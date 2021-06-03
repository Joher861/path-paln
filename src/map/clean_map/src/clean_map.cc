#include "clean_map.h"
#include "clean_map_log.h"
#include "clean_map_config.h"

#include "misc/robot_config.h"
#include "misc/planning_common_config.h"
#include "timer/timer.h"

using namespace planning_map;
using namespace planning_utils;

DeltaObMark::DeltaObMark()
{
}

DeltaObMark::DeltaObMark(uint8_t _old_type, uint8_t _new_type)
{
    old_type = _old_type;
    new_type = _new_type;
}

DeltaObCell::DeltaObCell()
{
}

DeltaObCell::DeltaObCell(const RobotGrid &_grid, ObMarkLayer::ObstacleType _type)
    : grid(_grid), type(_type)
{
}

DEFINE_CONFIG_TYPE(CONFIG_CLEAN_MAP, CleanMap);
CleanMap g_clean_map;

void CleanMap::init(ConfigManager &cfg_mgr,
                    const std::function<void(void)> &require_slam_map_func,
                    const std::function<int(uint8_t *)> &get_slam_map_func)
{
    configCleanMap = cfg_mgr;
    ConfigCleanMap *cfg_clean_map = dynamic_cast<ConfigCleanMap *>(
        cfg_mgr.GetSubConfig(CONFIG_CLEAN_MAP));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    ConfigRobot *cfg_robot = dynamic_cast<ConfigRobot *>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    cfg_clean_map->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_CLEAN_MAP_FLAG, LOG_CLEAN_MAP,
               cfg_clean_map->log_name, cfg_clean_map->log_path,
               cfg_clean_map->log_extension, cfg_clean_map->log_ts_mask,
               cfg_clean_map->log_print_to_console,
               (cfg_clean_map->log_max_file_size_mb)MB + (cfg_clean_map->log_max_file_size_kb)KB,
               cfg_clean_map->log_max_file_cnt, cfg_clean_map->log_level);

            float origin_x_temp = - (float)cfg_robot->robot_pose_in_local_map_x_default * cfg_robot->local_map_resolution;
            float origin_y_temp = - (float)cfg_robot->robot_pose_in_local_map_y_default * cfg_robot->local_map_resolution;

    initMapInfoLocal(cfg_clean_map->resolution, cfg_robot->local_map_resolution,
                     MapSize{cfg_clean_map->map_size_x, cfg_clean_map->map_size_y},
                     MapSize{cfg_robot->local_map_size_x, cfg_robot->local_map_size_y},
                     Point{cfg_clean_map->origin_pt_x, cfg_clean_map->origin_pt_y},
                     Point{origin_x_temp, origin_y_temp},
                     LOG_CLEAN_MAP);

    m_require_slam_map_func = require_slam_map_func;
    m_get_slam_map_func = get_slam_map_func;

    // 初始化sweep pattern
    m_sweep_path_length = 1;
    m_sweep_cleaned_length = static_cast<int32_t>(
        roundf(g_robot_cfg->main_brush_length / m_map_info->resolution));
    if (m_sweep_cleaned_length % 2 == 0)
        m_sweep_cleaned_length += 1;
    m_sweep_covered_length = static_cast<int32_t>(
        roundf(g_robot_cfg->footprint.width / m_map_info->resolution));
    if (m_sweep_covered_length % 2 == 0)
        m_sweep_covered_length += 1;
    createSweepUpdatePattern(SWEEP, CleanLayer::CLEANED_PATH,
                             CleanLayer::CLEANED, CleanLayer::COVERED);
    createSweepUpdatePattern(SWEEP_SIDE, CleanLayer::SIDE_CLEANED_PATH,
                             CleanLayer::SIDE_CLEANED, CleanLayer::COVERED);

    // 初始化清扫轨迹相关量
    m_clean_traj.clear();
    m_last_clean_traj_ts = 0;
    m_delta_ob_marks.clear();

    m_initialized = true;
}

void CleanMap::initLayer(std::string file_name)
{
    ConfigCleanMap *cfg_clean_map = dynamic_cast<ConfigCleanMap *>(
        configCleanMap.GetSubConfig(CONFIG_CLEAN_MAP));
    ConfigRobot *cfg_mgr = dynamic_cast<ConfigRobot *>(configCleanMap.GetSubConfig(CONFIG_ROBOT));

    if (file_name.size() > 3)
    {
        std::string tmp = file_name.substr(file_name.size() - 3, file_name.size());
        if (tmp == "pgm")
        {
            float resolution, origin_pt_x, origin_pt_y;
            const char *d = " ;";
            int map_size_x, map_size_y;
            std::ifstream pgm;
            pgm.open(file_name);
            std::string line;
            std::getline(pgm, line); // P5

            std::getline(pgm, line); // annotation

            char s[100];
            strcpy(s, line.c_str());
            char *p;
            p = strtok(s, d);
            for (int i = 0; i < 6; i++)
            {
                if (i == 3)
                    sscanf(p, "%f", &resolution);
                if (i == 4)
                    sscanf(p, "%f", &origin_pt_x);
                if (i == 5)
                    sscanf(p, "%f", &origin_pt_y);
                p = strtok(NULL, d);
            }

            std::getline(pgm, line); // annotation
            std::sscanf(line.c_str(), "%d %d", &map_size_x, &map_size_y);

            float origin_x_temp = - (float)cfg_mgr->robot_pose_in_local_map_x_default * cfg_mgr->local_map_resolution;
            float origin_y_temp = - (float)cfg_mgr->robot_pose_in_local_map_y_default * cfg_mgr->local_map_resolution;

            initMapInfoLocal(resolution, cfg_mgr->local_map_resolution,
                             MapSize{map_size_x, map_size_y},
                             MapSize{cfg_mgr->local_map_size_x, cfg_mgr->local_map_size_y},
                             Point{origin_pt_x, origin_pt_y},
                             Point{origin_x_temp, origin_y_temp},
                             LOG_CLEAN_MAP);
        }
    }

    std::unique_lock<shared_recursive_mutex> whole_lock(m_whole_clean_map_mtx);

    LayerNames names;
    for (auto &[name, layer] : m_layers)
        names.insert(name);
    deleteLayer(std::move(names));

    m_log_name = LOG_CLEAN_MAP;

    addLayer<CleanLayer, uint8_t>("clean", CleanLayer::UNKNOWN, m_log_name);
    m_clean_layer = std::dynamic_pointer_cast<CleanLayer>(getLayer("clean"));

    addLayer<ObMarkLayer, uint8_t>("ob_mark", ObMarkLayer::NONE, m_log_name);
    m_ob_mark_layer = std::dynamic_pointer_cast<ObMarkLayer>(getLayer("ob_mark"));

    addLayer<SlamLayer, uint8_t>("slam", SlamLayer::UNKNOWN, m_log_name);
    m_slam_layer = std::dynamic_pointer_cast<SlamLayer>(getLayer("slam"));

    addLayer<MaskLayer, uint8_t>("mask", MaskLayer::MASK_NONE, m_log_name);
    m_mask_layer = std::dynamic_pointer_cast<MaskLayer>(getLayer("mask"));

    addLayer<LocalLayer, uint8_t>("local", LocalLayer::UNKNOWN, m_log_name);
    m_local_layer = std::dynamic_pointer_cast<LocalLayer>(getLayer("local"));

    addLayer<InflateLayer, InflateCell>("inflate",
                                        InflateCell{InflateLayer::MASK_NONE, RobotGrid{0, 0},
                                                    Point{0.0f, 0.0f}, 0.0f},
                                        m_log_name);

    m_inflate_layer = std::dynamic_pointer_cast<InflateLayer>(getLayer("inflate"));

    m_inflate_layer->setMaxInflateRange(cfg_clean_map->max_inflate_range);
    m_inflate_layer->setMaskRange(std::vector<float>{
        cfg_clean_map->smaller_inflate_radius_offset, 0.0f});
}
void CleanMap::reset()
{
    std::unique_lock<shared_recursive_mutex> whole_lock(m_whole_clean_map_mtx);
    resetMap();
}

void CleanMap::createSweepUpdatePattern(std::string name,
                                        CleanLayer::CleanStatus path_status,
                                        CleanLayer::CleanStatus cleaned_status,
                                        CleanLayer::CleanStatus covered_status)
{
    std::vector<CleanLayer::CleanStatus> pattern;
    int32_t max_length = m_sweep_path_length;
    for (int32_t i = 0; i < m_sweep_path_length; ++i)
        pattern.push_back(path_status);
    if (m_sweep_cleaned_length > max_length)
    {
        int32_t cleaned_length = (m_sweep_cleaned_length - max_length) / 2;
        max_length = m_sweep_cleaned_length;
        pattern.insert(pattern.begin(), cleaned_length, cleaned_status);
        for (int32_t i = 0; i < cleaned_length; ++i)
            pattern.push_back(cleaned_status);
    }
    if (m_sweep_covered_length > max_length)
    {
        int32_t covered_length = (m_sweep_covered_length - max_length) / 2;
        pattern.insert(pattern.begin(), covered_length, covered_status);
        for (int32_t i = 0; i < covered_length; ++i)
            pattern.push_back(covered_status);
    }
    m_sweep_update_patterns.emplace(name, pattern);
}

void CleanMap::requireSlamMap()
{
    m_require_slam_map_func();
}

bool CleanMap::getSlamMap()
{
    return (m_get_slam_map_func(m_slam_layer->getMapAddr()) == 0);
}

void CleanMap::getInflateFreesAndObs(std::vector<RobotGrid> &frees,
                                     std::vector<std::pair<RobotGrid, float>> &obs)
{
    frees.clear();
    obs.clear();

    std::vector<RobotGrid> slam_frees, slam_obs;
    m_slam_layer->getFreesAndObstacles(slam_frees, slam_obs);
    for (auto &free : slam_frees)
    {
        frees.emplace_back(free);
    }
    for (auto &ob : slam_obs)
    {
        obs.emplace_back(ob, g_robot_cfg->footprint.inner_radius);
    }
}

void CleanMap::inflateMap()
{
    std::vector<RobotGrid> frees;
    std::vector<std::pair<RobotGrid, float>> obs;
    getInflateFreesAndObs(frees, obs);
    std::unique_lock<std::mutex> inflate_lock(m_inflate_mtx);
    m_inflate_layer->inflateMap(frees, obs);
    // std::string map_name = "/tmp/log/test/inflate_map." + std::to_string(cnt++)
    //     + ".bmp";
    // cv::imwrite(map_name.c_str(),
    //     m_inflate_layer->saveMapToMat(RobotGridRect{}, SAVE_WHOLE));
}

void CleanMap::getInflateMap(uint8_t mask, uint8_t **dest)
{
    m_inflate_layer->getMaskMap(mask, dest);
}

// void CleanMap::loadSlamMap(std::string img_name)
// {
//     cv::Mat map = cv::imread(img_name);
//     cv::cvtColor(map, map, CV_BGR2GRAY);
//     m_slam_layer->loadMapFromMat(m_map_info->grid_range, map);
// }

void CleanMap::getLocalMap(uint8_t **dest)
{
    m_local_layer->getLocalMap2(dest);
}

void CleanMap::updateCleanStatus(const RobotGrid &grid, CleanLayer::CleanStatus status)
{
    if (!inRange(grid))
        return;
    if (status >= CleanLayer::CLEAN_STATUS_MAX)
        return;

    uint8_t &target_status = (*m_clean_layer)[grid];
    if (status > target_status)
        target_status = status;
}

void CleanMap::updateCleanStatus(const RobotPose &pose, CleanLayer::CleanStatus status)
{
    RobotGrid grid = poseToGridWithoutBoundary(pose);
    updateCleanStatus(grid, status);
}

void CleanMap::updateCleanStatus(const RobotGrid &local_grid, const RobotPose &ref_pose,
                                 CleanLayer::CleanStatus status)
{
    Point local_pt{local_grid};
    local_pt = local_pt * m_map_info->resolution;
    RobotPose global_pose = transformFrame(ref_pose, RobotPose{local_pt});
    updateCleanStatus(global_pose, status);
}

void CleanMap::updateCleanStatus(const Point &local_pt, const RobotPose &ref_pose,
                                 CleanLayer::CleanStatus status)
{
    RobotPose global_pose = transformFrame(ref_pose, RobotPose{local_pt});
    updateCleanStatus(global_pose, status);
}

void CleanMap::updateCleanStatus(const std::vector<CleanLayer::CleanStatus> &pattern,
                                 const RobotPose &ref_pose, int32_t center_idx)
{
    int32_t status_size = static_cast<int32_t>(pattern.size());
    if (center_idx < 0)
    {
        if (status_size % 2 == 0)
        {
            CMAP_WARN_LOG("status size = %d is not odd", status_size);
            return;
        }
        center_idx = center_idx / 2;
    }
    else
    {
        if (center_idx >= status_size)
        {
            CMAP_WARN_LOG("center_idx = %d is larger than status size = %d",
                          center_idx, status_size);
            return;
        }
    }

    for (int32_t idx = 0; idx < status_size; ++idx)
    {
        RobotGrid local_grid;
        local_grid.y = center_idx - idx;
        updateCleanStatus(local_grid, ref_pose, pattern[idx]);
    }
}

void CleanMap::updateCleanStatus(const std::string &name, const RobotPose &ref_pose)
{
    auto it = m_sweep_update_patterns.find(name);
    if (it == m_sweep_update_patterns.end())
        return;

    updateCleanStatus(it->second, ref_pose);
}

void CleanMap::updateObMark(const RobotGrid &grid, ObMarkLayer::ObstacleType type)
{
    if (!inRange(grid))
        return;

    // TODO: 更新逻辑
    uint8_t old_type = 0;
    uint8_t new_type = 0;
    bool need_update = false;

    // 更新障碍变化量
    if (need_update)
    {
        std::lock_guard<std::mutex> delta_ob_marks_lock(m_delta_ob_marks_mtx);
        m_delta_ob_marks.emplace(grid, DeltaObMark{old_type, new_type});
    }
}

void CleanMap::updateCleanTraj(const RobotPose &pose)
{
    std::lock_guard<std::mutex> traj_lock(m_clean_traj_mtx);
    uint64_t cur_ts = Timer::getSystemTimestampUS();
    if (m_clean_traj.empty())
    {
        m_clean_traj.add(pose, cur_ts);
        return;
    }

    bool need_add = false;
    auto [last_traj_pose, last_traj_ts] = m_clean_traj.back();

    // 如果距离上一个点位置相差超过0.1m，将该点加入轨迹
    if (getDistance(pose.pt, last_traj_pose.pt) >= 0.1f)
        need_add = true;
    // 如果距离上一个点角度差超过30度，将该点加入轨迹
    else if (fabs(angleBetweenR(pose.theta, last_traj_pose.theta)) > deg2rad(30.0f))
        need_add = true;
    // 如果距离上一个点时间差超过5s，将该点加入轨迹
    else if (cur_ts - last_traj_ts > 5000000)
        need_add = true;

    if (need_add)
        m_clean_traj.add(pose, cur_ts);
}

std::list<RobotPose> CleanMap::getDeltaCleanTraj()
{
    std::lock_guard<std::mutex> traj_lock(m_clean_traj_mtx);
    std::list<RobotPose> poses;
    size_t traj_size = m_clean_traj.size();
    for (int32_t idx = traj_size - 1; idx >= 0; --idx)
    {
        if (auto &&[pose, ts] = m_clean_traj[idx]; ts >= m_last_clean_traj_ts)
            poses.push_front(pose);
    }
    m_last_clean_traj_ts = Timer::getSystemTimestampUS();
    return poses;
}

std::list<DeltaObCell> CleanMap::getDeltaObMark()
{
    std::lock_guard<std::mutex> delta_ob_marks_lock(m_delta_ob_marks_mtx);
    std::list<DeltaObCell> ob_cells;
    for (auto &&[grid, delta_mark] : m_delta_ob_marks)
    {
        auto old_type = m_ob_mark_layer->getType(delta_mark.old_type);
        auto new_type = m_ob_mark_layer->getType(delta_mark.new_type);
        if (old_type != new_type)
            ob_cells.emplace_back(grid, new_type);
    }
    m_delta_ob_marks.clear();
    return ob_cells;
}

RobotGridRect CleanMap::generatePersistMap(std::string map_name)
{
    CMAP_INFO_LOG("save persist map = %s", map_name.c_str());
    auto &&[ob_mark_map, rect] = m_ob_mark_layer->saveObMarkMapToMat(RobotGridRect{},
                                                                     SAVE_OCCUPIED);

    CMAP_INFO_LOG("persist map rect = %s", rect.toString().c_str());

    if (rect.getSize() == RobotGrid{0, 0})
        return rect;

    return rect;
    cv::imwrite(map_name, ob_mark_map);
}

bool CleanMap::loadPersistMap(std::string map_name, const RobotGridRect &region)
{
    CMAP_INFO_LOG("load persist map = %s", map_name.c_str());
    cv::Mat &&persist_ob_mark_map = cv::imread(map_name);

    bool loaded = m_ob_mark_layer->loadObMarkMapFromMat(region, persist_ob_mark_map);
    if (loaded)
        CMAP_INFO_LOG("load persist map success");
    else
        CMAP_INFO_LOG("load persist map failed");

    return loaded;
}

void CleanMap::updateLocalLayer()
{
    m_local_layer->update();
}
