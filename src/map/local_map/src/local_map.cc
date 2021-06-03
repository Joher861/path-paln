#include "local_map.h"
#include "include/local_map_log.h"
#include "include/local_map_config.h"
#include "include/footprint.h"
#include "include/line_iterator.h"
#include "vb_detect_log.h"

#include <tuple>

#include "misc/robot_config.h"
#include "misc/planning_common_config.h"
#include "misc/color_print.h"
#include "data_center/data_center.h"
#include "timer/sleep_timer.h"
#include "data/control_speed_data.h"
#include "event/global_common_event_def.h"
#include "task/task_manager.h"
#include "basic_planner/basic_task.h"

#include "geometry/geometry_func.h"
#include <stdio.h>
#include <sstream>


using namespace planning_map;
using namespace planning_utils;
using namespace planning_data;
using namespace planning_controller;

DEFINE_CONFIG_TYPE(CONFIG_LOCAL_MAP, LocalMap);
LocalMap g_local_map;

LocalMap::LocalMap()
    : GridMap(), m_run_local_map(false),
    m_print_local_map(false),
    m_print_local_inflate_map(false)
{}

LocalMap::~LocalMap()
{
    if (m_local_map_updating_thread)
    {
        m_local_map_updating_thread->join();
        delete m_local_map_updating_thread;
        m_local_map_updating_thread = nullptr;
    }

    if(m_viewer)
    {
        delete m_viewer;
        m_viewer = nullptr;
    }

}

void LocalMap::init(ConfigManager &cfg_mgr, 
        const std::function<int(SimpleLocalMap *local_map)> &get_local_map_func)
{
    m_log_name = LOG_LOCAL_MAP;

    ConfigLocalMap *cfg_local_map = dynamic_cast<ConfigLocalMap*>(
            cfg_mgr.GetSubConfig(CONFIG_LOCAL_MAP));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning*>(
            cfg_mgr.GetSubConfig(CONFIG_PLANNING));

    cfg_local_map->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_LOCAL_MAP_FLAG, LOG_LOCAL_MAP,
        cfg_local_map->log_name, cfg_local_map->log_path,
        cfg_local_map->log_extension, cfg_local_map->log_ts_mask,
        cfg_local_map->log_print_to_console,
        (cfg_local_map->log_max_file_size_mb) MB
        + (cfg_local_map->log_max_file_size_kb) KB,
        cfg_local_map->log_max_file_cnt, cfg_local_map->log_level);
   
    // CREATE_LOG(Img, LOG_LOCAL_MAP_IMG_FLAG, LOG_LOCAL_MAP_IMG, "local_map_img",
    //     "/tmp/testlog/test", "bmp", READABLE_US, true, 1000);

    //获取footprint相关参数
    // TODO:string 参数配置不对，字符串显示不出，里面没有内容
    // std::string footstr("[[1.05, -0.3],[1.05, 0.3], [-0.15,0.3],[-0.15,-0.3]]");//TODO:人为设置只为了测试
    std::string footstr("[[0.95, -0.2],[0.95, 0.2], [-0.15,0.2],[-0.15,-0.2]]");//TODO:人为设置只为了测试

    // makeFootprintFromString(cfg_local_map->foot_str, m_footprint_spec);
    makeFootprintFromString(footstr, m_footprint_spec);
    calculateMinAndMaxDistances(m_footprint_spec, m_inscribed_radius, m_circumscribed_radius);
    // printf("footprint_ptr ok:%s\r\n",cfg_local_map->foot_str.c_str());
    printf("footprint_ptr ok:%s\r\n",footstr);
    printf("footprint_spec{(%f,%f),(%f,%f),(%f,%f),(%f,%f)} m_inscribed_radius=%f m_circumscribed_radius=%f\r\n",m_footprint_spec[0].x,m_footprint_spec[0].y,
      m_footprint_spec[1].x,m_footprint_spec[1].y,m_footprint_spec[2].x,m_footprint_spec[2].y,
      m_footprint_spec[3].x,m_footprint_spec[3].y,m_inscribed_radius,m_circumscribed_radius);

    m_sdf_map.reset(new SDFMap);
    m_sdf_map->initMap(cfg_local_map->map_size_x, cfg_local_map->map_size_y,
        cfg_local_map->resolution,cfg_local_map->max_inflate_range);

    // 初始化base map
    initMapInfo(cfg_local_map->resolution,
        MapSize{cfg_local_map->map_size_x, cfg_local_map->map_size_y},
        Point{cfg_local_map->origin_pt_x, cfg_local_map->origin_pt_y},
        LOG_LOCAL_MAP);

    addLayer<LocalBaseLayer, LocalCell>("local_base", LocalCell(), m_log_name);
    m_local_base_layer
        = std::dynamic_pointer_cast<LocalBaseLayer>(getLayer("local_base"));
    addLayer<InflateLayer, InflateCell>("inflate_base", InflateCell(), m_log_name);
    m_inflate_base_layer
        = std::dynamic_pointer_cast<InflateLayer>(getLayer("inflate_base"));
    m_inflate_base_layer->setMaxInflateRange(cfg_local_map->max_inflate_range);
    m_inflate_base_layer->setMaskRange(std::vector<float>{
        0.0f,
        cfg_local_map->path_inflate_radius_offset
    });
    addLayer<LocalBaseLayer, LocalCell>("inflate_local_base", LocalCell(), m_log_name);
    m_inflate_local_base_layer
        = std::dynamic_pointer_cast<LocalBaseLayer>(getLayer("inflate_local_base"));

    addLayer<LocalCharBaseLayer, uint8_t>("char_local_base", 0, m_log_name);
    m_local_char_base_layer
        = std::dynamic_pointer_cast<LocalCharBaseLayer>(getLayer("char_local_base"));
    if(m_local_char_base_layer)
        LMAP_DEBUG_LOG("m_local_char_base_layer is ok.");

    // 设置其余参数
    m_print_local_map = cfg_local_map->print_local_map;
    m_print_local_inflate_map = cfg_local_map->print_local_inflate_map;
    m_rviz_display = cfg_local_map->rviz_display;
    m_use_esdf_map = cfg_local_map->use_esdf_map;

    printf("m_rviz_display=%d m_use_esdf_map=%d\r\n",cfg_local_map->rviz_display,m_use_esdf_map);

    // 初始化window map
    MapSize window_size{
        cfg_local_map->window_size_x,
        cfg_local_map->window_size_y
    };
    if (window_size.x > m_map_info->size.x || window_size.y > m_map_info->size.y)
    {
        std::string err_str = "window size " + window_size.toString()
            + " > map size " + m_map_info->size.toString() + ", not valid";
        LMAP_ERROR_LOG("%s", err_str.c_str());
        throw std::invalid_argument(err_str.c_str());
        return;
    }

    m_window_map.init(cfg_local_map);

    m_window_size = m_window_map.getMapInfo().size;

    m_to_window_center_offset
        = { m_window_size.x / 2, m_window_size.y / 2 };

    resetLocalMap();

    // 初始化footprint
    Point footprint_x_max_pt = m_map_info->origin_pt
        + Point{g_robot_cfg->footprint.inner_radius, 0.0f};
    Point footprint_y_max_pt = m_map_info->origin_pt
        + Point{0.0f, g_robot_cfg->footprint.inner_radius};
    if (!m_window_map.inRange(footprint_x_max_pt)
        || !m_window_map.inRange(footprint_y_max_pt))
    {
        std::string err_str = "footprint radius "
            + std::to_string(g_robot_cfg->footprint.inner_radius)
            + " too large for local map";
        LMAP_ERROR_LOG("%s", err_str.c_str());
        throw std::invalid_argument(err_str.c_str());
    }

    m_get_local_map_func = get_local_map_func;

    // 初始化更新线程
    m_local_map_polling_frequency = cfg_local_map->local_map_polling_frequency;
    m_local_map_updating_frequency = cfg_local_map->local_map_updating_frequency;
    m_cur_tof_ts = 0;

    m_local_map_frame_cnt = 0;
    m_inflate_map_frame_cnt = 0;
    m_frame_cnt = 0;

    m_viewer = new LocalMapViewer();

    m_local_map_updating_thread
        = new std::thread(&LocalMap::updateLocalMap, this);
    // if (!m_local_map_updating_thread)
    // {
    //     m_local_map_updating_thread->detach();
    // }

    
    m_initialized = true;
}

void LocalMap::start()
{
    m_run_local_map = true;
    printf("local map starting.\r\n");
}

void LocalMap::stop()
{
    m_run_local_map = false;
}

void LocalMap::resetLocalMap()
{
    std::unique_lock<shared_recursive_mutex> whole_lock(m_whole_map_mtx);
    std::unique_lock<shared_recursive_mutex> inflate_lock(m_inflate_map_mtx);

    LMAP_INFO_LOG("reset local map");

    DataSlam slam_data;
    int8_t ok = g_dc.getData<DataSlam>(slam_data);

    if (ok != 0)
    {
        // 如果没有获取到
        LMAP_WARN_LOG("failed to get slam data, reset map to { 0.0, 0.0, 0.0 }");
        slam_data.pose = RobotPose{};
    }

    m_map_info->setOriginPt(slam_data.pose.pt);
    m_window_map.setOriginPt(m_map_info->origin_pt);

    m_window_center_in_base_map = m_map_info->origin_grid;
    m_window_rect_in_base_map = RobotGridRect{
        m_map_info->origin_grid - m_to_window_center_offset,
        m_map_info->origin_grid + m_to_window_center_offset
    };
    
    resetMap();
    m_window_map.resetMap();
}


template <typename T>
void LocalMap::moveOffsetGrids(std::string layer_name, const RobotGrid &old_start,
    const RobotGrid &old_end, const RobotGrid &offset, const RobotGrid &new_end)
{
    auto it = m_layers.find(layer_name);
    if (it == m_layers.end())
        return;
    std::shared_ptr<LayerMap<T>> layer
        = std::reinterpret_pointer_cast<LayerMap<T>>(
            it->second);

    if (offset.x < 0 || offset.y < 0)
    {
        RobotGridRect reset_range{
            new_end,
            new_end + m_window_size - RobotGrid{1, 1}
        };

        for (auto & [name, layer] : m_layers)
        {
            layer->reset(reset_range);
        }

        return;
    }

    RobotGrid intersect_size = old_start - old_end + RobotGrid{1, 1};

    std::string layer_base = layer_name.substr(0, layer_name.find("base"));
    std::string window_layer_name = layer_base + "window";
    m_window_map.resetLayer(LayerNames{window_layer_name});

    std::shared_ptr<LayerMap<T>> window_layer
        = std::reinterpret_pointer_cast<LayerMap<T>>(
            m_window_map.getLayer(window_layer_name));

    RobotGridRect window_rect{
        offset,
        offset + intersect_size - RobotGrid{1, 1}
    };
    RobotGridRect base_old_rect{
        old_end,
        old_end + intersect_size - RobotGrid{1, 1}
    };
    RobotGridRect base_new_rect{
        new_end,
        new_end + m_window_size - RobotGrid{1, 1}
    };

    window_layer->setMapFromLayer(window_rect, layer, base_old_rect);
    layer->setMapFromLayer(base_new_rect, window_layer,
        m_window_map.getMapInfo().grid_range);
}

template <typename T>
void LocalMap::clearOffsetGrids(std::string layer_name, const RobotGrid &offset,
    const RobotGridRect &new_rect)
{
    auto it = m_layers.find(layer_name);
    if (it == m_layers.end())
        return;
    std::shared_ptr<LayerMap<T>> layer
        = std::reinterpret_pointer_cast<LayerMap<T>>(
            it->second);

    int32_t row_delete_start_x, row_delete_end_x;
    int32_t col_delete_start_x, col_delete_end_x;
    int32_t col_delete_start_y, col_delete_end_y;

    RobotGrid new_start = new_rect.max_grid;
    RobotGrid new_end = new_rect.min_grid;

    if (offset.x >= 0)
    {
        row_delete_start_x = new_start.x - offset.x + 1;
        row_delete_end_x = new_start.x;
        col_delete_start_x = new_end.x;
        col_delete_end_x = new_start.x - offset.x;
    }
    else
    {
        row_delete_start_x = new_end.x;
        row_delete_end_x = new_end.x - offset.x - 1;
        col_delete_start_x = new_end.x - offset.x;
        col_delete_end_x = new_start.x;
    }

    if (offset.y >= 0)
    {
        col_delete_start_y = new_start.y - offset.y + 1;
        col_delete_end_y = new_start.y;
    }
    else
    {
        col_delete_start_y = new_end.y;
        col_delete_end_y = new_end.y - offset.y - 1;
    }

    RobotGridRect clear_range1{
        RobotGrid{row_delete_start_x, new_end.y},
        RobotGrid{row_delete_end_x, new_end.y + m_window_size.y - 1}
    };
    layer->reset(clear_range1);

    RobotGridRect clear_range2{
        RobotGrid{col_delete_start_x, col_delete_start_y},
        RobotGrid{col_delete_end_x, col_delete_end_y}
    };
    layer->reset(clear_range2);

    return;
}

void LocalMap::resetWindow(const RobotPose &new_center)
{
    m_window_center_in_base_map = m_map_info->origin_grid;
    m_window_rect_in_base_map = RobotGridRect{
        m_map_info->origin_grid - m_to_window_center_offset,
        m_map_info->origin_grid + m_to_window_center_offset
    };

    m_window_map.setOriginPt(new_center.pt);

}

bool LocalMap::moveWindow(const RobotPose &old_center_pose,
    const RobotPose &new_center_pose)
{
    RobotGrid old_center_grid = poseToGridWithoutBoundary(old_center_pose);
    RobotGrid new_center_grid = poseToGridWithoutBoundary(new_center_pose);//新的window中心在老的window中的栅格坐标

    RobotGrid offset = new_center_grid - old_center_grid;

    RobotGrid new_window_center = new_center_grid;
    RobotGridRect new_window_rect{
        m_window_rect_in_base_map.min_grid + offset,
        m_window_rect_in_base_map.max_grid + offset
    };
    
    if (!inRange(new_window_rect.max_grid)  //移动后的窗口是否超出base_map的范围
        || !inRange(new_window_rect.min_grid))
    {
        return false;
    }
    else
    {
        // 移动window
        m_window_center_in_base_map = new_window_center;
        m_window_rect_in_base_map = new_window_rect;
        m_window_map.setOriginPt(new_center_pose.pt);

        RobotGrid rect_size = m_window_rect_in_base_map.getSize();
        RobotGrid max_to_center_offset
            = m_window_rect_in_base_map.max_grid - m_window_center_in_base_map;
        RobotGrid center_to_min_offset
            = m_window_center_in_base_map - m_window_rect_in_base_map.min_grid;
        if (rect_size != m_window_size
            || max_to_center_offset != m_to_window_center_offset
            || center_to_min_offset != m_to_window_center_offset)
        {
            LMAP_ERROR_LOG("new window size %s not match window size %s",
                m_window_rect_in_base_map.getSize().toString().c_str(),
                m_window_size.toString().c_str());
            LMAP_ERROR_LOG("or max to center %s not match to center offset %s",
                max_to_center_offset.toString().c_str(),
                m_to_window_center_offset.toString().c_str());
            LMAP_ERROR_LOG("or center to min %s not match to center offset %s",
                center_to_min_offset.toString().c_str(),
                m_to_window_center_offset.toString().c_str());
            int *segv_ptr = nullptr;
            int segv = *segv_ptr;
        }

        clearOffsetGrids<LocalCell>("local_base", offset, m_window_rect_in_base_map);
        clearOffsetGrids<InflateCell>("inflate_base", offset, m_window_rect_in_base_map);
        clearOffsetGrids<LocalCell>("inflate_local_base", offset, m_window_rect_in_base_map);
    }

    return true;
}

void LocalMap::moveBaseMap(const RobotPose &old_center_pose,
    const RobotPose &new_center_pose)
{
    RobotGrid old_base_map_grid = poseToGridWithoutBoundary(old_center_pose);
    RobotGrid new_base_map_grid = poseToGridWithoutBoundary(new_center_pose);

    RobotGrid offset = new_base_map_grid - old_base_map_grid;
    
    RobotGrid old_window_end_in_base_map = m_window_rect_in_base_map.min_grid;
    RobotGrid old_window_start_in_base_map = m_window_rect_in_base_map.max_grid;

    resetWindow(new_center_pose);

    // 如果新的区域和旧的区域没有相交的部分，reset window之后直接把window中的cell清空
    if (abs(offset.x) >= m_window_size.x
        || abs(offset.y) >= m_window_size.y)
    {
        LMAP_ERROR_LOG("old_center_pose = %s, new_center_pose = %s",
            old_center_pose.toString().c_str(),
            toMapCellPose(new_center_pose, m_map_info->resolution).toString().c_str());
        LMAP_ERROR_LOG("old_base_map_center = %s, new_base_map_center = %s",
            old_base_map_grid.toString().c_str(),
            new_base_map_grid.toString().c_str());
        LMAP_ERROR_LOG("offset = %s", offset.toString().c_str());
        m_local_base_layer->reset(m_window_rect_in_base_map);
        m_inflate_base_layer->reset(m_window_rect_in_base_map);
        m_inflate_local_base_layer->reset(m_window_rect_in_base_map);
        m_map_info->setOriginPt(new_center_pose.pt);

        return;
    }

    RobotGrid old_start_grid, new_start_grid, old_end_grid, new_end_grid;
    if (offset.x >= 0)
    {
        old_start_grid.x = old_window_start_in_base_map.x;
        new_start_grid.x = m_window_rect_in_base_map.max_grid.x - offset.x;
        old_end_grid.x = old_window_end_in_base_map.x + offset.x;
        new_end_grid.x = m_window_rect_in_base_map.min_grid.x;
    }
    else
    {
        old_start_grid.x = old_window_start_in_base_map.x + offset.x;
        new_start_grid.x = m_window_rect_in_base_map.max_grid.x;
        old_end_grid.x = old_window_end_in_base_map.x;
        new_end_grid.x = m_window_rect_in_base_map.min_grid.x - offset.x;
    }
    if (offset.y >= 0)
    {
        old_start_grid.y = old_window_start_in_base_map.y;
        new_start_grid.y = m_window_rect_in_base_map.max_grid.y - offset.y;
        old_end_grid.y = old_window_end_in_base_map.y + offset.y;
        new_end_grid.y = m_window_rect_in_base_map.min_grid.y;
    }
    else
    {
        old_start_grid.y = old_window_start_in_base_map.y + offset.y;
        new_start_grid.y = m_window_rect_in_base_map.max_grid.y;
        old_end_grid.y = old_window_end_in_base_map.y;
        new_end_grid.y = m_window_rect_in_base_map.min_grid.y - offset.y;
    }

    RobotGrid intersect_offset;
    RobotGrid real_old_start_grid, real_old_end_grid, real_new_end_grid;
    if (old_start_grid.x < 0
        || old_start_grid.y < 0
        || old_end_grid.x >= m_window_size.x
        || old_end_grid.y >= m_window_size.y)
    {
        intersect_offset = { -1, -1 };
    }
    else
    {
        real_old_start_grid.x = std::min(old_start_grid.x, m_window_size.x - 1);
        real_old_start_grid.y = std::min(old_start_grid.y, m_window_size.y - 1);
        real_old_end_grid.x = std::max(old_end_grid.x, 0);
        real_old_end_grid.y = std::max(old_end_grid.y, 0);
        RobotGrid inbound_end_offset = real_old_end_grid - old_end_grid;
        real_new_end_grid = new_end_grid + inbound_end_offset;

        intersect_offset = real_new_end_grid - m_window_rect_in_base_map.min_grid;
    }

    moveOffsetGrids<LocalCell>("local_base", real_old_start_grid, real_old_end_grid,
        intersect_offset, m_window_rect_in_base_map.min_grid);
    moveOffsetGrids<InflateCell>("inflate_base", real_old_start_grid, real_old_end_grid,
        intersect_offset, m_window_rect_in_base_map.min_grid);
    moveOffsetGrids<LocalCell>("inflate_inflate_base", real_old_start_grid,
        real_old_end_grid, intersect_offset, m_window_rect_in_base_map.min_grid);
    m_map_info->setOriginPt(new_center_pose.pt);
}

void LocalMap::clearRobotFootprint(const RobotPose &robot_pose, LocalBaseLayerPtr layer)
{
    int32_t clear_cnt = 0;
    for (auto &grid : g_robot_cfg->footprint.grids)
    {
        RobotPose pose{
            grid.x * g_robot_cfg->footprint.resolution,
            grid.y * g_robot_cfg->footprint.resolution,
            0.0f
        };

        RobotPose global_pose = transformFrame(robot_pose, pose);
        RobotGrid base_map_grid = poseToGridWithoutBoundary(global_pose);
        if (inRange(base_map_grid))
        {
            auto &local_cell = (*layer)[base_map_grid];
            if (local_cell.status == OCCUPIED)
                clear_cnt++;
            local_cell = LocalCell{};
            // layer->set(base_map_grid, LocalCell{});
        }
    }

    if (clear_cnt != 0)
        LMAP_WARN_LOG("clear footprint cnt = %d", clear_cnt);
}

void LocalMap::printLocalMap(const RobotPose &cur_pose)
{

}

void LocalMap::printLocalInflateMap(const RobotPose &cur_pose)
{
    printf("\033[2J\033[1;1H");
    MapSize size = m_window_rect_in_base_map.getSize();
    RobotGrid window_min_grid = m_window_rect_in_base_map.min_grid;
    RobotGrid origin_grid = m_window_center_in_base_map;
    InflateLayer &inflate_base_layer = *m_inflate_base_layer;
    LocalBaseLayer &inflate_local_base_layer = *m_inflate_local_base_layer;

    std::unordered_set<RobotGrid, RobotGridHash> contour_grids;
    std::unordered_set<RobotGrid, RobotGridHash> inner_grids;
    for (auto &grid : g_robot_cfg->footprint.contour_grids)
    {
        contour_grids.emplace(grid);
    }
    for (auto &grid : g_robot_cfg->footprint.grids)
    {
        inner_grids.emplace(grid);
    }

    auto contour_end = contour_grids.end();
    auto inner_end = inner_grids.end();

    for (int i = size.x - 1; i >= 0; --i)
    {
        for (int j = size.y - 1; j >= 0; --j)
        {
            RobotGrid grid = RobotGrid{i, j} + window_min_grid;

            RobotPose pose_w = gridToPose(grid);
            RobotPose pose_r = reverseTransformFrame(cur_pose, pose_w);
            RobotGrid grid_r{
                static_cast<int32_t>(roundf(pose_r.pt.x * 100.0f)),
                static_cast<int32_t>(roundf(pose_r.pt.y * 100.0f))
            };

            if (inner_grids.find(grid_r) != inner_end)
            {
                RobotGrid rp{origin_grid};
                RobotGrid tp{grid};
                float grid_angle_r = getAngleR(rp, tp);
                float angle_diff_r = angleBetweenR(cur_pose.theta, grid_angle_r);
                
                float range_r = deg2rad(10.0);
                if (withinAngleRangeR(grid_angle_r, cur_pose.theta - range_r,
                    cur_pose.theta + range_r))
                {
                    printf(COLOR_GREEN COLOR_BOLD "＃" COLOR_NONE);
                    continue;
                }
            }

            if (contour_grids.find(grid_r) != contour_end)
            {
                printf(COLOR_GREEN COLOR_BOLD "＃" COLOR_NONE);
                continue;
            }

            // float dist = getDistance(grid, origin_grid) * m_map_info->resolution;
            // if (dist <= g_robot_cfg->footprint.inner_radius)
            // {
            //     RobotGrid rp{origin_grid};
            //     RobotGrid tp{grid};
            //     float grid_angle_r = getAngleR(rp, tp);
            //     float angle_diff_r = angleBetweenR(cur_pose.theta, grid_angle_r);
                
            //     float range_r = deg2rad(10.0);
            //     if (withinAngleRangeR(grid_angle_r, cur_pose.theta - range_r,
            //         cur_pose.theta + range_r))
            //     {
            //         printf(COLOR_GREEN COLOR_BOLD "＃" COLOR_NONE);
            //         continue;
            //     }
            // }

            if (i == size.x - 1 || i == 0 || j == size.y - 1 || j == 0)
            {
                printf(COLOR_GREEN COLOR_BOLD "＃" COLOR_NONE);
                continue;
            }

            InflateCell &inflate_cell = inflate_base_layer[grid];
            uint8_t inflate_mask = inflate_cell.mask;

            if (m_inflate_base_layer->isMasked(grid, InflateLayer::MASK_SRC))
            {
                RobotGrid src_grid = inflate_cell.src_grid;
                uint8_t type = inflate_local_base_layer[src_grid].type;
                // if (type == HIGH_OBSTACLE)
                //     printf(BLUE BOLD "＠" NONE);
                // else if (type == LOW_OBSTACLE)
                //     printf(RED BOLD "＠" NONE);
                // else if (type == VERY_LOW_OBSTACLE)
                //     printf(L_BLACK BOLD "＠" NONE);
                // else if (type == TRAP_OBSTACLE)
                //     printf(PURPLE BOLD "＠" NONE);
                // else if (type == LEGS_OBSTACLE)
                //     printf(YELLOW BOLD "＠" NONE);
                // else if (type == THRESH_OBSTACLE)
                //     printf(GRAY BOLD "＠" NONE);
                // else
                //     printf(WHITE BOLD "＠" NONE);
                printf(COLOR_L_RED  COLOR_BOLD  "＠" COLOR_NONE);
            }
            else if (m_inflate_base_layer->isMasked(grid, NORMAL_INFLATE_RADIUS))
                printf(COLOR_BOLD "〇" COLOR_NONE);
            else if (m_inflate_base_layer->isMasked(grid, InflateLayer::MASK_FREE))
                printf(COLOR_BOLD "＋" COLOR_NONE);
            else if (m_inflate_base_layer->isMasked(grid, InflateLayer::MASK_NONE))
                printf("　");
        }
        printf("\n");
    }
    printf("\n");
}

void LocalMap::updateLocalMap()
{
    uint64_t update_step_time = 1000000 / m_local_map_updating_frequency;
    uint64_t loop_ts = Timer::getSystemTimestampUS();
    uint64_t last_update_ts = loop_ts;
    bool tof_updated = false;
    bool update_time_reached = false;
    uint32_t bmp_num = 0;

    // LocalCell *local_new_received_map
    //     = m_window_map.getLayerMapAddr<LocalCell>("local_new_received");
    // LocalBaseLayerPtr local_new_received_layer
    //     = std::reinterpret_pointer_cast<LocalBaseLayer>(
    //         m_window_map.getLayer("local_new_received"));
    // InflateLayerPtr inflate_window_layer
    //     = std::reinterpret_pointer_cast<InflateLayer>(
    //         m_window_map.getLayer("inflate_window"));

    DataSlam slam_data;

    // if (!local_new_received_map)
    // {
    //     std::string err_str{"local new received layer in null"};
    //     LMAP_ERROR_LOG("%s", err_str.c_str());
    //     throw std::runtime_error(err_str.c_str());
    //     return;
    // }

    SimpleLocalMap simple_local_map;
    //local_map_data_.grid[col][row] 中的row填入mx，col填入my
    auto getCost = [&](unsigned int mx, unsigned int my)->unsigned char
    {
        if(simple_local_map.grid[my][mx] != 0)
        {
            return OCCUPIED;
        }
        return FREE;
    };

    //local_map_data_.grid[col][row] 中的row对应mx，col对应my
    auto world2map = [&](float wx, float wy, unsigned int* mx, unsigned int* my)->bool
    {
        RobotPose world_pose(wx,wy,0);
        RobotPose origin_pose(simple_local_map.x, simple_local_map.y, simple_local_map.theta);
        RobotPose diff = reverseTransformFrame(origin_pose,world_pose);
        *my = (unsigned int)(diff.pt.x / getMapInfo().resolution + getMapInfo().origin_grid.x) + 1;//用map2world转换，再用world2map转换回来，实测少1,此处补偿
        *mx = (unsigned int)(diff.pt.y / getMapInfo().resolution + getMapInfo().origin_grid.y);

        return (*mx < getMapInfo().size.x && *my < getMapInfo().size.y);
    };

    //用法对应关系
    //local_map_data_.grid[col][row]   x->row  y->col
    //teb_local_map_.map2world(row,col,world_x,world_y);
    auto map2world = [&](unsigned int mx, unsigned int my, float* wx, float* wy)->bool 
    {
        RobotGrid grid(my,mx);
        RobotGrid grid_diff = grid - getMapInfo().origin_grid;
        RobotPose diff { grid_diff.x * getMapInfo().resolution, grid_diff.y * getMapInfo().resolution, 0};//在机器人坐标系下的坐标
        RobotPose origin_pose(simple_local_map.x, simple_local_map.y, simple_local_map.theta);
        RobotPose world_pose = transformFrame(diff, origin_pose);
        *wx = world_pose.pt.x;
        *wy = world_pose.pt.y;
        return true;
    };

    auto resetLocalMap = [&]()
    {
        simple_local_map.x = 0;
        simple_local_map.y = 0;
        simple_local_map.theta = 0;
        simple_local_map.obstacleInFront = 0;
        simple_local_map.x = 0;

        for (size_t i = 0; i < 161; i++)
        {
            for (size_t j = 0; j < 161; j++)
            {
                simple_local_map.grid[i][j] = 0;
            }
        }
    };

    SleepTimer timer(m_local_map_polling_frequency);
    while (true)
    {

        if (!m_run_local_map)
        {
            timer.sleep();
            continue;
        }

        loop_ts = Timer::getSystemTimestampUS();
        tof_updated = false;
        update_time_reached = false;
        
        // 查看是否有新的tof数据到达或是到达更新时间
        // printf("local map updating.\r\n");
        resetLocalMap(); 
        if (m_get_local_map_func(&simple_local_map) == 0)  //这里更新local_map的数据
        {
            m_cloud_vec.clear();
            m_local_char_base_layer->reset();
            LMAP_DEBUG_LOG("received new tof frame");
            tof_updated = true;

            // if (g_dc.getData<DataSlam>(slam_data) < 0)
            // {
            //     LMAP_DEBUG_LOG("cannot get current slam pose");
            //     timer.sleep();
            //     continue;
            // }

            // std::vector<Point> oriented_footprint;
            // transformFootprint(slam_data.pose, m_footprint_spec, oriented_footprint);
            // padFootprint(oriented_footprint, 1.3);

            uint64_t t1 = Timer::getSystemTimestampMS();
            //更新当前机器人坐标为原点
            getMapInfo().setOriginPt(Point{simple_local_map.x,simple_local_map.y});
            LMAP_DEBUG_LOG("robot origin point{%f,%f}",simple_local_map.x,simple_local_map.y);
            for(uint16_t i = 0;i < 161;++i)
            {
                for(uint16_t j = 0;j < 161;++j)
                {
                    uint8_t cost = getCost(i,j);
                    if(cost != 0)
                    {
                        float wx,wy;
                        map2world(i,j,&wx,&wy);//先转成全局坐标
                        Point point(wx,wy);

                        // if(!isInFootprint(point, oriented_footprint))
                            m_cloud_vec.emplace_back(wx,wy);

                        RobotPose pose(wx,wy,0);
                        RobotGrid grid = poseToGridWithoutBoundary(pose);

                        try
                        {
                            m_local_char_base_layer->set(grid,cost);//将障碍物数据更新进来
                        }
                        catch(std::out_of_range& e)
                        {
                            LMAP_ERROR_LOG("poseToGridWithoutBoundary out of range.");
                            continue;
                        }
                    }
                }
            }
            uint64_t t2 = Timer::getSystemTimestampMS();
            LMAP_DEBUG_LOG("Copy obstacle info time = %ld obs size = %d",t2-t1, m_cloud_vec.size());
        }
        else //如果没有更新就重新轮循
        {
            // timer.sleep();
            continue;
        }

        if(tof_updated && m_use_esdf_map) //障碍物一多后，运行时间超级长
        {
            //更新esdf
            uint64_t t1 = Timer::getSystemTimestampMS();
            m_sdf_map->odomCallback(simple_local_map.x,simple_local_map.y);
            LMAP_DEBUG_LOG("odomCallback.");
            m_sdf_map->setMapOrigin(simple_local_map.x,simple_local_map.y);
            LMAP_DEBUG_LOG("setMapOrigin.");

            m_sdf_map->cloudCallback(m_cloud_vec);
            LMAP_DEBUG_LOG("cloudCallback.");
            m_sdf_map->updateESDFCallback();
            LMAP_DEBUG_LOG("updateESDFCallback.");
            uint64_t t2 = Timer::getSystemTimestampMS();
            LMAP_DEBUG_LOG("updateESDF time = %ld ms",t2-t1);

            //test 
            // float dis1 = m_sdf_map->getDistance(Point{simple_local_map.x,simple_local_map.y+0.6});
            // float dis2 = m_sdf_map->getDistance(Point{simple_local_map.x,simple_local_map.y+0.4});
            // float dis3 = m_sdf_map->getDistance(Point{simple_local_map.x,simple_local_map.y+0.2});
            // float dis4 = m_sdf_map->getDistance(Point{simple_local_map.x+0.6,simple_local_map.y});
            // float dis5 = m_sdf_map->getDistance(Point{simple_local_map.x+0.4,simple_local_map.y});
            // float dis6 = m_sdf_map->getDistance(Point{simple_local_map.x+0.2,simple_local_map.y});
            // LMAP_DEBUG_LOG("dis1=%f dis2=%f dis3=%f dis4=%f dis5=%f dis6=%f ",dis1,dis2,dis3,dis4,dis5,dis6);

            // Eigen::Vector2d pos,grad;
            // pos(0) = simple_local_map.x;
            // pos(1) = simple_local_map.y+0.6;
            // float dis_tri1 = m_sdf_map->getDistWithGradTrilinear(pos, grad);
            // LMAP_DEBUG_LOG("dis_tri1=%f grad{%f,%f}",dis_tri1,grad(0),grad(1));
            // pos(0) = simple_local_map.x;
            // pos(1) = simple_local_map.y+0.4;
            // float dis_tri2 = m_sdf_map->getDistWithGradTrilinear(pos, grad);
            // LMAP_DEBUG_LOG("dis_tri2=%f grad{%f,%f}",dis_tri2,grad(0),grad(1));
            // pos(0) = simple_local_map.x;
            // pos(1) = simple_local_map.y+0.2;
            // float dis_tri3 = m_sdf_map->getDistWithGradTrilinear(pos, grad);
            // LMAP_DEBUG_LOG("dis_tri3=%f grad{%f,%f}",dis_tri3,grad(0),grad(1));
            // pos(0) = simple_local_map.x+0.6;
            // pos(1) = simple_local_map.y;
            // float dis_tri4 = m_sdf_map->getDistWithGradTrilinear(pos, grad);
            // LMAP_DEBUG_LOG("dis_tri4=%f grad{%f,%f}",dis_tri4,grad(0),grad(1));
            // pos(0) = simple_local_map.x+0.4;
            // pos(1) = simple_local_map.y;
            // float dis_tri5 = m_sdf_map->getDistWithGradTrilinear(pos, grad);
            // LMAP_DEBUG_LOG("dis_tri5=%f grad{%f,%f}",dis_tri5,grad(0),grad(1));
            // pos(0) = simple_local_map.x+0.2;
            // pos(1) = simple_local_map.y;
            // float dis_tri6 = m_sdf_map->getDistWithGradTrilinear(pos, grad);
            // LMAP_DEBUG_LOG("dis_tri5=%f grad{%f,%f}",dis_tri6,grad(0),grad(1));

            if(m_rviz_display)
            {
                //发布esdf地图
                m_viewer->publishESDF(getMapInfo(),m_sdf_map,m_local_char_base_layer);
            }
        
        }
        

        // if (loop_ts - last_update_ts > update_step_time)
        // {
        //     LMAP_DEBUG_LOG("local map update time reached");
        //     last_update_ts = loop_ts;
        //     update_time_reached = true;
        // }

        // if (!tof_updated && !update_time_reached)
        // {
        //     LMAP_DEBUG_LOG("no need update local map");
        //     timer.sleep();
        //     continue;
        // }

        // 获取数据
        if (g_dc.getData<DataSlam>(slam_data) < 0)
        {
            LMAP_DEBUG_LOG("cannot get current slam pose");
            timer.sleep();
            continue;
        }

        // 以下开始地图更新相关操作
        // std::unique_lock<shared_recursive_mutex> whole_lock(m_whole_map_mtx);

        RobotPose cur_robot_pose = slam_data.pose;

        // RobotPose cur_tof_cell_pose
        //     = toMapCellPose(m_cur_tof_global_pose, m_map_info->resolution);
        RobotPose cur_robot_cell_pose
            = toMapCellPose(cur_robot_pose, m_map_info->resolution);//TODO：这里是否需用cur_robot_pose

        clearRobotFootprint(cur_robot_cell_pose, m_local_base_layer);

        // 在base map上获取所有要膨胀的障碍点
        LMAP_DEBUG_LOG("get inflate obstacle pts");
        uint64_t t1 = Timer::getSystemTimestampMS();

        auto &&inflate_obs = m_local_char_base_layer->getInflateObstacles(
            m_window_rect_in_base_map, g_robot_cfg->footprint.inner_radius);
            
        LMAP_DEBUG_LOG("obstacle pts size = %u", inflate_obs.size());

        // 地图膨胀
        LMAP_DEBUG_LOG("inflate map");
        m_inflate_base_layer->reset();
        m_inflate_base_layer->inflateMap(std::vector<RobotGrid>{}, inflate_obs);

        uint64_t t2 = Timer::getSystemTimestampMS();
        LMAP_DEBUG_LOG("inflating map cost %ld ms",t2-t1);

        //测试用，保存膨胀图片
        // RobotGridRect grid = RobotGridRect(Grid(0,0),Grid(161,161));
        // std::string str;
        // std::stringstream ss;
        // bmp_num++;
        // ss << bmp_num;
        // ss >> str;
        // str = "/tmp/log/test/local_inflate" + str + ".bmp";   //std::string("/tmp/log/test/local_inflate.bmp")
        // bool success = saveMapToFile(str, std::string("inflate_base"),
        //     SAVE_WHOLE, grid);

        if(m_rviz_display)
        {
            //用于rviz显示
            m_viewer->publishRobotPose(cur_robot_pose);
            // m_viewer->publishObstacles(m_cloud_vec);
        }
            m_viewer->publishObstacles(inflate_obs,m_local_char_base_layer);

        {

            if (m_print_local_inflate_map)
                printLocalInflateMap(cur_robot_cell_pose);


        }

        m_global_pose = cur_robot_pose;

        LMAP_DEBUG_LOG("frame cnt updated");
        m_frame_cnt++;
        if (m_frame_cnt > 10000000)
            m_frame_cnt = 0;
        m_local_map_data_ready = true;

        timer.sleep();
    }
}


bool LocalMap::checkTrajectoryPointFeasible(double x, double y, double theta, InfeasiableDir& infeasiable_side)
{

    if (footprintCost(x, y, theta, m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) < 0)
    {
        RobotPose pose(x, y, theta);
        m_viewer->publishInfeasibleRobotPose(pose);
        return false;
    }
    return true;
}

//检测2米半径内，轨迹是否可行
bool LocalMap::checkTrajectoryFeasible(const vector<RobotPose>& path, double& distance, InfeasiableDir& infeasiable_side,
        int look_ahead_idx)
{
    if (look_ahead_idx < 0 || look_ahead_idx >= path.size())
        look_ahead_idx = path.size() - 1;

    double radius = 0.0;
    int i = 0;

    while(radius < 3.0 && i < look_ahead_idx)//TODO:检测距离变成可调的
    {
        if (footprintCost(path[i].pt.x, path[i].pt.y, path[i].theta, m_footprint_spec, m_inscribed_radius,
            m_circumscribed_radius, infeasiable_side) < 0)
        {
            distance = radius;
            // printf(COLOR_L_RED  COLOR_BOLD  "path_pose{%f,%f}\r\n" COLOR_NONE,path[i].pt.x, path[i].pt.y);
            RobotPose pose(path[i].pt.x, path[i].pt.y, path[i].theta);
            m_viewer->publishInfeasibleRobotPose(pose);
            return false;
        }
        // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
        // and interpolates in that case.
        // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!

        // double delta_rot = angleBetweenR(path[i].theta, path[i + 1].theta);

        Point delta_dist = path[i + 1].pt - path[i].pt;
        // if (fabs(delta_rot) > M_PI || delta_dist.norm() > m_inscribed_radius)
        // {
        //     int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / M_PI),
        //                 std::ceil((double)(delta_dist.norm() / m_inscribed_radius))) - 1;

        //     RobotPose intermediate_pose = path[i];
        //     RobotPose last_intermediate_pose = intermediate_pose;

        //     for (int step = 0; step < n_additional_samples; ++step)
        //     {
        //         intermediate_pose.pt = intermediate_pose.pt + delta_dist / (n_additional_samples + 1.0);
        //         intermediate_pose.theta = normalizeAngle(intermediate_pose.theta +
        //                                                     delta_rot / (n_additional_samples + 1.0));
        //         if (footprintCost(intermediate_pose.pt.x, intermediate_pose.pt.y, intermediate_pose.theta,
        //                             m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) == -1)
        //         {
        //             distance = radius;
        //             // printf(COLOR_L_RED  COLOR_BOLD  "intermediate_pose{%f,%f}\r\n" COLOR_NONE,intermediate_pose.pt.x, intermediate_pose.pt.y);
        //             RobotPose pose(intermediate_pose.pt.x, intermediate_pose.pt.y, intermediate_pose.theta);
        //             m_viewer->publishInfeasibleRobotPose(pose);
        //             return false;
        //         }
        //         radius += (intermediate_pose.pt - last_intermediate_pose.pt).norm();
        //     }
        // }

        radius += delta_dist.norm();
        ++i;
    }
    return true;
}

bool LocalMap::checkTrajectoryFeasible(const PosePathPtr path, double& distance, InfeasiableDir& infeasiable_side,
        int look_ahead_idx)
{
    if (look_ahead_idx < 0 || look_ahead_idx >= path->size())
        look_ahead_idx = path->size() - 1;

    double radius = 0.0;
    int i = 0;

    while(radius < 10.0 && i < look_ahead_idx)
    {
        if (footprintCost(path->at(i).pt.x, path->at(i).pt.y, path->at(i).theta, m_footprint_spec, m_inscribed_radius,
            m_circumscribed_radius, infeasiable_side) < 0)
        {
            distance = radius;
            // printf(COLOR_L_RED  COLOR_BOLD  "path_pose{%f,%f}\r\n" COLOR_NONE,path->at(i).pt.x, path->at(i).pt.y);
            RobotPose pose(path->at(i).pt.x, path->at(i).pt.y, path->at(i).theta);
            m_viewer->publishInfeasibleRobotPose(pose);
            return false;
        }
        // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
        // and interpolates in that case.
        // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!

        double delta_rot = angleBetweenR(path->at(i).theta, path->at(i+1).theta);

        Point delta_dist = path->at(i+1).pt - path->at(i).pt;
        if (fabs(delta_rot) > M_PI || delta_dist.norm() > m_inscribed_radius)
        {
            int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / M_PI),
                        std::ceil((double)(delta_dist.norm() / m_inscribed_radius))) - 1;

            RobotPose intermediate_pose = path->at(i);
            RobotPose last_intermediate_pose = intermediate_pose;

            for (int step = 0; step < n_additional_samples; ++step)
            {
                intermediate_pose.pt = intermediate_pose.pt + delta_dist / (n_additional_samples + 1.0);
                intermediate_pose.theta = normalizeAngle(intermediate_pose.theta +
                                                            delta_rot / (n_additional_samples + 1.0));
                if (footprintCost(intermediate_pose.pt.x, intermediate_pose.pt.y, intermediate_pose.theta,
                                    m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) == -1)
                {
                    distance = radius;
                    // printf(COLOR_L_RED  COLOR_BOLD  "intermediate_pose{%f,%f}\r\n" COLOR_NONE,intermediate_pose.pt.x, intermediate_pose.pt.y);
                    RobotPose pose(intermediate_pose.pt.x, intermediate_pose.pt.y, intermediate_pose.theta);
                    m_viewer->publishInfeasibleRobotPose(pose);
                    return false;
                }
                radius += (intermediate_pose.pt - last_intermediate_pose.pt).norm();
            }
        }
        radius += delta_dist.norm();
        ++i;
    }
    return true;
}


bool LocalMap::checkRotateFeasible(double x, double y, double theta, double target_angle, InfeasiableDir& infeasiable_side)
{
    double resolution_collision_check_angular = 0.262; //以15度间隔来判断

    //先进行一次目标角度的判断
    if (footprintCost(x, y, target_angle, m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) < 0)
    {
        RobotPose pose(x, y, target_angle);
        m_viewer->publishInfeasibleRobotPose(pose);
        return false;
    }

    double delta_rot = normalizeAngle(normalizeAngle(target_angle) - normalizeAngle(theta));//normalizeAngle的效果是以最近的角度差来进行旋转测试
                                            
    if (fabs(delta_rot) > resolution_collision_check_angular)
    {
        int n_additional_samples = std::ceil(fabs(delta_rot) / resolution_collision_check_angular) - 1;

        double intermediate_theta = theta;
        for (int step = 0; step < n_additional_samples; ++step)
        {
            intermediate_theta = normalizeAngle(intermediate_theta +
                                                      delta_rot / (n_additional_samples + 1.0));
            if (footprintCost(x, y, intermediate_theta, m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) < 0)
            {
                RobotPose pose(x, y, intermediate_theta);
                m_viewer->publishInfeasibleRobotPose(pose);
                return false;
            }
        }
    }
    return true;
}

//feasible_dir left->1  right->0
//检测两边旋转是否可以，优先检测夹角近的那边
bool LocalMap::checkRotateFeasible(double x, double y, double theta, double target_angle, InfeasiableDir& infeasiable_side, bool& feasible_dir, double& delta_angel)
{
    double resolution_collision_check_angular = 0.262; //以15度间隔来判断

    //先进行一次目标角度的判断
    if (footprintCost(x, y, target_angle, m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) < 0)
    {
        RobotPose pose(x, y, target_angle);
        m_viewer->publishInfeasibleRobotPose(pose);
        return false;
    }

    double delta_rot = normalizeAngle(target_angle) - normalizeAngle(theta);//normalizeAngle的效果是以最近的角度差来进行旋转测试
    if(delta_rot > 0)
    {
        if(fabs(delta_rot) > M_PI)
        {
            delta_rot = normalizeAngle(delta_rot);
            feasible_dir = 0;
        }
        else
        {
            feasible_dir = 1;
        }
    }
    else
    {
        if(fabs(delta_rot) > M_PI)
        {
            delta_rot = normalizeAngle(delta_rot);
            feasible_dir = 1;
        }
        else
        {
            feasible_dir = 0;
        }
    }

    bool collision_flag = false;

    if (fabs(delta_rot) > resolution_collision_check_angular)
    {
        int n_additional_samples = std::ceil(fabs(delta_rot) / resolution_collision_check_angular) - 1;

        double intermediate_theta = theta;
        for (int step = 0; step < n_additional_samples; ++step)
        {
            intermediate_theta = normalizeAngle(intermediate_theta +
                                                      delta_rot / (n_additional_samples + 1.0));
            if (footprintCost(x, y, intermediate_theta, m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) < 0)
            {
                collision_flag = true;
                RobotPose pose(x, y, intermediate_theta);
                m_viewer->publishInfeasibleRobotPose(pose);
                break;
            }
        }
    }
    delta_angel = delta_rot;

    if(!collision_flag)
        return true;

    if(delta_rot > 0)
    {
        delta_rot -= M_PI; 
    }
    else
    {
        delta_rot += M_PI;
    }
    feasible_dir = !feasible_dir;
    delta_angel = delta_rot;

    if (fabs(delta_rot) > resolution_collision_check_angular)
    {
        int n_additional_samples = std::ceil(fabs(delta_rot) / resolution_collision_check_angular) - 1;

        double intermediate_theta = theta;
        for (int step = 0; step < n_additional_samples; ++step)
        {
            intermediate_theta = normalizeAngle(intermediate_theta +
                                                      delta_rot / (n_additional_samples + 1.0));
            if (footprintCost(x, y, intermediate_theta, m_footprint_spec, m_inscribed_radius, m_circumscribed_radius, infeasiable_side) < 0)
            {
                RobotPose pose(x, y, intermediate_theta);
                m_viewer->publishInfeasibleRobotPose(pose);
                return false;
            }
        }
    }

    return true;

}

double LocalMap::footprintCost(double x, double y, double theta, const std::vector<Point> &footprint_spec,
    double inscribed_radius, double circumscribed_radius,InfeasiableDir& infeasiable_side)
{

    double cos_th = cos(theta);
    double sin_th = sin(theta);
    std::vector<Point> oriented_footprint;
    for (unsigned int i = 0; i < footprint_spec.size(); ++i)
    {
        Point new_pt;
        new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
        new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
        oriented_footprint.push_back(new_pt);
    }

    //对footprint进行膨胀
    // padFootprint(oriented_footprint,0.05);

    Point robot_position;
    robot_position.x = x;
    robot_position.y = y;

    if (inscribed_radius == 0.0)
    {
        calculateMinAndMaxDistances(footprint_spec, inscribed_radius, circumscribed_radius);
    }

    return footprintCost(robot_position, oriented_footprint, inscribed_radius, circumscribed_radius, infeasiable_side);
}

double LocalMap::footprintCost(const Point &position, const std::vector<Point> &footprint,
                               double inscribed_radius, double circumscribed_radius,
                               InfeasiableDir& infeasiable_side)
{

    //used to put things into grid coordinates
    unsigned int cell_x, cell_y;

    RobotGrid grid = m_local_char_base_layer->poseToGridWithoutBoundary(RobotPose(position));
    cell_x = grid.x;
    cell_y = grid.y;
    if(!inRange(grid))
    {
        // LMAP_DEBUG_LOG("NOT in range.");
        printf(COLOR_L_RED  COLOR_BOLD  "NOT in range.\r\n" COLOR_NONE);
        return -1.0;
    }

    //if number of points in the footprint is less than 3, we'll just assume a circular robot
    if (footprint.size() < 3)
    {
        return pointCost(cell_x, cell_y);
    }

    //now we really have to lay down the footprint in the costmap grid
    unsigned int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    //we need to rasterize each line in the footprint
    for (unsigned int i = 0; i < footprint.size() - 1; ++i)
    {
        //get the cell coord of the first point
        grid = m_local_char_base_layer->poseToGridWithoutBoundary(RobotPose(footprint[i]));//TODO:注意如果出现转换失败会报异常
        x0 = grid.x;                                                     //检测的轨迹长度不能超出local_map的覆盖范围。
        y0 = grid.y;
        if(!inRange(grid))
        {
            // LMAP_DEBUG_LOG("NOT in range.");
            printf(COLOR_L_RED  COLOR_BOLD  "NOT in range.\r\n" COLOR_NONE);
            return -1.0;
        }

        grid = m_local_char_base_layer->poseToGridWithoutBoundary(RobotPose(footprint[i+1]));
        x1 = grid.x;
        y1 = grid.y;
        if(!inRange(grid))
        {
            // LMAP_DEBUG_LOG("NOT in range.");
            printf(COLOR_L_RED  COLOR_BOLD  "NOT in range.\r\n" COLOR_NONE);
            return -1.0;
        }

        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = std::max(line_cost, footprint_cost);

        //if there is an obstacle that hits the line... we know that we can return false right away
        if (line_cost < 0)
        {
            switch(i)
            {
                case 0:
                    infeasiable_side = LocalMap::FRONT;
                    printf(COLOR_L_RED  COLOR_BOLD  "front line_cost < 0\r\n" COLOR_NONE);
                    break;
                case 1:
                    infeasiable_side = LocalMap::LEFT;
                    printf(COLOR_L_RED  COLOR_BOLD  "left front line_cost < 0\r\n" COLOR_NONE);
                    break;
                case 2:
                    infeasiable_side = LocalMap::BACK;
                    printf(COLOR_L_RED  COLOR_BOLD  "back front line_cost < 0\r\n" COLOR_NONE);
                    break;
            }
            // LMAP_DEBUG_LOG("line_cost < 0");
            return -1.0;
        }
    }

    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    grid = m_local_char_base_layer->poseToGridWithoutBoundary(RobotPose(footprint.back()));
    x0 = grid.x;                                                     
    y0 = grid.y;
    if(!inRange(grid))
    {
        // LMAP_DEBUG_LOG("NOT in range.");
        printf(COLOR_L_RED  COLOR_BOLD  "NOT in range\r\n" COLOR_NONE);
        return -1.0;
    }

    //get the cell coord of the first point
    grid = m_local_char_base_layer->poseToGridWithoutBoundary(RobotPose(footprint.front()));
    x1 = grid.x;
    y1 = grid.y;
    if(!inRange(grid))
    {
        // LMAP_DEBUG_LOG("NOT in range.");
        printf(COLOR_L_RED  COLOR_BOLD  "NOT in range\r\n" COLOR_NONE);
        return -1.0;
    }

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = std::max(line_cost, footprint_cost);

    if (line_cost < 0) //右侧干涉检测
    {
        // LMAP_DEBUG_LOG("line_cost < 0");
        infeasiable_side = LocalMap::RIGHT;
        printf(COLOR_L_RED  COLOR_BOLD  "right line_cost  < 0\r\n" COLOR_NONE);
        return -1.0;
    }

    //if all line costs are legal... then we can return that the footprint is legal
    return footprint_cost;
}

//calculate the cost of a ray-traced line
double LocalMap::lineCost(int x0, int x1,
                          int y0, int y1)
{

    double line_cost = 0.0;
    double point_cost = -1.0;

    for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
    {
        point_cost = pointCost(line.getX(), line.getY()); //Score the current point

        if (point_cost < 0)
            return -1;

        if (line_cost < point_cost)
            line_cost = point_cost;
    }

    return line_cost;
}

double LocalMap::pointCost(int x, int y)
{
    uint8_t cost;
    m_local_char_base_layer->get(RobotGrid{x, y}, cost);
    //if the cell is in an obstacle the path is invalid
    RobotPose pos = m_local_char_base_layer->gridToPose(RobotGrid{x, y});
    if (cost != FREE ) //|| (m_sdf_map->getDistance(pos.pt) < 0.05 && m_use_esdf_map)
    {
        // RobotPose pose = gridToPose(RobotGrid{x, y});
        // printf(COLOR_L_RED  COLOR_BOLD  "point{%f,%f}\r\n" COLOR_NONE,pose.pt.x,pose.pt.y);
        return -1;
    }
    // unsigned char cost = 0;
    return cost;
}


void LocalMap::transformFootprint(RobotPose pose, const std::vector<Point>& footprint_spec,
                        std::vector<Point>& oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.clear();
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i)
  {
    Point new_pt;
    new_pt.x = pose.pt.x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = pose.pt.y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

//padding 取0.1这样的
void LocalMap::padFootprint(std::vector<Point>& footprint, double padding)
{
  // pad footprint in place
  for (unsigned int i = 0; i < footprint.size(); i++)
  {
    Point& pt = footprint[ i ];
    pt.x += sign0(pt.x) * padding;
    pt.y += sign0(pt.y) * padding;
  }
}

bool LocalMap::isInFootprint(Point point, std::vector<Point>& oriented_footprint)
{

    auto GetCross = [](Point& p1, Point& p2,Point& p) -> float
    {
        return (p2.x - p1.x) * (p.y - p1.y) -(p.x - p1.x) * (p2.y - p1.y);
    };

	return (GetCross(oriented_footprint[0],oriented_footprint[1],point) * GetCross(oriented_footprint[2],oriented_footprint[3],point) >= 0
        && GetCross(oriented_footprint[1],oriented_footprint[2],point) * GetCross(oriented_footprint[3],oriented_footprint[0],point) >= 0);

}

bool LocalMap::getInflateOccupancy(Eigen::Vector2d point)
{
    RobotPose pose(point.x(),point.y(),0);
    RobotGrid grid = poseToGridWithoutBoundary(pose);
    if (!inRange(grid))
        return true;
    return m_inflate_base_layer->isOccupied(grid);
}



