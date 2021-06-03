***********************************************************************/

#ifndef MAP_CLEAN_MAP_H
#define MAP_CLEAN_MAP_H

#include <mutex>
#include <functional>

#include "map/grid_map.h"
#include "clean_layer.h"
#include "obstacle_mark_layer.h"
#include "slam_layer.h"
#include "mask_layer.h"
#include "map/inflate_layer.h"
#include "config/config_manager.h"
#include "misc/planning_typedefs.h"
#include "local_layer.h"

using planning_utils::ConfigManager;

namespace planning_map
{
#define SWEEP "clean"
#define SWEEP_SIDE "clean_side"
#define SWEEP_UPDATE_PATTERN(...) \
  std::vector<CleanLayer::CleanStatus> { __VA_ARGS__ }

  struct DeltaObMark
  {
    DeltaObMark();
    DeltaObMark(uint8_t old_type, uint8_t new_type);

    uint8_t old_type;
    uint8_t new_type;
  };

  struct DeltaObCell
  {
    DeltaObCell();
    DeltaObCell(const RobotGrid &grid, ObMarkLayer::ObstacleType type);

    RobotGrid grid;
    ObMarkLayer::ObstacleType type;
  };

  class CleanMap : public GridMap
  {
  public:
    enum CleanMapInflateMask
    {
      SMALLER_INFLATE_RADIUS = 4,
      NORMAL_INFLATE_RADIUS = 8
    };

    /**
          * @brief  读取配置参数，初始化clean_map
          * @param  cfg_mgr                 配置管理器
          *         requrire_slam_map_func  请求获取slam地图函数
          *         get_slam_map_func       获取最新slam地图到clean_map的函数
          * @retval None
          */
    void init(ConfigManager &cfg_mgr,
              const std::function<void(void)> &require_slam_map_func,
              const std::function<int(uint8_t *)> &get_slam_map_func);
    /**
          * @brief  初始化图层
          * @param  None
          * @retval None
          */
    void initLayer(std::string file_name);
    /**
          * @brief  重置清扫地图
          * @param  file_name
          * @retval None
          */
    void reset();

    /**
          * @brief  向slam模块请求获取最新slam地图
          * @param  None
          * @retval None
          */
    void requireSlamMap();

    /**
          * @brief  获取最新slam地图到clean_map内部slam_layer
          * @param  None
          * @retval None
          */
    bool getSlamMap();

    /**
          * @brief  膨胀最新的地图，保存在inflate_layer
          * @param  None
          * @retval None
          */
    void inflateMap();

    /**
          * @brief  获取膨胀地图
          * @param  mask        膨胀mask，将获取该mask对应的膨胀地图
          *         dest        膨胀地图拷贝到的位置
          * @retval None
          */
    void getInflateMap(uint8_t mask, uint8_t **dest);
    // void loadSlamMap(std::string img_name);

    void getLocalMap(uint8_t **dest);

    // ***************************** 清扫地图更新 ****************************
    void updateCleanStatus(const RobotGrid &grid, CleanLayer::CleanStatus status);
    void updateCleanStatus(const RobotPose &pose, CleanLayer::CleanStatus status);
    void updateCleanStatus(const RobotGrid &local_grid, const RobotPose &ref_pose,
                           CleanLayer::CleanStatus status);
    void updateCleanStatus(const Point &local_pt, const RobotPose &ref_pose,
                           CleanLayer::CleanStatus status);
    void updateCleanStatus(const std::vector<CleanLayer::CleanStatus> &pattern,
                           const RobotPose &ref_pose, int32_t center_idx = -1);
    void updateCleanStatus(const std::string &name, const RobotPose &ref_pose);

    // ***************************** 障碍地图更新 ****************************
    void updateObMark(const RobotGrid &grid, ObMarkLayer::ObstacleType type);

    // **************************** 实时清扫信息相关 **************************
    /**
          * @brief  更新清扫轨迹
          * @param  pose        当前轨迹点位置
          * @retval None
          */
    void updateCleanTraj(const RobotPose &pose);

    /**
          * @brief  获取清扫轨迹变化量
          * @param  None
          * @retval None
          */
    std::list<RobotPose> getDeltaCleanTraj();

    /**
          * @brief  获取障碍标记的增量
          * @param  None
          * @retval None
          */
    std::list<DeltaObCell> getDeltaObMark();

    // **************************** 实时清扫信息相关 **************************
    RobotGridRect generatePersistMap(std::string map_name);
    bool loadPersistMap(std::string map_name, const RobotGridRect &region);
    void updateLocalLayer();

    ConfigManager configCleanMap;

  private:
    void createSweepUpdatePattern(std::string name,
                                  CleanLayer::CleanStatus path_status,
                                  CleanLayer::CleanStatus cleaned_status,
                                  CleanLayer::CleanStatus covered_status);
    void getInflateFreesAndObs(std::vector<RobotGrid> &frees,
                               std::vector<std::pair<RobotGrid, float>> &obs);

    CleanLayerPtr m_clean_layer;
    ObMarkLayerPtr m_ob_mark_layer;
    SlamLayerPtr m_slam_layer;
    MaskLayerPtr m_mask_layer;
    InflateLayerPtr m_inflate_layer;
    LocalLayerPtr m_local_layer;

    std::function<void(void)> m_require_slam_map_func;
    std::function<int(uint8_t *)> m_get_slam_map_func;

    shared_recursive_mutex m_whole_clean_map_mtx;
    std::mutex m_inflate_mtx;
    std::mutex m_clean_traj_mtx;
    std::mutex m_delta_ob_marks_mtx;

    int32_t m_sweep_covered_length;
    int32_t m_sweep_cleaned_length;
    int32_t m_sweep_path_length;
    std::unordered_map<std::string, std::vector<CleanLayer::CleanStatus>>
        m_sweep_update_patterns;

    RobotPoseTrajectory m_clean_traj;
    uint64_t m_last_clean_traj_ts;
    std::map<RobotGrid, DeltaObMark> m_delta_ob_marks;
  };
} // namespace planning_map

extern planning_map::CleanMap g_clean_map;

#endif // MAP_CLEAN_MAP_H