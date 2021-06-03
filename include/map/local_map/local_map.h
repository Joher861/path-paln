#ifndef MAP_LOCAL_MAP_H
#define MAP_LOCAL_MAP_H

#include "map/grid_map.h"
#include "map/inflate_layer.h"
#include "local_map/local_base_layer.h"
#include "local_map/local_uint8_layer.h"
#include "local_map/include/local_window_map.h"
#include "misc/footprint.h"
#include "config/config_manager.h"
#include "data/slam_data.h"
#include "data/local_map_data.h"
#include "local_map/include/local_map_viewer.h"

#include "map/map_cost_value.h"
#include "local_map/include/sdf_map.h"

using planning_utils::ConfigManager;

namespace planning_map
{
    struct ObContourInfo
    {
        PosePathPtr ob_contour_path = nullptr;
        LocalCell nearest_local_src_cell;
        RobotPose nearest_local_src_pose;
        InflateCell nearest_contour_inflate_cell;
        RobotPose nearest_contour_pose;
    };

    class LocalMap : public GridMap
    {
      public:

        enum LocalMapInflateMask
        {
            NORMAL_INFLATE_RADIUS   = 4,
            PATH_INFLATE_RADIUS     = 8,
        };

        enum InfeasiableDir
        {
            FRONT   = 0,
            BACK       ,
            LEFT       ,
            RIGHT      ,
        };

        LocalMap();
        virtual ~LocalMap();
        
        virtual void init(ConfigManager &cfg_mgr, 
                const std::function<int(SimpleLocalMap *local_map)> &get_local_map_func);
        void start();
        void stop();
        void resetLocalMap();
        SDFMap::Ptr getSdfMap() {return m_sdf_map;}
        
        bool checkTrajectoryFeasible(const PosePathPtr path, double& distance, InfeasiableDir& infeasiable_side, int look_ahead_idx = 1000);
        bool checkTrajectoryFeasible(const vector<RobotPose>& path, double& distance, InfeasiableDir& infeasiable_side, int look_ahead_idx = 1000);
        bool checkTrajectoryPointFeasible(double x, double y, double theta,  InfeasiableDir& infeasiable_side);

        std::vector<Eigen::Vector2d>& getObstacleVec() {return m_cloud_vec;}
        bool getInflateOccupancy(Eigen::Vector2d point);
        bool checkRotateFeasible(double x, double y, double theta, double target_angle,  InfeasiableDir& infeasiable_side);
        bool checkRotateFeasible(double x, double y, double theta, double target_angle, InfeasiableDir& infeasiable_side,
            bool & feasible_dir, double& delta_angel);
      private:

        RobotGrid getGridOffset(const RobotGrid &grid_src,
            const RobotGrid &grid_dest) const;
        RobotGrid getGridOffsetToWindowCenter(const RobotGrid &grid) const;
        RobotGrid getGridOffsetToWindowCenter(const Point &pt) const;
        RobotGrid getGridOffsetToBaseMapCenter(const RobotGrid &grid) const;
        RobotGrid getGridOffsetToBaseMapCenter(const Point &pt) const;

        bool windowGridToBaseMapGrid(const RobotGrid &window_grid,
            RobotGrid &base_map_grid) const;
        bool baseMapGridToWindowGrid(const RobotGrid &base_map_grid,
            RobotGrid &window_grid) const;

        template <typename T>
        void moveOffsetGrids(std::string layer_name, const RobotGrid &old_start,
            const RobotGrid &old_end, const RobotGrid &offset, const RobotGrid &new_end);
        template <typename T>
        void clearOffsetGrids(std::string layer_name, const RobotGrid &offset,
            const RobotGridRect &new_rect);

        void resetWindow(const RobotPose &new_center);
        bool moveWindow(const RobotPose &old_center_pose,
            const RobotPose &new_center_pose);
        void moveBaseMap(const RobotPose &old_center_pose,
            const RobotPose &new_center_pose);

        void clearRobotFootprint(const RobotPose &robot_pose, LocalBaseLayerPtr layer);

        RobotGridRect getMapWindowFromCenterPose(const RobotPose &center_pose) const;

        void printLocalMap(const RobotPose &cur_pose);
        void printLocalInflateMap(const RobotPose &cur_pose);

        void updateLocalMap();
        
        double pointCost(int x, int y);
        double lineCost(int x0, int x1,int y0, int y1);
        double footprintCost(const Point &position, const std::vector<Point> &footprint,
                               double inscribed_radius, double circumscribed_radius, InfeasiableDir& infeasiable_side);                
        double footprintCost(double x, double y, double theta, const std::vector<Point> &footprint_spec,
            double inscribed_radius, double circumscribed_radius, InfeasiableDir& infeasiable_side);

        double sign0(double x)
        {
            return x < 0.0 ? -1.0 : (x > 0.0 ? 1.0 : 0.0);
        }

        void transformFootprint(RobotPose pose, const std::vector<Point>& footprint_spec,
                                std::vector<Point>& oriented_footprint);

        void padFootprint(std::vector<Point>& footprint, double padding);           

        bool isInFootprint(Point point, std::vector<Point>& oriented_footprint);

        float m_local_map_polling_frequency;
        float m_local_map_updating_frequency;

        LocalBaseLayerPtr m_local_base_layer;
        InflateLayerPtr m_inflate_base_layer;
        LocalBaseLayerPtr m_inflate_local_base_layer;
        LocalCharBaseLayerPtr m_local_char_base_layer;

        LocalWindowMap m_window_map;
        MapSize m_window_size;
        RobotGrid m_to_window_center_offset;
        RobotGrid m_window_center_in_base_map;
        RobotGridRect m_window_rect_in_base_map;

        std::thread *m_local_map_updating_thread;

        RobotPose m_cur_tof_global_pose;
        uint64_t m_cur_tof_ts;
        RobotPose m_global_pose;

        bool m_run_local_map;
        bool m_local_map_data_ready;
        uint64_t m_local_map_frame_cnt;
        uint64_t m_inflate_map_frame_cnt;
        uint64_t m_frame_cnt;


        std::function<int(SimpleLocalMap *local_map)> m_get_local_map_func;

        // local_map用到的共享数据
        // DataSlam m_slam_data;

        // 锁
        shared_recursive_mutex m_whole_map_mtx;
        shared_recursive_mutex m_inflate_map_mtx;

        // 其他参数
        bool m_print_local_map;
        bool m_print_local_inflate_map;
        bool m_rviz_display; //是否用rviz显示local map障碍物信息
        bool m_use_esdf_map = false;
        LocalMapViewer * m_viewer;

        std::vector<Point> m_footprint_spec;
        double m_inscribed_radius,m_circumscribed_radius;

        SDFMap::Ptr m_sdf_map;
        std::vector<Eigen::Vector2d> m_cloud_vec;
    };

    
    inline RobotGrid LocalMap::getGridOffsetToWindowCenter(const RobotGrid &grid) const
    {
        return grid - m_window_map.getMapInfo().origin_grid;
    }

    inline RobotGrid LocalMap::getGridOffsetToWindowCenter(const Point &pt) const
    {
        Point cell_pt = toMapCellPt(pt, m_map_info->resolution);
        const MapInfo &window_map_info = m_window_map.getMapInfo();
        RobotGrid offset;
        offset.x = static_cast<int>(round((cell_pt.x
            - window_map_info.origin_pt.x) / window_map_info.resolution));
        offset.y = static_cast<int>(round((cell_pt.y
            - window_map_info.origin_pt.y) / window_map_info.resolution));
        return offset;
    }

    inline RobotGrid LocalMap::getGridOffsetToBaseMapCenter(const RobotGrid &grid) const
    {
        return grid - m_map_info->origin_grid;
    }

    inline RobotGrid LocalMap::getGridOffsetToBaseMapCenter(const Point &pt) const
    {
        Point cell_pt = toMapCellPt(pt, m_map_info->resolution);
        RobotGrid offset;
        offset.x = static_cast<int32_t>(round((cell_pt.x
            - m_map_info->origin_pt.x) / m_map_info->resolution));
        offset.y = static_cast<int32_t>(round((cell_pt.y
            - m_map_info->origin_pt.y) / m_map_info->resolution));
        return offset;
    }

    inline bool LocalMap::windowGridToBaseMapGrid(const RobotGrid &window_grid,
        RobotGrid &base_map_grid) const
    {
        RobotGrid offset = getGridOffsetToWindowCenter(window_grid);
        base_map_grid = m_window_center_in_base_map + offset;
        return inRange(base_map_grid);
    }

    inline bool LocalMap::baseMapGridToWindowGrid(const RobotGrid &base_map_grid,
        RobotGrid &window_grid) const
    {
        RobotGrid offset = base_map_grid - m_window_center_in_base_map;
        window_grid = m_window_map.getMapInfo().origin_grid + offset;
        return m_window_map.inRange(window_grid);
    }

    inline RobotGridRect LocalMap::getMapWindowFromCenterPose(
        const RobotPose &center_pose) const
    {
        RobotGrid center_grid = poseToGrid(center_pose);
        return RobotGridRect{
            center_grid - m_to_window_center_offset,
            center_grid + m_to_window_center_offset
        };
    }
}

extern planning_map::LocalMap g_local_map;

#endif // MAP_LOCAL_MAP_H