#ifndef DETECTOR_COLLISION_DETECTOR_H
#define DETECTOR_COLLISION_DETECTOR_H

#include "detector/detector.h"
// #include "detector/detector_param.h"
// #include "detector/detector_context.h"
#include "collision_detector_param.h"
// #include "collision_detector_log.h"
#include "collision_detector_context.h"
// #include "collision_detector_config.h"
#include "collision_detector_events.h"

#include "data/data_center/data_center.h"
#include "data/event_center/event_center.h"
#include "event_center/event.h"

// #include "misc/planning_typedefs.h"
#include "data/slam_data.h"
// #include "data/chassis_data.h"
#include "data/local_map_data.h"
// #include "data/control_speed_data.h"

using namespace std;

namespace planning_detector
{
    class CollisionDetector : public planning_detector::Detector
    {
    // DEFINE_DETECTOR(Collision)
      public:
        // static CollisionDetector &getInstance(ThreadPool *pool);
        static CollisionDetector &getInstance();

      private:

        // CollisionDetector(ThreadPool *pool);
        CollisionDetector();
        virtual float loadConfig(ConfigManager &cfg_mgr);
        virtual void initRunSM();
        virtual void reset();
        virtual void getData();
        virtual bool handleParam(DetectorParamPtr param);
        virtual void finishDetect();

        virtual ContextPtr saveContext();
        virtual Transition restoreContext(ContextPtr ctx);  

        virtual void record_collision_pic();
        virtual void record_collision_detected_pic(const int &i_, const int &j_, const size_t &k_);

        // DataSlam m_slam_data;
        // DataUltrasonic m_ultrasonic_data;
        DataLocalMap m_simple_local_map;
        // DataControlSpeed m_control_speed;

        float m_obstacle_radius;
        float m_vehicle_longitude_size;
        float m_vehicle_lateral_size;
        float m_inflate_longitude_dist_small;
        float m_inflate_lateral_dist_small;
        float m_inflate_longitude_dist_large;
        float m_inflate_lateral_dist_large;
        int m_robot_pose_in_local_map_x_default;
        int m_robot_pose_in_local_map_y_default;
        int m_robot_pose_in_local_map_x;
        int m_robot_pose_in_local_map_y;
        float m_local_map_resolution;
        int m_local_map_size_x;
        int m_local_map_size_y;
        int m_max_det_path_points_num_default; // 默认最大保留路径点数
        int m_max_det_path_points_num;
        float m_predict_horizon;

        PosePath m_current_path_points;
        PosePath m_current_path_points_in_local_map;

        friend struct CollisionDetectorStates;

        // END_GLOBAL_PLANNER(Collision)
    };
} // namespace planning_detector

extern planning_detector::CollisionDetector &g_collision_detector; 

#endif // DETECTOR_COLLISION_DETECTOR_H