#ifndef DETECTOR_OBSTACLE_DETECTOR_H
#define DETECTOR_OBSTACLE_DETECTOR_H

#include "detector/detector.h"
// #include "detector/detector_param.h"
// #include "detector/detector_context.h"
#include "obstacle_detector_param.h"
// #include "obstacle_detector_log.h"
#include "obstacle_detector_context.h"
// #include "obstacle_detector_config.h"
#include "obstacle_detector_events.h"

#include "data/data_center/data_center.h"
#include "data/event_center/event_center.h"
#include "event_center/event.h"

// #include "misc/planning_typedefs.h"
#include "data/slam_data.h"
#include "data/chassis_data.h"
#include "data/local_map_data.h"
#include "data/control_speed_data.h"

using namespace std;

namespace planning_detector
{
    class ObstacleDetector : public planning_detector::Detector
    {
    // DEFINE_DETECTOR(Obstacle)
      public:
        // static ObstacleDetector &getInstance(ThreadPool *pool);
        static ObstacleDetector &getInstance();

      private:

        // ObstacleDetector(ThreadPool *pool);
        ObstacleDetector();
        virtual float loadConfig(ConfigManager &cfg_mgr);
        virtual void initRunSM();
        virtual void reset();
        virtual void getData();
        virtual bool handleParam(DetectorParamPtr param);
        virtual void finishDetect();

        virtual ContextPtr saveContext();
        virtual Transition restoreContext(ContextPtr ctx);  

        void update_segmentations(const size_t &segmentation_num_, vector<float> &angles_of_field_, vector<float> &k_longitude_, vector<float> &k_lateral_,
                                  vector<float> &bias_longitude_l_in_function_, vector<float> &bias_longitude_r_in_function_, 
                                  vector<float> &bias_lateral_in_function_, vector<float> &bias_lateral_bottom_in_function_,
                                  vector<vector<float>> &F_l_, vector<vector<float>> &F_r_, vector<vector<float>> &F_n_, vector<vector<float>> &F_f_);
        void record_grid_of_interest_pic(const vector<float> &angles_of_field_,
                                        const vector<vector<float>> &F_l_, const vector<vector<float>> &F_r_,
                                        const vector<vector<float>> &F_n_, const vector<vector<float>> &F_f_);
        

        DataSlam m_slam_data;
        DataFeedbackSpeed m_feedback_control;
        DataUltrasonic m_ultrasonic_data;
        DataLocalMap m_simple_local_map;
        DataControlSpeed m_control_speed;
        float m_intended_velocity;
        float m_max_break_acc;

        float m_lasting_period;
        float m_reset_lasting_period;
        float m_vehicle_longitude_size; // 车长，加过冗余距离, unit:m
        float m_vehicle_lateral_size;   // 车宽，　加过冗余距离, unit:m
        float m_vehicle_lateral_inflate_size;
        float bias_longitude;
        float bias_lateral_near;
        float m_forward_safe_distance_default; 
        float m_front_side_safe_distance_default; 
        float m_forward_safe_distance; // velocity adaptively
        float m_front_side_safe_distance; // velocity adaptively
        float m_side_safe_distance; 
        float m_left_safe_dis_bias;
        float m_right_safe_dis_bias;
        float bias_lateral;
        float front_seg_bottom;

        float m_grid_of_interest_radius_default;    
        float m_grid_of_far_circle_radius_default;  
        float m_grid_of_interest_radius;    // velocity adaptively, 若扇形segmentation，则要根据车宽设置，1/(2*sin(0.5*min_OA_Angle))
        float m_grid_of_far_circle_radius;  // velocity adaptively
        float m_grid_of_near_circle_radius; // velocity adaptively
        vector<RobotGrid> grid_of_interest;
        vector<RobotGrid> grid_of_far_circle;
        vector<RobotGrid> grid_of_near_circle;
        int m_robot_pose_in_local_map_x_default;
        int m_robot_pose_in_local_map_y_default;
        int m_robot_pose_in_local_map_x;
        int m_robot_pose_in_local_map_y;
        float m_local_map_resolution;
        int m_local_map_size_x;
        int m_local_map_size_y;

        // bool m_small_obstacle_flag;
        // bool m_must_stop;

        friend struct ObstacleDetectorStates;

        // END_GLOBAL_PLANNER(Obstacle)
    };
} // namespace planning_detector

extern planning_detector::ObstacleDetector &g_obstacle_detector; 

#endif // DETECTOR_OBSTACLE_DETECTOR_H