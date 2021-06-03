#ifndef DETECTOR_EMERGENCY_STOP_DETECTOR_H
#define DETECTOR_EMERGENCY_STOP_DETECTOR_H

#include "detector/detector.h"
// #include "detector/detector_param.h"
// #include "detector/detector_context.h"
#include "emergency_stop_detector_param.h"
// #include "emergency_stop_detector_log.h"
#include "emergency_stop_detector_context.h"
// #include "emergency_stop_detector_config.h"
#include "emergency_stop_detector_events.h"

#include "data/data_center/data_center.h"
#include "data/event_center/event_center.h"
#include "event_center/event.h"

// #include "misc/planning_typedefs.h"
#include "data/slam_data.h"
#include "data/chassis_data.h"
#include "data/local_map_data.h"
#include "data/local_map_ready.h"
#include "data/control_speed_data.h"
// #include "data/local_map_data.h"

using namespace std;

namespace planning_detector
{
    class EmergencyStopDetector : public planning_detector::Detector
    {
    // DEFINE_DETECTOR(EmergencyStop)
      public:
        // static EmergencyStopDetector &getInstance(ThreadPool *pool);
        static EmergencyStopDetector &getInstance();

      private:

        // EmergencyStopDetector(ThreadPool *pool);
        EmergencyStopDetector();
        virtual float loadConfig(ConfigManager &cfg_mgr);
        virtual void initRunSM();
        virtual void reset();
        virtual void getData();
        virtual bool handleParam(DetectorParamPtr param);
        virtual void finishDetect();

        virtual ContextPtr saveContext();
        virtual Transition restoreContext(ContextPtr ctx);  

        float m_intended_velocity;
        float m_max_break_acc;
        DataSlam m_slam_data;
        DataFeedbackSpeed m_feedback_control;
        DataUltrasonic m_ultrasonic_data;
        DataLocalMap m_simple_local_map;
        DataControlSpeed m_control_speed;
        DataLocalMapReady m_local_map_ready;
        bool m_must_stop;
        float m_lasting_period;
        float m_reset_lasting_period;
        float m_jumped_dist_threshold;
        float m_threshold_ultrasonic_front_default;
        float m_threshold_ultrasonic_front_left_default;
        float m_threshold_ultrasonic_front_right_default;
        float m_threshold_ultrasonic_front;
        float m_threshold_ultrasonic_front_left;
        float m_threshold_ultrasonic_front_right;
        float m_threshold_ultrasonic_left_down_1;
        float m_threshold_ultrasonic_left_down_2;
        float m_threshold_ultrasonic_right_down_1;
        float m_threshold_ultrasonic_right_down_2;
        float m_threshold_slowdown_front;
        float m_threshold_slowdown_side;
        int m_print_us;

        friend struct EmergencyStopDetectorStates;

        // END_GLOBAL_PLANNER(EmergencyStop)
    };
} // namespace planning_detector

extern planning_detector::EmergencyStopDetector &g_emergency_stop_detector; //定义闭合检测器

#endif // DETECTOR_EMERGENCY_STOP_DETECTOR_H