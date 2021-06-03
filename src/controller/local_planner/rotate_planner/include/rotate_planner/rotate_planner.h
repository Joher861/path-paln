#ifndef CONTROLLER_ROTATE_PLANNER_H
#define CONTROLLER_ROTATE_PLANNER_H

#include "local_planner/local_planner.h"
#include "rotate_task.h"
#include "rotate_planner_events.h"
#include "data/slam_data.h"

namespace planning_controller
{
    // using TaskRotate::RotateType;
    // using TaskRotate::RotateDirection;

    DEFINE_LOCAL_PLANNER(Rotate)

      public:
    
        static RotatePlannerPtr& getInstance(ThreadPool *pool);

      private:

        RotatePlanner(ThreadPool *pool);
        virtual float loadConfig(ConfigManager &cfg_mgr);
        virtual void initRunSM();
        virtual void reset();
        virtual void getData();
        virtual bool handleTask(const Task& task);
        virtual void finishTask();

        DataSlam m_slam_data;

        TaskRotate::RotateType m_type;
        TaskRotate::RotateDirection m_dir;
        TaskRotate::RotateReason m_reason;
        float m_rotate_vel;
        float m_target_angle;
        float m_delta_angle;
        float m_remained_angle;
        bool left_dir;
        bool rotation_finished;
        bool stuck_flag;
        bool standby_flag;

        friend struct RotatePlannerStates;

    END_LOCAL_PLANNER(Rotate)
}

#endif // CONTROLLER_ROTATE_PLANNER_H