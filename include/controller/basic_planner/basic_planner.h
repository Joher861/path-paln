
#ifndef CONTROLLER_BASIC_PLANNER_H
#define CONTROLLER_BASIC_PLANNER_H

#include "local_planner/local_planner.h"
#include "basic_task.h"

namespace planning_controller
{


    DEFINE_LOCAL_PLANNER(Basic)

      public:
    
        static BasicPlannerPtr& getInstance(ThreadPool *pool);
        
      private:

        BasicPlanner(ThreadPool *pool);
        virtual float loadConfig(ConfigManager &cfg_mgr);
        virtual void initRunSM();
        virtual void reset();
        virtual void getData();
        virtual bool handleTask(const Task& task);
        virtual void finishTask();


        TaskBasic::BasicType type = TaskBasic::MAX_BASIC_TYPE;
        float v                 = 0.0f;
        float w                 = 0.0f;
        float vl                = 0.0f;
        float vr                = 0.0f;
        bool smooth             = false;
        uint64_t smooth_time    = 200;
        float radius            = 0.0f;

        friend struct BasicPlannerStates;

    END_LOCAL_PLANNER(Basic)
}

#endif // CONTROLLER_BASIC_PLANNER_H