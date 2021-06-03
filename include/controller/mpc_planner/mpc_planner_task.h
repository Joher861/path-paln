#ifndef CONTROLLER_MPC_PLANNER_TASK_H
#define CONTROLLER_MPC_PLANNER_TASK_H

#include "task/task.h"
#include "misc/planning_typedefs.h"

namespace planning_controller
{
    DEFINE_TASK(MPC, NONE_PREEMPT, true)
        enum MPCType 
        {
            MPC_FOLLOW = 0, 
            MAX_MPC_TYPE
        };

        MPCType type            = MAX_MPC_TYPE;
        PosePath mpc_ref_path;            
        size_t start_segment_index = 0;     
        int global_path_index = -1;
        float velocity = 0.3;
    END_TASK(MPC)
}

#endif // CONTROLLER_MPC_TASK_H