#ifndef CONTROLLER_MPC_PLANNER_TASK_H
#define CONTROLLER_MPC_PLANNER_TASK_H

#include "task/task.h"
#include "misc/planning_typedefs.h"

namespace planning_controller
{
    DEFINE_TASK(MPC, NONE_PREEMPT, true)
        enum MPCType 
        {
            MPC_FOLLOW = 0,       // MPC跟随路径
            MAX_MPC_TYPE
        };

        MPCType type            = MAX_MPC_TYPE;       // MPC任务类型
        PosePath mpc_ref_path;                      // MPC要跟随的路径
        size_t start_segment_index = 0;               // 路径起点
        int global_path_index = -1;
        float velocity = 0.3;
    END_TASK(MPC)
}

#endif // CONTROLLER_MPC_TASK_H