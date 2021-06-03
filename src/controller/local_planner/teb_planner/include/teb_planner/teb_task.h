#ifndef CONTROLLER_NAV_TASK_H
#define CONTROLLER_NAV_TASK_H

#include "task/task.h"
#include <vector>
#include "misc/planning_typedefs.h"

namespace planning_controller
{
    //nav任务可以抢占其他任务，自身可被抢占
    //TaskTeb is the class name, TaskTebPtr is its shared ptr.
    DEFINE_TASK(Teb, PREEMPT, true)
        enum RunState 
        {
            ONCONTROLLING = 0,
            ONRECOVERY,
            MAX_TEB_RUN_STATE
        };

        enum RecoveryType 
        {
            ROTATE = 0,
            BACKWARD,
            BACKWARD_AND_ROTATE,
            FORWARD,
            FORWARD_AND_ROTATE,
            CLEAR_LOCALMAP,
            MAX_RECOVERY_TYPE
        };
        RunState state = ONCONTROLLING;
        PosePathPtr path;
        bool need_theta = false;//到点后是否需要旋转到目标角度
        size_t global_path_index = 0;
        RecoveryType recovery_type = MAX_RECOVERY_TYPE;
    END_TASK(Teb)
}

#endif // CONTROLLER_ROTATE_TASK_H