#ifndef PLANNER_TEB_INPUT_H
#define PLANNER_TEB_INPUT_H

#include "planner/global_planner_input.h"
#include <vector>
#include "misc/planning_typedefs.h"

namespace planning_planner
{

    DEFINE_GLOBAL_PLANNER_INPUT(Teb) //TebPlannerInput
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
    END_GLOBAL_PLANNER_INPUT(Teb)
}

// eg.
// CREATE_GLOBAL_PLANNER_INPUT(TebPlannerInput, global_teb_input);

// std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());
// for (uint16_t i = 0; i < test_path_tmp.size(); i++)
// {
//     if (i % 10 == 0)
//     {
//         RobotPose pose;
//         pose.pt.x = test_path_tmp.at(i).pt.x;
//         pose.pt.y = test_path_tmp.at(i).pt.y;
//         pose.theta = test_path_tmp.at(i).theta;
//         teb_path_ptr->add(pose);
//     }
// }
// global_teb_input->path = teb_path_ptr;

// g_teb_planner.startPlanner(global_teb_input);


#endif // CONTROLLER_ROTATE_TASK_H