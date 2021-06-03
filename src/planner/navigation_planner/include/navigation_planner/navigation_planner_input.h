#ifndef PLANNER_NAVIGATION_PLANNER_INPUT_H
#define PLANNER_NAVIGATION_PLANNER_INPUT_H

#include "planner/global_planner_input.h"
#include "misc/planning_typedefs.h"

namespace planning_planner
{
    DEFINE_GLOBAL_PLANNER_INPUT(Navigation)

    float AStarGridScale = 0.4;
    RobotPose start;
    RobotPose goal;    
    float safe_distance = 0.2;
    
    END_GLOBAL_PLANNER_INPUT(Navigation)
} // namespace planning_planner

#endif // PLANNER_NAVIGATION_PLANNER_INPUT_H