#ifndef PLANNER_PATH_FOLLOW_PLANNER_INPUT_H
#define PLANNER_PATH_FOLLOW_PLANNER_INPUT_H

#include "planner/global_planner_input.h"
#include "misc/planning_typedefs.h"

namespace planning_planner
{
    DEFINE_GLOBAL_PLANNER_INPUT(PathFollow)
    bool need_path_insert = false;
    int64_t input_idx = -1;
    bool feedback_path_points_states = false;
    PosePath global_path;
    END_GLOBAL_PLANNER_INPUT(PathFollow)
} // namespace planning_planner

#endif // PLANNER_PATH_FOLLOW_PLANNER_INPUT_H