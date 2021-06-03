#ifndef PLANNER_CCPP_PLANNER_INPUT_H
#define PLANNER_CCPP_PLANNER_INPUT_H

#include "planner/global_planner_input.h"
#include "misc/planning_typedefs.h"
#include "ccpp_data.h"
namespace planning_planner
{
    DEFINE_GLOBAL_PLANNER_INPUT(CCPP)
    std::vector<double> START_POS = {0,0,0};

    int planMode = 0;//模式0 ccpp规划 模式1 对指定区域重新规划
    int replanIndex = 0;
    std::vector<ccppCellData> ccppDataInput;
    std::vector<missOutPath> missOutDataInput;
    END_GLOBAL_PLANNER_INPUT(CCPP)
} // namespace planning_planner

#endif