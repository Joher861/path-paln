#ifndef PLANNER_CCPP_PLANNER_H
#define PLANNER_CCPP_PLANNER_H

#include "planner/global_planner.h"
#include "ccpp_coverage_planner_input.h"
#include "ccpp_data.h"
#include "pre_process.h"

namespace planning_planner
{

    DEFINE_GLOBAL_PLANNER(ccppCoverage)

public:
    DataCCPP get_global_path()
    {
        return ccpp_coverage_path_output;
    }

private:
    float loadConfig(ConfigManager &cfg_mgr);
    void initRunSM();
    void getData();
    void reset();
    void finishPlanning();
    bool handleInput(const GlobalPlannerInputPtr input);
    ContextPtr saveContext();
    Transition restoreContext(ContextPtr ctx);

    void stopRobot();
    param param_ccpp;
    // log相关函数
    void saveCleanStatusLog();
    void saveEndPtLog();

    // update到data_center的path
    DataCCPP ccpp_coverage_path_output;
    int ccpp_planner_finished = 0;
    friend struct ccppPlannerStates;

    END_GLOBAL_PLANNER(ccppCoverage)
} // namespace planning_planner

extern planning_planner::ccppCoveragePlanner &g_ccpp_coverage_planner;

#endif