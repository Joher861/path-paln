#ifndef PLANNER_NAVIGATION_PLANNER_H
#define PLANNER_NAVIGATION_PLANNER_H

#include "planner/global_planner.h"
#include "navigation_planner_input.h"
#include "data/slam_data.h"
#include "data/local_map_data.h"

namespace planning_planner
{

    DEFINE_GLOBAL_PLANNER(navigation)

public:
    PosePath get_global_path()
    {
        return Hybrid_A_start_path;
    }

private:
    float loadConfig(ConfigManager &cfg_mgr);
    void initRunSM();
    void reset();
    void getData();
    void finishPlanning();
    bool handleInput(const GlobalPlannerInputPtr input);
    ContextPtr saveContext();
    Transition restoreContext(ContextPtr ctx);

    void stopRobot();

    void saveCleanStatusLog();
    void saveEndPtLog();

    bool Hybrid_A_star();


    int navigation_planner_finished = 0;

    float gridOriginX = 0.0f;
    float gridOriginY = 0.0f;
    float gridPrecision = 0.05f;
    std::vector<double> START = {0, 0, 0};
    std::vector<double> GOAL = {0, 0, 0};
    float safe_distance = 0;
    float AStarGridScale = 0.4;

    std::vector<uint8_t> map = {0, 0, 0};

    PosePath Hybrid_A_start_path;

    friend struct navigationPlannerStates;

    int64_t m_path_follow_planner_input_id;

    END_GLOBAL_PLANNER(navigation)
} // namespace planning_planner

extern planning_planner::navigationPlanner &g_navigation_planner;

#endif // PLANNER_navigation_PLANNER_H