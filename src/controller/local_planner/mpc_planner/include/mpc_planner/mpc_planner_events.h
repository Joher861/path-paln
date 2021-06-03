#ifndef CONTROLLER_MPC_PLANNER_EVENTS_H
#define CONTROLLER_MPC_PLANNER_EVENTS_H

#include "event_center/event.h"
#include "event_center/event_center.h"

#include "misc/planning_typedefs.h"

using namespace planning_data;

/**
  * @desc   EvCurIndexInCurTask_MPC  定义CurIndexInCurTask_MPC
  * @eg     None
  */
DEFINE_EVENT(CurIndexInCurTask_MPC)

    size_t cur_index_in_cur_task;
    int global_path_index;
    std::vector<int> path_points_states;

END_EVENT(CurIndexInCurTask_MPC)

/**
  * @desc   EvTaskPathInfeasible  TaskPathInfeasible
  * @eg     None
  */
DEFINE_EVENT(TaskPathInfeasible_MPC)
int id;
END_EVENT(TaskPathInfeasible_MPC)

/**
  * @desc   EvTaskFinished_MPC  TaskFinished_MPC
  * @eg     None
  */
DEFINE_EVENT(TaskFinished_MPC)

END_EVENT(TaskFinished_MPC)

#endif // end CONTROLLER_MPC_PLANNER_EVENTS_H