#ifndef CONTROLLER_TEB_PLANNER_EVENTS_H
#define CONTROLLER_TEB_PLANNER_EVENTS_H

#include "event_center/event.h"
#include <vector>
using namespace planning_data;

/**
  * @desc   EvTaskFinished 任务完成时间
  * @eg     None
  */
DEFINE_EVENT(TaskTebFinished)
    int64_t task_id;        // 完成的任务id
END_EVENT(TaskTebFinished)
/**
  * @desc   EvTaskFinished 任务完成时间
  * @eg     None
  */
DEFINE_EVENT(TaskTebError)
    int64_t task_id;        // 完成的任务id
END_EVENT(TaskTebError)

/**
  * @desc   EvTaskFinished 任务完成时间
  * @eg     None
  */
DEFINE_EVENT(TaskTebReplan)
    int64_t task_id;        // 完成的任务id
END_EVENT(TaskTebReplan)

/**
  * @desc   EvCurIndexInCurTask_TEB  定义CurIndexInCurTask_TEB
  * @eg     None
  */
DEFINE_EVENT(CurIndexInCurTask_TEB)

    size_t cur_index_in_cur_task; //全局路径段的第几个点
    size_t global_path_index; //记录下发的全局路径起始点的index
    std::vector<int> path_points_states;

END_EVENT(CurIndexInCurTask_TEB)

#endif // CONTROLLER_TEB_PLANNER_EVENTS_H