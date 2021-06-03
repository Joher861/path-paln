#ifndef CONTROLLER_LOCAL_PLANNER_EVENTS_H
#define CONTROLLER_LOCAL_PLANNER_EVENTS_H

#include "event_center/event.h"

using namespace planning_data;

/**
  * @desc   EvTaskFinished 任务完成事件
  * @eg     None
  */
DEFINE_EVENT(TaskFinished)
    int64_t task_id;        // 完成的任务id
END_EVENT(TaskFinished)

/**
  * @desc   EvTaskError    任务异常事件
  * @eg     None
  */
DEFINE_EVENT(TaskError)
    int64_t task_id;        // 异常任务id
    uint8_t err_type;       // 异常的类型
END_EVENT(TaskError)

#endif // CONTROLLER_LOCAL_PLANNER_EVENTS_H