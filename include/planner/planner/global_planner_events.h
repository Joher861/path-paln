#ifndef PLANNER_GLOBAL_PLANNER_EVENTS_H
#define PLANNER_GLOBAL_PLANNER_EVENTS_H

#include "event_center/event.h"

using planning_data::Event;

/**
  * @desc   EvPlannerFinished 规划任务完成事件
  * @eg     None
  */
DEFINE_EVENT(PlannerFinished)
    int64_t id;        // 对应的input的id
END_EVENT(PlannerFinished)

/**
  * @desc   EvPlannerFinished 任务完成时间
  * @eg     None
  */
DEFINE_EVENT(PlannerError)
    int64_t id;         // 对应input的id
    uint8_t error_type; // 对应异常的类型
END_EVENT(PlannerError)

#endif // PLANNER_GLOBAL_PLANNER_EVENTS_H