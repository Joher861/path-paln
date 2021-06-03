#ifndef PLANNER_GLOBAL_PLANNER_EVENTS_H
#define PLANNER_GLOBAL_PLANNER_EVENTS_H

#include "event_center/event.h"

using planning_data::Event;

DEFINE_EVENT(PlannerFinished)
    int64_t id;        
END_EVENT(PlannerFinished)


DEFINE_EVENT(PlannerError)
    int64_t id;        
    uint8_t error_type;
END_EVENT(PlannerError)

#endif // PLANNER_GLOBAL_PLANNER_EVENTS_H