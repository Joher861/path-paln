#ifndef CONTROLLER_ROTATE_PLANNER_EVENTS_H
#define CONTROLLER_ROTATE_PLANNER_EVENTS_H

#include "event_center/event.h"
#include "event_center/event_center.h"

#include "misc/planning_typedefs.h"

using namespace planning_data;

/**
  * @desc   EvRotationStuck  定义RotationStuck
  * @eg     None
  */
DEFINE_EVENT(RotationStuck)

END_EVENT(RotationStuck)

/**
  * @desc   EvRotationStandby  定义RotationStandby
  * @eg     None
  */
DEFINE_EVENT(RotationStandby)

END_EVENT(RotationStandby)

#endif // end CONTROLLER_ROTATE_PLANNER_EVENTS_H