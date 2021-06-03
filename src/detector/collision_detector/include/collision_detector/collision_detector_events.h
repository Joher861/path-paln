#ifndef DETECTOR_COLLISION_DETECTOR_EVENTS_H
#define DETECTOR_COLLISION_DETECTOR_EVENTS_H

#include "event_center/event.h"
#include "event_center/event_center.h"

#include "misc/planning_typedefs.h"

using namespace planning_data;

/**
  * @desc   EvCollisionTrue  定义CollisionTrue
  * @eg     None
  */
DEFINE_EVENT(CollisionTrue)

END_EVENT(CollisionTrue)

#endif // end DETECTOR_COLLISION_DETECTOR_EVENTS_H