#ifndef DETECTOR_OBSTACLE_DETECTOR_EVENTS_H
#define DETECTOR_OBSTACLE_DETECTOR_EVENTS_H

#include "event_center/event.h"
#include "event_center/event_center.h"

#include "misc/planning_typedefs.h"

using namespace planning_data;

/**
  * @desc   EvSmallObstacle  定义SmallObstacle
  * @eg     None
  */
DEFINE_EVENT(SmallObstacle)

END_EVENT(SmallObstacle)

/**
  * @desc   EvLargeObstacle  定义LargeObstacle
  * @eg     None
  */
DEFINE_EVENT(LargeObstacle)

END_EVENT(LargeObstacle)

/**
  * @desc   EvFarCircleObstacle  定义FarCircleObstacle
  * @eg     None
  */
DEFINE_EVENT(FarCircleObstacle)
    int obstacle_dir;
END_EVENT(FarCircleObstacle)

/**
  * @desc   EvNearCircleObstacle  定义NearCircleObstacle
  * @eg     None
  */
DEFINE_EVENT(NearCircleObstacle)
    int obstacle_dir;
END_EVENT(NearCircleObstacle)

/**
  * @desc   EvHaveObstacle  定义HaveObstacle
  * @eg     None
  */
DEFINE_EVENT(HaveObstacle)
    std::vector<int> have_obstacle;
END_EVENT(HaveObstacle)

/**
  * @desc   EvEmergencyLMFront  定义EmergencyLMFront
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMFront)
END_EVENT(EmergencyLMFront)

/**
  * @desc   EvEmergencyLMLeft  定义EmergencyLMLeft
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMLeft)
END_EVENT(EmergencyLMLeft)

/**
  * @desc   EvEmergencyLMFrontLeft  定义EmergencyLMFrontLeft
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMFrontLeft)
END_EVENT(EmergencyLMFrontLeft)

/**
  * @desc   EvEmergencyLMRight  定义EmergencyLMRight
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMRight)
END_EVENT(EmergencyLMRight)

/**
  * @desc   EvEmergencyLMFrontRight  定义EmergencyLMFrontRight
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMFrontRight)
END_EVENT(EmergencyLMFrontRight)

/**
  * @desc   EvEmergencyLMRightPure  定义EmergencyLMRightPure
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMRightPure)
END_EVENT(EmergencyLMRightPure)

/**
  * @desc   EvEmergencyLMLeftPure  定义EmergencyLMLeftPure
  * @eg     None
  */
DEFINE_EVENT(EmergencyLMLeftPure)
END_EVENT(EmergencyLMLeftPure)

/**
  * @desc   EvLMFrontLasting  定义LMFrontLasting
  * @eg     None
  */
DEFINE_EVENT(LMFrontLasting)
END_EVENT(LMFrontLasting)

/**
  * @desc   EvLMLeftLasting  定义LMLeftLasting
  * @eg     None
  */
DEFINE_EVENT(LMLeftLasting)
END_EVENT(LMLeftLasting)

/**
  * @desc   EvLMFrontLeftLasting  定义LMFrontLeftLasting
  * @eg     None
  */
DEFINE_EVENT(LMFrontLeftLasting)
END_EVENT(LMFrontLeftLasting)

/**
  * @desc   EvLMRightLasting  定义LMRightLasting
  * @eg     None
  */
DEFINE_EVENT(LMRightLasting)
END_EVENT(LMRightLasting)

/**
  * @desc   EvLMFrontRightLasting  定义LMFrontRightLasting
  * @eg     None
  */
DEFINE_EVENT(LMFrontRightLasting)
END_EVENT(LMFrontRightLasting)

/**
  * @desc   EvLMFreezeStop  定义LMFreezeStop
  * @eg     None
  */
DEFINE_EVENT(LMFreezeStop)
END_EVENT(LMFreezeStop)

/**
  * @desc   EvLMEmergencyStop  定义LMEmergencyStop
  * @eg     None
  */
DEFINE_EVENT(LMEmergencyStop)
END_EVENT(LMEmergencyStop)

/**
  * @desc   EvLMEmergencySlowdown  定义LMEmergencySlowdown
  * @eg     None
  */
DEFINE_EVENT(LMEmergencySlowdown)
float discount = 0.5;
END_EVENT(LMEmergencySlowdown)

#endif // end DETECTOR_OBSTACLE_DETECTOR_EVENTS_H