#ifndef DETECTOR_EMERGENCY_STOP_DETECTOR_EVENTS_H
#define DETECTOR_EMERGENCY_STOP_DETECTOR_EVENTS_H

#include "event_center/event.h"
#include "event_center/event_center.h"

#include "misc/planning_typedefs.h"

using namespace planning_data;

/**
  * @desc   EvEmergencyStop  定义EmergencyStop
  * @eg     None
  */
DEFINE_EVENT(EmergencyStop)
END_EVENT(EmergencyStop)

/**
  * @desc   EvEmergencySlowdown  定义EmergencySlowdown
  * @eg     None
  */
DEFINE_EVENT(EmergencySlowdown)
float discount = 0.5;
END_EVENT(EmergencySlowdown)

/**
  * @desc   EvCoordJumped  定义CoordJumped
  * @eg     None
  */
DEFINE_EVENT(CoordJumped)
END_EVENT(CoordJumped)

/**
  * @desc   EvEmergencyUSFront 定义EmergencyUSFront
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSFront)
END_EVENT(EmergencyUSFront)

/**
  * @desc   EvEmergencyUSFrontLeft  定义EmergencyUSFrontLeft
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSFrontLeft)
END_EVENT(EmergencyUSFrontLeft)

/**
  * @desc   EvEmergencyUSFrontRight  定义EmergencyUSFrontRight
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSFrontRight)
END_EVENT(EmergencyUSFrontRight)

/**
  * @desc   EvEmergencyUSLeft  定义EmergencyUSLeft
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSLeft)
END_EVENT(EmergencyUSLeft)

/**
  * @desc   EvEmergencyUSRight  定义EmergencyUSRight
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSRight)
END_EVENT(EmergencyUSRight)

/**
  * @desc   EvEmergencyUSRightPure  定义EmergencyUSRightPure
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSRightPure)
END_EVENT(EmergencyUSRightPure)

/**
  * @desc   EvEmergencyUSLeftPure  定义EmergencyUSLeftPure
  * @eg     None
  */
DEFINE_EVENT(EmergencyUSLeftPure)
END_EVENT(EmergencyUSLeftPure)

/**
  * @desc   EvUSEscapeFRLeftRotate  定义USEscapeFRLeftRotate
  * @eg     None
  */
DEFINE_EVENT(USEscapeFRLeftRotate)
END_EVENT(USEscapeFRLeftRotate)

/**
  * @desc   EvUSEscapeRLeftRotate  定义USEscapeRLeftRotate
  * @eg     None
  */
DEFINE_EVENT(USEscapeRLeftRotate)
END_EVENT(USEscapeRLeftRotate)

/**
  * @desc   EvUSEscapeFLRightRotate  定义USEscapeFLRightRotate
  * @eg     None
  */
DEFINE_EVENT(USEscapeFLRightRotate)
END_EVENT(USEscapeFLRightRotate)

/**
  * @desc   EvUSEscapeLRightRotate  定义USEscapeLRightRotate
  * @eg     None
  */
DEFINE_EVENT(USEscapeLRightRotate)
END_EVENT(USEscapeLRightRotate)

/**
  * @desc   EvUSEscapeFRightRotate  定义USEscapeFRightRotate
  * @eg     None
  */
DEFINE_EVENT(USEscapeFRightRotate)
END_EVENT(USEscapeFRightRotate)

/**
  * @desc   EvUSEscapeFLeftRotate  定义USEscapeFLeftRotate
  * @eg     None
  */
DEFINE_EVENT(USEscapeFLeftRotate)
END_EVENT(USEscapeFLeftRotate)

/**
  * @desc   EvUSFrontLasting  定义USFrontLasting
  * @eg     None
  */
DEFINE_EVENT(USFrontLasting)
END_EVENT(USFrontLasting)

/**
  * @desc   EvUSFrontLeftLasting  定义USFrontLeftLasting
  * @eg     None
  */
DEFINE_EVENT(USFrontLeftLasting)
END_EVENT(USFrontLeftLasting)

/**
  * @desc   EvUSFrontRightLasting  定义USFrontRightLasting
  * @eg     None
  */
DEFINE_EVENT(USFrontRightLasting)
END_EVENT(USFrontRightLasting)

/**
  * @desc   EvUSLeftLasting  定义USLeftLasting
  * @eg     None
  */
DEFINE_EVENT(USLeftLasting)
END_EVENT(USLeftLasting)

/**
  * @desc   EvUSRightLasting  定义USRightLasting
  * @eg     None
  */
DEFINE_EVENT(USRightLasting)
END_EVENT(USRightLasting)

#endif // end DETECTOR_EMERGENCY_STOP_DETECTOR_EVENTS_H