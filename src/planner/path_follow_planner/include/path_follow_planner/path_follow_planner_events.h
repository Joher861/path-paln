#ifndef PLANNER_PATH_FOLLOW_PLANNER_EVENTS_H
#define PLANNER_PATH_FOLLOW_PLANNER_EVENTS_H

#include "event_center/event.h"
#include "event_center/event_center.h"

#include "misc/planning_typedefs.h"

using namespace planning_data;

/**
  * @desc   EvPathFollowGoExit
  * @eg     None
  */
DEFINE_EVENT(PathFollowGoExit)

END_EVENT(PathFollowGoExit)

/**
  * @desc   EvPathFollowPlannerError
  * @eg     None
  */
DEFINE_EVENT(PathFollowPlannerError)
int64_t id;
END_EVENT(PathFollowPlannerError)

/**
  * @desc   EvPFPDetIntendVel
  * @eg     None
  */
DEFINE_EVENT(PFPDetIntendVel)
float intend_velocity;
END_EVENT(PFPDetIntendVel)

/**
  * @desc   EvPFPtebPlanningReq
  * @eg     None
  */
DEFINE_EVENT(PFPtebPlanningReq)
PosePathPtr path_to_teb_ptr;
// size_t start_idx;
size_t end_idx;
bool is_first = false;
bool is_end = false;
bool robot_pose_as_start = false;
END_EVENT(PFPtebPlanningReq)

/**
  * @desc   EvPFPtebRelax
  * @eg     None
  */
DEFINE_EVENT(PFPtebRelax)
END_EVENT(PFPtebRelax)

/**
  * @desc   EvPFPOAMode
  * @eg     None
  */
DEFINE_EVENT(PFPOAMode)
bool oa_is_on;
END_EVENT(PFPOAMode)

/**
  * @desc   EvPFPFeedbackPathStates
  * @eg     None
  */
DEFINE_EVENT(PFPFeedbackPathStates)
int64_t input_idx;
std::vector<int> path_points_situation;
END_EVENT(PFPFeedbackPathStates)

/**
  * @desc   EvPFPCollisionFreeze
  * @eg     None
  */
DEFINE_EVENT(PFPCollisionFreeze)
END_EVENT(PFPCollisionFreeze)


#endif // end PLANNER_PATH_FOLLOW_PLANNER_EVENTS_H