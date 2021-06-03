#ifndef GLOBAL_TEB_PLANNER_EVENTS_H
#define GLOBAL_TEB_PLANNER_EVENTS_H

#include "event_center/event.h"
#include "misc/planning_typedefs.h"
#include <vector>
using namespace planning_data;



/**
  * @desc   EvTebOptPath teb优化路径
  * @eg     None
  */
DEFINE_EVENT(TebOptPath)
    PosePathPtr opt_path_ptr;
    size_t end_idx;
    bool is_robot_as_start;
END_EVENT(TebOptPath)

/**
  * @desc   EvTebOptPath teb优化路径
  * @eg     None
  */
DEFINE_EVENT(TebRotateReq)
    float  target_yaw;
    bool dir; //1->left 0->right
END_EVENT(TebRotateReq)


/**
  * @desc   EvTebPlannerError 
  * @eg     None
  */
DEFINE_EVENT(GTebPlannerError)
    int64_t input_id;
END_EVENT(GTebPlannerError)

/**
  * @desc   EvTebPlannerReplan 
  * @eg     None
  */
DEFINE_EVENT(GTebPlannerReplan)
    int64_t input_id;
END_EVENT(GTebPlannerError)


#endif // CONTROLLER_TEB_PLANNER_EVENTS_H