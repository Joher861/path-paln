#ifndef DETECTOR_COLLISION_DETECTOR_PARAM_H
#define DETECTOR_COLLISION_DETECTOR_PARAM_H

#include "detector/detector_param.h"
#include "misc/planning_typedefs.h"

namespace planning_detector
{
    DEFINE_DETECTOR_PARAM(Collision)
        PosePath current_path_points;
        float vehicle_velocity;
        float path_points_resolution;
    END_DETECTOR_PARAM(Collision)
}

#endif // DETECTOR_COLLISION_DETECTOR_PARAM_H