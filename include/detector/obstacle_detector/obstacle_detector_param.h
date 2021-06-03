#ifndef DETECTOR_OBSTACLE_DETECTOR_PARAM_H
#define DETECTOR_OBSTACLE_DETECTOR_PARAM_H

#include "detector/detector_param.h"
// #include "misc/planning_typedefs.h"

namespace planning_detector
{
    DEFINE_DETECTOR_PARAM(Obstacle)
        float vehicle_velocity = 1.5;
    END_DETECTOR_PARAM(Obstacle)
}

#endif // DETECTOR_OBSTACLE_DETECTOR_PARAM_H