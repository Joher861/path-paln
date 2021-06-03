#ifndef DATA_CONTROL_SPEED_DATA_H
#define DATA_CONTROL_SPEED_DATA_H

#include <cinttypes>

#include "misc/planning_typedefs.h"
#include "data_center/data.h"

using planning_data::Data;

DEFINE_DATA(ControlSpeed)
    float linear_vel;
    float angular_steer;

    // uint64_t ts;                // 坐标对应的时间戳

END_DATA(ControlSpeed)

#endif