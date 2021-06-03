
#ifndef DATA_CENTER_CHASSIS_MODE_H
#define DATA_CENTER_CHASSIS_MODE_H

#include "misc/planning_typedefs.h"
#include "data_center/data.h"

using planning_data::Data;

DEFINE_DATA(ChassisMode)
    uint8_t chassis_mode;
    uint8_t start_button;
END_DATA(ChassisMode)

#endif // DATA_CENTER_CHASSIS_MODE_H