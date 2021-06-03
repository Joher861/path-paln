
#ifndef DATA_CENTER_LOCAL_MAP_READY_H
#define DATA_CENTER_LOCAL_MAP_READY_H

#include "misc/planning_typedefs.h"
#include "data_center/data.h"

using planning_data::Data;

DEFINE_DATA(LocalMapReady)
    int flag = 0;
    uint64_t ts;
    
END_DATA(LocalMapReady)

#endif // DATA_CENTER_LOCAL_MAP_READY_H