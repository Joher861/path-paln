
#ifndef DATA_CENTER_LOCAL_MAP_DATA_H
#define DATA_CENTER_LOCAL_MAP_DATA_H

#include "misc/planning_typedefs.h"
// #include "misc/planning_defs.h"
#include "data_center/data.h"

using planning_data::Data;

DEFINE_DATA(LocalMap)
    // robot_pose pose;
    uint64_t ts;
    float x;
    float y;
    float theta;    
    bool obstacleInFront=false;
    int distanceToFrontObstacle=-1;

    uint16_t grid[161][161];
    
END_DATA(LocalMap)

#endif // DATA_CENTER_LOCAL_MAP_DATA_H