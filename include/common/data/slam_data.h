#ifndef DATA_CENTER_SLAM_DATA_H
#define DATA_CENTER_SLAM_DATA_H

#include <cinttypes>

#include "misc/planning_typedefs.h"
#include "data_center/data.h"

using planning_data::Data;

DEFINE_DATA(Slam)
    // 坐标信息
    RobotPose pose;                // 坐标
    int8_t pose_status;         // 坐标状态（正常、丢失、重定位中、重定位成功、下位机数据异常等）

    uint64_t ts;                // 坐标对应的时间戳

    // imu rpy信息
    float roll;
    float pitch;
    float yaw;

    // 打滑状态
    int8_t slip_status;

    // 地图信息
    int64_t map_id;
    
    // 提线数据
    uint64_t compass_ts;
    RobotTransform compass_tf;

    // 重定位数据
    uint64_t reloc_ts;
    RobotTransform reloc_tf;
END_DATA(Slam)

#endif