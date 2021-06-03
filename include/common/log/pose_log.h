
#ifndef POSE_LOG_H
#define POSE_LOG_H

#include "log/csv_log_data.h"
#include "log/log_manager.h"
#include "geometry/geometry_func.h"

using namespace planning_utils;

struct CSVPoseData : CSVLogData
{
    float x;
    float y;
    float z;

    float roll;
    float pitch;
    float yaw;

    //TODO:slam 全部信息记录

    CSVPoseData(float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
        : x(_x), y(_y), z(_z), roll(_roll), pitch(_pitch), yaw(_yaw)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(x) + delimiter
                            + std::to_string(y) + delimiter
                            + std::to_string(rad2deg(z)) + delimiter
                            + std::to_string(roll) + delimiter
                            + std::to_string(pitch) + delimiter
                            + std::to_string(yaw);
        return log_str;
    }
};

#define LOG_POSE_FLAG pose
#define LOG_POSE      "pose"

#define POSE_LOG(x, y, z, roll, pitch, yaw)   \
    LOG(LOG_NAMES(LOG_POSE), std::make_shared<CSVPoseData>(x, y, z, roll, pitch, yaw).get())



#endif //POSE_LOG_H