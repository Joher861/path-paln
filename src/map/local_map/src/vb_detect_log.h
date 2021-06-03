#ifndef MAP_VB_DETECT_LOG_H
#define MAP_VB_DETECT_LOG_H

#include "log/csv_log_data.h"
#include "misc/planning_typedefs.h"
#include "geometry/geometry_func.h"

using planning_utils::CSVLogData;
using planning_utils::rad2deg;

struct CSVVBBaseInfo : CSVLogData
{
    float r;
    Point curve_center;

    CSVVBBaseInfo(float _r, const Point &_curve_center)
        : r(_r), curve_center(_curve_center)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(r) + delimiter
            + std::to_string(curve_center.x) + delimiter
            + std::to_string(curve_center.y);

        return log_str;
    }
};

#define LOG_VB_BASE_INFO_FLAG  vb_base_info
#define LOG_VB_BASE_INFO       "vb_base_info"

#define VB_BASE_INFO_LOG(r, curve_center)                                      \
    LOG(LOG_NAMES(LOG_VB_BASE_INFO),                                           \
        std::make_shared<CSVVBBaseInfo>(r, curve_center).get())

struct CSVVBEdgeInfo : CSVLogData
{
    RobotPose pose;
    RobotGrid grid;
    float heading;

    CSVVBEdgeInfo(const RobotPose &_pose, const RobotGrid &_grid, float _heading)
        : pose(_pose), grid(_grid), heading(_heading)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(pose.pt.x) + delimiter
            + std::to_string(pose.pt.y) + delimiter
            + std::to_string(rad2deg(pose.theta)) + delimiter
            + std::to_string(grid.x) + delimiter
            + std::to_string(grid.y) + delimiter
            + std::to_string(rad2deg(heading));

        return log_str;
    }
};

#define LOG_VB_EDGE_INFO_FLAG  vb_edge_info
#define LOG_VB_EDGE_INFO       "vb_edge_info"

#define VB_EDGE_INFO_LOG(pose, grid, heading)                                  \
    LOG(LOG_NAMES(LOG_VB_EDGE_INFO),                                           \
        std::make_shared<CSVVBEdgeInfo>(pose, grid, heading).get())

struct CSVVBObInfo : CSVLogData
{
    RobotPose pose;
    RobotGrid grid;

    CSVVBObInfo(const RobotPose &_pose, const RobotGrid &_grid)
        : pose(_pose), grid(_grid)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(pose.pt.x) + delimiter
            + std::to_string(pose.pt.y) + delimiter
            + std::to_string(rad2deg(pose.theta)) + delimiter
            + std::to_string(grid.x) + delimiter
            + std::to_string(grid.y);

        return log_str;
    }
};

#define LOG_VB_OB_INFO_FLAG  vb_ob_info
#define LOG_VB_OB_INFO       "vb_ob_info"

#define VB_OB_INFO_LOG(pose, grid)                                             \
    LOG(LOG_NAMES(LOG_VB_OB_INFO),                                             \
        std::make_shared<CSVVBObInfo>(pose, grid).get())

struct CSVVBResult : CSVLogData
{
    RobotPose pose;
    RobotGrid grid;
    RobotPose bump_pose;
    float angle_diff;
    float dist_diff;

    CSVVBResult(const RobotPose &_pose, const RobotGrid &_grid, const RobotPose &_bump_pose,
        float _angle_diff, float _dist_diff)
        : pose(_pose), grid(_grid), bump_pose(_bump_pose), angle_diff(_angle_diff),
            dist_diff(_dist_diff)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(pose.pt.x) + delimiter
            + std::to_string(pose.pt.y) + delimiter
            + std::to_string(rad2deg(pose.theta)) + delimiter
            + std::to_string(grid.x) + delimiter
            + std::to_string(grid.y) + delimiter
            + std::to_string(bump_pose.pt.x) + delimiter
            + std::to_string(bump_pose.pt.y) + delimiter
            + std::to_string(rad2deg(bump_pose.theta)) + delimiter
            + std::to_string(rad2deg(angle_diff)) + delimiter
            + std::to_string(dist_diff);

        return log_str;
    }
};

#define LOG_VB_RESULT_FLAG  vb_result
#define LOG_VB_RESULT       "vb_result"

#define VB_RESULT_LOG(pose, grid, bump_pose, angle_diff, dist_diff)            \
    LOG(LOG_NAMES(LOG_VB_RESULT),                                              \
        std::make_shared<CSVVBResult>(pose, grid, bump_pose, angle_diff, dist_diff).get())

#endif // MAP_VB_DETECT_LOG_H