#ifndef MISC_ROBOT_CONFIG_H
#define MISC_ROBOT_CONFIG_H

#include "config/config.h"
#include "misc/footprint.h"
#include "geometry/geometry_func.h"

using planning_utils::deg2rad;

#define CONFIG_ROBOT    "cfg_robot"

DEFINE_CONFIG(Robot)
    float vehicle_longitude_size;  // unit: m
    float vehicle_lateral_size;    // unit: m
    int local_map_size_x;
    int local_map_size_y;
    int robot_pose_in_local_map_x_default;
    int robot_pose_in_local_map_y_default;
    float local_map_resolution;    // unit m
    float rotate_vel_default;      // unit m/s
    float max_linear_vel;          // unit: m/s
    float min_linear_vel;          // unit: m/s
    float max_break_acc;           // unit: m/s^2

    float wheels_distance; //unit:m
    float wheel_perimeter;
    float main_brush_length;
    float footprint_resolution;
    cv::Mat footprint_description;
    cv::Mat side_sensor_description;
    std::vector<RobotPose> side_sensor_poses;
    Footprint footprint;

    void setFoortprint()
    {
        footprint.setFootprint(footprint_resolution, footprint_description);
    }
END_CONFIG(Robot)

DEFINE_CONFIG_READ_FUNC(Robot)
    READ_CONFIG_MEMBER(vehicle_longitude_size);
    READ_CONFIG_MEMBER(vehicle_lateral_size);
    READ_CONFIG_MEMBER(local_map_size_x);
    READ_CONFIG_MEMBER(local_map_size_y);
    READ_CONFIG_MEMBER(robot_pose_in_local_map_x_default);
    READ_CONFIG_MEMBER(robot_pose_in_local_map_y_default);
    READ_CONFIG_MEMBER(local_map_resolution);
    READ_CONFIG_MEMBER(rotate_vel_default);
    READ_CONFIG_MEMBER(max_linear_vel);
    READ_CONFIG_MEMBER(min_linear_vel);
    READ_CONFIG_MEMBER(max_break_acc);

    READ_CONFIG_MEMBER(wheels_distance);
    READ_CONFIG_MEMBER(wheel_perimeter);
    READ_CONFIG_MEMBER(main_brush_length);
    READ_CONFIG_MEMBER(footprint_resolution);
    READ_CONFIG_MEMBER(footprint_description);
    READ_CONFIG_MEMBER(side_sensor_description);

    auto &ssd = cfg->side_sensor_description;
    for (int32_t i = 0; i < ssd.rows; ++i)
    {
        RobotPose side_sensor_pose{
            ssd.at<float>(i, 0),
            ssd.at<float>(i, 1),
            deg2rad(ssd.at<float>(i, 2))
        };
        cfg->side_sensor_poses.emplace_back(side_sensor_pose);
    }

END_CONFIG_READ_FUNC(Robot)

extern ConfigRobot *g_robot_cfg;

#endif // MISC_ROBOT_CONFIG_H