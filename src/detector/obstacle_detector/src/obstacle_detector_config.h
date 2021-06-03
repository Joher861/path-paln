#ifndef DETECTOR_OBSTACLE_DETECTOR_CONFIG_H
#define DETECTOR_OBSTACLE_DETECTOR_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_OBSTACLE_DETECTOR    "cfg_obstacle_detector"

DEFINE_CONFIG(ObstacleDetector)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    float lasting_period;
    float reset_lasting_period;
    float vehicle_lateral_inflate_size;
    float forward_safe_distance_default;
    float front_side_safe_distance_default;
    float side_safe_distance;
    float left_safe_dis_bias;
    float right_safe_dis_bias;
    float grid_of_interest_radius_default;
    float grid_of_far_circle_radius_default;
    float grid_of_near_circle_radius;
    float polling_frequency;
END_CONFIG(ObstacleDetector)

DEFINE_CONFIG_READ_FUNC(ObstacleDetector)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(lasting_period);
    READ_CONFIG_MEMBER(reset_lasting_period);
    READ_CONFIG_MEMBER(vehicle_lateral_inflate_size);
    READ_CONFIG_MEMBER(forward_safe_distance_default);
    READ_CONFIG_MEMBER(front_side_safe_distance_default);
    READ_CONFIG_MEMBER(side_safe_distance);
    READ_CONFIG_MEMBER(left_safe_dis_bias);
    READ_CONFIG_MEMBER(right_safe_dis_bias);
    READ_CONFIG_MEMBER(grid_of_interest_radius_default);
    READ_CONFIG_MEMBER(grid_of_far_circle_radius_default);
    READ_CONFIG_MEMBER(grid_of_near_circle_radius);
    READ_CONFIG_MEMBER(polling_frequency);
END_CONFIG_READ_FUNC(ObstacleDetector)

#endif // DETECTOR_OBSTACLE_DETECTOR_CONFIG_H