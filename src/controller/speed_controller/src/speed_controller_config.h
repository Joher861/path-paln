#ifndef CONTROLLER_SPEED_CONTROLLER_H
#define CONTROLLER_SPEED_CONTROLLER_H

#include <string>

#include "config/config.h"

#define CONFIG_SPEED_CONTROLLER    "cfg_speed_controller"

DEFINE_CONFIG(SpeedController)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    float max_angular_steer;     // unit: degree
    float min_angular_steer;     // unit: degree
    float rotate_vel;            // unit: m/s
    float steer_bias;            // deg
    float sending_speed_frequency;
END_CONFIG(SpeedController)

DEFINE_CONFIG_READ_FUNC(SpeedController)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(max_angular_steer);
    READ_CONFIG_MEMBER(min_angular_steer);
    READ_CONFIG_MEMBER(rotate_vel);
    READ_CONFIG_MEMBER(steer_bias);
    READ_CONFIG_MEMBER(sending_speed_frequency);
END_CONFIG_READ_FUNC(SpeedController)

#endif // CONTROLLER_SPEED_CONTROLLER_H