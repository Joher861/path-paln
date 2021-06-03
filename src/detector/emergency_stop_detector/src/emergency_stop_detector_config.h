#ifndef DETECTOR_EMERGENCY_STOP_DETECTOR_CONFIG_H
#define DETECTOR_EMERGENCY_STOP_DETECTOR_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_EMERGENCY_STOP_DETECTOR    "cfg_emergency_stop_detector"

DEFINE_CONFIG(EmergencyStopDetector)
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
    float jumped_dist_threshold;
    float threshold_ultrasonic_front_default;
    float threshold_ultrasonic_front_left_default;
    float threshold_ultrasonic_front_right_default;
    float threshold_ultrasonic_left_down_1;
    float threshold_ultrasonic_left_down_2;
    float threshold_ultrasonic_right_down_1;
    float threshold_ultrasonic_right_down_2;
    int print_us;
    float polling_frequency;
END_CONFIG(EmergencyStopDetector)

DEFINE_CONFIG_READ_FUNC(EmergencyStopDetector)
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
    READ_CONFIG_MEMBER(jumped_dist_threshold);
    READ_CONFIG_MEMBER(threshold_ultrasonic_front_default);
    READ_CONFIG_MEMBER(threshold_ultrasonic_front_left_default);
    READ_CONFIG_MEMBER(threshold_ultrasonic_front_right_default);
    READ_CONFIG_MEMBER(threshold_ultrasonic_left_down_1);
    READ_CONFIG_MEMBER(threshold_ultrasonic_left_down_2);
    READ_CONFIG_MEMBER(threshold_ultrasonic_right_down_1);
    READ_CONFIG_MEMBER(threshold_ultrasonic_right_down_2);
    READ_CONFIG_MEMBER(print_us);
    READ_CONFIG_MEMBER(polling_frequency);
END_CONFIG_READ_FUNC(EmergencyStopDetector)

#endif // DETECTOR_EMERGENCY_STOP_DETECTOR_CONFIG_H