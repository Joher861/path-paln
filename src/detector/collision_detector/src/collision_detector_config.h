#ifndef DETECTOR_COLLISION_DETECTOR_CONFIG_H
#define DETECTOR_COLLISION_DETECTOR_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_COLLISION_DETECTOR    "cfg_collision_detector"

DEFINE_CONFIG(CollisionDetector)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    float obstacle_radius;
    float inflate_longitude_dist_small;
    float inflate_lateral_dist_small;
    float inflate_longitude_dist_large;
    float inflate_lateral_dist_large;
    int max_det_path_points_num_default;
    float predict_horizon;
    float polling_frequency;
END_CONFIG(CollisionDetector)

DEFINE_CONFIG_READ_FUNC(CollisionDetector)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(obstacle_radius);
    READ_CONFIG_MEMBER(inflate_longitude_dist_small);
    READ_CONFIG_MEMBER(inflate_lateral_dist_small);
    READ_CONFIG_MEMBER(inflate_longitude_dist_large);
    READ_CONFIG_MEMBER(inflate_lateral_dist_large);
    READ_CONFIG_MEMBER(max_det_path_points_num_default);
    READ_CONFIG_MEMBER(predict_horizon);
    READ_CONFIG_MEMBER(polling_frequency);
END_CONFIG_READ_FUNC(CollisionDetector)

#endif // DETECTOR_COLLISION_DETECTOR_CONFIG_H