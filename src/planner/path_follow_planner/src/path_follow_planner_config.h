#ifndef CONTROLLER_PATH_FOLLOW_PLANNER_CONFIG_H
#define CONTROLLER_PATH_FOLLOW_PLANNER_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_PATH_FOLLOW_PLANNER    "cfg_path_follow_planner"

DEFINE_CONFIG(PathFollowPlanner)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    float interval_dense;
    float interval_sparse;
    int ev_deduction_period_for_us;
    int set_oa_off;
    int set_collis_lasting_off;
    float collision_lasting_threshold;
    float delta_rotate_angle;
    float max_velocity;
    float min_velocity;
    float max_curvature;
    float min_curvature;
    int const_velocity;
    float test_velocity;
    float planning_frequency;
END_CONFIG(PathFollowPlanner)

DEFINE_CONFIG_READ_FUNC(PathFollowPlanner)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(interval_dense);
    READ_CONFIG_MEMBER(interval_sparse);
    READ_CONFIG_MEMBER(ev_deduction_period_for_us);
    READ_CONFIG_MEMBER(set_oa_off);
    READ_CONFIG_MEMBER(set_collis_lasting_off);
    READ_CONFIG_MEMBER(collision_lasting_threshold);
    READ_CONFIG_MEMBER(delta_rotate_angle);
    READ_CONFIG_MEMBER(max_velocity);
    READ_CONFIG_MEMBER(min_velocity);
    READ_CONFIG_MEMBER(max_curvature);
    READ_CONFIG_MEMBER(min_curvature);
    READ_CONFIG_MEMBER(const_velocity);
    READ_CONFIG_MEMBER(test_velocity);
    READ_CONFIG_MEMBER(planning_frequency);
END_CONFIG_READ_FUNC(PathFollowPlanner)

#endif // CONTROLLER_PATH_FOLLOW_PLANNER_CONFIG_H