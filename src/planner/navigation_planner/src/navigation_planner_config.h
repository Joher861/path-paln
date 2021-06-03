#ifndef PLANNER__NAVIGATION_PLANNER_CONFIG_H
#define PLANNER__NAVIGATION_PLANNER_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_NAVIGATION_PLANNER    "cfg_navigation_planner"

DEFINE_CONFIG(NavigationPlanner)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    float planning_frequency;
END_CONFIG(NavigationPlanner)

DEFINE_CONFIG_READ_FUNC(NavigationPlanner)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(planning_frequency);
END_CONFIG_READ_FUNC(NavigationPlanner)

#endif // CONTROLLER_path_follow_PLANNER_CONFIG_H