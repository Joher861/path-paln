#ifndef PLANNER_CCPP_PLANNER_CONFIG_H
#define PLANNER_CCPP_PLANNER_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_CCPP_COVERAGE_PLANNER    "cfg_ccpp_planner"

DEFINE_CONFIG(CCPPCoveragePlanner)
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
        
    std::vector<double> start_pos;
    
    double resolution;
    
    double robot_radius;
    
    double grid_spacing_in_meter;
    
    double coverage_radius;
    
    float min_line_cleaning_distance;
    
    float complementary_path_distance;
    
    float need_hybrid_distance;
    
    float distance_to_boundary;
    
    double min_cell_area_;
END_CONFIG(CCPPCoveragePlanner)

DEFINE_CONFIG_READ_FUNC(CCPPCoveragePlanner)
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
    READ_CONFIG_MEMBER(complementary_path_distance);
    READ_CONFIG_MEMBER(coverage_radius);
    READ_CONFIG_MEMBER(distance_to_boundary);
    READ_CONFIG_MEMBER(grid_spacing_in_meter);
    READ_CONFIG_MEMBER(min_cell_area_);
    READ_CONFIG_MEMBER(min_line_cleaning_distance);
    READ_CONFIG_MEMBER(need_hybrid_distance);
    READ_CONFIG_MEMBER(resolution);
    READ_CONFIG_MEMBER(robot_radius);

END_CONFIG_READ_FUNC(CCPPCoveragePlanner)

#endif