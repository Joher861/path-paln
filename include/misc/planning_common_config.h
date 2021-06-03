#ifndef MISC_PLANNING_COMMON_CONFIG_H
#define MISC_PLANNING_COMMON_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_PLANNING    "cfg_planning"

DEFINE_CONFIG(Planing)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size;
    int log_max_file_cnt;
    int log_level;
END_CONFIG(Planing)

DEFINE_CONFIG_READ_FUNC(Planing)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
END_CONFIG_READ_FUNC(Planing)

#endif // MISC_PLANNING_COMMON_CONFIG_H