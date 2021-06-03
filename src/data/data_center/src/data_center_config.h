#ifndef DATA_DATA_CENTER_CONFIG_H
#define DATA_DATA_CENTER_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_DATA_CENTER    "cfg_data_center"

DEFINE_CONFIG(DataCenter)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
END_CONFIG(DataCenter)

DEFINE_CONFIG_READ_FUNC(DataCenter)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
END_CONFIG_READ_FUNC(DataCenter)

#endif // DATA_DATA_CENTER_CONFIG_H