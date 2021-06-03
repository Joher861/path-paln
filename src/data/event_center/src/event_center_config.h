#ifndef DATA_EVENT_CENTER_H
#define DATA_EVENT_CENTER_H

#include <string>

#include "config/config.h"

#define CONFIG_EVENT_CENTER    "cfg_event_center"

DEFINE_CONFIG(EventCenter)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    int event_timeout;
END_CONFIG(EventCenter)

DEFINE_CONFIG_READ_FUNC(EventCenter)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(event_timeout);
END_CONFIG_READ_FUNC(EventCenter)

#endif // DATA_EVENT_CENTER_H