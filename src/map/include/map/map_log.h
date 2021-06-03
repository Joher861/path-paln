#ifndef MAP_MAP_LOG_H
#define MAP_MAP_LOG_H

#include "log/log_manager.h"

using planning_utils::LEVEL_DEBUG;
using planning_utils::LEVEL_ERROR;
using planning_utils::LEVEL_INFO;
using planning_utils::LEVEL_WARN;

#define MAP_DEBUG_LOG(name, fmt, ...)                                        \
    LOG(LOG_NAMES(name), __LINE__, __FILE__, planning_utils::LEVEL_DEBUG, \
        fmt, ##__VA_ARGS__)
#define MAP_INFO_LOG(name, fmt, ...)                                        \
    LOG(LOG_NAMES(name), __LINE__, __FILE__, planning_utils::LEVEL_INFO, \
        fmt, ##__VA_ARGS__)
#define MAP_WARN_LOG(name, fmt, ...)                                        \
    LOG(LOG_NAMES(name), __LINE__, __FILE__, planning_utils::LEVEL_WARN, \
        fmt, ##__VA_ARGS__)
#define MAP_ERROR_LOG(name, fmt, ...)                                        \
    LOG(LOG_NAMES(name), __LINE__, __FILE__, planning_utils::LEVEL_ERROR, \
        fmt, ##__VA_ARGS__)

#endif // MAP_MAP_LOG_H
