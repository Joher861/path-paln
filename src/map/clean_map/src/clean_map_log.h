#ifndef MAP_CLEAN_MAP_LOG_H
#define MAP_CLEAN_MAP_LOG_H

#include "log/log_manager.h"

#define LOG_CLEAN_MAP_FLAG     clean_map
#define LOG_CLEAN_MAP          "clean_map"
#define CMAP_DEBUG_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_CLEAN_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define CMAP_INFO_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_CLEAN_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define CMAP_WARN_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_CLEAN_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define CMAP_ERROR_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_CLEAN_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif // MAP_CLEAN_MAP_LOG_H