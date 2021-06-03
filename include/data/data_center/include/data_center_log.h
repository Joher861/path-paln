#ifndef DATA_DATA_CENTER_LOG_H
#define DATA_DATA_CENTER_LOG_H

#include "log/log_manager.h"

#define LOG_DATA_CENTER_FLAG     data_center
#define LOG_DATA_CENTER          "data_center"
#define DATA_DEBUG_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_DATA_CENTER), __LINE__, __FILE__,                         \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define DATA_INFO_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_DATA_CENTER), __LINE__, __FILE__,                         \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define DATA_WARN_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_DATA_CENTER), __LINE__, __FILE__,                         \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define DATA_ERROR_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_DATA_CENTER), __LINE__, __FILE__,                         \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif // DATA_DATA_CENTER_LOG_H