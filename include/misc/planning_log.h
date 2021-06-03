#ifndef MISC_PLANNING_LOG_H
#define MISC_PLANNING_LOG_H

#include "log/log_manager.h"

#define LOG_PLANNING_FLAG     planning
#define LOG_PLANNING          "planning"
#define PLAN_DEBUG_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_PLANNING), __LINE__, __FILE__,                          \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define PLAN_INFO_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_PLANNING), __LINE__, __FILE__,                          \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define PLAN_WARN_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_PLANNING), __LINE__, __FILE__,                          \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define PLAN_ERROR_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_PLANNING), __LINE__, __FILE__,                          \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif // MISC_PLANNING_LOG_H