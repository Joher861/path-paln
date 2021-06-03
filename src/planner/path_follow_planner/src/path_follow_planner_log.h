#ifndef PLANNER_PATHFOLLOW_PLANNER_LOG_H
#define PLANNER_PATHFOLLOW_PLANNER_LOG_H

#include "log/log_manager.h"

#define LOG_PATHFOLLOW_PLANNER_FLAG     pathfollow_planner
#define LOG_PATHFOLLOW_PLANNER          "pathfollow_planner"
#define PATHFOLLOW_PLANNER_DEBUG_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_PATHFOLLOW_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define PATHFOLLOW_PLANNER_INFO_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_PATHFOLLOW_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define PATHFOLLOW_PLANNER_WARN_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_PATHFOLLOW_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define PATHFOLLOW_PLANNER_ERROR_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_PATHFOLLOW_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // CONTROLLER_PATHFOLLOW_PLANNER_LOG_H