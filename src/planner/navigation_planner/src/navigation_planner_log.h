#ifndef CONTROLLER_NAVIGATION_PLANNER_LOG_H
#define CONTROLLER_NAVIGATION_PLANNER_LOG_H

#include "log/log_manager.h"

#define LOG_NAVIGATION_PLANNER_FLAG navigation_planner
#define LOG_NAVIGATION_PLANNER "navigation_planner"
#define NAVIGATION_DEBUG_LOG(fmt, ...)                        \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define NAVIGATION_INFO_LOG(fmt, ...)                          \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define NAVIGATION_WARN_LOG(fmt, ...)                             \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define NAVIGATION_ERROR_LOG(fmt, ...)                        \
    LOG(LOG_NAME(LOG_NAVIGATION_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif // CONTROLLER_NAVIGATION_PLANNER_LOG_H