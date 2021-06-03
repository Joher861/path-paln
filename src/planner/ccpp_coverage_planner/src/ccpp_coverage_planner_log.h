#ifndef PLANNER_CCPP_PLANNER_LOG_H
#define PLANNER_CCPP_PLANNER_LOG_H

#include "log/log_manager.h"

#define LOG_CCPP_PLANNER_FLAG navigation_planner
#define LOG_CCPP_PLANNER "  ccpp_planner"
#define CCPP_DEBUG_LOG(fmt, ...)                        \
    LOG(LOG_NAME(LOG_CCPP_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define CCPP_INFO_LOG(fmt, ...)                          \
    LOG(LOG_NAME(LOG_CCPP_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define CCPP_WARN_LOG(fmt, ...)                             \
    LOG(LOG_NAME(LOG_CCPP_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define CCPP_ERROR_LOG(fmt, ...)                        \
    LOG(LOG_NAME(LOG_CCPP_PLANNER), __LINE__, __FILE__, \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif