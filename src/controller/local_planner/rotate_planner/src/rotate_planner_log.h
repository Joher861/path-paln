#ifndef CONTROLLER_ROTATE_PLANNER_LOG_H
#define CONTROLLER_ROTATE_PLANNER_LOG_H

#include "log/log_manager.h"

#define LOG_ROTATE_PLANNER_FLAG     rotate_planner
#define LOG_ROTATE_PLANNER          "rotate_planner"
#define ROTATE_DEBUG_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_ROTATE_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define ROTATE_INFO_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_ROTATE_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define ROTATE_WARN_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_ROTATE_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define ROTATE_ERROR_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_ROTATE_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // CONTROLLER_ROTATE_PLANNER_LOG_H