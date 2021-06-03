#ifndef CONTROLLER_SPEED_CONTROLLER_LOG_H
#define CONTROLLER_SPEED_CONTROLLER_LOG_H

#include "log/log_manager.h"

#define LOG_SPEED_CONTROLLER_FLAG speed_controller
#define LOG_SPEED_CONTROLLER      "speed_controller"
#define SPEED_CONTROLLER_DEBUG_LOG(fmt, ...)                                   \
    LOG(LOG_NAME(LOG_SPEED_CONTROLLER), __LINE__, __FILE__,                    \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define SPEED_CONTROLLER_INFO_LOG(fmt, ...)                                    \
    LOG(LOG_NAME(LOG_SPEED_CONTROLLER), __LINE__, __FILE__,                    \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define SPEED_CONTROLLER_WARN_LOG(fmt, ...)                                    \
    LOG(LOG_NAME(LOG_SPEED_CONTROLLER), __LINE__, __FILE__,                    \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define SPEED_CONTROLLER_ERROR_LOG(fmt, ...)                                   \
    LOG(LOG_NAME(LOG_SPEED_CONTROLLER), __LINE__, __FILE__,                    \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif