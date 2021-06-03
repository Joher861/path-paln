#ifndef CONTROLLER_EVENT_CENTER_LOG_H
#define CONTROLLER_EVENT_CENTER_LOG_H

#include "log/log_manager.h"

#define LOG_EVENT_CENTER_FLAG     event_center
#define LOG_EVENT_CENTER          "event_center"
#define EVENT_DEBUG_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_EVENT_CENTER), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_DEBUG,fmt, ##__VA_ARGS__)
#define EVENT_INFO_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_EVENT_CENTER), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define EVENT_WARN_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_EVENT_CENTER), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define EVENT_ERROR_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_EVENT_CENTER), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // CONTROLLER_EVENT_CENTER_LOG_H