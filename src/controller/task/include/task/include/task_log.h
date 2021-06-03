#ifndef CONTROLLER_TASK_LOG_H
#define CONTROLLER_TASK_LOG_H

#include "log/log_manager.h"

#define LOG_TASK_FLAG task
#define LOG_TASK      "task"
#define TASK_DEBUG_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_TASK), __LINE__, __FILE__,                                \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define TASK_INFO_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_TASK), __LINE__, __FILE__,                                \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define TASK_WARN_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_TASK), __LINE__, __FILE__,                                \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define TASK_ERROR_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_TASK), __LINE__, __FILE__,                                \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#endif