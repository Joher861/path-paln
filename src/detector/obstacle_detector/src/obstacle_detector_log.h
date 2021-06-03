#ifndef DETECTOR_OBSTACLE_DETECTOR_LOG_H
#define DETECTOR_OBSTACLE_DETECTOR_LOG_H

#include "log/log_manager.h"

#define LOG_OBSTACLE_DETECTOR_FLAG     obstacle_detector
#define LOG_OBSTACLE_DETECTOR         "obstacle_detector"
#define OBSTACLE_DET_DEBUG_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_OBSTACLE_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define OBSTACLE_DET_INFO_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_OBSTACLE_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define OBSTACLE_DET_WARN_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_OBSTACLE_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define OBSTACLE_DET_ERROR_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_OBSTACLE_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // DETECTOR_OBSTACLE_DETECTOR_LOG_H