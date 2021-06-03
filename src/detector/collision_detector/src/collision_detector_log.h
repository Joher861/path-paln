#ifndef DETECTOR_COLLISION_DETECTOR_LOG_H
#define DETECTOR_COLLISION_DETECTOR_LOG_H

#include "log/log_manager.h"

#define LOG_COLLISION_DETECTOR_FLAG     collision_detector
#define LOG_COLLISION_DETECTOR         "collision_detector"
#define COLLISION_DET_DEBUG_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_COLLISION_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define COLLISION_DET_INFO_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_COLLISION_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define COLLISION_DET_WARN_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_COLLISION_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define COLLISION_DET_ERROR_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_COLLISION_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // DETECTOR_COLLISION_DETECTOR_LOG_H