#ifndef DETECTOR_EMERGENCY_STOP_DETECTOR_LOG_H
#define DETECTOR_EMERGENCY_STOP_DETECTOR_LOG_H

#include "log/log_manager.h"

#define LOG_EMERGENCY_STOP_DETECTOR_FLAG     emergency_stop_detector
#define LOG_EMERGENCY_STOP_DETECTOR        "emergency_stop_detector"
#define EMERGENCY_STOP_DET_DEBUG_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_EMERGENCY_STOP_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define EMERGENCY_STOP_DET_INFO_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_EMERGENCY_STOP_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define EMERGENCY_STOP_DET_WARN_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_EMERGENCY_STOP_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define EMERGENCY_STOP_DET_ERROR_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_EMERGENCY_STOP_DETECTOR), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // DETECTOR_EMERGENCY_STOP_DETECTOR_LOG_H