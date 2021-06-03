#ifndef MAP_LOCAL_MAP_LOG_H
#define MAP_LOCAL_MAP_LOG_H

#include "log/log_manager.h"

#define LOG_LOCAL_MAP_FLAG     local_map
#define LOG_LOCAL_MAP          "local_map"
#define LMAP_DEBUG_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_LOCAL_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LMAP_INFO_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_LOCAL_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define LMAP_WARN_LOG(fmt, ...)                                                \
    LOG(LOG_NAME(LOG_LOCAL_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define LMAP_ERROR_LOG(fmt, ...)                                               \
    LOG(LOG_NAME(LOG_LOCAL_MAP), __LINE__, __FILE__,                           \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#define LOG_VIRTUAL_BUMP_FLAG     virtual_bump
#define LOG_VIRTUAL_BUMP          "virtual_bump"
#define VB_DEBUG_LOG(fmt, ...)                                                 \
    LOG(LOG_NAME(LOG_VIRTUAL_BUMP), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define VB_INFO_LOG(fmt, ...)                                                  \
    LOG(LOG_NAME(LOG_VIRTUAL_BUMP), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define VB_WARN_LOG(fmt, ...)                                                  \
    LOG(LOG_NAME(LOG_VIRTUAL_BUMP), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define VB_ERROR_LOG(fmt, ...)                                                 \
    LOG(LOG_NAME(LOG_VIRTUAL_BUMP), __LINE__, __FILE__,                        \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

#define LOG_OB_CONTOUR_FLAG     obstacle_contour
#define LOG_OB_CONTOUR          "obstacle_contour"
#define OB_CONTOUR_DEBUG_LOG(fmt, ...) \
    LOG(LOG_NAME(LOG_OB_CONTOUR), __LINE__, __FILE__, \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define OB_CONTOUR_INFO_LOG(fmt, ...) \
    LOG(LOG_NAME(LOG_OB_CONTOUR), __LINE__, __FILE__, \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define OB_CONTOUR_WARN_LOG(fmt, ...) \
    LOG(LOG_NAME(LOG_OB_CONTOUR), __LINE__, __FILE__, \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define OB_CONTOUR_ERROR_LOG(fmt, ...) \
    LOG(LOG_NAME(LOG_OB_CONTOUR), __LINE__, __FILE__, \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)

// log map img
#define LOG_LOCAL_MAP_IMG_FLAG local_map_img
#define LOG_LOCAL_MAP_IMG      "local_map_img"

#define LMAP_IMG_LOG(img)                                                      \
    LOG(LOG_NAME(LOG_LOCAL_MAP_IMG), img)

// obstacle contour img
#define LOG_OBSTACLE_CONTOUR_IMG_FLAG   obstacle_contour_img
#define LOG_OBSTACLE_CONTOUR_IMG        "obstacle_contour_img"

#define OB_CONTOUR_IMG_LOG(img)                                                    \
    LOG(LOG_NAME(LOG_OBSTACLE_CONTOUR_IMG), img)

// path mask img
#define LOG_PATH_MASK_IMG_FLAG   path_mask_img
#define LOG_PATH_MASK_IMG        "path_mask_img"

#define PATH_MASK_LOG(img)                                                    \
    LOG(LOG_NAME(LOG_PATH_MASK_IMG), img)

#endif // MAP_LOCAL_MAP_LOG_H