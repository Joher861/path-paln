
#ifndef CONTROLLER_MPC_PLANNER_LOG_H
#define CONTROLLER_MPC_PLANNER_LOG_H

#include "log/log_manager.h"

#define LOG_MPC_PLANNER_FLAG     mpc_planner
#define LOG_MPC_PLANNER          "mpc_planner"
#define MPC_DEBUG_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_MPC_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define MPC_INFO_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_MPC_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_INFO, fmt, ##__VA_ARGS__)
#define MPC_WARN_LOG(fmt, ...)                                              \
    LOG(LOG_NAME(LOG_MPC_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_WARN, fmt, ##__VA_ARGS__)
#define MPC_ERROR_LOG(fmt, ...)                                             \
    LOG(LOG_NAME(LOG_MPC_PLANNER), __LINE__, __FILE__,                      \
        planning_utils::LEVEL_ERROR, fmt, ##__VA_ARGS__)


#endif // CONTROLLER_MPC_PLANNER_LOG_H