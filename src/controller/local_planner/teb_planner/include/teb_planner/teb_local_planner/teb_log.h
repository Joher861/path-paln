#ifndef LOG_PMS_H
#define LOG_PMS_H

#include <iostream>
#include "log/log_manager.h"

namespace planning_controller
{
// #define TEB_NDEBUG
#define LOG_TEB_PLANNER_FLAG     teb_planner
#define LOG_TEB_PLANNER          "teb_planner"
//TODO:后面替换成pms系统里的log接口
#ifndef TEB_NDEBUG


#define TEB_DEBUG_LOG(fmt, ...)                                           \
    LOG(LOG_NAMES(LOG_TEB_PLANNER), __LINE__, __FILE__, planning_utils::LEVEL_DEBUG,           \
        fmt, ##__VA_ARGS__)
#define TEB_INFO_LOG(fmt, ...)                                            \
    LOG(LOG_NAMES(LOG_TEB_PLANNER), __LINE__, __FILE__, planning_utils::LEVEL_INFO,            \
        fmt, ##__VA_ARGS__)
#define TEB_WARN_LOG(fmt, ...)                                            \
    LOG(LOG_NAMES(LOG_TEB_PLANNER), __LINE__, __FILE__, planning_utils::LEVEL_WARN,            \
        fmt, ##__VA_ARGS__)
#define TEB_ERROR_LOG(fmt, ...)                                           \
    LOG(LOG_NAMES(LOG_TEB_PLANNER), __LINE__, __FILE__, planning_utils::LEVEL_ERROR,           \
        fmt, ##__VA_ARGS__)
#define TEB_FATAL_LOG(fmt, ...)                                           \
    LOG(LOG_NAMES(LOG_TEB_PLANNER), __LINE__, __FILE__, planning_utils::LEVEL_ERROR,           \
        fmt, ##__VA_ARGS__)


// #define TEB_INFO_LOG(...)       printf(__VA_ARGS__)
// #define TEB_DEBUG_LOG(...)      printf(__VA_ARGS__)
// #define TEB_WARN_LOG(...)       printf(__VA_ARGS__)
// #define TEB_ERROR_LOG(...)      printf(__VA_ARGS__)
// #define TEB_FATAL_LOG(...)      printf(__VA_ARGS__)
#else

#define TEB_INFO_LOG(...)       
#define TEB_DEBUG_LOG(...)       
#define TEB_WARN_LOG(...)       
#define TEB_ERROR_LOG(...)      
#define TEB_FATAL_LOG(...)      

#endif






// #ifdef WIN32
// # if defined (__MINGW32__)
// #  define ROS_ISSUE_BREAK() DebugBreak();
// # else // MSVC
// #  define ROS_ISSUE_BREAK() __debugbreak();
// # endif
// #elif defined(__powerpc64__)
// # define ROS_ISSUE_BREAK() asm volatile ("tw 31,1,1");
// #elif defined(__i386__) || defined(__ia64__) || defined(__x86_64__)
// # define ROS_ISSUE_BREAK() asm("int $3");
// #else
# include <stdlib.h>
# define ROS_ISSUE_BREAK() abort();
// #endif

#ifndef TEB_NDEBUG
#ifndef ROS_ASSERT_ENABLED
#define ROS_ASSERT_ENABLED
#endif
#endif

#ifdef ROS_ASSERT_ENABLED
#define ROS_BREAK() \
  do { \
    TEB_FATAL_LOG("BREAKPOINT HIT\n\tfile = %s\n\tline=%d\n", __FILE__, __LINE__); \
    ROS_ISSUE_BREAK() \
  } while (false)

#define ROS_ASSERT(cond) \
  do { \
    if (!(cond)) { \
      TEB_FATAL_LOG("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
      ROS_ISSUE_BREAK() \
    } \
  } while (false)

#define ROS_ASSERT_MSG(cond, ...) \
  do { \
    if (!(cond)) { \
      TEB_FATAL_LOG("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n\tmessage = ", __FILE__, __LINE__, #cond); \
      TEB_FATAL_LOG(__VA_ARGS__); \
      TEB_FATAL_LOG("\n"); \
      ROS_ISSUE_BREAK(); \
    } \
  } while (false)

#define ROS_ASSERT_CMD(cond, cmd) \
  do { \
    if (!(cond)) { \
      cmd; \
    } \
  } while (false)


#else
#define ROS_BREAK()
#define ROS_ASSERT(cond)
#define ROS_ASSERT_MSG(cond, ...)
#define ROS_ASSERT_CMD(cond, cmd)
#endif




}





#endif