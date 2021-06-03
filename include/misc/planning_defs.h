#ifndef MISC_PLANNING_DEF_H
#define MISC_PLANNING_DEF_H

#define FLOAT_EPS           0.0001f
#define ABNORMAL_ANGLE       1000000.0f

/**
 * @brief  string拼接的宏
 * @param  X, Y分别为两个拼接对象
 * @retval None
 */
#define STR_CONN(X, Y)  #X#Y

/**
 * @brief  string拼接的宏
 * @param  X, Y, Z分别为三个拼接对象
 * @retval None
 */
#define STR_CONN3(X, Y, Z)  #X#Y#Z

/**
 * @brief  比较某一std::type_index对象是否为类型T
 * @param  type_idx 一个std::type_index对象
 *         T        某一事件类型
 * @retval 为同一类型返回true，否则返回false
 */
#define TYPE_EQUALS(type_idx, T)    (type_idx == typeid(T))

#define TRIG_SEGFAULT()                                                        \
    int* segv_ptr = nullptr;                                                   \
    int segv = *segv_ptr;


// *********************************** log *************************************

#define LOG_AK_PLANNING_FLAG     planning
#define LOG_AK_PLANNING          "planning"
#define PLANNING_DEBUG_LOG(fmt, ...)                                           \
    LOG(LOG_NAME(LOG_AK_PLANNING), __LINE__, __FILE__, LEVEL_DEBUG,            \
        fmt, ##__VA_ARGS__)
#define PLANNING_INFO_LOG(fmt, ...)                                            \
    LOG(LOG_NAME(LOG_AK_PLANNING), __LINE__, __FILE__, LEVEL_INFO,             \
        fmt, ##__VA_ARGS__)
#define PLANNING_WARN_LOG(fmt, ...)                                            \
    LOG(LOG_NAME(LOG_AK_PLANNING), __LINE__, __FILE__, LEVEL_WARN,             \
        fmt, ##__VA_ARGS__)
#define PLANNING_ERROR_LOG(fmt, ...)                                           \
    LOG(LOG_NAME(LOG_AK_PLANNING), __LINE__, __FILE__, LEVEL_ERROR,            \
        fmt, ##__VA_ARGS__)

#endif // MISC_PLANNING_DEF_H
