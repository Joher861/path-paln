#ifndef LOG_LOG_FUNC_H
#define LOG_LOG_FUNC_H

#include <typeindex>
#include <string>
#include <cstdarg>
#include <cstdio>

#include "log_handler.h"

namespace planning_utils
{
    enum LogTSMask        // log的时间戳配置掩码
    {
        READABLE_MS = 1,    // 可读的时间戳（单位ms）
        READABLE_US = 2,    // 可读的时间戳（单位us）
        RAW_MS = 4,         // 原始时间戳（1970-01-01开始，单位ms）
        RAW_US = 8,         // 原始时间戳（1970-01-01开始，单位us）
        SYSTEM_MS = 16,     // 系统上点时间戳（单位ms）
        SYSTEM_US = 32,     // 系统上点时间戳（单位us）
    };
    
    std::vector<std::string> getLogTimestamps(uint8_t ts_mask,
        bool use_use_fast_readable_ts = true, int32_t timezone_offset = 8);
    uint8_t getLogTimestampMask(bool use_readable_ms, bool use_readable_us,
        bool use_raw_ms, bool use_raw_us, bool use_system_ms, bool use_system_us);

    // struct LogRunTimeInfo
    // {
    //     int32_t line;
    //     std::string file;
    // };

    /******************************* FUNC PARAM *******************************/
    
    #define DEFINE_FUNC_PARAM(ParamType, BaseType)    \
        struct ParamType##FuncParam;    \
        using ParamType##FuncParamPtr    \
            = std::shared_ptr<ParamType##FuncParam>;    \
        struct ParamType##FuncParam : BaseType##FuncParam \
        {   \
          public:
    #define END_FUNC_PARAM(ParamType)    \
        };

    #define CREATE_FUNC_PARAM(ParamType, ...)    \
        std::dynamic_pointer_cast<FuncParam>( \
            std::make_shared<ParamType##FuncParam>(__VA_ARGS__))

    struct FuncParam;
    using FuncParamPtr = std::shared_ptr<FuncParam>;
    struct FuncParam
    {
        FuncParam(uint8_t _ts_mask, bool _print_to_console)
            : ts_mask(_ts_mask), print_to_console(_print_to_console)
        {}
        virtual ~FuncParam() {}

        virtual bool isMatch(FuncParamPtr param) = 0;

        uint8_t ts_mask;
        bool print_to_console;
    };

    /******************************* FUNC PARAM *******************************/

    #define DEFINE_LOG_FUNC(FuncType, BaseType)    \
        struct FuncType##LogFunc;   \
        using FuncType##LogFuncPtr    \
            = std::shared_ptr<FuncType##LogFunc>;   \
        struct FuncType##LogFunc : BaseType##LogFunc \
        {   \
          protected:   \
            using BaseType##LogFunc::BaseType##LogFunc; \
          public:   \
            static FuncType##LogFuncPtr createLogFunc() \
            {   \
                return std::make_shared<FuncType##LogFunc>(); \
            }
    #define END_LOG_FUNC(FuncType)    \
        };

    #define CREATE_LOG_FUNC(FuncType)   \
        FuncType##LogFunc::createLogFunc()
    /* #define CREATE_LOG_FUNC(FuncType, ...)    \
        std::dynamic_pointer_cast<LogFunc>( \
            std::make_shared<FuncType##LogFunc>( \
                new FuncType##LogFunc {##__VA_ARGS__})) */

    struct LogFunc;
    using LogFuncPtr = std::shared_ptr<LogFunc>;
    struct LogFunc
    {
      public:

        virtual ~LogFunc() {}
        virtual void setParam(bool use_fast_readable_ts, int32_t timezone_offset);
        virtual void log(LogHandlerPtr handler, FuncParamPtr func_param,
            va_list args) = 0;
        virtual std::type_index getHandlerType() = 0;
    
      protected:

        bool m_use_fast_readable_ts         = true;
        int32_t m_timezone_offset           = 8;
    };
}

#endif // LOG_LOG_FUNC_H