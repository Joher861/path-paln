#ifndef LOG_LOG_CONFIG_H
#define LOG_LOG_CONFIG_H

#include "config/config.h"

#define CONFIG_LOG    "cfg_log"

DEFINE_CONFIG(Log)
    bool enable_log;
    bool use_fast_readable_ts;
    int32_t timezone_offset;
    bool disable_sync_with_stdio;
    bool untie_cin_cout;
END_CONFIG(Log)

DEFINE_CONFIG_READ_FUNC(Log)
    READ_CONFIG_MEMBER(enable_log);
    READ_CONFIG_MEMBER(use_fast_readable_ts);
    READ_CONFIG_MEMBER(timezone_offset);
    READ_CONFIG_MEMBER(disable_sync_with_stdio);
    READ_CONFIG_MEMBER(untie_cin_cout);
END_CONFIG_READ_FUNC(Log)

#endif // LOG_LOG_CONFIG_H