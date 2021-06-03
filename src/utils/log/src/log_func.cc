#include "log_func.h"

#include "timer/timer.h"

namespace planning_utils
{
    std::vector<std::string> getLogTimestamps(uint8_t ts_mask,
        bool use_use_fast_readable_ts, int32_t timezone_offset)
    {
        std::vector<std::string> timestamps;

        if (!(ts_mask & 0x02) && (ts_mask & 0x01))
            timestamps.emplace_back(Timer::getReadableTimestampMS(
                use_use_fast_readable_ts, timezone_offset));
        else
            timestamps.emplace_back("");

        if (ts_mask & 0x02)
            timestamps.emplace_back(Timer::getReadableTimestampUS(
                use_use_fast_readable_ts, timezone_offset));
        else
            timestamps.emplace_back("");

        if (!(ts_mask & 0x08) && (ts_mask & 0x04))
            timestamps.emplace_back(std::to_string(Timer::getTimestampMS()));
        else
            timestamps.emplace_back("");

        if (ts_mask & 0x08)
            timestamps.emplace_back(std::to_string(Timer::getTimestampMS()));
        else
            timestamps.emplace_back("");

        if (!(ts_mask & 0x20) && (ts_mask & 0x10))
            timestamps.emplace_back(std::to_string(Timer::getSystemTimestampMS()));
        else
            timestamps.emplace_back("");

        if (ts_mask & 0x20)
            timestamps.emplace_back(std::to_string(Timer::getSystemTimestampUS()));
        else
            timestamps.emplace_back("");

        return timestamps;
    }

    uint8_t getLogTimestampMask(bool use_readable_ms, bool use_readable_us,
        bool use_raw_ms, bool use_raw_us, bool use_system_ms, bool use_system_us)
    {
        uint8_t ts_mask = 0;
        if (use_readable_ms)
            ts_mask |= READABLE_MS;
        if (use_readable_us)
            ts_mask |= READABLE_US;
        if (use_raw_ms)
            ts_mask |= RAW_MS;
        if (use_raw_us)
            ts_mask |= RAW_US;
        if (use_system_ms)
            ts_mask |= SYSTEM_MS;
        if (use_system_us)
            ts_mask |= SYSTEM_US;
        return ts_mask;
    }

    void LogFunc::setParam(bool use_fast_readable_ts, int32_t timezone_offset)
    {
        m_use_fast_readable_ts = use_fast_readable_ts;
        m_timezone_offset = timezone_offset;
    }
}