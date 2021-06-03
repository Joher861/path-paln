#include "sleep_timer.h"
#include <thread>

using namespace planning_utils;

SleepTimer::SleepTimer(float frequency, uint64_t min_sleep_time)
    : m_loop_ts_us(1e6 / frequency),
      m_min_sleep_time(min_sleep_time)
{
    m_last_ts = Timer::getSystemTimestampUS();
}

uint64_t SleepTimer::sleep()
{
    uint64_t cur_ts = Timer::getSystemTimestampUS();
    uint64_t ts_diff = cur_ts - m_last_ts;
    if (ts_diff >= m_loop_ts_us)
    {
        std::this_thread::sleep_for(
            std::chrono::microseconds(m_min_sleep_time));
        m_last_ts = Timer::getSystemTimestampUS();
        return m_last_ts;
    }

    std::this_thread::sleep_for(
        std::chrono::microseconds(m_loop_ts_us - ts_diff));
    m_last_ts = Timer::getSystemTimestampUS();
    return m_last_ts;
}