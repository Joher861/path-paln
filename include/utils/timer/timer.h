#ifndef UTILS_TIMER_H
#define UTILS_TIMER_H

#include <string>
#include <cinttypes>
#include <ctime>
#include <sys/time.h>

namespace planning_utils
{
    class Timer
    {
      public:

        inline static std::string getReadableTimestampUS(bool fast = false,
            int zone_offset = 8)
        {
            char            fmt[64], buf[64];
            struct timeval  tv;
            struct tm       tm;

            gettimeofday(&tv, NULL);
            auto getTime = fast ? gmtime : localtime;
            if (auto res = getTime(&tv.tv_sec); res != nullptr)
            {
                tm = *res;
                if (fast)
                {
                    tm.tm_hour += zone_offset;
                    if (tm.tm_hour > 23)
                        tm.tm_hour -= 24;
                }
                strftime(fmt, sizeof fmt, "%H.%M.%S.%%06u", &tm);
                snprintf(buf, sizeof buf, fmt, tv.tv_usec);
                return std::string(buf);
            }
            else
            {	
                time_t rawtime;
                struct tm *info;
                char buffer[80];
                time( &rawtime );
                info = localtime( &rawtime );
                strftime(buffer, 80, "%H_%M_%S", info);
                return std::string(buffer);
            }
        }

        inline static std::string getReadableTimestampMS(bool fast = false,
            int zone_offset = 8)
        {
            char            fmt[64], buf[64];
            struct timeval  tv;
            struct tm       tm;

            gettimeofday(&tv, NULL);
            auto getTime = fast ? gmtime : localtime;
            if (auto res = getTime(&tv.tv_sec); res != nullptr)
            {
                tm = *res;
                if (fast)
                {
                    tm.tm_hour += zone_offset;
                    if (tm.tm_hour > 23)
                        tm.tm_hour -= 24;
                }
                strftime(fmt, sizeof fmt, "%H.%M.%S.%%03u", &tm);
                snprintf(buf, sizeof buf, fmt, tv.tv_usec / 1000);
                return std::string(buf);
            }
            else
            {	
                time_t rawtime;
                struct tm *info;
                char buffer[80];
                time( &rawtime );
                info = localtime( &rawtime );
                strftime(buffer, 80, "%H_%M_%S", info);
                return std::string(buffer);
            }
        }

        inline static uint64_t getSystemTimestampUS()
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            return (uint64_t)(ts.tv_sec * 1e6 + ts.tv_nsec * 0.001);
        }

        inline static uint64_t getTimestampUS()
        {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            return (uint64_t)(tv.tv_sec * 1e6 + tv.tv_usec);
        }

        inline static uint64_t getSystemTimestampMS()
        {
            return getSystemTimestampUS() / 1000;
        }

        inline static uint64_t getTimestampMS()
        {
            return getTimestampUS() / 1000;
        }
    };
}

#endif // UTILS_TIMER_H