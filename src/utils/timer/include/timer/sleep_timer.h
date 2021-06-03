#include "timer.h"

namespace planning_utils
{
    class SleepTimer
    {
      public:

        SleepTimer(float frequency, uint64_t min_sleep_time = 1000);
        
        uint64_t sleep();

      private:

        uint64_t m_loop_ts_us;
        uint64_t m_min_sleep_time;
        uint64_t m_last_ts;
    };
}