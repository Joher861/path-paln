#include <cstdio>

#include "sleep_timer.h"

using namespace planning_utils;

int main()
{
    SleepTimer t(10.0f);
    while (true)
    {
        printf("ts = %lu\n", t.sleep());
    }
    return 0;
}