#ifndef LOG_CSV_LOG_DATA_H
#define LOG_CSV_LOG_DATA_H

#include <string>

namespace planning_utils
{
    struct CSVLogData
    {
        virtual ~CSVLogData() {}
        virtual std::string getLogStr(std::string delimiter) const = 0;
    };
}

#endif // LOG_CSV_LOG_DATA_H