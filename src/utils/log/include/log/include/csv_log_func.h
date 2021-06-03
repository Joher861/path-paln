#ifndef LOG_CSV_LOG_LEVEL_H
#define LOG_CSV_LOG_LEVEL_H

#include <cstring>
#include <iostream>

#include "log_func.h"
#include "text_log_handler.h"
#include "../csv_log_data.h"
#include "misc/color_print.h"

namespace planning_utils
{
    DEFINE_FUNC_PARAM(CSV,)
        CSVFuncParam(
            uint8_t _ts_mask,
            bool _print_to_console,
            std::string _delimiter)
            : FuncParam(_ts_mask, _print_to_console),
              delimiter(_delimiter)
        {}

        virtual bool isMatch(FuncParamPtr param)
        {
            CSVFuncParamPtr csv_param
                = std::dynamic_pointer_cast<CSVFuncParam>(param);
            
            return (delimiter == csv_param->delimiter);
        }

        std::string delimiter;
    END_FUNC_PARAM(CSV)

    DEFINE_LOG_FUNC(CSV,)
        virtual void log(LogHandlerPtr log_handler, FuncParamPtr func_param,
            va_list args)
        {
            if (!log_handler->target_ready)
                return;

            CSVFuncParamPtr param
                = std::reinterpret_pointer_cast<CSVFuncParam>(func_param);
            
            const CSVLogData* data = va_arg(args, const CSVLogData*);
            std::string&& data_str = data->getLogStr(param->delimiter);

            std::string time_part = "";
            if (param->ts_mask != 0)
            {
                std::vector<std::string> &&timestamps
                    = getLogTimestamps(param->ts_mask, m_use_fast_readable_ts,
                        m_timezone_offset);
                
                for (auto &ts : timestamps)
                {
                    if (ts == "")
                        continue;
                    
                    time_part += ts + param->delimiter;
                }
            }

            std::unique_lock<std::mutex> lock(log_handler->target_mtx);
            TextHandlerTargetPtr target
                = std::reinterpret_pointer_cast<TextHandlerTarget>
                    (log_handler->getCurrentTarget());

            std::string whole_log_str = time_part + data_str + "\n";
            target->fs << whole_log_str;
            
            if (param->print_to_console)
            {
                std::cout << whole_log_str;
            }

            log_handler->tryChangeHandler();
        }

        virtual std::type_index getHandlerType()
        {
            return typeid(TextLogHandler);
        }
    END_LOG_FUNC(CSV)
}

#endif // LOG_CSV_LOG_LEVEL_H