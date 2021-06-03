#ifndef LOG_PLAIN_TEXT_LOG_FUNC_H
#define LOG_PLAIN_TEXT_LOG_FUNC_H

#include <cstring>
#include <iostream>

#include "log_func.h"
#include "text_log_handler.h"
#include "misc/color_print.h"

namespace planning_utils
{
    enum PlainTextLogLevel
    {
        LEVEL_NONE = 0,
        LEVEL_DEBUG,
        LEVEL_INFO,
        LEVEL_WARN,
        LEVEL_ERROR,
        LEVEL_MAX_LEVEL
    };

    DEFINE_FUNC_PARAM(PlainText,)
        PlainTextFuncParam(
            uint8_t _ts_mask,
            bool _print_to_console,
            uint8_t _level)
            : FuncParam(_ts_mask, _print_to_console),
              level(_level)
        {}

        virtual bool isMatch(FuncParamPtr param)
        {
            return true;
        }

        uint8_t level;
    END_FUNC_PARAM(PlainText)

    DEFINE_LOG_FUNC(PlainText,)
        virtual void log(LogHandlerPtr log_handler, FuncParamPtr func_param,
            va_list args)
        {
            static const std::string ts_delimiter = "*";
            // static const std::string ts_delimiter_print
            //     = std::string(GREEN) + ts_delimiter + NONE;
            
            if (!log_handler->target_ready)
                return;

            PlainTextFuncParamPtr param
                = std::reinterpret_pointer_cast<PlainTextFuncParam>(func_param);
            
            if (param->level == LEVEL_NONE)
                return;
            
            int32_t line = va_arg(args, int32_t);
            const char* file = va_arg(args, const char *);
            uint8_t log_level(va_arg(args, uint32_t));
            if (log_level == LEVEL_NONE)
                return;
            if (log_level < param->level)
                return;

            const char *fmt = va_arg(args, const char *);
            char log[512];
            vsprintf(log, fmt, args);
            
            if (log[strlen(log) - 1] == '\n')
                log[strlen(log) - 1] = '\0';

            std::string filename(file);
            size_t idx = filename.find_last_of('/');
            if (idx != std::string::npos)
                filename = filename.substr(idx + 1);

            std::string log_level_str;
            std::string log_level_color;
            std::string loc_part_color = COLOR_L_BLUE;
            switch (log_level)
            {
                case LEVEL_DEBUG:
                    log_level_str = "DEBUG";
                    log_level_color = COLOR_L_CYAN;
                    break;
                case LEVEL_INFO:
                    log_level_str = "INFO";
                    log_level_color = COLOR_YELLOW;
                    break;
                case LEVEL_WARN:
                    log_level_str = "WARN";
                    log_level_color = COLOR_L_PURPLE;
                    break;
                case LEVEL_ERROR:
                    log_level_str = "ERROR";
                    log_level_color = COLOR_L_RED;
                    break;
                default:
                    log_level_str = "NONE";
                    log_level_color = COLOR_NONE;
                    break;
            }

            std::string time_part = "";
            if (param->ts_mask != 0)
            {
                time_part += "[";

                std::vector<std::string> &&timestamps
                    = getLogTimestamps(param->ts_mask, m_use_fast_readable_ts,
                        m_timezone_offset);

                bool first = true;
                for (auto &ts : timestamps)
                {
                    if (ts == "")
                        continue;
                    
                    if (first)
                    {
                        time_part += ts;
                        first = false;
                    }
                    else
                    {
                        time_part += ts_delimiter + ts;
                    }
                }
                time_part += "]";
            }
            std::string level_part = std::string("[") + log_level_str + "]";
            std::string loc_part = std::string("[") + filename
                + ":" + std::to_string(line) + "]";

            std::unique_lock<std::mutex> lock(log_handler->target_mtx);
            TextHandlerTargetPtr target
                = std::reinterpret_pointer_cast<TextHandlerTarget>
                    (log_handler->getCurrentTarget());

            std::string whole_log_str = time_part + level_part + loc_part + " "
                + log + "\n";
            target->fs << whole_log_str;
            
            if (param->print_to_console)
            {
                std::string print_whole_log_str = COLOR_NONE + time_part
                    + log_level_color + level_part + loc_part_color
                    + loc_part + log_level_color + " " + log + COLOR_NONE + "\n";
                std::cout << print_whole_log_str;
            }

            log_handler->tryChangeHandler();
        }

        virtual std::type_index getHandlerType()
        {
            return typeid(TextLogHandler);
        }

    END_LOG_FUNC(PlainText)
}

#endif // LOG_PLAIN_TEXT_LOG_FUNC_H