#ifndef LOG_IMG_LOG_FUNC_H
#define LOG_IMG_LOG_FUNC_H

#include <cstring>
#include <iostream>

#include "opencv2/imgcodecs.hpp"

#include "log_func.h"
#include "img_log_handler.h"

namespace planning_utils
{
    DEFINE_FUNC_PARAM(Img,)
        ImgFuncParam(
            uint8_t _ts_mask,
            bool _print_to_console,
            std::string _img_name,
            std::string _extension)
            : FuncParam(_ts_mask, _print_to_console),
              img_name(_img_name), extension(_extension)
        {}

        virtual bool isMatch(FuncParamPtr param)
        {
            return true;
        }

        std::string img_name;
        std::string extension;

    END_FUNC_PARAM(Img)

    DEFINE_LOG_FUNC(Img,)
        virtual void log(LogHandlerPtr log_handler, FuncParamPtr func_param,
            va_list args)
        {
            if (!log_handler->target_ready)
                return;

            ImgFuncParamPtr param
                = std::reinterpret_pointer_cast<ImgFuncParam>(func_param);
            
            uint64_t img_addr(va_arg(args, uint64_t));
            cv::Mat *img = reinterpret_cast<cv::Mat *>(img_addr);

            std::string time_part = "";
            if (param->ts_mask == 0)
            {
                param->ts_mask = 2;
            }

            std::vector<std::string> &&timestamps
                = getLogTimestamps(param->ts_mask, m_use_fast_readable_ts,
                        m_timezone_offset);
            
            for (auto &ts : timestamps)
            {
                if (ts == "")
                    continue;

                time_part += "_" + ts;
            }

            std::unique_lock<std::mutex> lock(log_handler->target_mtx);
            ImgHandlerTargetPtr target
                = std::reinterpret_pointer_cast<ImgHandlerTarget>
                    (log_handler->getCurrentTarget());
            
            std::string img_whole_name = target->dir + "/" + param->img_name
                + time_part + "." + param->extension;

            cv::imwrite(img_whole_name, *img);
            
            log_handler->tryChangeHandler();
        }

        virtual std::type_index getHandlerType()
        {
            return typeid(ImgLogHandler);
        }
    END_LOG_FUNC(Img)
}

#endif // LOG_IMG_LOG_FUNC_H