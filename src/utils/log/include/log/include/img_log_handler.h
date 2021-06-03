#ifndef LOG_IMG_LOG_HANDLER_H
#define LOG_IMG_LOG_HANDLER_H

#include <fstream>

#include "log_handler.h"

namespace planning_utils
{
    DEFINE_HANDLER_TARGET(Img,)
        std::string dir;
    END_HANDLER_Target(Img)

    DEFINE_HANDLER_PARAM(Img,)
        ImgHandlerParam(
            std::string _name,
            std::string _path,
            std::string _extension,
            uint64_t _max_img_cnt)
            : HandlerParam(_name, _path),
              extension(_extension),
              max_img_cnt(_max_img_cnt)
        {}

        virtual bool isMatch(HandlerParamPtr param)
        {
            ImgHandlerParamPtr img_param
                = std::dynamic_pointer_cast<ImgHandlerParam>(param);

            if (extension != img_param->extension)
                return false;
            
            if (max_img_cnt == 0 && img_param->max_img_cnt != 0)
            {
                img_param->max_img_cnt = 0;
            }
            else if (max_img_cnt != 0 && img_param->max_img_cnt == 0) 
            {
                max_img_cnt = 0;
            }
            else if (max_img_cnt != 0 && img_param->max_img_cnt != 0)
            {
                uint64_t max_cnt
                    = std::max(max_img_cnt, img_param->max_img_cnt);
                max_img_cnt = img_param->max_img_cnt = max_cnt;
            }
            
            return true;
        }

        std::string extension;
        uint64_t max_img_cnt;
    END_HANDLER_PARAM(Img)

    DEFINE_LOG_HANDLER(Img,)
        virtual bool getHandler()
        {
            ImgHandlerParamPtr img_param
                = std::reinterpret_pointer_cast<ImgHandlerParam>(param);
            
            std::unique_lock<std::mutex> tg_lock(target_mtx);

            if (m_targets.empty())
            {
                ImgHandlerTargetPtr target = CREATE_HANDLER_TARGET(Img);
                target->dir = img_param->path;

                std::string cmd = std::string("mkdir -p ") + target->dir;
                system(cmd.c_str());

                m_targets.emplace_back(
                    std::reinterpret_pointer_cast<HandlerTarget>(target));
                target_ready = true;
                return true;
            }

            ImgHandlerTargetPtr target
                = std::reinterpret_pointer_cast<ImgHandlerTarget>(m_targets.back());
            std::string cmd = std::string("mkdir -p ") + target->dir;
                system(cmd.c_str());
            target_ready = true; 
            return true;
        }

        virtual void releaseHandler()
        {
            std::unique_lock<std::mutex> tg_lock(target_mtx);

            if (m_targets.empty())
            {
                target_ready = false;
                return;
            }

            ImgHandlerTargetPtr target
                = std::dynamic_pointer_cast<ImgHandlerTarget>(m_targets.back());
            target_ready = false; 
            return;
        }

        virtual void tryChangeHandler()
        {
            return;
        }
        
        virtual void clearHandler()
        {
            std::unique_lock<std::mutex> tg_lock(target_mtx);

            if (!m_targets.empty())
            {
                ImgHandlerTargetPtr target
                    = std::dynamic_pointer_cast<ImgHandlerTarget>(m_targets.back());
                m_targets.clear();
            }
            target_ready = false; 
            ImgHandlerParamPtr img_param
                = std::dynamic_pointer_cast<ImgHandlerParam>(param);
            std::string cmd = "rm -rf " + img_param->path;
            system(cmd.c_str());
            return;
        }
    END_LOG_HANDLER(Img);
}

#endif // LOG_IMG_LOG_HANDLER_H  
