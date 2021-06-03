#ifndef LOG_TEXT_LOG_HANDLER_H
#define LOG_TEXT_LOG_HANDLER_H

#include <fstream>

#include "log_handler.h"

namespace planning_utils
{
    DEFINE_HANDLER_TARGET(Text,)
        std::fstream fs;
        std::string filename;
        uint8_t file_cnt;
    END_HANDLER_Target(Text)

    DEFINE_HANDLER_PARAM(Text,)
        TextHandlerParam(
            std::string _name,
            std::string _path,
            std::string _extension,
            uint64_t _max_file_size,
            uint8_t _max_file_cnt)
            : HandlerParam(_name, _path),
              extension(_extension),
              max_file_size(_max_file_size),
              max_file_cnt(_max_file_cnt)
        {}

        virtual bool isMatch(HandlerParamPtr param)
        {
            TextHandlerParamPtr text_param
                = std::dynamic_pointer_cast<TextHandlerParam>(param);

            if (extension != text_param->extension)
                return false;
            
            if (max_file_size == 0 && text_param->max_file_size != 0)
            {
                text_param->max_file_size = 0;
            }
            else if (max_file_size != 0 && text_param->max_file_size == 0) 
            {
                max_file_size = 0;
            }
            else if (max_file_size != 0 && text_param->max_file_size != 0)
            {
                uint64_t max_size
                    = std::max(max_file_size, text_param->max_file_size);
                max_file_size = text_param->max_file_size = max_size;

                if (max_file_cnt == 0 || text_param->max_file_cnt == 0)
                {
                    max_file_cnt = text_param->max_file_cnt = 0;
                }
                else
                {
                    uint8_t max_cnt
                        = std::max(max_file_cnt, text_param->max_file_cnt);
                    max_file_cnt = text_param->max_file_cnt = max_cnt;
                }
            }
            
            return true;
        }

        std::string extension;
        uint64_t max_file_size;
        uint8_t max_file_cnt;
    END_HANDLER_PARAM(Text);

    DEFINE_LOG_HANDLER(Text,)
        virtual bool getHandler()
        {
            TextHandlerParamPtr text_param
                = std::reinterpret_pointer_cast<TextHandlerParam>(param);
            std::string cmd = std::string("mkdir -p ") + text_param->path;
            system(cmd.c_str());
            
            std::unique_lock<std::mutex> tg_lock(target_mtx);

            if (m_targets.empty())
            {
                TextHandlerTargetPtr target = CREATE_HANDLER_TARGET(Text);
                target->filename = text_param->path + "/" + text_param->name
                    + "." + text_param->extension;
                target->file_cnt = 0;

                target->fs.open(target->filename,
                    std::fstream::out | std::fstream::trunc);
                if (!target->fs.is_open())
                {
                    target_ready = false;
                    return false;
                }
                m_targets.emplace_back(
                    std::reinterpret_pointer_cast<HandlerTarget>(target));
                target_ready = true;
                return true;
            }

            TextHandlerTargetPtr target
                = std::reinterpret_pointer_cast<TextHandlerTarget>(m_targets.back());
            if (!target->fs.is_open())
            {
                target->fs.open(target->filename,
                    std::fstream::out | std::fstream::trunc);
                if (!target->fs.is_open())
                {
                    target_ready = false;
                    return false;
                }
            }
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

            TextHandlerTargetPtr target
                = std::dynamic_pointer_cast<TextHandlerTarget>(m_targets.back());
            if (target->fs.is_open())
            {
                target->fs.close();
            }
            target_ready = false; 
            return;
        }

        virtual void tryChangeHandler()
        {
            if (!target_ready)
                return;

            TextHandlerParamPtr text_param
                = std::reinterpret_pointer_cast<TextHandlerParam>(param);
            if (text_param->max_file_size == 0)
                return;
            

            if (m_targets.empty())
                return;
            
            TextHandlerTargetPtr cur_target
                = std::reinterpret_pointer_cast<TextHandlerTarget>(m_targets.back());            
            size_t file_size = (size_t)cur_target->fs.tellp();

            if (file_size < text_param->max_file_size)
                return;

            cur_target->fs.close();
            cur_target->filename = text_param->path + "/" + text_param->name
                + "." + std::to_string(cur_target->file_cnt) + ".log";
            std::string origin_log_name = text_param->path + "/"
                + text_param->name + "." + text_param->extension;
            std::string cmd = std::string("mv ") + origin_log_name + " "
                + cur_target->filename;
            system(cmd.c_str());

            if (text_param->max_file_cnt != 0
                && m_targets.size() > text_param->max_file_cnt)
            {
                TextHandlerTargetPtr del_target
                    = std::reinterpret_pointer_cast<TextHandlerTarget>(
                        m_targets.front());
                std::string cmd = std::string("rm -f ") + del_target->filename;
                system(cmd.c_str());
                m_targets.erase(m_targets.begin());
            }

            TextHandlerTargetPtr target = CREATE_HANDLER_TARGET(Text);
            target->filename = text_param->path + "/" + text_param->name
                + "." + text_param->extension;
            target->file_cnt = cur_target->file_cnt + 1;

            target->fs.open(target->filename,
                std::fstream::out | std::fstream::trunc);
            if (!target->fs.is_open())
            {
                target_ready = false;
                return;
            }
            m_targets.emplace_back(
                std::reinterpret_pointer_cast<HandlerTarget>(target));
            target_ready = true;
            return;
        }
        
        virtual void clearHandler()
        {
            std::unique_lock<std::mutex> tg_lock(target_mtx);

            if (!m_targets.empty())
            {
                TextHandlerTargetPtr target
                    = std::dynamic_pointer_cast<TextHandlerTarget>(m_targets.back());
                if (target->fs.is_open())
                {
                    target->fs.close();
                }
                m_targets.clear();
            }
            target_ready = false; 
            TextHandlerParamPtr text_param
                = std::dynamic_pointer_cast<TextHandlerParam>(param);
            std::string cmd = std::string("ls -dp ") + text_param->path
                +"/* | grep -P \".*/" + text_param->name + "\\.([0-9]+\\.)?"
                + text_param->extension + "$\" | xargs -d \"\\n\" rm -f";
            system(cmd.c_str());
            return;
        }
    END_LOG_HANDLER(Text);
}

#endif // LOG_TEXT_LOG_HANDLER_H  
