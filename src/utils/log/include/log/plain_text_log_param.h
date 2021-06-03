#ifndef LOG_PLAIN_TEXT_LOG_PARAM_H
#define LOG_PLAIN_TEXT_LOG_PARAM_H

#include "include/text_log_param.h"
#include "include/plain_text_log_func.h"

namespace planning_utils
{
    #define PLAIN_TEXT_DEBUG_LOG(log_names, ...)  \
        LOG(log_names, __LINE__, __FILE__, LEVEL_DEBUG, ##__VA_ARGS__)
    #define PLAIN_TEXT_INFO_LOG(log_names, ...)  \
        LOG(log_names, __LINE__, __FILE__, LEVEL_INFO, ##__VA_ARGS__)
    #define PLAIN_TEXT_WARN_LOG(log_names, ...)  \
        LOG(log_names, __LINE__, __FILE__, LEVEL_WARN, ##__VA_ARGS__)
    #define PLAIN_TEXT_ERROR_LOG(log_names, ...)  \
        LOG(log_names, __LINE__, __FILE__, LEVEL_ERROR, ##__VA_ARGS__)

    struct PlainTextLogParam : TextLogParam
    {
      public:

        PlainTextLogParam(std::string _name,
            std::string _path,
            std::string _extension,
            uint8_t _ts_mask,
            bool _print_to_console,
            uint64_t _max_file_size,
            uint8_t _max_file_cnt,
            uint8_t _level)
        {
            handler_param = CREATE_HANDLER_PARAM(Text,
                _name, _path, _extension, _max_file_size, _max_file_cnt);
            func_param = CREATE_FUNC_PARAM(PlainText,
                _ts_mask, _print_to_console, _level);
        }

        uint8_t level;

      private:

        virtual bool isValid() const
        {
            if (handler_param->name == "" || handler_param->path == "")
                return false;
            TextHandlerParamPtr hp
                = std::dynamic_pointer_cast<TextHandlerParam>(handler_param);
            if (hp->extension == "")
                return false;

            PlainTextFuncParamPtr fp
                = std::dynamic_pointer_cast<PlainTextFuncParam>(func_param);
            if (fp->level >= LEVEL_MAX_LEVEL)
                return false;
            
            return true;
        }

        virtual LogHandlerPtr generateNewHandler(HandlerParamPtr param) const
        {
            return CREATE_LOG_HANDLER(Text, param);
        }

        virtual LogFuncPtr findFunc(
            const std::unordered_map<std::type_index, LogFuncPtr> &funcs) const
        {
            std::type_index func_type = typeid(PlainTextLogFunc);
            auto it = funcs.find(func_type);
            if (it == funcs.end())
                return nullptr;
            else
                return it->second;
        }
    };
}

#endif // LOG_PLAIN_TEXT_LOG_PARAM_H
