#ifndef LOG_CSV_LOG_PARAM_H
#define LOG_CSV_LOG_PARAM_H


#include "include/text_log_param.h"
#include "include/csv_log_func.h"

namespace planning_utils
{
    #define CSV_LOG(log_names, CSVDataType, ...)  \
        LOG(log_names, std::make_shared<CSVDataType>(__VA_ARGS__).get())

    struct CSVLogParam : TextLogParam
    {
      public:

        CSVLogParam(std::string _name,
            std::string _path,
            std::string _extension,
            uint8_t _ts_mask,
            bool _print_to_console,
            uint64_t _max_file_size,
            uint8_t _max_file_cnt,
            std::string _delimiter)
        {
            handler_param = CREATE_HANDLER_PARAM(Text,
                _name, _path, _extension, _max_file_size, _max_file_cnt);
            func_param = CREATE_FUNC_PARAM(CSV,
                _ts_mask, _print_to_console, _delimiter);
        }

        std::string delimiter;

      private:

        virtual bool isValid() const
        {
            if (handler_param->name == "" || handler_param->path == "")
                return false;
            TextHandlerParamPtr hp
                = std::dynamic_pointer_cast<TextHandlerParam>(handler_param);
            if (hp->extension == "")
                return false;

            CSVFuncParamPtr fp
                = std::dynamic_pointer_cast<CSVFuncParam>(func_param);
            if (fp->delimiter.find('\n') != std::string::npos
                || fp->delimiter.find('\r') != std::string::npos)
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
            std::type_index func_type = typeid(CSVLogFunc);
            auto it = funcs.find(func_type);
            if (it == funcs.end())
                return nullptr;
            else
                return it->second;
        }
    };
}

#endif // LOG_CSV_LOG_PARAM_H