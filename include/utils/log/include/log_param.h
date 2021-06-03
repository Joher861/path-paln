#ifndef LOG_LOG_PARAM_H
#define LOG_LOG_PARAM_H

#include <tuple>
#include <unordered_map>

#include "log.h"

namespace planning_utils
{
    struct LogParam;
    using LogParamPtr = std::shared_ptr<LogParam>;
    struct LogParam
    {
      public:

        virtual ~LogParam() {}
        std::tuple<LogHandlerPtr, LogFuncPtr> referHandlerAndFunc(
            // const std::map<uint64_t, Log> &logs,
            const std::unordered_map<std::string, Log> &logs,
            const std::unordered_map<std::type_index, LogFuncPtr> &funcs) const
        {
            LogHandlerPtr handler;
            LogFuncPtr func;
            if (!isValid())
                return std::make_tuple(nullptr, nullptr);
            if (!validateParamWithLogs(logs, handler, func))
                return std::make_tuple(nullptr, nullptr);
            
            if (func == nullptr)
            {
                func = findFunc(funcs);
                if (func == nullptr)
                    return std::make_tuple(nullptr, nullptr);
            }
            if (handler == nullptr)
            {
                handler = generateNewHandler(handler_param);
            }
            return std::make_tuple(handler, func);
        }

        HandlerParamPtr handler_param;
        FuncParamPtr func_param;

      protected:

        virtual bool isValid() const = 0;
        // bool validateParamWithLogs(const std::map<uint64_t, Log> &logs,
        bool validateParamWithLogs(const std::unordered_map<std::string, Log> &logs,
            LogHandlerPtr &handler, LogFuncPtr &func) const
        {
            handler = nullptr;
            func = nullptr;
            for (auto & [name, log] : logs)
            // for (auto & [id, log] : logs)
            {
                if (log.handler->param->name == handler_param->name
                    && log.handler->param->path == handler_param->path)
                {
                    if (typeid(*log.handler->param) != typeid(*handler_param))
                        return false;
                    if (typeid(*log.func_param) != typeid(*func_param))
                        return false;
                    // if (!isFuncParamMatch(log.func_param))
                    //     return false;
                    // if (!isHandlerParamMatch(log.handler->param))
                    //     return false;
                    if (!func_param->isMatch(log.func_param))
                        return false;
                    if (!handler_param->isMatch(log.handler->param))
                        return false;
                    
                    handler = log.handler;
                    func = log.func;
                    return true;
                }
            }

            return true;
        }
        // virtual bool isHandlerParamMatch(HandlerParamPtr param) const = 0;
        // virtual bool isFuncParamMatch(FuncParamPtr param) const = 0;
        virtual LogHandlerPtr generateNewHandler(
            HandlerParamPtr param) const = 0;
        virtual LogFuncPtr findFunc(
            const std::unordered_map<std::type_index, LogFuncPtr> &funcs) const = 0;
    };
}

#endif // LOG_LOG_PARAM_H