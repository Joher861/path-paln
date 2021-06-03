#ifndef LOG_LOG_H
#define LOG_LOG_H

#include "log_handler.h"
#include "log_func.h"

namespace planning_utils
{
    struct Log;

    struct Log
    {
        // Log(uint64_t _id, std::type_index _type, LogHandlerPtr _handler,
        //         FuncParamPtr _func_param, LogFuncPtr _func)
        //     : id(_id),
        Log(std::string _name, std::type_index _type, LogHandlerPtr _handler,
                FuncParamPtr _func_param, LogFuncPtr _func)
            : name(_name),
              type(_type),
              handler(_handler),
              func_param(_func_param),
              func(_func)
        {}

        // const uint64_t id;
        const std::string name;
        const std::type_index type;
        const LogHandlerPtr handler;
        const FuncParamPtr func_param;
        const LogFuncPtr func;
    };
}

#endif // LOG_LOG_H