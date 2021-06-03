
#ifndef LOG_LOG_HANDLER_H
#define LOG_LOG_HANDLER_H

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

namespace planning_utils
{
    /***************************** Handler Target *****************************/
    
    #define DEFINE_HANDLER_TARGET(TargetType, BaseType)    \
        struct TargetType##HandlerTarget;   \
        using TargetType##HandlerTargetPtr    \
            = std::shared_ptr<TargetType##HandlerTarget>;   \
        struct TargetType##HandlerTarget : BaseType##HandlerTarget \
        {   \
          public:
    #define END_HANDLER_Target(TargetType)    \
        };
        

    /* #define CREATE_HANDLER_TARGET(TargetType)    \
        std::dynamic_pointer_cast<HandlerTarget>( \
            std::make_shared<TargetType##HandlerTarget>( \
                new TargetType##HandlerTarget)) */
    #define CREATE_HANDLER_TARGET(TargetType)    \
        std::make_shared<TargetType##HandlerTarget>()

    struct HandlerTarget;
    using HandlerTargetPtr = std::shared_ptr<HandlerTarget>;
    struct HandlerTarget
    {
        virtual ~HandlerTarget() {}
    };


    /***************************** Handler Param ******************************/
    
    #define DEFINE_HANDLER_PARAM(ParamType, BaseType)    \
        struct ParamType##HandlerParam; \
        using ParamType##HandlerParamPtr    \
            = std::shared_ptr<ParamType##HandlerParam>; \
        struct ParamType##HandlerParam : BaseType##HandlerParam \
        {   \
          public:
    #define END_HANDLER_PARAM(ParamType)    \
        };

    #define CREATE_HANDLER_PARAM(ParamType, ...)    \
        std::dynamic_pointer_cast<HandlerParam>( \
            std::make_shared<ParamType##HandlerParam>(__VA_ARGS__))

    struct HandlerParam;   
    using HandlerParamPtr = std::shared_ptr<HandlerParam>;
    struct HandlerParam
    {
        HandlerParam(std::string _name, std::string _path)
            : name(_name), path(_path)
        {}
        virtual ~HandlerParam() {}

        virtual bool isMatch(HandlerParamPtr param) = 0;
        
        std::string name;
        std::string path;
    };

    /******************************** Handler *********************************/
    
    #define DEFINE_LOG_HANDLER(HandlerType, BaseType)    \
        struct HandlerType##LogHandler;     \
        using HandlerType##LogHandlerPtr    \
            = std::shared_ptr<HandlerType##LogHandler>; \
        struct HandlerType##LogHandler : BaseType##LogHandler \
        {   \
          public:   \
            using BaseType##LogHandler::BaseType##LogHandler;
    #define END_LOG_HANDLER(HandlerType)    \
        };

    #define CREATE_LOG_HANDLER(HandlerType, ...)    \
        std::dynamic_pointer_cast<LogHandler>( \
            std::make_shared<HandlerType##LogHandler>(__VA_ARGS__))

    struct LogHandler;
    using LogHandlerPtr = std::shared_ptr<LogHandler>;
    struct LogHandler
    {
      public:

        LogHandler(HandlerParamPtr _param)
            : target_ready(false), param(_param)
        {}
        virtual ~LogHandler() {}

        virtual bool getHandler() = 0;
        virtual void releaseHandler() = 0;
        virtual void tryChangeHandler() = 0;
        virtual void clearHandler() = 0;
        HandlerTargetPtr getCurrentTarget()
        {
            // std::shared_lock<std::shared_timed_mutex> lock(m_targets_mtx);
            return m_targets.back();
        }

        std::mutex target_mtx;
        bool target_ready;

        HandlerParamPtr param;
      
      protected:

        std::vector<HandlerTargetPtr> m_targets; 
        // std::shared_timed_mutex m_targets_mtx;
    };
}

#endif // LOG_LOG_HANDLER_H