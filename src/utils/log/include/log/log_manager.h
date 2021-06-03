#ifndef LOG_LOG_MANAGER_H
#define LOG_LOG_MANAGER_H

#include <typeindex>
#include <set>
#include <functional>
#include <mutex>
#include <shared_mutex>

#include "include/log_param.h"
#include "include/log_config.h"
#include "plain_text_log_param.h"
#include "csv_log_param.h"
#include "img_log_param.h"

#include "config/config_manager.h"

using planning_utils::ConfigManager;

// ********************************* log相关宏定义 ***************************

/**
  * @brief  注册自定义的log func
  * @param  FuncType 要注册的log func类型
  * @retval None
  */
#define REGISTER_LOG_FUNC(FuncType)                                        \
    g_lm.registerLogFunc<FuncType>()

/**
  * @brief  生成对应的log创建标志变量，代表目标log是否创建成功
  * @param  flag_name log创建标志名，一般和log名对应
  * @retval None
  */
#define LOG_CREATE_FLAG(flag_name)   _log_create_flag_##flag_name##_

/**
  * @brief  创建log
  * @param  LogParamType    log参数类型
  *         flag_name       log创建标志名
  *         log_name        log名
  *         ...             可变参数，对应该log参数类型的具体参数
  * @retval None
  */
#define CREATE_LOG(LogParamType, flag_name, log_name, ...)                 \
    int8_t LOG_CREATE_FLAG(flag_name) = g_lm.createLog(log_name,           \
        LogParamType##LogParam(__VA_ARGS__))

/**
  * @brief  打开指定的log
  * @param  ... 可变参数，所有要打开的log名，如log1, log2, log3
  * @retval None
  */
#define OPEN_LOG(...)   \
    g_lm.openLog(std::set<std::string>{__VA_ARGS__})

/**
  * @brief  除去某些log，打开所有其他已创建的log
  * @param  ... 可变参数，所有要除去的log名，如log1, log2, log3
  * @retval None
  */
#define OPEN_LOG_EXCEPT(...)  \
    g_lm.openLog(std::set<std::string>{__VA_ARGS__}, true)

/**
  * @brief  关闭指定的log
  * @param  ... 可变参数，所有要关闭的log名，如log1, log2, log3
  * @retval None
  */
#define CLOSE_LOG(...)  \
    g_lm.closeLog(std::set<std::string>{__VA_ARGS__})

/**
  * @brief  除去某些log，关闭所有其他已创建的log
  * @param  ... 可变参数，所有要除去的log名，如log1, log2, log3
  * @retval None
  */
#define CLOSE_LOG_EXCEPT(...)  \
    g_lm.closeLog(std::set<std::string>{__VA_ARGS__}, true)

/**
  * @brief  清空指定的log（连文件一起删除）
  * @param  ... 可变参数，所有要清空的log名，如log1, log2, log3
  * @retval None
  */
#define CLEAR_LOG(...)  \
    g_lm.clearLog(std::set<std::string>{__VA_ARGS__})

/**
  * @brief  除去某些log，清空所有其他已创建的log
  * @param  ... 可变参数，所有要除去的log名，如log1, log2, log3
  * @retval None
  */
#define CLEAR_LOG_EXCEPT(...)  \
    g_lm.clearLog(std::set<std::string>{__VA_ARGS__}, true)

/**
  * @brief  备份所有已创建的log
  * @param  backup_path 备份的目标路径
  * @retval None
  */
#define BACKUP_LOG(backup_path)     g_lm.backupLog(backup_path);

/**
  * @brief  创建所有log名的集合，和LOG配合使用
  * @param  ...可变参数 所需包括的log名
  * @retval None
  */
#define LOG_NAMES(...)  std::set<std::string>{__VA_ARGS__}

/**
  * @brief  创建log名，和LOG配合使用
  * @param  log_name log名
  * @retval None
  */
#define LOG_NAME(log_name)  std::string{log_name}

/**
  * @brief  写入log
  * @param  log_names   所有要写入的log，可以是LOG_NAMES集合，也可以是单个log名
  *         ...         可变参数，写入的内容（不同log写入的内容不同）
  * @retval None
  */
#define LOG(log_names, ...)  g_lm.log(log_names, ##__VA_ARGS__)

namespace planning_utils
{
    class LogManager
    {
      private:

        using LogFormat = std::type_index;
        using HandlerType = std::type_index;
        using FuncType = std::type_index;
        using FormatToFuncMap = std::unordered_map<LogFormat, FuncType>;
        using FuncToHandlerMap = std::unordered_map<FuncType, HandlerType>;

      public:

        /**
          * @brief  日志管理器的单例
          * @param  None
          * @retval None
          */
        static LogManager &getInstance();

        // 禁止拷贝和赋值构造函数
        LogManager(const LogManager &) = delete;
        void operator=(const LogManager &) = delete;

        /**
          * @brief  初始化log模块
          * @param  cfg_mgr     配置管理器对象
          * @retval None
          */
        void init(ConfigManager &cfg_mgr);

        /**
          * @brief  创建log
          * @param  log_name    log名字，要求不同的log名字不可重复
          *         param       log参数，不同子类的LogParam，代表不同类型的log
          * @retval 是否成功创建  0     成功
          *                     -1    不成功，名字重复
          *                     -2    不成功，没有对应的Func或者Handler创建失败
          *                     -3    不成功，该参数类型的Func和Handler不匹配
          */
        int8_t createLog(std::string log_name, const LogParam &param);

        /**
          * @brief  注册自定义的log func
          * @param  LogFuncType 自定义的log func类型
          * @retval None
          */
        template<typename LogFuncType>
        void registerLogFunc()
        {
            std::unique_lock<std::shared_timed_mutex> lock(m_log_mtx);
            std::shared_ptr<LogFuncType> func = LogFuncType::createLogFunc();
            FuncType func_type = typeid(LogFuncType);
            m_log_funcs_to_handlers.emplace(func_type, func->getHandlerType());
            m_funcs[func_type] = std::dynamic_pointer_cast<LogFunc>(func);
        }

        /**
          * @brief  打开log
          * @param  log_names   所有目标的log名
          *         exclude     包括log_names还是除去log_names
          * @retval None
          */
        void openLog(std::set<std::string>&& log_names, bool exclude = false);

        /**
          * @brief  关闭log
          * @param  log_names   所有目标的log名
          *         exclude     包括log_names还是除去log_names
          * @retval None
          */
        void closeLog(std::set<std::string>&& log_names, bool exclude = false);
        
        /**
          * @brief  清空log（连文件一起清空）
          * @param  log_names   所有目标的log名
          *         exclude     包括log_names还是除去log_names
          * @retval None
          */
        void clearLog(std::set<std::string>&& log_names, bool exclude = false);
        
        /**
          * @brief  写入log
          * @param  log_names   所有目标的log名（可以同时写入多个log，但需要这些log类型
          *                     相同）
          *         ...         可变参数，写入的内容（不同类型的log内容不同）
          * @retval None
          */
        void log(std::set<std::string>&& log_names, ...);

        /**
          * @brief  写入log
          * @param  log_name    所有目标的log名
          *         ...         可变参数，写入的内容（不同类型的log内容不同）
          * @retval None
          */
        void log(std::string &&log_name, ...);

        /**
          * @brief  备份所有log
          * @param  backup_path 备份到的目录
          * @retval None
          */
        void backupLog(std::string backup_path);

      private:

        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        LogManager();

        /**
          * @brief  辅助函数，多目标log同时操作
          * @param  log_names   目标log的集合
          *         exclude     是要包括还是排除
          *         func        具体的操作函数
          * @retval None
          */
        void bundleLogExecute(std::set<std::string>&& log_names, bool exclude,
            std::function<void(std::string)> func);

        FuncToHandlerMap m_log_funcs_to_handlers;       // Func和Handler的对应关系
        std::unordered_map<std::type_index, LogFuncPtr> m_funcs;  // 所有注册的func
        std::unordered_map<LogHandlerPtr, std::set<std::string>> m_handlers;
                                                    // 所有handler和log间的对应关系
        std::unordered_map<std::string, Log> m_logs;    // 所有创建的log
        std::shared_timed_mutex m_log_mtx;              // log操作的互斥锁

        // 参数
        bool m_enable_log;
        bool m_use_fast_readable_ts;
        int32_t m_timezone_offset;
        bool m_disable_sync_with_stdio;
        bool m_untie_cin_cout;
    };
}

extern planning_utils::LogManager& g_lm;         // log_manager的单例全局变量

#endif // LOG_LOG_MANAGER_Hd