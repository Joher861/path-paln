
#include <cstdarg>

#include "log_manager.h"

using namespace planning_utils;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_LOG, Log);

LogManager& g_lm = LogManager::getInstance();

LogManager & LogManager::getInstance()
{
    static LogManager instance;
    return instance;
}

LogManager::LogManager()
    : m_enable_log(false), m_use_fast_readable_ts(true), m_timezone_offset(8)
{
    // 注册内建的一些LogFunc
    registerLogFunc<PlainTextLogFunc>();
    registerLogFunc<CSVLogFunc>();
    registerLogFunc<ImgLogFunc>();
}

void LogManager::init(ConfigManager &cfg_mgr)
{
    ConfigLog *cfg_log = dynamic_cast<ConfigLog*>(
        cfg_mgr.GetSubConfig(CONFIG_LOG));
    
    m_enable_log = cfg_log->enable_log;
    m_use_fast_readable_ts = cfg_log->use_fast_readable_ts;
    m_timezone_offset = cfg_log->timezone_offset;
    m_disable_sync_with_stdio = cfg_log->disable_sync_with_stdio;
    m_untie_cin_cout = cfg_log->untie_cin_cout;

    FuncType plain_text_func_type = typeid(PlainTextLogFunc);
    m_funcs[plain_text_func_type]->setParam(m_use_fast_readable_ts, m_timezone_offset);
    FuncType csv_func_type = typeid(CSVLogFunc);
    m_funcs[csv_func_type]->setParam(m_use_fast_readable_ts, m_timezone_offset);
    FuncType img_func_type = typeid(ImgLogFunc);
    m_funcs[img_func_type]->setParam(m_use_fast_readable_ts, m_timezone_offset);

    // 关闭和stdio的兼容
    if (m_disable_sync_with_stdio)
        std::ios::sync_with_stdio(false);  
    // 解除cin cout绑定
    if (m_untie_cin_cout)
        std::cin.tie(0);
}

int8_t LogManager::createLog(std::string log_name, const LogParam &param)
{
    std::unique_lock<std::shared_timed_mutex> lock(m_log_mtx);

    if (m_logs.find(log_name) != m_logs.end())
    {
        printf("log name %s has been used already.\n", log_name.c_str());
        return -1;
    }

    auto [ handler, func ] = param.referHandlerAndFunc(m_logs, m_funcs);
    if (handler == nullptr || func == nullptr)
    {
        printf("cannot get handler or func according to param\n");
        return -2;
    }
    else
    {
        FuncType func_type = typeid(*func);
        if (m_log_funcs_to_handlers.at(func_type) != typeid(*handler))
            return -3;
    }
    m_handlers[handler].insert(log_name);
    m_logs.emplace(log_name, Log(log_name, typeid(param), handler,
        param.func_param, func));

    return 0;    
}

void LogManager::openLog(std::set<std::string>&& log_names, bool exclude)
{
    std::shared_lock<std::shared_timed_mutex> lock(m_log_mtx);
    std::set<std::string> valid_names;
    for (auto &name : log_names)
        if (m_logs.find(name) != m_logs.end())
            valid_names.insert(name);
    auto func = [&] (std::string name)
        {
            m_logs.at(name).handler->getHandler(); 
        };

    bundleLogExecute(std::move(valid_names), exclude, func);
}

void LogManager::closeLog(std::set<std::string>&& log_names, bool exclude)
{
    std::shared_lock<std::shared_timed_mutex> lock(m_log_mtx);
    std::set<std::string> valid_names;
    for (auto &name : log_names)
        if (m_logs.find(name) != m_logs.end())
            valid_names.insert(name);
    auto func = [&] (std::string name)
        {
            m_logs.at(name).handler->releaseHandler(); 
        };

    bundleLogExecute(std::move(valid_names), exclude, func);
}

void LogManager::clearLog(std::set<std::string>&& log_names, bool exclude)
{
    std::shared_lock<std::shared_timed_mutex> lock(m_log_mtx);
    std::set<std::string> valid_names;
    for (auto &name : log_names)
        if (m_logs.find(name) != m_logs.end())
            valid_names.insert(name);
    auto func = [&] (std::string name)
        {
            m_logs.at(name).handler->clearHandler(); 
        };

    bundleLogExecute(std::move(valid_names), exclude, func);
}

void LogManager::bundleLogExecute(std::set<std::string>&& log_names, bool exclude,
    std::function<void(std::string)> func)
{
    std::set<std::string> names; 
    if (exclude == false)
    {
        names = log_names;
    }
    else
    {
        for(auto & [name, log] : m_logs)
        {
            if (log_names.find(name) == log_names.end())
                names.insert(name);
        }
    }

    for(auto & [handler, name_set] : m_handlers)
    {
        for (auto &name : name_set)
        {
            if (names.find(name) != names.end())
            {
                func(name);
            }
        }
    }
}

void LogManager::log(std::set<std::string>&& log_names, ...)
{
    if (log_names.empty())
        return;

    std::shared_lock<std::shared_timed_mutex> lock(m_log_mtx);   

    std::vector<std::string> valid_names;
    for (auto &name : log_names)
        if (m_logs.find(name) != m_logs.end())
            valid_names.push_back(name);

    if (valid_names.empty())
        return;

    bool same_type = true;
    for (size_t i = 0; i < valid_names.size() - 1; i++)
    {
        if (m_logs.at(valid_names[i]).type
            != m_logs.at(valid_names[i + 1]).type)
        {
            same_type = false;
            break;
        }
    }

    if (!same_type)
        return;

    for(auto log_name : valid_names)
    {
        va_list arg;
        va_start(arg, log_names);
        const Log& log = m_logs.at(log_name);
        log.func->log(log.handler, log.func_param, arg);
        va_end(arg);
    }
}

void LogManager::log(string &&log_name, ...)
{
    std::shared_lock<std::shared_timed_mutex> lock(m_log_mtx);   

    auto it = m_logs.find(log_name);
    if (it == m_logs.end())
        return;

    va_list arg;
    va_start(arg, log_name);
    const Log& log = it->second;
    log.func->log(log.handler, log.func_param, arg);
    va_end(arg);
}

void LogManager::backupLog(std::string backup_path)
{
    std::unique_lock<std::shared_timed_mutex> lock(m_log_mtx);
    std::set<std::string> old_paths;
    std::string cmd;
    cmd = std::string("mkdir -p ") + backup_path;
    system(cmd.c_str());
    cmd = std::string("rm -rf ") + backup_path + "/*";
    system(cmd.c_str());
    auto func = [&] (std::string name) mutable
        {
            m_logs.at(name).handler->releaseHandler();
            std::string path = m_logs.at(name).handler->param->path;
            printf("old_path: %s\n", path.c_str());
            if (old_paths.find(path) != old_paths.end())
                return;
            old_paths.insert(path);
            std::string cmd = std::string("mv ") + path + "/* " + backup_path;
            printf("mv cmd : %s\n", cmd.c_str());
            system(cmd.c_str());
        };

    bundleLogExecute(std::set<std::string>{}, true, func);  
}