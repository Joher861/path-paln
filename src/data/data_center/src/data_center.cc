#include "data_center.h"
#include "data_center_log.h"
#include "data_center_config.h"

#include "log/log_manager.h"
#include "misc/planning_common_config.h"

using namespace planning_data;
using namespace planning_utils;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_DATA_CENTER, DataCenter);

DataCenter& g_dc = DataCenter::getInstance();   // 获取DataCenter单例对象

DataCenter & DataCenter::getInstance()
{
    static DataCenter instance;
    return instance;
}

DataCenter::DataCenter()
{}

void DataCenter::init(ConfigManager &cfg_mgr)
{
    ConfigDataCenter *cfg_data_center = dynamic_cast<ConfigDataCenter*>(
            cfg_mgr.GetSubConfig(CONFIG_DATA_CENTER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning*>(
            cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    
    cfg_data_center->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_DATA_CENTER_FLAG, LOG_DATA_CENTER,
        cfg_data_center->log_name, cfg_data_center->log_path,
        cfg_data_center->log_extension, cfg_data_center->log_ts_mask,
        cfg_data_center->log_print_to_console,
        (cfg_data_center->log_max_file_size_mb) MB
        + (cfg_data_center->log_max_file_size_kb) KB,
        cfg_data_center->log_max_file_cnt, cfg_data_center->log_level); 
}

DataCenter::~DataCenter()
{
    for (auto & [idx, detail] : m_data_details)
    {
        if (!detail.addr)
            delete detail.addr;
    }
}