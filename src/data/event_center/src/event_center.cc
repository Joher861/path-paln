#include "event_center.h"
#include "event_center_log.h"
#include "event_center_config.h"

#include "timer/timer.h"
#include "log/log_manager.h"
#include "misc/planning_common_config.h"

using namespace planning_data;
using namespace planning_utils;

DEFINE_CONFIG_TYPE(CONFIG_EVENT_CENTER, EventCenter);
EventCenter& g_ec = EventCenter::getInstance();

EventCenter & EventCenter::getInstance()
{
    static EventCenter instance;
    return instance;
}

EventCenter::EventCenter()
{}

void EventCenter::init(ConfigManager &cfg_mgr)
{
    ConfigEventCenter *cfg_event_center = dynamic_cast<ConfigEventCenter*>(
            cfg_mgr.GetSubConfig(CONFIG_EVENT_CENTER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning*>(
            cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    
    cfg_event_center->log_path = cfg_planning->log_path;

    m_evt_timeout = cfg_event_center->event_timeout;

    CREATE_LOG(PlainText, LOG_EVENT_CENTER_FLAG, LOG_EVENT_CENTER,
        cfg_event_center->log_name, cfg_event_center->log_path,
        cfg_event_center->log_extension, cfg_event_center->log_ts_mask,
        cfg_event_center->log_print_to_console,
        (cfg_event_center->log_max_file_size_mb) MB
        + (cfg_event_center->log_max_file_size_kb) KB,
        cfg_event_center->log_max_file_cnt, cfg_event_center->log_level);
}

void EventCenter::pushEvent(EventPtr event)
{
    if (event == nullptr)
        return;

    std::lock_guard<std::mutex> evt_list_lock(m_evt_list_mtx);

    event->ts = Timer::getSystemTimestampUS();

    EVENT_INFO_LOG("push new event, type = %s, ts = %lu",
        event->getTypeName().c_str(), event->ts);

    m_evt_list.push_back(event);
}

void EventCenter::eventDeduction(uint64_t last_ts, uint64_t cur_ts,
                        std::list<EventPtr>& evt_list)
{
    std::lock_guard<std::mutex> evt_list_lock(m_evt_list_mtx);

    /* 删除所有超时的事件 */
    uint64_t ts_limit = cur_ts - m_evt_timeout;
    for (auto it = m_evt_list.begin(); it != m_evt_list.end();)
    {
        EventPtr evt = *it;
        uint64_t evt_ts = evt->ts;
        if (evt_ts < ts_limit)
        {
            EVENT_DEBUG_LOG("delete timeout event, ts = %lu, cur_ts = %lu，"   \
                "ts_diff = %lu", evt_ts, cur_ts, cur_ts - evt_ts);
            it = m_evt_list.erase(it);
        }
        else
        {
            break;
        }
    }

    /* 获取所有在last_ts和cur_ts时间段内的事件 */
    evt_list.clear();
    for (auto it = m_evt_list.rbegin(); it != m_evt_list.rend(); ++it)
    {
        EventPtr evt = *it;
        uint64_t evt_ts = evt->ts;

        if (evt_ts > last_ts && evt_ts <= cur_ts)
        {
            evt_list.push_front(evt);
        }
        else if (evt_ts <= last_ts)
        {
            break;
        }
    }
    EVENT_DEBUG_LOG("event in (%lu, %lu] size = %u", last_ts, cur_ts,
        evt_list.size());
}