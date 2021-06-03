#include <thread>

#include "event_center.h"
#include "event_center_log.h"
#include "utils/timer/timer.h"
#include "utils/config/config_manager.h"
#include "misc/planning_common_config.h"

using namespace planning_data;
using namespace planning_utils;

using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);

// 定义事件
DEFINE_EVENT(Bumper)
    int8_t bumper;
END_EVENT(Bumper)

DEFINE_EVENT(VC)
    int32_t vc;
END_EVENT(VC)

int main()
{
    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("../../../../../config/planning.yaml");
    g_ec.init(cfg_mgr);

    OPEN_LOG_EXCEPT();

    EVENT_INFO_LOG("EvBumper name = %s", EvBumper::type_name.c_str());
    EVENT_INFO_LOG("EvVC name = %s", EvVC::type_name.c_str());

    list<EventPtr> evt_list;    

    uint64_t ts0, ts1, ts2;

    ts0 = Timer::getSystemTimestampUS();

    CREATE_EVENT(EvBumper, b_ev);
    b_ev->bumper = 3;
    g_ec.pushEvent(b_ev);
    this_thread::sleep_for(chrono::milliseconds(100));

    ts1 = Timer::getSystemTimestampUS();

    CREATE_EVENT(EvVC, vc_ev);
    vc_ev->vc = 32;
    g_ec.pushEvent(vc_ev);
    this_thread::sleep_for(chrono::milliseconds(100));

    ts2 = Timer::getSystemTimestampUS();

    // 获取(ts0, ts1]内发生的事件
    g_ec.eventDeduction(ts0, ts1, evt_list);
    EVENT_INFO_LOG("get event size = %u", evt_list.size());
    for (auto evt : evt_list)
    {
        EVENT_INFO_LOG("event type name = %s", evt->getTypeName().c_str());
    }
    EVENT_INFO_LOG("****************");

    // 获取(ts1, ts2]内发生的事件
    evt_list.clear();
    g_ec.eventDeduction(ts1, ts2, evt_list);
    EVENT_INFO_LOG("get event size = %u", evt_list.size());
    for (auto evt : evt_list)
    {
        EVENT_INFO_LOG("event type name = %s", evt->getTypeName().c_str());
    }
    EVENT_INFO_LOG("****************");

    // 获取(ts0, ts2]内发生的事件
    evt_list.clear();
    g_ec.eventDeduction(ts0, ts2, evt_list);
    EVENT_INFO_LOG("get event size = %u", evt_list.size());
    for (auto evt : evt_list)
    {
        std::type_index type = evt->getType();
        if (TYPE_EQUALS(type, EvBumper))
        // if (evt->getType() == typeid(EvBumper))
        {
            EvBumperPtr bumper_evt = dynamic_pointer_cast<EvBumper>(evt);
            EVENT_INFO_LOG("event type name = %s", evt->getTypeName().c_str());
            EVENT_INFO_LOG("bumper = %d", static_cast<int>(bumper_evt->bumper));
        }
    }
    EVENT_INFO_LOG("****************");

    EVENT_INFO_LOG("ts0 = %lu, ts1 = %lu, ts2 = %lu");

    return 0;
}