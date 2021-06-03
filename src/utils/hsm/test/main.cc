#include <iostream>

#include "test_hsm.h"
#include "event_center/event_center.h"
#include "log/log_manager.h"
#include "misc/planning_common_config.h"

using namespace planning_utils;
using namespace planning_data;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);

int main()
{
    ThreadPool pool(2);

    CREATE_LOG(PlainText, LOG_TEST_HSM_FLAG, LOG_TEST_HSM, "test_hsm",
        "/tmp/testlog/test", "log", READABLE_US, true, 2 MB, 2, LEVEL_DEBUG);

    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("Work/Project/k905/planning/config/planning.yaml");
    
    g_ec.init(cfg_mgr);

    OPEN_LOG_EXCEPT();

    ContextTestPtr test_ctx;

    TestHSM hsm(&pool);
    hsm.init(3);
    
    this_thread::sleep_for(chrono::seconds(2));

    hsm.start();

    this_thread::sleep_for(chrono::seconds(1));

    CREATE_EVENT(EvToSecond, to_2nd_evt);
    TEST_HSM_INFO_LOG("put 2nd evt, ts = %lu", to_2nd_evt->ts);
    g_ec.pushEvent(to_2nd_evt);

    this_thread::sleep_for(chrono::seconds(1));

    CREATE_EVENT(EvToThird, to_3rd_evt);
    TEST_HSM_INFO_LOG("put 3rd evt, ts = %lu", to_3rd_evt->ts);
    g_ec.pushEvent(to_3rd_evt);

    this_thread::sleep_for(chrono::seconds(3));

    test_ctx = std::dynamic_pointer_cast<ContextTest>(hsm.pause());

    this_thread::sleep_for(chrono::seconds(2));

    hsm.start(test_ctx);

    this_thread::sleep_for(chrono::seconds(1));

    hsm.stop();

    this_thread::sleep_for(chrono::seconds(2));

    // ThreadPool pool(1);

    // TestHSM hsm(3, pool);

    // this_thread::sleep_for(chrono::seconds(1));

    // hsm.start();
    
    // this_thread::sleep_for(chrono::seconds(2));

    // hsm.pause();

    // this_thread::sleep_for(chrono::seconds(1));

    // hsm.start();

    // this_thread::sleep_for(chrono::seconds(3));

    // hsm.stop();

    // this_thread::sleep_for(chrono::seconds(1));

    // hsm.start();

    // this_thread::sleep_for(chrono::seconds(1));

    // hsm.pause();

    // this_thread::sleep_for(chrono::seconds(2));

    // hsm.stop();

    // this_thread::sleep_for(chrono::seconds(2));

    return 0;
}