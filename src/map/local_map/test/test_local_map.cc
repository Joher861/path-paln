#include "thread_pool/ThreadPool.h"

#include "local_map.h"
#include "utils/config.h"
#include "misc/planning_common_config.h"
#include "misc/robot_config.h"
#include "data/data_center.h"
#include "data/event_center.h"
#include "utils/sleep_timer.h"

using namespace planning_map;
using namespace planning_utils;

DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);
DEFINE_CONFIG_TYPE(CONFIG_ROBOT, Robot);

ConfigRobot *g_robot_cfg;

int main()
{
    ThreadPool pool(6);
    
    ConfigManager cfg_mgr;

    cfg_mgr.LoadConfig("Work/Project/k905/planning/config/planning.yaml");
    cfg_mgr.LoadConfig("Work/Project/k905/pms/config/robot.yaml");	

    g_robot_cfg = dynamic_cast<ConfigRobot*>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    g_robot_cfg->setFoortprint();

    g_dc.init(cfg_mgr);
    g_ec.init(cfg_mgr);
    g_local_map.init(cfg_mgr,
        [&](LocalCell *local_map, RobotPose &robot_pose, uint64_t &ts) {
            LMAP_DEBUG_LOG("getting local map");
            // printf("getting local map\n");
            return -1;
        });

    OPEN_LOG_EXCEPT();

    SleepTimer t(1);
    while (true)
    {
        t.sleep();
    }

    return 0;
}