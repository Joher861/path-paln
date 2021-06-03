#include "thread_pool/ThreadPool.h"

#include "rotate_planner_log.h"
#include "rotate_task.h"
#include "rotate_planner.h"
#include "task/task_manager.h"
#include "speed_controller/speed_controller.h"
#include "geometry/geometry_func.h"
#include "log/log_manager.h"
#include "timer/sleep_timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "misc/robot_config.h"
#include "misc/planning_common_config.h"

using namespace std;
using namespace planning_controller;
using namespace planning_data;
using namespace planning_utils;

DEFINE_CONFIG_TYPE(CONFIG_ROBOT, Robot);
DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);
ConfigRobot *g_robot_cfg;

int main()
{
    ThreadPool pool(10);

    ConfigManager cfg_mgr;

    // cfg_mgr.LoadConfig("../../../../../../../pms/config/robot.yaml");
    // cfg_mgr.LoadConfig("../../../../../../config/planning.yaml");
    cfg_mgr.LoadConfig("Work/Project/k905/pms/config/robot.yaml");
    cfg_mgr.LoadConfig("Work/Project/k905/planning/config/planning.yaml");

    g_robot_cfg = dynamic_cast<ConfigRobot*>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    g_robot_cfg->setFoortprint();
    
    RotatePlannerPtr& rotate_planner = RotatePlanner::getInstance(&pool);

    g_dc.init(cfg_mgr);
    g_ec.init(cfg_mgr);
    g_speed_controller.init(cfg_mgr, [](float vl, float vr){});
    g_tm.init(cfg_mgr);
    rotate_planner->init(cfg_mgr);

    OPEN_LOG_EXCEPT();

    g_tm.registerPlanner<TaskRotate>(rotate_planner);
                
    int64_t rotate_task1_id;

    pool.enqueue(
        [&]() mutable
        {
            CREATE_TASK(TaskRotate, test_rt);
            test_rt->w = deg2rad(90);
            test_rt->type = TaskRotate::DELTA_TIME;
            test_rt->dir = TaskRotate::ROTATE_LEFT;
            test_rt->time = 2000;
            ROTATE_INFO_LOG("add task1");
            rotate_task1_id = g_tm.addTask(test_rt);
        }
    );

    pool.enqueue(
        [&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            CREATE_TASK(TaskRotate, test_rt);
            test_rt->w = deg2rad(45);
            test_rt->type = TaskRotate::DELTA_TIME;
            test_rt->dir = TaskRotate::ROTATE_LEFT;
            test_rt->time = 2000;
            ROTATE_INFO_LOG("add task2");
            g_tm.addTask(test_rt);
        }
    );

    pool.enqueue(
        [&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            ROTATE_INFO_LOG("delete task1");
            g_tm.deleteTask(rotate_task1_id);
        }
    );

    pool.enqueue(
        [&]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            CREATE_TASK(TaskRotate, test_rt);
            test_rt->w = deg2rad(45);
            test_rt->type = TaskRotate::DELTA_TIME;
            test_rt->dir = TaskRotate::ROTATE_LEFT;
            test_rt->time = 3000;
            test_rt->preempt_type = PREEMPT;
            ROTATE_INFO_LOG("add task3");
            g_tm.addTask(test_rt);
        }
    );

    SleepTimer t(1.0f);
    while (true)
    {
        t.sleep();
    }

    return 0;
}