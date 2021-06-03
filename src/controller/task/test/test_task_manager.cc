#include <iostream>
#include <thread>

#include "task_manager.h"
#include "log/log_manager.h"
#include "timer/timer.h"
#include "task_log.h"
#include "misc/planning_common_config.h"

using namespace planning_controller;
using namespace planning_utils;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);

//定义任务
DEFINE_TASK(Line, NONE_PREEMPT, true)
    int line_dis;
    float line_speed;    
END_TASK(Line)

DEFINE_TASK(Rotate, NONE_PREEMPT, true)
    int dir;
    float rotate_angle;
    float rotate_speed;
END_TASK(Rotate)

DEFINE_TASK(Edge, NONE_PREEMPT, true)
    int dir;
END_TASK(Edge)

DEFINE_TASK(Follow, NONE_PREEMPT, true)
    int speed;
END_TASK(Follow)

DEFINE_TASK(Dwa, NONE_PREEMPT, true)
    int speed;
END_TASK(Dwa)


//planning_controller::TaskManager g_tm;       // 任务管理器的全局对象

int main()
{
    cout << "test task manager......" << endl;

    uint64_t ts0, ts1, ts2;

    ConfigManager cfg_mgr;
    cfg_mgr.LoadConfig("../../../../../config/planning.yaml");
    // cfg_mgr.LoadConfig("Work/Project/k905/planning/config/planning.yaml");

    g_tm.init(cfg_mgr);

    OPEN_LOG_EXCEPT();

    while (true)
    {
        ts0 = Timer::getSystemTimestampUS();
        cout << "ts0 = " << ts0 << endl;

        CREATE_TASK(TaskLine, b_line_task);

        cout << "test task manager -- 0" << endl;
        b_line_task->preemptible = false;
        cout << "test task manager -- 1" << endl;
        b_line_task->preempt_type = PREEMPT;
        b_line_task->line_dis = 10;
        b_line_task->line_speed = 100;
        cout << "test task manager -- 2" << endl;
        g_tm.addTask(b_line_task);
        this_thread::sleep_for(chrono::milliseconds(2000));

        ts1 = Timer::getSystemTimestampUS();
        cout << "ts1 = " << ts1 << endl;

        CREATE_TASK(TaskRotate, b_rotate_task);
        b_rotate_task->preemptible = true;
        b_rotate_task->preempt_type = NONE_PREEMPT;
        b_rotate_task->dir = 1;
        b_rotate_task->rotate_angle = 1;
        b_rotate_task->rotate_speed = 100;
        g_tm.addTask(b_rotate_task);
        this_thread::sleep_for(chrono::milliseconds(5000));    

        ts2 = Timer::getSystemTimestampUS();
        cout << "ts2 = " << ts2 << endl;

        CREATE_TASK(TaskEdge, b_edge_task);
        b_edge_task->preemptible = false;
        b_edge_task->preempt_type = PREEMPT_DELETE;
        b_edge_task->dir = 2;
        g_tm.addTask(b_edge_task);
        this_thread::sleep_for(chrono::milliseconds(5000));

        CREATE_TASK(TaskFollow, b_follow_task);
        b_follow_task->preemptible = true;
        b_follow_task->preempt_type = NONE_PREEMPT;
        b_follow_task->speed = 2;
        g_tm.addTask(b_follow_task);
        this_thread::sleep_for(chrono::milliseconds(5000));     

        CREATE_TASK(TaskDwa, b_dwa_task);
        b_dwa_task->preemptible = true;
        b_dwa_task->preempt_type = NONE_PREEMPT;
        b_dwa_task->speed = 2;
        g_tm.addTask(b_dwa_task);
        this_thread::sleep_for(chrono::milliseconds(5000));           

        TASK_INFO_LOG("************deleteTask task_id = 4");

        g_tm.deleteTask(4);
        g_tm.deleteTask(5);
        this_thread::sleep_for(chrono::milliseconds(5000));

        TASK_ERROR_LOG("error in deconstructer");
        return 0;
    }
}
