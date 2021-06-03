#include <ctime>
#include "speed_controller.h"
#include "log/log_manager.h"
#include "misc/robot_config.h"
#include "misc/planning_common_config.h"

using namespace planning_controller;
using namespace planning_utils;
// using namespace Communication;
using namespace std;

DEFINE_CONFIG_TYPE(CONFIG_ROBOT, Robot);
DEFINE_CONFIG_TYPE(CONFIG_PLANNING, Planing);
ConfigRobot *g_robot_cfg;

void Delay(int time) //time*1000为秒数 
{
    clock_t now = clock();
    while (clock() - now < time);
}

uint64_t getTime_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)(ts.tv_sec * 1e6 + ts.tv_nsec * 0.001);    
}

uint64_t getTime_ms()
{
    return getTime_us() / 1000;
}

// zwz test
void test(float a, float b)
{
    std::cout << a << "" << b << '\n';
}

int main()
{
    static int step = 0;

    ConfigManager cfg_mgr;

    cfg_mgr.LoadConfig("../../../../../pms/config/robot.yaml");
    cfg_mgr.LoadConfig("../../../../config/planning.yaml");

    g_robot_cfg = dynamic_cast<ConfigRobot*>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    g_robot_cfg->setFoortprint();
    cout << "set speed controller init Parameter..." << endl;
    g_speed_controller.init(cfg_mgr,test);
    // g_speed_controller.init(cfg_mgr,
    //     [&](float vl, float vr) { g_comm.sendSpeed(vl, vr); });
    cout << "start test speed controller..." << endl;

    OPEN_LOG_EXCEPT();

    while (true)
    {
        switch (step)
        {
            case 0:
                g_speed_controller.SetChassisControl(0.1, 0.0);  
                step = 1;
                cout << "g_speed_controller.SetChassisControl(0.1, 0.0)" << endl;
                break;            
            case 1:
                g_speed_controller.SetStop();  
                step = 2;
                cout << "g_speed_controller.SetStop()" << endl;
                break;
            case 2:
                g_speed_controller.SetChassisControl(0.1, M_PI_2);  
                step = 3;
                cout << "g_speed_controller.SetChassisControl(0.1, M_PI_2)" << endl;
                break;
            case 3:
                g_speed_controller.SetChassisControl(0.1, -M_PI_2);  
                step = 4;
                cout << "g_speed_controller.SetChassisControl(0.1, -M_PI_2)" << endl;
                break;
            case 4:
                g_speed_controller.SetChassisControl(0.3, 0.0);
                step = 0;
                cout << "g_speed_controller.SetChassisControl(0.1, 0.0)" << endl;
                break;
            default:
                break;
        }
        this_thread::sleep_for(chrono::milliseconds(2000));  
    }
}