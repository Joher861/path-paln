#include "thread_pool/ThreadPool.h"

#include "mpc_planner_log.h"
#include "mpc_planner_task.h"
#include "mpc_planner.h"
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

    cfg_mgr.LoadConfig("../../../../../../../pms/config/robot.yaml");
    cfg_mgr.LoadConfig("../../../../../../config/planning.yaml");
    // cfg_mgr.LoadConfig("Work/Project/k905/pms/config/robot.yaml");
    // cfg_mgr.LoadConfig("Work/Project/k905/planning/config/planning.yaml");

    g_robot_cfg = dynamic_cast<ConfigRobot*>(cfg_mgr.GetSubConfig(CONFIG_ROBOT));
    g_robot_cfg->setFoortprint();
    
    MPCPlannerPtr& mpc_planner = MPCPlanner::getInstance(&pool);

    g_dc.init(cfg_mgr);
    g_ec.init(cfg_mgr);
    g_speed_controller.init(cfg_mgr, [](float vl, float vr){});
    g_tm.init(cfg_mgr);
    mpc_planner->init(cfg_mgr);

    OPEN_LOG_EXCEPT();

    g_tm.registerPlanner<TaskMPC>(mpc_planner);
                
    int64_t mpc_task1_id;

    // test mpc_planner ===============================================================
    
    // pool.enqueue(
    //     [&]() mutable
    //     {
    //         CREATE_TASK(TaskMPC, test_mpc);

    //         PosePath test_path;
    //         RobotPose tmp_path_point;
    //         tmp_path_point.pt.x = 0.0;
    //         tmp_path_point.pt.y = 0.0;
    //         tmp_path_point.theta = 0.0;
    //         for (int i = 0; i < 30; i++)
    //         {
    //             tmp_path_point.pt.x = 0.3 * (i+1);
    //             test_path.add(tmp_path_point);
    //         }
    //         test_mpc->type = TaskMPC::MPC_FOLLOW;
    //         test_mpc->mpc_ref_path = test_path;
    //         test_mpc->start_segment_index = 0;

    //         // MPC_INFO_LOG("add task1");
    //         std::cout << "===============add task1===============" << std::endl;
    //         mpc_task1_id = g_tm.addTask(test_mpc);
    //     }
    // );

    pool.enqueue(
        [&]() mutable
        {
            CREATE_TASK(TaskMPC, test_mpc);
            PosePath test_path;
            PosePath test_path_tmp;
            // read file
            ifstream f;
            f.open("/home/CCPP_path_points_distance_no_minor_direcetion_542p.csv");
            while (!f.eof())
            {
                std::string s;
                getline(f, s);
                if (!s.empty())
                {
                    vector<string> ss = Split(s, ",");
                    // stringstream ss_1;
                    // ss_1 << s;
                    // cout << s << endl;
                    // string timestamp;
                    string pose_x = ss[0];
                    string pose_y = ss[1];
                    string pose_theta = ss[2];
                    // ss_1 >> timestamp;
                    // ss_1 >> pose_x;
                    // ss_1 >> pose_y;
                    // ss_1 >> pose_theta;
                    // 加入路径点中
                    RobotPose pose_single;
                    pose_single.pt.x = atof(pose_x.c_str());
                    pose_single.pt.y = atof(pose_y.c_str());
                    pose_single.theta = atof(pose_theta.c_str());
                    // cout << pose_single.x << " | " << pose_single.y << endl;
                    test_path_tmp.add(pose_single);
                }
            }
            f.close();
            std::cout << "test_path_tmp.size(): " << test_path_tmp.size() << std::endl;

            // for ccpp path points no_minor_direction_insert_ellipse only
            for (size_t i = 0; i < test_path_tmp.size(); i++)
            {
                // swap x&y because the map is wrong
                float pose_y = test_path_tmp[i].pt.x;
                float pose_x = test_path_tmp[i].pt.y;
                float pose_theta = test_path_tmp[i].theta;
                // multiply resolution
                pose_y *= 0.05;
                pose_x *= 0.05;

                // coordinate transformation
                if (pose_theta > 1.57 - 0.01 && pose_theta < 1.57 + 0.01)
                {
                    pose_theta = 0;
                }
                else if (pose_theta > 0 - 0.01 && pose_theta < 0 + 0.01)
                {
                    pose_theta = 1.57;
                }
                else if (pose_theta > -1.57 - 0.01 && pose_theta < -1.57 + 0.01)
                {
                    pose_theta = 3.14;
                }
                else
                {
                    pose_theta = -1.57;
                }

                RobotPose current_point;
                current_point.pt.x = pose_x;
                current_point.pt.y = pose_y;
                current_point.theta = pose_theta;
                //  cout << "show current point: " << i << "   " << pose_x << " " << pose_y << " " << pose_theta << endl;
                // cout << "show current point: " << i << "   " << current_point.x << " " << current_point.y << " " << current_point.theta << endl;

                // if not the last point
                if (i + 1 <= test_path_tmp.size() - 1)
                {
                    // check whether insert ellipse
                    float next_pose_y = test_path_tmp[i + 1].pt.x;
                    float next_pose_x = test_path_tmp[i + 1].pt.y;
                    next_pose_y *= 0.05;
                    next_pose_x *= 0.05;
                    float next_pose_theta = test_path_tmp[i + 1].theta;
                    float distance = hypot((next_pose_x - pose_x), (next_pose_y - pose_y));
                    if (distance > 0.5 && next_pose_theta != pose_theta) // here means another clean line
                    {
                        // insert ellipse
                        float a = distance / 2;
                        float b = 0.5; // adaptive to environment
                        float res = 0.1;
                        int N_ellipse = b / res;

                        // insert the first point on ellipse
                        test_path.add(current_point);

                        // ellipse part 1
                        for (int i = 1; i < N_ellipse; i++)
                        {
                            // ellipse local coordinates
                            float ellipse_local_x = i * res;
                            float ellipse_local_y_1 = -sqrt((1 - pow(ellipse_local_x, 2) / pow(b, 2)) * pow(a, 2)) + a;
                            float d_ellipse_local_y_1 = pow(a, 2) / pow(b, 2) * ellipse_local_x / (sqrt((1 - pow(ellipse_local_x, 2) / pow(b, 2)) * pow(a, 2)));
                            float ellipse_local_yaw_1 = atan(d_ellipse_local_y_1);

                            // translate to global coordinates
                            RobotPose ellipse_global_1;
                            ellipse_global_1.pt.x = pose_x + ellipse_local_x * cos(pose_theta) - ellipse_local_y_1 * sin(pose_theta);
                            ellipse_global_1.pt.y = pose_y + ellipse_local_y_1 * cos(pose_theta) + ellipse_local_x * sin(pose_theta);
                            ellipse_global_1.theta = toNPPiAngleRangeR(ellipse_local_yaw_1 + pose_theta);

                            // insert
                            test_path.add(ellipse_global_1);
                        }

                        // insert the top point on ellipse
                        RobotPose top_ellipse_point_local;
                        RobotPose top_ellipse_point_global;
                        top_ellipse_point_local.pt.x = b;
                        top_ellipse_point_local.pt.y = a;
                        top_ellipse_point_local.theta = 3.14 / 2;
                        top_ellipse_point_global.pt.x = pose_x + top_ellipse_point_local.pt.x * cos(pose_theta) - top_ellipse_point_local.pt.y * sin(pose_theta);
                        top_ellipse_point_global.pt.y = pose_y + top_ellipse_point_local.pt.y * cos(pose_theta) + top_ellipse_point_local.pt.x * sin(pose_theta);
                        top_ellipse_point_global.theta = toNPPiAngleRangeR(top_ellipse_point_local.theta + pose_theta);
                        test_path.add(top_ellipse_point_global);

                        // ellipse part 2
                        for (int i = N_ellipse - 1; i > 0; i--)
                        {
                            // ellipse local coordinates
                            float ellipse_local_x = i * res;
                            float ellipse_local_y_2 = sqrt((1 - pow(ellipse_local_x, 2) / pow(b, 2)) * pow(a, 2)) + a;
                            float d_ellipse_local_y_2 = -pow(a, 2) / pow(b, 2) * ellipse_local_x / (sqrt((1 - pow(ellipse_local_x, 2) / pow(b, 2)) * pow(a, 2)));
                            float ellipse_local_yaw_2 = atan(d_ellipse_local_y_2) + 3.14;

                            // translate to global coordinates
                            RobotPose ellipse_global_2;
                            ellipse_global_2.pt.x = pose_x + ellipse_local_x * cos(pose_theta) - ellipse_local_y_2 * sin(pose_theta);
                            ellipse_global_2.pt.y = pose_y + ellipse_local_y_2 * cos(pose_theta) + ellipse_local_x * sin(pose_theta);
                            ellipse_global_2.theta = toNPPiAngleRangeR(ellipse_local_yaw_2 + pose_theta);

                            // insert
                            test_path.add(ellipse_global_2);
                        }
                        // insert the last point on ellipse
                        // DataPose last_point_on_ellipse;
                        // last_point_on_ellipse.x = next_pose_x;
                        // last_point_on_ellipse.y = next_pose_y;
                        // last_point_on_ellipse.theta = next_pose_theta;
                        // data_robot.goal_path_points.push_back(last_point_on_ellipse);
                    }
                    else
                    {
                        test_path.add(current_point);
                    }
                }
                else
                {
                    test_path.add(current_point);
                }
            }
            std::cout << "test_path.size(): " << test_path.size() << std::endl;

            test_mpc->type = TaskMPC::MPC_FOLLOW;
            test_mpc->mpc_ref_path = test_path;
            test_mpc->start_segment_index = 0;

            std::cout << "===============add task2===============" << std::endl;
            mpc_task1_id = g_tm.addTask(test_mpc);
        }
    );

    SleepTimer t(1.0f);
    while (true)
    {
        t.sleep();
    }

    return 0;
}