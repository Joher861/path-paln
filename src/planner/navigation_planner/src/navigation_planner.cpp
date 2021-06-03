#include "navigation_planner.h"
#include "navigation_planner_events.h"
#include "navigation_planner_config.h"
#include "navigation_planner_log.h"
#include "hybrid_breadth_first.h"

#include "path_follow_planner/path_follow_planner.h"
#include "path_follow_planner/path_follow_planner_input.h"

#include "planner/global_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "task/task_manager.h"

#include "local_planner/local_planner_events.h"

#include "rotate_planner/rotate_task.h"
#include "basic_planner/basic_task.h"

#include "misc/planning_common_config.h"
#include "misc/robot_config.h"
#include "event/global_common_event_def.h"
#include "sys/stat.h"
#include "clean_map/clean_map.h"
#include "unistd.h"

using namespace planning_planner;
using namespace planning_utils;
using namespace planning_data;
using namespace planning_controller;
using namespace planning_map;

#define NAVITEST 0
#define SAVEFILE 1

#if NAVITEST
string saveFileAdress2 = "data/";
#else
string saveFileAdress2 = "/tmp/log/navigationLog/";
#endif

namespace planning_planner
{
    struct navigationPlannerStates
    { // StateWithOwner
        // Class that clients can use instead of deriving directly from State that provides convenient
        // typed access to the Owner. This class can also be chained via the StateBaseType parameter,
        // which is useful when inheriting state machines.
        struct BaseState : StateWithOwner<navigationPlanner>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter() // OnEnter is invoked when a State is created
            {
            }

            virtual void OnExit()
            {
                NAVIGATION_INFO_LOG("Exit Disable...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Enable>();
                }
                else
                {
                    return NoTransition();
                }
            }

            virtual void Update()
            {
                NAVIGATION_INFO_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                NAVIGATION_INFO_LOG("Enter Enable...");
            }

            virtual void OnExit()
            {
                NAVIGATION_INFO_LOG("Exit Enable...");
            }

            virtual Transition GetTransition()
            {
                if (!Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Disable>();
                }
                return InnerEntryTransition<NavigationPlannerGo>();
            }

            virtual void Update()
            {
                NAVIGATION_INFO_LOG("Enable...");
            }
        };
        struct NavigationPlannerGo : BaseState
        {
            virtual void OnEnter()
            {
                NAVIGATION_INFO_LOG("Enter NavigationPlannerGo...");
                Owner().dispatchThread();
                bool find_path = g_navigation_planner.Hybrid_A_star();

                PosePath m_global_path = g_navigation_planner.get_global_path();
                // Owner().Hybrid_A_start_path_output.Hybrid_A_star_path = m_global_path;
                if (find_path == true && m_global_path.size() > 2)
                {
                    // update到data_center
                    // g_dc.updateData<DataHybridAStar>(&Owner().Hybrid_A_start_path_output);
                    NAVIGATION_INFO_LOG("find global path");
                }
                else
                {
                    NAVIGATION_WARN_LOG("fail to find global path");
                    // g_dc.updateData<DataHybridAStar>(&Owner().Hybrid_A_start_path_output);
                }
            }

            virtual void OnExit()
            {
                NAVIGATION_INFO_LOG("Exit NavigationPlannerGo...");
                Owner().stopThread();
            }

            virtual Transition GetTransition()
            {

                return SiblingTransition<Finished>();
            }

            virtual void Update()
            {
                NAVIGATION_INFO_LOG("NavigationPlannerGoUpdate");
            }
        };

        struct Finished : BaseState
        {
            virtual void OnEnter()
            {
                Owner().finishPlanning();
                NAVIGATION_INFO_LOG("set stop flag, change to Idle");
                Owner().m_stop_flag = true;
                Owner().stopThread();
            }

            virtual void OnExit()
            {
                NAVIGATION_INFO_LOG("Exit Finished");
            }

            virtual Transition GetTransition()
            {
                return NoTransition();
            }
        };
    };
} // namespace planning_planner

DEFINE_CONFIG_TYPE(CONFIG_NAVIGATION_PLANNER, NavigationPlanner);
navigationPlanner &g_navigation_planner = navigationPlanner::getInstance();

float navigationPlanner::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigNavigationPlanner *cfg_np = dynamic_cast<ConfigNavigationPlanner *>(
        cfg_mgr.GetSubConfig(CONFIG_NAVIGATION_PLANNER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));

    cfg_np->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_NAVIGATION_PLANNER_FLAG, LOG_NAVIGATION_PLANNER,
               cfg_np->log_name, cfg_np->log_path,
               cfg_np->log_extension, cfg_np->log_ts_mask,
               cfg_np->log_print_to_console,
               (cfg_np->log_max_file_size_mb)MB + (cfg_np->log_max_file_size_kb)KB,
               cfg_np->log_max_file_cnt, cfg_np->log_level);

    return cfg_np->planning_frequency;
}

bool navigationPlanner::handleInput(const GlobalPlannerInputPtr input)
{
    NavigationPlannerInputPtr navigationPlanner_input = std::dynamic_pointer_cast<NavigationPlannerInput>(input);

    safe_distance = navigationPlanner_input->safe_distance;
    AStarGridScale = navigationPlanner_input->AStarGridScale;

    START[0] = navigationPlanner_input->start.pt.x;
    START[1] = navigationPlanner_input->start.pt.y;
    START[2] = navigationPlanner_input->start.theta;

    GOAL[0] = navigationPlanner_input->goal.pt.x;
    GOAL[1] = navigationPlanner_input->goal.pt.y;
    GOAL[2] = navigationPlanner_input->goal.theta;

    return true;
}

void navigationPlanner::initRunSM()
{
    m_run_sm.Initialize<navigationPlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("navigationPlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void navigationPlanner::reset()
{
}

void navigationPlanner::stopRobot()
{
}

ContextPtr navigationPlanner::saveContext()
{
}

Transition navigationPlanner::restoreContext(ContextPtr ctx)
{
}

void navigationPlanner::getData()
{
}

void navigationPlanner::finishPlanning()
{
    stopRobot();
    NAVIGATION_INFO_LOG("navigation_planner navigationPlanner::finishPlanning()   put_event");

    CREATE_EVENT(EvPlannerFinished, ev_navigation_finished);
    ev_navigation_finished->id = m_input_id;
    g_ec.pushEvent(ev_navigation_finished);
}

bool navigationPlanner::Hybrid_A_star()
{
    Hybrid_A_start_path.clear();

#if SAVEFILE
    long curTime = Timer::getSystemTimestampUS();
    string time = std::to_string(curTime);
    if (access(saveFileAdress2.c_str(), 00) != 0)
    {
        mkdir(saveFileAdress2.c_str(), 0777);
        chmod(saveFileAdress2.c_str(), 0777);
    }
    ofstream SaveFile_navigation(saveFileAdress2 + time + "navigationPlanner.txt");
    string SaveFileImg = saveFileAdress2 + time + "image.png";
    string SaveFileImgPre = saveFileAdress2 + time + "imagePre.png";

    gridOriginX = -(g_clean_map.getMapInfo().origin_pt.x / g_clean_map.getMapInfo().resolution);
    gridOriginY = -(g_clean_map.getMapInfo().origin_pt.y / g_clean_map.getMapInfo().resolution);
    gridPrecision = g_clean_map.getMapInfo().resolution;

    SaveFile_navigation << "origin " << gridOriginX << " " << gridOriginY << " " << gridPrecision << endl;
    SaveFile_navigation << "start_raw " << START[0] << " " << START[1] << " " << START[2] << endl;
    SaveFile_navigation << "goal_raw " << GOAL[0] << " " << GOAL[1] << " " << GOAL[2] << endl;
    SaveFile_navigation << "centerIndex sacle " << safe_distance << " " << AStarGridScale << endl;
#endif

    MapSize map_size = g_clean_map.getMapInfo().size;

    uint8_t **slamMap = new uint8_t *[map_size.x];
    for (int i = 0; i < map_size.x; i++)
    {
        slamMap[i] = new uint8_t[map_size.y];
    }

    for (int i = 0; i < map_size.x; i++)
    {
        for (int j = 0; j < map_size.y; j++)
        {
            RobotGrid grid;
            unsigned char temp;
            grid.x = i;
            grid.y = j;
            g_clean_map.get("slam", grid, temp);
            slamMap[i][j] = temp;
        }
    }

    uint8_t **localMap = new uint8_t *[map_size.x];
    for (int i = 0; i < map_size.x; i++)
    {
        localMap[i] = new uint8_t[map_size.y];
    }
    g_clean_map.getLocalMap(localMap);

    cv::Mat map_origin(map_size.x, map_size.y, CV_8UC1, cv::Scalar(0, 0, 0));
    for (int i = 0; i < map_size.x; i++)
    {
        for (int j = 0; j < map_size.y; j++)
        {
            if ((slamMap[i][j] == 255) && localMap[i][j] != 255)
                map_origin.at<unsigned char>(map_size.x - 1 - i, map_size.y - 1 - j) = 255; //no obstacle
            else
                map_origin.at<unsigned char>(map_size.x - 1 - i, map_size.y - 1 - j) = 0; 
        }
    }

    cv::Size dsize = cv::Size(map_origin.cols * AStarGridScale, map_origin.rows * AStarGridScale);
    cv::Mat image_scale = cv::Mat(dsize, 0);
    cv::resize(map_origin, image_scale, dsize, 0, 0, cv::INTER_NEAREST);

    cv::Mat room_map_hybrid = image_scale;

    // gird_map: Hybrid A* 

    vector<vector<int>> gird_map(room_map_hybrid.rows, vector<int>(room_map_hybrid.cols));
    MapInfo mapInfo = g_clean_map.getMapInfo();
    float erodeDis = 0.2;
    int temp2 = (erodeDis / mapInfo.resolution * AStarGridScale);
    int temp = ceil(safe_distance / mapInfo.resolution * AStarGridScale);
    temp2 = max(temp2, 1);
    cv::morphologyEx(room_map_hybrid, room_map_hybrid, cv::MORPH_ERODE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(temp2, temp2)));

    cv::morphologyEx(room_map_hybrid, room_map_hybrid, cv::MORPH_DILATE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(temp2, temp2)));
    if (temp > 0)
        cv::morphologyEx(room_map_hybrid, room_map_hybrid, cv::MORPH_ERODE,
                         cv::getStructuringElement(cv::MORPH_RECT, cv::Size(temp, temp)));

    for (int i = 0; i < room_map_hybrid.rows; i++)
    {
        for (int j = 0; j < room_map_hybrid.cols; j++)
        {
            if (image_scale.at<unsigned char>(i, j) == 255)
            {
                image_scale.at<unsigned char>(i, j) = 255;
                gird_map[room_map_hybrid.rows - 1 - i][room_map_hybrid.cols - 1 - j] = 0; // 没有障碍物
            }
            else
            {
                image_scale.at<unsigned char>(i, j) = 0;
                gird_map[room_map_hybrid.rows - 1 - i][room_map_hybrid.cols - 1 - j] = 1; // 有障碍物
            }
        }
    }
    // 判断起点和终点是否有效

    std::vector<double> gridStart = {0, 0, 0};
    std::vector<double> gridGoal = {0, 0, 0};

    gridStart[0] = START[0] / gridPrecision * AStarGridScale + gridOriginX * AStarGridScale;
    gridStart[1] = START[1] / gridPrecision * AStarGridScale + gridOriginY * AStarGridScale;
    gridStart[2] = START[2];

    gridGoal[0] = GOAL[0] / gridPrecision * AStarGridScale + gridOriginX * AStarGridScale;
    gridGoal[1] = GOAL[1] / gridPrecision * AStarGridScale + gridOriginY * AStarGridScale;
    gridGoal[2] = GOAL[2];

    cv::Mat image_test = cv::Mat(dsize, 0);
    for (int i = 0; i < room_map_hybrid.rows; i++)
    {
        for (int j = 0; j < room_map_hybrid.cols; j++)
        {
            if (gird_map[room_map_hybrid.rows - 1 - i][room_map_hybrid.cols - 1 - j] == 1)
                image_test.at<unsigned char>(i, j) = 0;
            else
            {
                image_test.at<unsigned char>(i, j) = 255;
            }
        }
    }
    int x = room_map_hybrid.rows - gridStart[0];
    int y = room_map_hybrid.cols - gridStart[1];
    cv::Point p1(y, x);
    cv::circle(image_test, p1, 3, cv::Scalar(100), -1);
    cv::line(image_test, cv::Point(room_map_hybrid.rows / 2, 0), cv::Point(room_map_hybrid.rows / 2, room_map_hybrid.cols), cv::Scalar(150), 1, cv::LINE_AA);
    cv::line(image_test, cv::Point(0, room_map_hybrid.cols / 2), cv::Point(room_map_hybrid.rows, room_map_hybrid.cols / 2), cv::Scalar(150), 1, cv::LINE_AA);

    x = room_map_hybrid.rows - gridGoal[0];
    y = room_map_hybrid.cols - gridGoal[1];
    cv::Point p2(y, x);

    cv::circle(image_test, p2, 3, cv::Scalar(100), -1);

#if SAVEFILE
    SaveFile_navigation << "map_origin.rows  " << map_origin.rows << "  map_origin.cols  " << map_origin.cols << std::endl;
    SaveFile_navigation << "image_scale.rows  " << image_scale.rows << "  image_scale.cols  " << image_scale.cols << std::endl;
    SaveFile_navigation << "start_grid " << gridStart[0] << " " << gridStart[1] << " " << gridStart[2] << endl;
    SaveFile_navigation << "goal_grid " << gridGoal[0] << " " << gridGoal[1] << " " << gridGoal[2] << endl;

#endif
    imwrite(SaveFileImgPre, image_test);

    if (floor(gridStart[0]) >= (int)gird_map.size() || floor(gridStart[1]) >= (int)gird_map[0].size() || floor(gridStart[0]) < 0 || floor(gridStart[1]) < 0)
    {
        printf("Error, the start_point is out of range\n");
        return false;
    }

    if (floor(gridGoal[0]) > (int)gird_map.size() || floor(gridGoal[1]) > (int)gird_map[0].size() || floor(gridGoal[0]) < 0 || floor(gridGoal[1]) < 0)
    {
        printf("Error, the goal_point is out of range\n");
        return false;
    }

    if ((gird_map[floor(gridStart[0])][floor(gridStart[1])] != 0))
    {
        printf("Error, the start_point is aroung abstacle \n");
        return false;
    }

    // 判断起点和终点是否有效
    if ((gird_map[floor(gridGoal[0])][floor(gridGoal[1])] != 0))
    {
        printf("Error, the goal_point is aroung abstacle \n");
        return false;
    }

    HBF hbf = HBF();

    std::chrono::system_clock::system_clock::time_point start_time =
        std::chrono::system_clock::system_clock::now();

    float agvLength = 1.2;
    float agvWidth = 0.7;
    float AStarPrecision = gridPrecision / AStarGridScale;

    vector<HBF::maze_s> show_path = hbf.hybrid_a_star(gird_map, gridStart, gridGoal, AStarPrecision, agvLength, agvWidth, g_clean_map.getMapInfo().origin_pt.x, g_clean_map.getMapInfo().origin_pt.y);

    std::chrono::system_clock::system_clock::time_point end_time =
        std::chrono::system_clock::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
#if SAVEFILE
    SaveFile_navigation << "time spent:" << elapsed_seconds.count() << " s" << std::endl;
#endif

    vector<HBF::maze_s> pathDense;
    HBF::maze_s pathTemp;
    float perPathDis = 0.06;
    float perPathDisTemp = 0;
    if ((int)show_path.size() > 2)
    {
        pathDense.push_back(show_path.front());
        for (int i = 1; i < (int)show_path.size();)
        {
            float xTemp = show_path[i].x;
            float yTemp = show_path[i].y;
            float thetaTemp = show_path[i].theta;

            float xLastTemp = show_path[i - 1].x;
            float yLastTemp = show_path[i - 1].y;
            float thetaLastTemp = show_path[i - 1].theta;

            float x_last = pathDense.back().x;
            float y_last = pathDense.back().y;
            float theta_last = pathDense.back().theta;

            float dis = sqrt((xTemp - xLastTemp) * (xTemp - xLastTemp) + (yTemp - yLastTemp) * (yTemp - yLastTemp));
            if (dis - perPathDisTemp > perPathDis)
            {
                perPathDisTemp = perPathDisTemp + perPathDis;
                float scale = perPathDisTemp / dis;
                pathTemp.x = xLastTemp + (xTemp - xLastTemp) * scale;
                pathTemp.y = yLastTemp + (yTemp - yLastTemp) * scale;
                float deltaTheta = thetaTemp - thetaLastTemp;


                pathTemp.theta = thetaLastTemp;
                pathDense.push_back(pathTemp);
            }
            else
            {
                perPathDisTemp = perPathDisTemp - dis;
                i++;
            }
        }
        pathTemp.x = GOAL[0];
        pathTemp.y = GOAL[1];
        pathTemp.theta = show_path.back().theta;
        pathDense.push_back(pathTemp);
        pathTemp.theta = GOAL[2];
        pathDense.push_back(pathTemp);
    }

    int thetaFiliterNum = 3;
    for (int i = 0; i < (int)pathDense.size(); i++)
    {
        Pose pose_tmp;
        pose_tmp.pt.x = pathDense[i].x;
        pose_tmp.pt.y = pathDense[i].y;
        if (i <= (int)pathDense.size() - 1 - thetaFiliterNum)
            pose_tmp.theta = atan2(pathDense[i + thetaFiliterNum].y - pathDense[i].y, pathDense[i + thetaFiliterNum].x - pathDense[i].x);
        else
        {
            if ((int)pathDense.size() == 1)
            {
                pose_tmp.theta = 0;
            }
            else if (i == (int)pathDense.size() - 1)
                pose_tmp.theta = pathDense[i].theta;
            else
            {
                pose_tmp.theta = atan2(pathDense[i].y - pathDense[i-1].y, pathDense[i].x - pathDense[i-1].x);
            }
        }

        Hybrid_A_start_path.add(pose_tmp);
    }
    NAVIGATION_INFO_LOG("end_A_star");

    if ((int)Hybrid_A_start_path.size() > 2)
    {
    }
    else
    {
        return false;
    }

    if ((int)show_path.size() > 2)
    {
        return true;
    }
    else
    {
        return false;
    }
}
