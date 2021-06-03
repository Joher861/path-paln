#include "mpc_planner.h"
#include "mpc_planner_log.h"
#include "mpc_planner_config.h"

#include "local_planner/local_planner_events.h"
#include "path_follow_planner/path_follow_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "speed_controller/speed_controller.h"
#include "misc/planning_common_config.h"
#include "misc/robot_config.h"

#include "emergency_stop_detector/emergency_stop_detector.h"
#include "obstacle_detector/obstacle_detector.h"

// #include "IpIpoptApplication.hpp"
// #include "IpSolveStatistics.hpp"
#include "robot_nlp.hpp"

using namespace planning_controller;
using namespace planning_utils;
using namespace std;

#define LOG_OUTPUT
// #define SOLO_TEST

#ifdef LOG_OUTPUT
ofstream MPC_DEBUG_LOG_OUTPUT;
std::string filename_mpc = "/tmp/log/pms/MPC_DEBUG_LOG_OUTPUT.csv";
#endif

namespace planning_controller
{
    struct MPCPlannerStates
    {
        struct BaseState : StateWithOwner<MPCPlanner>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Enter Disable...");
            }

            virtual void OnExit()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Exit Disable...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Enable>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Disable...");
            }
        };

        struct Enable : BaseState
        {
            uint64_t last_ts_enable = Timer::getSystemTimestampUS();
            bool path_follow_go_exit_flag = false;

            virtual void OnEnter()
            {
                MPC_INFO_LOG(COLOR_L_GREEN "Enter Enable...");
                Owner().dispatchThread();
            }

            virtual void OnExit()
            {
                MPC_INFO_LOG(COLOR_L_GREEN "Exit Enable...");
                // uncomment for debug
                Owner().stopThread();
                if (path_follow_go_exit_flag) // 因为pfp停了退出的话，要把速度置0
                {
                    g_speed_controller.SetStop(true);
                }
            }

            virtual Transition GetTransition()
            {
                if (!Owner().m_run_loop_enabled || path_follow_go_exit_flag)
                {
                    return SiblingTransition<Disable>();
                }

                if (Owner().m_global_mpc_path.empty())
                {
                    return InnerEntryTransition<StopEmptyInput>();
                }
                else if (Owner().m_type == TaskMPC::MPC_FOLLOW)
                {
                    return InnerEntryTransition<DoMPC>();
                }
                return NoTransition();
            }

            virtual void Update()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Enable...");
                // path_follow_go_exit_flag = false;
                // check events
                EvtList evt_list_enable;
                evt_list_enable.clear();
                uint64_t cur_ts_enable = Timer::getSystemTimestampUS();
                // uint64_t delta_t_enable = cur_ts_enable - last_ts_enable;
                // MPC_DEBUG_LOG(COLOR_L_GREEN "last_ts_enable:  %u    cur_ts_enable:  %u    delta_t_enable:  %u", last_ts_enable, cur_ts_enable, delta_t_enable);
                g_ec.eventDeduction(last_ts_enable, cur_ts_enable, evt_list_enable);
                last_ts_enable = cur_ts_enable;
                for (auto &evt : evt_list_enable)
                {
                    std::type_index evt_type = evt->getType();
                    if (TYPE_EQUALS(evt_type, EvPathFollowGoExit)) // Path_Follow_Go Exit
                    {
                        MPC_WARN_LOG(COLOR_L_GREEN "=============Path_Follow_Go Exit=============");
                        path_follow_go_exit_flag = true;
                    }
                }
            }
        };

        struct DoMPC : BaseState
        {
            vector<float> robot_current_pose;
            vector<MPCRefPoints> local_mpc_path;
            uint64_t mpc_ref_path_size;
            // vector<bool> path_reached;
            // vector<bool> path_skipped;
            vector<int> path_points_states;
            uint64 last_skipped_index = -1;
            uint64 last_reached_index = -1;
            float small_velocity = 0.1; // 0.001

            vector<float> X_OPT_out;
            vector<float> opt_global_out;

            float mpc_linear_vel;
            float mpc_angular_steer;

            uint64_t last_ts = Timer::getSystemTimestampUS();

            virtual void OnEnter()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Enter DoMPC...");
#ifdef LOG_OUTPUT
#ifdef SOLO_TEST
                MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::ate);
                MPC_DEBUG_LOG_OUTPUT.close();
#else
                MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
                MPC_DEBUG_LOG_OUTPUT.close();
#endif
#endif
                mpc_ref_path_size = Owner().m_global_mpc_path.size();
                Owner().m_final_path_point_arrived = false;
                Owner().m_final_path_point_infeasible = 0;
                MPC_DEBUG_LOG(COLOR_L_GREEN "mpc_ref_path_size: %u", mpc_ref_path_size);

                for (size_t i = 0; i < mpc_ref_path_size; i++)
                {
                    if (i < Owner().m_start_segment_index)
                    {
                        path_points_states.push_back(9);
                    }
                    else
                    {
                        path_points_states.push_back(0);
                    }
                }

                // ipopt init
                Owner().app = IpoptApplicationFactory();
                // Owner().app->Options()->SetStringValue("option_file_name", "/tmp/config/ipopt.opt"); // ipopt path
                // app->Options()->SetStringValue("print_level", "0");
                // app->Options()->SetStringValue("hassian_approximation", "limited-memory");
                // app->Options()->SetStringValue("limited_memory_update_type", "bfgs");
                Owner().app->Options()->SetNumericValue("max_cpu_time", (double)Owner().m_max_ipopt_cpu_time);
                Owner().status = Owner().app->Initialize("/tmp/config/ipopt.opt", true);
            }

            virtual void OnExit()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Exit DoMPC...");
                // clear
                // path_skipped.clear();
                // path_reached.clear();
                path_points_states.clear();
                robot_current_pose.clear();
                local_mpc_path.clear();
                // Owner().m_global_mpc_path.clear();
                Owner().m_type = TaskMPC::MAX_MPC_TYPE;
            }

            virtual Transition GetTransition()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Enter DoMPC Transition...");

                if (!Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Disable>();
                }
                else if (Owner().m_final_path_point_infeasible != 0)
                {
                    return SiblingTransition<Infeasible>();
                }
                else if (Owner().m_final_path_point_arrived)
                {
                    return SiblingTransition<Finished>();
                }
                else if (Owner().m_type != TaskMPC::MPC_FOLLOW)
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
                MPC_INFO_LOG(COLOR_L_GREEN "================================== Enter DoMPC Update ==================================");
                uint64_t update_start_ts = Timer::getTimestampUS();
#ifdef LOG_OUTPUT
                MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
                struct timespec curtime;
                clock_gettime(CLOCK_MONOTONIC, &curtime);
                uint64_t mpc_debug_log_ts = curtime.tv_sec * 1e6 + curtime.tv_nsec / 1e3;
                MPC_DEBUG_LOG_OUTPUT << mpc_debug_log_ts << " ";
                MPC_DEBUG_LOG_OUTPUT << Owner().m_slam_data.pose.pt.x << " " << Owner().m_slam_data.pose.pt.y << " " << rad2deg(Owner().m_slam_data.pose.theta) << " ";
                MPC_DEBUG_LOG_OUTPUT.close();
#endif

                bool mpc_anomaly_output = false;
                bool emergency_stop_flag = false;
                bool emergency_slowdown_flag = false;
                float us_slowdown_discount = 1.0;
                float lm_slowdown_discount = 1.0;

                // update robot pose
                robot_current_pose.push_back(Owner().m_slam_data.pose.pt.x);
                robot_current_pose.push_back(Owner().m_slam_data.pose.pt.y);
                robot_current_pose.push_back(Owner().m_slam_data.pose.theta);
                MPC_INFO_LOG(COLOR_L_GREEN "Get Robot Current Pose: %f %f %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, Owner().m_slam_data.pose.theta);

                size_t selected_index = mpc_ref_path_size - 1;
                size_t selected_index_for_infeasible = 0;

                // get start index
                for (size_t i = 0; i < mpc_ref_path_size; i++)
                {
                    if (path_points_states[i] == 0)
                    {
                        selected_index = i;
                        // check if arrived selected_index
                        float distance_to_selected_point = sqrtf((powf((Owner().m_global_mpc_path[selected_index].pt.y - robot_current_pose[1]), 2) + powf((Owner().m_global_mpc_path[selected_index].pt.x - robot_current_pose[0]), 2)));
                        if (abs(distance_to_selected_point) <= Owner().m_search_radius)
                        {
                            path_points_states[selected_index] = 1;
                            last_reached_index = selected_index;
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                Owner().m_selected_index = selected_index;

                // check if arrived last point
                float distance_to_last_point = hypot((robot_current_pose[0] - Owner().m_global_mpc_path[mpc_ref_path_size - 1].pt.x), (robot_current_pose[1] - Owner().m_global_mpc_path[mpc_ref_path_size - 1].pt.y));
                if (distance_to_last_point < Owner().m_final_radius && selected_index == mpc_ref_path_size - 1)
                {
                    g_speed_controller.SetStop(true);
                    Owner().m_final_path_point_arrived = true;
                    MPC_WARN_LOG(COLOR_L_GREEN "==================final path point arrived=====================");
                }
                else
                {
                    Owner().prepare_data_for_mpc(Owner().m_global_mpc_path, selected_index, robot_current_pose, local_mpc_path, Owner().m_N, path_points_states);
                    MPC_DEBUG_LOG(COLOR_L_GREEN "last_skipped_index %ld   last_reached_index %ld ", last_skipped_index, last_reached_index);
                    Owner().do_mpc(robot_current_pose, local_mpc_path, X_OPT_out, opt_global_out);
                    // check if infeasible point
                    if (selected_index == mpc_ref_path_size - 1 && X_OPT_out[Owner().m_NX] < small_velocity) // optimized velocity == 0;
                    {
                        MPC_INFO_LOG(COLOR_L_GREEN "=================final point infeasible========================");
                        path_points_states[selected_index] = 2;
                        last_skipped_index = selected_index;
                        Owner().m_final_path_point_infeasible = 1;
                    }
                    else if (selected_index < mpc_ref_path_size - 1 && X_OPT_out[Owner().m_NX] < small_velocity) // optimized velocity == 0;
                    {
                        MPC_INFO_LOG(COLOR_L_GREEN "===============infeasible point: %u", selected_index);
                        path_points_states[selected_index] = 2;
                        last_skipped_index = selected_index;
                        // for next iter
                        selected_index += 1;
                        selected_index_for_infeasible = selected_index;

                        // try to find feasible point
                        for (size_t i = 0; selected_index_for_infeasible < mpc_ref_path_size; i++)
                        {
                            selected_index_for_infeasible = selected_index + i;
                            MPC_INFO_LOG(COLOR_L_GREEN "===============loop to find feasible point=====================");
                            Owner().prepare_data_for_mpc(Owner().m_global_mpc_path, selected_index_for_infeasible, robot_current_pose, local_mpc_path, Owner().m_N, path_points_states);
                            Owner().do_mpc(robot_current_pose, local_mpc_path, X_OPT_out, opt_global_out);
                            if (selected_index_for_infeasible == mpc_ref_path_size - 1 && X_OPT_out[Owner().m_NX] < small_velocity)
                            {
                                MPC_INFO_LOG(COLOR_L_GREEN "=========final point infeasible(in find_feasible loop)=========");
                                path_points_states[selected_index_for_infeasible] = 2;
                                last_skipped_index = selected_index_for_infeasible;
                                Owner().m_final_path_point_infeasible = 1;
                            }
                            else if (selected_index_for_infeasible < mpc_ref_path_size - 1 && X_OPT_out[Owner().m_NX] < small_velocity)
                            {
                                path_points_states[selected_index_for_infeasible] = 2;
                                MPC_INFO_LOG(COLOR_L_GREEN "============not feasible: %u", selected_index_for_infeasible);
                            }
                            else
                            {
                                selected_index = selected_index_for_infeasible;
                                MPC_INFO_LOG(COLOR_L_GREEN "============feasible found: %u", selected_index);
                                break;
                            }
                        }
                    }

                    // mpc_output anomaly detection
                    // MPC_DEBUG_LOG(COLOR_L_GREEN "Enter anomaly detection ============");
                    mpc_linear_vel = X_OPT_out[Owner().m_NX];
                    mpc_angular_steer = X_OPT_out[Owner().m_NX + 1];
                    if (mpc_linear_vel >= Owner().m_v_upper_bound &&
                        (mpc_angular_steer >= Owner().m_delta_upper_bound || mpc_angular_steer <= Owner().m_delta_lower_bound))
                    {
                        // anomaly
                        // g_speed_controller.SetStop(true);
                        mpc_anomaly_output = true;
                        MPC_WARN_LOG(COLOR_L_GREEN "=========mpc output anomaly detected: dangerous output=========");
                        // marked as infeasible, request for new path
                        // Owner().m_final_path_point_infeasible = 1;
                        // selected_index++;
                        // path_skipped[selected_index] = true;
                        path_points_states[selected_index] = 5;
                    }
                    else if (mpc_linear_vel < small_velocity)
                    {
                        // g_speed_controller.SetStop(true);
                        mpc_anomaly_output = true;
                        MPC_WARN_LOG(COLOR_L_GREEN "=========mpc output anomaly detected: small output=========");
                        // selected_index++;
                        // path_skipped[selected_index] = true;
                        path_points_states[selected_index] = 6;
                    }
                    else
                    {
                        MPC_INFO_LOG(COLOR_L_GREEN "The first pair control variables, not send yet==================");
                        MPC_INFO_LOG(COLOR_L_GREEN "v[0]: %f    delta[0]: %f  %f", mpc_linear_vel, mpc_angular_steer, rad2deg(mpc_angular_steer));
                    }

                    if (selected_index == mpc_ref_path_size - 1 && mpc_anomaly_output)
                    {
                        Owner().m_final_path_point_infeasible = 2;
                    }
#ifdef LOG_OUTPUT
                    MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
                    MPC_DEBUG_LOG_OUTPUT << mpc_linear_vel << " " << rad2deg(mpc_angular_steer) << " ";
                    MPC_DEBUG_LOG_OUTPUT.close();
#endif

                    // check events
                    EvtList evt_list;
                    evt_list.clear();
                    uint64_t cur_ts = Timer::getSystemTimestampUS();
                    last_ts = cur_ts - Owner().m_ev_deduction_period_for_us; // 3s
                    // uint64_t delta_t = cur_ts - last_ts;
                    // MPC_DEBUG_LOG(COLOR_L_GREEN "last_ts:  %u    cur_ts:  %u    delta_t:  %u", last_ts, cur_ts, delta_t);
                    g_ec.eventDeduction(last_ts, cur_ts, evt_list);

                    for (auto &evt : evt_list)
                    {
                        std::type_index evt_type = evt->getType();
                        if (TYPE_EQUALS(evt_type, EvEmergencyStop)) // US Emergency Stop
                        {
                            // MPC_INFO_LOG(COLOR_L_GREEN "===============US Emergency Stop===============");
                            emergency_stop_flag = true;
                        }
                        if (TYPE_EQUALS(evt_type, EvEmergencySlowdown)) // US Emergency Slowdown
                        {
                            // MPC_INFO_LOG(COLOR_L_GREEN "=============US Emergency Slowdown=============");
                            emergency_slowdown_flag = true;
                            EvEmergencySlowdownPtr slowdown_esd_evt = dynamic_pointer_cast<EvEmergencySlowdown>(evt);
                            us_slowdown_discount = slowdown_esd_evt->discount;
                        }
                        if (TYPE_EQUALS(evt_type, EvLMEmergencyStop)) // LM Emergency Stop
                        {
                            // MPC_INFO_LOG(COLOR_L_GREEN "===============LM Emergency Stop===============");
                            emergency_stop_flag = true;
                        }
                        if (TYPE_EQUALS(evt_type, EvLMEmergencySlowdown)) // LM Emergency Slowdown
                        {
                            // MPC_INFO_LOG(COLOR_L_GREEN "=============LM Emergency Slowdown=============");
                            emergency_slowdown_flag = true;
                            EvLMEmergencySlowdownPtr slowdown_od_evt = dynamic_pointer_cast<EvLMEmergencySlowdown>(evt);
                            lm_slowdown_discount = slowdown_od_evt->discount;
                        }
                    }

                    if (emergency_stop_flag)
                    {
                        g_speed_controller.SetStopKeepSteering(true);
                        MPC_WARN_LOG(COLOR_L_GREEN "=============== Emergency Stop ===============");
                    }
                    else if (mpc_anomaly_output)
                    {
                        g_speed_controller.SetStopKeepSteering(true);
                        MPC_WARN_LOG(COLOR_L_GREEN "=============== Anomaly Output ===============");
                    }
                    else if (!emergency_stop_flag && emergency_slowdown_flag)
                    {
                        float discount = min(us_slowdown_discount, lm_slowdown_discount);
                        if (mpc_linear_vel * discount > small_velocity)
                        {
                            g_speed_controller.SetChassisControl(mpc_linear_vel * discount, mpc_angular_steer, true);
                        }
                        else
                        {
                            g_speed_controller.SetChassisControl(small_velocity, mpc_angular_steer, true);
                        }

                        MPC_WARN_LOG(COLOR_L_GREEN "============= Emergency Slowdown: discount %0.2f =============", discount);
                    }
                    else
                    {
                        g_speed_controller.SetChassisControl(mpc_linear_vel, mpc_angular_steer, true);
                        MPC_WARN_LOG(COLOR_L_GREEN "v[0]: %f  %f   delta[0]: %f  %f", mpc_linear_vel, 100 * mpc_linear_vel / Owner().m_v_upper_bound, mpc_angular_steer, rad2deg(mpc_angular_steer));
                        MPC_INFO_LOG(COLOR_L_GREEN "Control variables sent============================");
                    }
                }

                // create event for upper planner
                CREATE_EVENT(EvCurIndexInCurTask_MPC, ev_cur_index_in_cur_task);
                // MPC_INFO_LOG(COLOR_L_GREEN "selected_index:  %u   selected_index_for_infeasible:  %u", selected_index, selected_index_for_infeasible);
                ev_cur_index_in_cur_task->cur_index_in_cur_task = max(selected_index, selected_index_for_infeasible);
                ev_cur_index_in_cur_task->global_path_index = Owner().m_global_path_index;
                ev_cur_index_in_cur_task->path_points_states = path_points_states;
                g_ec.pushEvent(ev_cur_index_in_cur_task);
                MPC_INFO_LOG(COLOR_L_GREEN "CurIndexInCurTask_MPC Event pushed: %u", ev_cur_index_in_cur_task->cur_index_in_cur_task);

                // for next iter
                robot_current_pose.clear();

                // send reached and skipped information to map here

#ifdef LOG_OUTPUT
                MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
                MPC_DEBUG_LOG_OUTPUT << "\n";
                MPC_DEBUG_LOG_OUTPUT.close();
#endif

                uint64_t update_end_ts = Timer::getTimestampUS();
                float update_period = float(update_end_ts - update_start_ts) / 1000000;
                if (update_period > Owner().m_T_MPC)
                {
                    MPC_WARN_LOG(COLOR_L_RED "update_period:  %f/%f", update_period, Owner().m_T_MPC);
                }
            }
        };

        struct StopEmptyInput : BaseState
        {
            virtual void OnEnter()
            {
                MPC_WARN_LOG(COLOR_L_GREEN "Enter StopEmptyInput,...");
                g_speed_controller.SetStopKeepSteering(true);
                // CREATE_EVENT();
                // g_ec.pushEvent();
            }

            virtual void OnExit()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Exit StopEmptyInput...");
            }

            virtual Transition GetTransition()
            {
                return NoTransition();
            }

            // virtual void Update()
            // {
            //     MPC_DEBUG_LOG(COLOR_L_GREEN "StopEmptyInput Update...");
            // }
        };

        struct Infeasible : BaseState
        {
            virtual void OnEnter()
            {
                MPC_WARN_LOG(COLOR_L_GREEN "Enter Infeasible, Final Path Point Infeasible, Push Event");
                // send infeasible event or do something here, for future work
                g_speed_controller.SetStop(true);
                CREATE_EVENT(EvTaskPathInfeasible_MPC, ev_task_path_infeasible);
                ev_task_path_infeasible-> id = Owner().m_final_path_point_infeasible;
                g_ec.pushEvent(ev_task_path_infeasible);
            }

            virtual void OnExit()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Exit Infeasible...");
            }

            virtual Transition GetTransition()
            {
                // 应该加跳出逻辑，比如新任务下发后应该能跳出执行新任务
                if (Owner().m_type == TaskMPC::MPC_FOLLOW)
                {
                    return SiblingTransition<DoMPC>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Infeasible Update...");
            }
        };

        struct Finished : BaseState
        {
            virtual void OnEnter()
            {
                MPC_INFO_LOG(COLOR_L_GREEN "Enter Finished, MPC task finished, TaskFinished_MPC Event pushed");
                Owner().finishTask();

                MPC_INFO_LOG(COLOR_L_GREEN "set stop flag, change to Idle");
                Owner().m_stop_flag = true;

                CREATE_EVENT(EvTaskFinished_MPC, ev_task_mpc_finished);
                g_ec.pushEvent(ev_task_mpc_finished);
            }

            virtual void OnExit()
            {
                MPC_DEBUG_LOG(COLOR_L_GREEN "Exit Finished");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_type == TaskMPC::MPC_FOLLOW)
                {
                    return SiblingTransition<DoMPC>();
                }
                return NoTransition();
            }
        };
    };
} // namespace planning_controller

DEFINE_CONFIG_TYPE(CONFIG_MPC_PLANNER, MPCPlanner);

MPCPlannerPtr &MPCPlanner::getInstance(ThreadPool *pool)
{
    static MPCPlannerPtr instance = MPCPlannerPtr(new MPCPlanner(pool));
    return instance;
}

MPCPlanner::MPCPlanner(ThreadPool *pool)
    : LocalPlanner(pool)
{
    m_log_name = LOG_MPC_PLANNER;
    m_type = TaskMPC::MAX_MPC_TYPE;
    m_selected_index = 0;
    m_final_path_point_arrived = false;
    m_final_path_point_infeasible = 0;
    m_start_segment_index = 0;
    m_global_path_index = -1;
    m_search_radius = 0.2;
    m_final_radius = 0.2;
    m_max_ipopt_cpu_time = 0.3;
}

float MPCPlanner::loadConfig(ConfigManager &cfg_mgr)
{
    MPC_DEBUG_LOG(COLOR_L_GREEN " Enter MPCPlanner::loadConfig ");
    ConfigMPCPlanner *cfg_mpcp = dynamic_cast<ConfigMPCPlanner *>(
        cfg_mgr.GetSubConfig(CONFIG_MPC_PLANNER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    ConfigRobot *cfg_robot = dynamic_cast<ConfigRobot *>(
        cfg_mgr.GetSubConfig(CONFIG_ROBOT));

    m_v_upper_bound_default = cfg_robot->max_linear_vel; // unit: m/s
    m_v_lower_bound = cfg_robot->min_linear_vel;

    m_lr = cfg_mpcp->conf_lr;
    m_lf = cfg_mpcp->conf_lf;
    m_l = m_lr + m_lf;
    m_rho = m_lf / m_l;

    m_NX = cfg_mpcp->conf_NX;
    m_NU = cfg_mpcp->conf_NU;
    m_NXU = m_NX + m_NU;
    m_N = cfg_mpcp->conf_N;

    m_delta_lower_bound = deg2rad(cfg_mpcp->conf_delta_lower_bound);
    m_delta_upper_bound = deg2rad(cfg_mpcp->conf_delta_upper_bound);

    m_Qx = cfg_mpcp->conf_Qx;
    m_Qy = cfg_mpcp->conf_Qy;
    m_Qphi = cfg_mpcp->conf_Qphi;
    m_Qdelta = cfg_mpcp->conf_Qdelta;

    m_ev_deduction_period_for_us = cfg_mpcp->ev_deduction_period_for_us;

    cfg_mpcp->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_MPC_PLANNER_FLAG, LOG_MPC_PLANNER,
               cfg_mpcp->log_name, cfg_mpcp->log_path,
               cfg_mpcp->log_extension, cfg_mpcp->log_ts_mask,
               cfg_mpcp->log_print_to_console,
               (cfg_mpcp->log_max_file_size_mb)MB + (cfg_mpcp->log_max_file_size_kb)KB,
               cfg_mpcp->log_max_file_cnt, cfg_mpcp->log_level);

    return cfg_mpcp->planning_frequency;
}

void MPCPlanner::initRunSM()
{
    m_run_sm.Initialize<MPCPlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("MPCPlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void MPCPlanner::reset()
{
    m_log_name = LOG_MPC_PLANNER;
    m_type = TaskMPC::MAX_MPC_TYPE;
    m_selected_index = 0;
    m_final_path_point_arrived = false;
    m_final_path_point_infeasible = 0;
    m_start_segment_index = 0;
    m_global_path_index = -1;
    m_search_radius = 0.2;
    m_final_radius = 0.2;
    m_max_ipopt_cpu_time = 0.3;
}

void MPCPlanner::getData()
{
    g_dc.getData<DataSlam>(m_slam_data);
}

bool MPCPlanner::handleTask(const Task &task)
{
    const TaskMPC &mpc_task = dynamic_cast<const TaskMPC &>(task);

    MPC_DEBUG_LOG(COLOR_L_GREEN "receive mpc task: %d, type = %d, start_segment_index = %lu, path_size = %lu, global_path_index = %d",
                  mpc_task.id, mpc_task.type, mpc_task.start_segment_index, mpc_task.mpc_ref_path.size(), mpc_task.global_path_index);

    if (mpc_task.type >= TaskMPC::MAX_MPC_TYPE)
        return false;

    m_type = mpc_task.type;

    if (m_type == TaskMPC::MPC_FOLLOW)
    {
        m_global_mpc_path.clear();
        m_global_mpc_path = mpc_task.mpc_ref_path;
        m_start_segment_index = mpc_task.start_segment_index;
        m_global_path_index = mpc_task.global_path_index;

        // set velocity upper bound
        m_v_upper_bound = mpc_task.velocity > m_v_upper_bound_default ? m_v_upper_bound_default : mpc_task.velocity;

        // set T_MPC
        if (mpc_task.velocity > m_v_upper_bound_default)
        {
            m_T_MPC = m_search_radius / m_v_upper_bound_default;
        }
        else if (mpc_task.velocity < 0.1)
        {
            m_T_MPC = m_search_radius / 0.1;
        }
        else
        {
            m_T_MPC = m_search_radius / mpc_task.velocity;
        }
        if (m_T_MPC < 0.1)
            m_T_MPC = 0.1;

        // set m_max_ipopt_cpu_time
        m_max_ipopt_cpu_time = m_T_MPC - 0.05;
    }

    return true;
}

void MPCPlanner::finishTask()
{
    MPC_INFO_LOG(COLOR_L_GREEN "generate mpc task finished evt id = %ld", m_task_id);

    g_speed_controller.SetStop(true);

    CREATE_EVENT(EvTaskFinished, ev_mpc_finished);
    ev_mpc_finished->task_id = m_task_id;
    g_ec.pushEvent(ev_mpc_finished);
}

size_t MPCPlanner::get_current_index()
{
    return m_selected_index;
}

void MPCPlanner::prepare_data_for_mpc(const PosePath &m_global_mpc_path_, size_t &index_selected_point,
                                      const vector<float> &robot_current_pose_, vector<MPCRefPoints> &local_mpc_path_, size_t N_, vector<int> &path_points_states)
{
    MPC_INFO_LOG(COLOR_L_GREEN "In prepare_data_for_mpc=======================");
    local_mpc_path_.clear();
    vector<MPCRefPoints> temp_local_mpc_path;

    bool trans2local_coord_flag = false;
    while (!trans2local_coord_flag)
    {
        trans2local_coord_flag = trans2local_coord(m_global_mpc_path_, index_selected_point, robot_current_pose_, temp_local_mpc_path, N_);
        if (!trans2local_coord_flag && index_selected_point < m_global_mpc_path_.size() - 1)
        {
            path_points_states[index_selected_point] = 3;
            index_selected_point += 1;
            // MPC_INFO_LOG(COLOR_L_GREEN "index_selected_point + 1");
            // for (int i = 0; i <= index_selected_point; i++)
            // {
            //     MPC_INFO_LOG(COLOR_L_GREEN "%d", path_points_states[i]);
            // }
        }
    }
    local_mpc_path_ = temp_local_mpc_path;

#ifdef LOG_OUTPUT
    MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
    MPC_DEBUG_LOG_OUTPUT << local_mpc_path_[0].x << " " << local_mpc_path_[0].y << " " << rad2deg(local_mpc_path_[0].theta) << " ";
    MPC_DEBUG_LOG_OUTPUT.close();
#endif

    // show first ref point
    MPC_DEBUG_LOG(COLOR_L_GREEN "MPC start=========================================");
    MPC_DEBUG_LOG(COLOR_L_GREEN "index_current:   %u   points_num: %u", index_selected_point, m_global_mpc_path_.size());
    MPC_DEBUG_LOG(COLOR_L_GREEN "curr_robot_cor:  %f %f %f ", robot_current_pose_[0], robot_current_pose_[1], robot_current_pose_[2]);
    MPC_DEBUG_LOG(COLOR_L_GREEN "ref_point_cor:   %f %f %f ", m_global_mpc_path_[index_selected_point].pt.x, m_global_mpc_path_[index_selected_point].pt.y, m_global_mpc_path_[index_selected_point].theta);
    MPC_DEBUG_LOG(COLOR_L_GREEN "ref_next_p_cor:  %f %f %f ", m_global_mpc_path_[index_selected_point + 1].pt.x, m_global_mpc_path_[index_selected_point + 1].pt.y, m_global_mpc_path_[index_selected_point + 1].theta);

#ifdef LOG_OUTPUT
    MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
    MPC_DEBUG_LOG_OUTPUT << index_selected_point << " " << m_global_mpc_path_.size() << " ";
    MPC_DEBUG_LOG_OUTPUT << m_global_mpc_path_[index_selected_point].pt.x << " " << m_global_mpc_path_[index_selected_point].pt.y << " " << rad2deg(m_global_mpc_path_[index_selected_point].theta) << " ";
    MPC_DEBUG_LOG_OUTPUT << m_global_mpc_path_[index_selected_point + 1].pt.x << " " << m_global_mpc_path_[index_selected_point + 1].pt.y << " " << rad2deg(m_global_mpc_path_[index_selected_point + 1].theta) << " ";
    MPC_DEBUG_LOG_OUTPUT.close();
#endif
}

bool MPCPlanner::trans2local_coord(const PosePath &m_global_mpc_path_, const size_t &index_selected_point,
                                   const vector<float> &robot_current_pose_, vector<MPCRefPoints> &temp_local_mpc_path_, size_t N_)
{
    temp_local_mpc_path_.clear();
    MPCRefPoints temp_point_mpc_local;
    // get first N points, trans to local coord, give to temp_local_mpc_path_
    for (size_t i = 0; i < N_; i++)
    {
#ifdef LINEARIZATION // not tested
        if (i + index_selected_point <= m_global_mpc_path_.size() - 1)
        {
            temp_point_mpc_local.x = m_global_mpc_path_[i + index_selected_point].pt.x;
            temp_point_mpc_local.y = m_global_mpc_path_[i + index_selected_point].pt.y;
            temp_point_mpc_local.theta = m_global_mpc_path_[i + index_selected_point].theta;
        }
        else
        {
            temp_point_mpc_local.x = m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.x;
            temp_point_mpc_local.y = m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.y;
            temp_point_mpc_local.theta = m_global_mpc_path_[m_global_mpc_path_.size() - 1].theta;
        }
        temp_local_mpc_path_.push_back(temp_point_mpc_local);
#else
        // translate to mpc local coordinate
        if (i + index_selected_point <= m_global_mpc_path_.size() - 1)
        {
            temp_point_mpc_local.x = (m_global_mpc_path_[i + index_selected_point].pt.x - robot_current_pose_[0]) * cos(robot_current_pose_[2]) + (m_global_mpc_path_[i + index_selected_point].pt.y - robot_current_pose_[1]) * sin(robot_current_pose_[2]);
            temp_point_mpc_local.y = (m_global_mpc_path_[i + index_selected_point].pt.x - robot_current_pose_[0]) * (-sin(robot_current_pose_[2])) + (m_global_mpc_path_[i + index_selected_point].pt.y - robot_current_pose_[1]) * cos(robot_current_pose_[2]);
            // temp_point_mpc_local.theta = wrapAngle_rad(m_global_mpc_path_[i + index_selected_point].theta - robot_current_pose_[2]);
            temp_point_mpc_local.theta = toNPPiAngleRangeR(m_global_mpc_path_[i + index_selected_point].theta - robot_current_pose_[2]);
            MPC_INFO_LOG(COLOR_L_GREEN "trans to local_coord, index = %u, temp_point_mpc_global = %f, %f, %f", i, m_global_mpc_path_[i + index_selected_point].pt.x, m_global_mpc_path_[i + index_selected_point].pt.y, m_global_mpc_path_[i + index_selected_point].theta);
        }
        else
        {
            temp_point_mpc_local.x = (m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.x - robot_current_pose_[0]) * cos(robot_current_pose_[2]) + (m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.y - robot_current_pose_[1]) * sin(robot_current_pose_[2]);
            temp_point_mpc_local.y = (m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.x - robot_current_pose_[0]) * (-sin(robot_current_pose_[2])) + (m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.y - robot_current_pose_[1]) * cos(robot_current_pose_[2]);
            // temp_point_mpc_local.theta = wrapAngle_rad(m_global_mpc_path_[m_global_mpc_path_.size() - 1].theta - robot_current_pose_[2]);
            temp_point_mpc_local.theta = toNPPiAngleRangeR(m_global_mpc_path_[m_global_mpc_path_.size() - 1].theta - robot_current_pose_[2]);
            MPC_INFO_LOG(COLOR_L_GREEN "trans to local_coord, index = %u, temp_point_mpc_global = %f, %f, %f", i, m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.x, m_global_mpc_path_[m_global_mpc_path_.size() - 1].pt.y, m_global_mpc_path_[m_global_mpc_path_.size() - 1].theta);
        }

        if (temp_point_mpc_local.x < 0 && i == 0 && i + index_selected_point < m_global_mpc_path_.size() - 1)
        {
            MPC_INFO_LOG(COLOR_L_GREEN "trans to local coord failed, first temp_point_mpc_local.x < 0");
            return false;
        }

        temp_local_mpc_path_.push_back(temp_point_mpc_local);
        MPC_INFO_LOG(COLOR_L_GREEN "trans to local_coord, index = %u, temp_point_mpc_local  = %f, %f, %f", i, temp_point_mpc_local.x, temp_point_mpc_local.y, temp_point_mpc_local.theta);

#endif
    }
    return true;
}

void MPCPlanner::do_mpc(const vector<float> &robot_current_pose_, vector<MPCRefPoints> &local_mpc_path_, vector<float> &X_OPT_, vector<float> &opt_global_)
{
    // Ipopt::SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    // app->Options()->SetStringValue("option_file_name", "/tmp/config/ipopt.opt"); // ipopt path
    // // app->Options()->SetStringValue("print_level", "0");
    // // app->Options()->SetStringValue("hassian_approximation", "limited-memory");
    // // app->Options()->SetStringValue("limited_memory_update_type", "bfgs");

    Ipopt::SmartPtr<ROBOT_NLP> robot_nlp = new ROBOT_NLP();
    robot_nlp->set_vehicle_para(m_lr, m_lf, m_l, m_rho);
    // MPC_DEBUG_LOG(COLOR_L_GREEN "Vehicle parameters set=====================");
    robot_nlp->set_opti_para(m_NX, m_NU, m_NXU, m_N, m_T_MPC);
    // MPC_DEBUG_LOG(COLOR_L_GREEN "Opimization parameters set=================");
    robot_nlp->set_weights(m_Qx, m_Qy, m_Qphi, m_Qdelta);
    // MPC_DEBUG_LOG(COLOR_L_GREEN "MPC weights set============================");
    robot_nlp->set_bounds(m_v_lower_bound, m_v_upper_bound, m_delta_lower_bound, m_delta_upper_bound);
    // MPC_DEBUG_LOG(COLOR_L_GREEN "MPC bounds set=============================");
    robot_nlp->set_current_pose(robot_current_pose_);
    // MPC_DEBUG_LOG(COLOR_L_GREEN "Current pose set===========================");
    robot_nlp->set_local_points(local_mpc_path_);
    // MPC_DEBUG_LOG(COLOR_L_GREEN "MPC local points set=======================");

    // ApplicationReturnStatus status;
    // status = app->Initialize();
    status = app->OptimizeTNLP(robot_nlp);

    robot_nlp->get_opt_res(X_OPT_, opt_global_);

#ifdef LOG_OUTPUT
    MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
    MPC_DEBUG_LOG_OUTPUT << opt_global_[0] << " " << opt_global_[1] << " " << rad2deg(opt_global_[2]) << " ";
    MPC_DEBUG_LOG_OUTPUT.close();
#endif

    int solved_flag = 0;

    if (status == Solve_Succeeded)
    {
        solved_flag = 1;
        // Retrieve some statistics about the solve
        Index iter_count = app->Statistics()->IterationCount();
        MPC_DEBUG_LOG(COLOR_L_GREEN "*** The problem solved in %d.", iter_count);
        // std::cout << "*** The problem solved in " << iter_count << " iterations!" << std::endl;
        // Number final_obj = app->Statistics()->FinalObjective();
        // std::cout << "*** The final value of the objective function is " << final_obj << '.'<< std::endl;
        Number total_cpu_time = app->Statistics()->TotalCPUTime();
        MPC_INFO_LOG(COLOR_L_GREEN "*** The total cpu time is %f/%f.", total_cpu_time, m_max_ipopt_cpu_time);
        // std::cout << "*** The total cpu time is " << total_cpu_time << '.' << std::endl;
        Number total_sys_time = app->Statistics()->TotalSysTime();
        MPC_DEBUG_LOG(COLOR_L_GREEN "*** The total sys time is %f.", total_sys_time);
        // std::cout << "*** The total sys time is " << total_sys_time << '.' << std::endl;

        // #ifdef LOG_OUTPUT
        //         MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
        //         MPC_DEBUG_LOG_OUTPUT << iter_count << " " << total_cpu_time << " " << total_sys_time << " ";
        //         MPC_DEBUG_LOG_OUTPUT.close();
        // #endif
    }
#ifdef LOG_OUTPUT
    MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::app);
    MPC_DEBUG_LOG_OUTPUT << solved_flag << " ";
    MPC_DEBUG_LOG_OUTPUT.close();
#endif
}