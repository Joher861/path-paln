#include "path_follow_planner.h"
#include "path_follow_planner_input.h"
#include "path_follow_planner_log.h"
#include "path_follow_planner_config.h"
#include "path_follow_planner_events.h"
// #include "local_trajectory_generator/local_trajectory_generator.h"
// #include "rollout_generator/rollout_generator.h"

#include "global_teb_planner/teb_planner.h"
#include "global_teb_planner/teb_planner_events.h"
#include "global_teb_planner/global_teb_input.h"

#include "speed_controller/speed_controller.h"
// #include "teb_planner/teb_planner.h"
#include "pure_pursuit_planner/pure_pursuit_planner.h"
#include "mpc_planner/mpc_planner.h"
#include "rotate_planner/rotate_planner.h"

#include "planner/global_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "task/task_manager.h"

#include "local_planner/local_planner_events.h"

// #include "rotate_planner/rotate_task.h"
// #include "basic_planner/basic_task.h"

#include "misc/planning_common_config.h"
#include "misc/robot_config.h"
#include "event/global_common_event_def.h"

#include "emergency_stop_detector/emergency_stop_detector.h"
#include "obstacle_detector/obstacle_detector.h"
#include "collision_detector/collision_detector.h"

using namespace planning_planner;
using namespace planning_utils;
using namespace planning_data;
using namespace planning_controller;
using namespace planning_detector;

#define LOG_OUTPUT
#define WZ_TEST


#ifdef LOG_OUTPUT
ofstream PATH_FOLLOW_DEBUG_LOG_OUTPUT;
std::string filename = "/tmp/log/pms/PATH_FOLLOW_DEBUG_LOG_OUTPUT.csv";
ofstream MPC_DEBUG_LOG_OUTPUT;
std::string filename_mpc = "/tmp/log/pms/MPC_DEBUG_LOG_OUTPUT.csv";
ofstream PURE_PURSUIT_DEBUG_LOG_OUTPUT;
std::string filename_pp = "/tmp/log/pms/PURE_PURSUIT_DEBUG_LOG_OUTPUT.csv";
ofstream ALL_PATH_POINTS_DEBUG_LOG_OUTPUT;
std::string filename_all = "/tmp/log/pms/ALL_PATH_POINTS_DEBUG_LOG_OUTPUT.csv";
#endif

#ifdef OD_SWITCH
const static size_t segmentation_num = 7; // 3
#endif

namespace planning_planner
{
    struct PathFollowPlannerStates
    { // StateWithOwner
        // Class that clients can use instead of deriving directly from State that provides convenient
        // typed access to the Owner. This class can also be chained via the StateBaseType parameter,
        // which is useful when inheriting state machines.
        struct BaseState : StateWithOwner<PathFollowPlanner>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter() // OnEnter is invoked when a State is created
            {
                PATHFOLLOW_PLANNER_DEBUG_LOG("Enter Disable...");
            }

            virtual void OnExit()
            {
                PATHFOLLOW_PLANNER_DEBUG_LOG("Exit Disable...");
            }

            virtual Transition GetTransition()
            {
                PATHFOLLOW_PLANNER_DEBUG_LOG("Disable GetTransition...");
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
                // navigtion_DEBUG_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                Owner().dispatchThread();
                CREATE_GLOBAL_PLANNER_INPUT(TebPlannerInput, g_teb_input);
                g_teb_planner.startPlanner(g_teb_input);
            }

            virtual void OnExit()
            {
                // navigtion_DEBUG_LOG("Exit Enable...");
                g_teb_planner.stopPlanner();
                Owner().stopThread();
            }

            virtual Transition GetTransition()
            {
                if (!Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Disable>();
                }

                return InnerEntryTransition<Path_Follow_Go>();
            }

            virtual void Update()
            {
            }
        };

        struct Path_Follow_Go : BaseState
        {
            int64_t mpc_task_id;
            int64_t purepursuit_task_id;
            int64_t rotate_task_id;
            int64_t ending_rotate_task_id = -2;

            uint64_t ts_ev_cur_index_mpc;
            uint64_t ts_ev_cur_index_purepursuit;
            uint64_t ts_ev_escape_request_purepursuit;
            uint64_t ts_ev_escape_request_teb;
            uint64_t ts_ev_teb_opt_path;

            bool rotate_request_left_dir;
            float rotate_request_target_angle;
            int rotate_request_flag;
            int ignore_rotate_request_flag;

            uint64_t last_ts;

            int mpc_rx_flag;
            int mpc_infeasible_flag;
            int pp_infeasible_flag;
            int teb_infeasible_flag;
            int teb_infeasible_timer_flag;
            uint64_t teb_infeasible_timer_start_ts;
            int teb_rotate_request_flag;
            int mpc_finished;
            int pp_finished;
            int mode_obstacle_avoidance;
            int last_mode_obstacle_avoidance;

            size_t print_mpc_return_next_index;
            size_t print_ppp_return_next_index;
            size_t print_teb_return_next_index;

            bool last_rotate_task_finished = true;
            bool ending_rotate_task_finished = false;
            bool ending_rotate_task_sent = false;
            float delta_rotate_angle;
            int last_rotate_dir;

            float last_curvature;
            float last_velocity;

            int ending_type;

            int teb_is_planning;
            int oa_switched_off_idx;
            bool force_to_OA_0;

            float max_period;

            bool teb_path_is_robot_as_start;

            bool freezing_replanning_req_to_teb;

            int teb_infeasible_ending_timer;
            uint64_t teb_infeasible_ending_timer_start_ts;

            int collision_lasting_timer;
            uint64_t collision_lasting_start_ts;

            CREATE_EVENT(EvPFPDetIntendVel, ev_det_intend_vel);

#ifdef OD_SWITCH
            uint64_t ts_ev_have_obstacle;
            int obstacle_dir_in_far_circle;
            int obstacle_dir_in_near_circle;
            std::vector<int> have_obstacle;
#else
            PosePath path_points_to_cd;
#endif

            virtual void OnEnter()
            {
                PATHFOLLOW_PLANNER_INFO_LOG("Enter Path_Follow_Go...");
                last_ts = Timer::getSystemTimestampUS();
                ending_type = 0;
                teb_is_planning = 0;
                max_period = 0;
                force_to_OA_0 = false;
                teb_path_is_robot_as_start = false;
                freezing_replanning_req_to_teb = false;
                teb_infeasible_ending_timer = 0;
                // Owner().m_interval_mpc = 0.05;
                // Owner().m_interval_pure_pursuit = 1;
                Owner().jmp_r_mpc2pp = 2;
                Owner().max_jmp_r_pp2mpc = 2;
                Owner().jmp_tol_angle = 60;
                Owner().oa_switch_radius = 0.35; // 要比各local_planner里的reach_radius大一点
                Owner().goal_index_mpc_last = 0;
                Owner().goal_index_pure_pursuit_last = 0;
                Owner().goal_index_teb_last = 0;
                Owner().goal_index_mpc = 0;
                Owner().goal_index_pure_pursuit = 0;
                Owner().mpc_return_next_dense_index = 0;
                Owner().pure_pursuit_return_next_sparse_index = 0;

                Owner().m_path_follow_finished = 0;
                Owner().m_path_follow_error = 0;
                Owner().m_start_deviation = 0;
                Owner().path_points_mpc.clear();
                Owner().path_points_pure_pursuit.clear();
                Owner().path_points_mpc_situation.clear();
                Owner().path_points_pure_pursuit_situation.clear();
                Owner().pp_index_correspond_mpc_index.clear();
                Owner().pair_pp_index_to_mpc_index.clear();

                Owner().end_idx_from_teb = -1;

                teb_infeasible_timer_flag = 0;

                ts_ev_cur_index_mpc = 0;
                ts_ev_cur_index_purepursuit = 0;
                ts_ev_teb_opt_path = 0;
#ifdef OD_SWITCH
                ts_ev_have_obstacle = 0;
#endif
                collision_lasting_timer = 0;
                Owner().m_collision_lasting_flag = false;

                mode_obstacle_avoidance = 0;
                last_mode_obstacle_avoidance = 0;
                delta_rotate_angle = deg2rad(Owner().m_delta_rotate_angle); // deg in config, need deg2rad
                last_rotate_dir = 0;
                last_curvature = 0;
                last_velocity = 0;

#ifdef LOG_OUTPUT
                // PATH_FOLLOW_DEBUG_LOG_OUTPUT.open(filename, ios::out | ios::ate);
                // PATH_FOLLOW_DEBUG_LOG_OUTPUT.close();
                // MPC_DEBUG_LOG_OUTPUT.open(filename_mpc, ios::out | ios::ate);
                // MPC_DEBUG_LOG_OUTPUT.close();
                // PURE_PURSUIT_DEBUG_LOG_OUTPUT.open(filename_pp, ios::out | ios::ate);
                // PURE_PURSUIT_DEBUG_LOG_OUTPUT.close();
#endif

                if (Owner().global_path_points.empty())
                {
                    Owner().m_path_follow_error = 1;
                }
                else
                {
                    if (Owner().m_need_path_insert)
                    {
                        PosePath path_after_ellipse_insert;
                        // Owner().insert_ellipse_between_clean_lines(Owner().global_path_points, path_after_ellipse_insert);
                        // Owner().insert_straight_line_between_clean_lines(Owner().global_path_points, path_after_ellipse_insert);
                        Owner().insert_bezier_between_clean_lines(Owner().global_path_points, path_after_ellipse_insert);
                        Owner().global_path_points.clear();
                        Owner().global_path_points = path_after_ellipse_insert;
                        PATHFOLLOW_PLANNER_WARN_LOG("insert_bezier_between_clean_lines() finished...");
                    }

#ifdef LOG_OUTPUT
                    // 全部路径点文档
                    ALL_PATH_POINTS_DEBUG_LOG_OUTPUT.open(filename_all, ios::out | ios::app);
                    for (size_t i = 0; i < Owner().global_path_points.size(); i++)
                    {
                        ALL_PATH_POINTS_DEBUG_LOG_OUTPUT << Owner().global_path_points[i].pt.x << " " << Owner().global_path_points[i].pt.y << " " << Owner().global_path_points[i].theta << "\n";
                    }
                    ALL_PATH_POINTS_DEBUG_LOG_OUTPUT.close();
#endif

                    g_dc.getData<DataSlam>(Owner().m_slam_data);
                    float starting_dist = hypot(Owner().global_path_points[0].pt.y - Owner().m_slam_data.pose.pt.y, Owner().global_path_points[0].pt.x - Owner().m_slam_data.pose.pt.x);
                    float starting_yaw = atan2(Owner().global_path_points[0].pt.y - Owner().m_slam_data.pose.pt.y, Owner().global_path_points[0].pt.x - Owner().m_slam_data.pose.pt.x);
                    float starting_robot_yaw_diff = rad2deg(toNPPiAngleRangeR(starting_yaw - Owner().m_slam_data.pose.theta));
                    float starting_path_yaw_diff = rad2deg(toNPPiAngleRangeR(starting_yaw - Owner().global_path_points[0].theta));
                    if (starting_dist > 0.5 || fabs(starting_robot_yaw_diff) >= 90 || fabs(starting_path_yaw_diff) >= 90)
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("Start deviation...");
                        PATHFOLLOW_PLANNER_INFO_LOG("Owner().global_path_points[0].theta:  %f  robot_yaw:  %f  starting_yaw:  %f", rad2deg(Owner().global_path_points[0].theta), rad2deg(Owner().m_slam_data.pose.theta), rad2deg(starting_yaw));
                        PATHFOLLOW_PLANNER_INFO_LOG("starting_robot_yaw_diff:  %f  starting_path_yaw_diff:  %f", starting_robot_yaw_diff, starting_path_yaw_diff);
                        // 选用　start_deviation用teb方案
                        // Owner().m_start_deviation = 1;

                        PosePath tmp_path = Owner().global_path_points;
                        Owner().global_path_points.clear();
                        int insert_num = (int)(starting_dist / Owner().m_interval_mpc);
                        float dy = (Owner().global_path_points[0].pt.y - Owner().m_slam_data.pose.pt.y) / (float)insert_num;
                        float dx = (Owner().global_path_points[0].pt.x - Owner().m_slam_data.pose.pt.x) / (float)insert_num;
                        for (int i = 0; i < insert_num; i++)
                        {
                            RobotPose tmp_pose;
                            tmp_pose.pt.x = Owner().m_slam_data.pose.pt.x + (float)i * dx;
                            tmp_pose.pt.y = Owner().m_slam_data.pose.pt.y + (float)i * dy;
                            tmp_pose.theta = starting_yaw;

                            Owner().global_path_points.add(tmp_pose);
                        }

                        for (size_t i = 0; i < tmp_path.size(); i++)
                        {
                            Owner().global_path_points.add(tmp_path[i]);
                        }
                    }

                    Owner().preprocess_global_path_points();
                }
            }

            virtual void OnExit()
            {
                PATHFOLLOW_PLANNER_INFO_LOG("Exit Path_Follow_Go...");
                g_collision_detector.stopDetect();
                Owner().print_reached_and_skipped();
                if (Owner().m_feedback_path_points_states)
                {
                    CREATE_EVENT(EvPFPFeedbackPathStates, ev_pfp_feedback_path_states);
                    ev_pfp_feedback_path_states->input_idx = Owner().m_input_idx;
                    ev_pfp_feedback_path_states->path_points_situation = Owner().path_points_mpc_situation;
                    g_ec.pushEvent(ev_pfp_feedback_path_states);
                }
                CREATE_EVENT(EvPathFollowGoExit, ev_path_follow_go_exit);
                g_ec.pushEvent(ev_path_follow_go_exit);
                PATHFOLLOW_PLANNER_INFO_LOG("+++++++++++++ Path_Follow_Go Exit Event pushed +++++++++++++");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_path_follow_error != 0)
                {
                    return SiblingTransition<Error>();
                }
                if (Owner().m_path_follow_finished == 1 && ending_rotate_task_finished)
                {
                    return SiblingTransition<Finished>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                PATHFOLLOW_PLANNER_INFO_LOG("========================Path_Follow_Go Update========================");
                uint64_t update_start_ts = Timer::getTimestampUS();
#ifdef LOG_OUTPUT
                PATH_FOLLOW_DEBUG_LOG_OUTPUT.open(filename, ios::out | ios::app);
                struct timespec curtime;
                clock_gettime(CLOCK_MONOTONIC, &curtime);
                uint64_t path_follow_debug_log_ts = curtime.tv_sec * 1e6 + curtime.tv_nsec / 1e3;
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << path_follow_debug_log_ts << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT.close();
#endif

#ifdef OD_SWITCH
                Owner().m_small_obstacle_flag = false;
                Owner().m_far_circle_obstacle_flag = false;
                Owner().m_near_circle_obstacle_flag = false;
                obstacle_dir_in_far_circle = 0;
                obstacle_dir_in_near_circle = 0;
                // define have_obstacle
                have_obstacle.clear();
                for (size_t i = 0; i < segmentation_num; i++)
                {
                    have_obstacle.push_back(0);
                }
#else
                Owner().m_collision_flag = false;
#endif

                Owner().mpc_return_index = 0;
                Owner().mpc_path_points_states.clear();

                Owner().pure_pursuit_return_index = 0;
                Owner().pure_pursuit_path_points_states.clear();

                Owner().m_rx_teb_path = 0;

                Owner().teb_return_index = 0;
                Owner().teb_path_points_states.clear();

                mpc_rx_flag = 0;
                // mpc_infeasible_flag = 0;
                teb_infeasible_flag = 0;
                pp_infeasible_flag = 0;
                mpc_finished = 0;
                pp_finished = 0;
                teb_rotate_request_flag = 0;

                ts_ev_escape_request_purepursuit = UINT64_MAX;
                ts_ev_escape_request_teb = UINT64_MAX;
                rotate_request_flag = 0;
                ignore_rotate_request_flag = 0;

                bool escape_fl_rotate_right = false;
                bool escape_l_rotate_right = false;
                bool escape_fr_rotate_left = false;
                bool escape_r_rotate_left = false;
                bool escape_f_rotate_left = false;
                bool escape_f_rotate_right = false;

                bool us_lasting_front = false;
                bool us_lasting_front_left = false;
                bool us_lasting_front_right = false;
                bool us_lasting_left = false;
                bool us_lasting_right = false;

                bool lm_lasting_front = false;
                bool lm_lasting_front_left = false;
                bool lm_lasting_front_right = false;
                bool lm_lasting_left = false;
                bool lm_lasting_right = false;

                // bool emergency_us_front = false;
                bool emergency_us_front_left = false;
                bool emergency_us_front_right = false;
                bool emergency_us_left = false;
                bool emergency_us_right = false;
                bool emergency_us_left_pure = false;
                bool emergency_us_right_pure = false;

                // bool emergency_lm_front = false;
                bool emergency_lm_front_left = false;
                bool emergency_lm_front_right = false;
                bool emergency_lm_left = false;
                bool emergency_lm_right = false;
                bool emergency_lm_left_pure = false;
                bool emergency_lm_right_pure = false;

                bool slam_coord_jumped = false;

                bool freeze_stop_flag = false;

                bool rotation_stuck_flag = false;
                bool rotation_standby_flag = false;

                int index_jumped = 0;
                oa_switched_off_idx = -1;
                // Owner().m_path_follow_finished = 0;

                // check events
                EvtList evt_list;
                EvtList evt_list_for_us;
                evt_list.clear();
                evt_list_for_us.clear();
                uint64_t cur_ts = Timer::getSystemTimestampUS();
                uint64_t last_ts_for_us_events = cur_ts - Owner().m_ev_deduction_period_for_us; // us
                // uint64_t delta_t = cur_ts - last_ts;
                // PATHFOLLOW_PLANNER_INFO_LOG("last_ts:  %u    cur_ts:  %u    delta_t:  %u", last_ts, cur_ts, delta_t);
                g_ec.eventDeduction(last_ts, cur_ts, evt_list);
                g_ec.eventDeduction(last_ts_for_us_events, cur_ts, evt_list_for_us);
                last_ts = cur_ts;
                // for normal events
                for (auto &evt : evt_list)
                {
                    std::type_index evt_type = evt->getType();
#ifdef OD_SWITCH
                    // ********** obstacle_detector
                    if (TYPE_EQUALS(evt_type, EvSmallObstacle))
                    {
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "+++++++++++++++Small Obstacle+++++++++++++++");
                        Owner().m_small_obstacle_flag = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvFarCircleObstacle))
                    {
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "++++++++++++Far Circle Obstacle++++++++++++");
                        Owner().m_far_circle_obstacle_flag = true;
                        EvFarCircleObstaclePtr far_circle_obs_det_evt = dynamic_pointer_cast<EvFarCircleObstacle>(evt);
                        obstacle_dir_in_far_circle = far_circle_obs_det_evt->obstacle_dir;
                    }
                    if (TYPE_EQUALS(evt_type, EvNearCircleObstacle))
                    {
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "++++++++++++Near Circle Obstacle++++++++++++");
                        Owner().m_near_circle_obstacle_flag = true;
                        EvNearCircleObstaclePtr near_circle_obs_det_evt = dynamic_pointer_cast<EvNearCircleObstacle>(evt);
                        obstacle_dir_in_near_circle = near_circle_obs_det_evt->obstacle_dir;
                    }
                    if (TYPE_EQUALS(evt_type, EvHaveObstacle))
                    {
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "++++++++++++Receive HaveObstacle++++++++++++");
                        EvHaveObstaclePtr have_obstacle_obs_det_evt = dynamic_pointer_cast<EvHaveObstacle>(evt);
                        if (have_obstacle_obs_det_evt->ts > ts_ev_have_obstacle)
                        {
                            ts_ev_have_obstacle = have_obstacle_obs_det_evt->ts;
                            have_obstacle = have_obstacle_obs_det_evt->have_obstacle;
                        }
                    }
#else
                    // ********** collision_detector
                    if (TYPE_EQUALS(evt_type, EvCollisionTrue))
                    {
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "+++++++++++++++ collision event received +++++++++++++++");
                        Owner().m_collision_flag = true;
                    }
#endif
                    // ********** emergency_stop_detector
                    // Coord Jumped Event
                    if (TYPE_EQUALS(evt_type, EvCoordJumped))
                    {
                        slam_coord_jumped = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ EvCoordJumped ++++++++++++");
                    }
                    // US Lasting Events
                    if (TYPE_EQUALS(evt_type, EvUSFrontLasting))
                    {
                        us_lasting_front = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_front = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvUSFrontLeftLasting))
                    {
                        us_lasting_front_left = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_front_left = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvUSFrontRightLasting))
                    {
                        us_lasting_front_right = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_front_right = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvUSLeftLasting))
                    {
                        us_lasting_left = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_left = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvUSRightLasting))
                    {
                        us_lasting_right = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_right = true ++++++++++++");
                    }
                    // LM Lasting Events
                    if (TYPE_EQUALS(evt_type, EvLMFrontLasting))
                    {
                        lm_lasting_front = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_front = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvLMFrontLeftLasting))
                    {
                        lm_lasting_front_left = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_front_left = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvLMFrontRightLasting))
                    {
                        lm_lasting_front_right = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_front_right = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvLMLeftLasting))
                    {
                        lm_lasting_left = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_left = true ++++++++++++");
                    }
                    if (TYPE_EQUALS(evt_type, EvLMRightLasting))
                    {
                        lm_lasting_right = true;
                        // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_right = true ++++++++++++");
                    }
                    // ********** teb_planner
                    if (TYPE_EQUALS(evt_type, EvTebOptPath))
                    {
                        EvTebOptPathPtr teb_opt_path_evt = dynamic_pointer_cast<EvTebOptPath>(evt);

                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "teb_opt_path_evt->ts: %lu", teb_opt_path_evt->ts);
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "ts_ev_teb_opt_path:   %lu", ts_ev_teb_opt_path);

                        if (teb_opt_path_evt->ts > ts_ev_teb_opt_path)
                        {
                            ts_ev_teb_opt_path = teb_opt_path_evt->ts;
                            Owner().m_rx_teb_path = 1;
                            Owner().path_from_teb = *(teb_opt_path_evt->opt_path_ptr);
                            // Owner().start_idx_from_teb = teb_opt_path_evt->start_idx;
                            Owner().end_idx_from_teb = (int)(teb_opt_path_evt->end_idx);
                            teb_path_is_robot_as_start = teb_opt_path_evt->is_robot_as_start;
                            teb_is_planning = 0;
                            // for (size_t i = 0; i < Owner().path_from_teb.size(); i++)
                            // {
                            //     PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "RX_TEB_PATH [%lu]:  %f  %f  %f", i, Owner().path_from_teb[i].pt.x, Owner().path_from_teb[i].pt.y, rad2deg(Owner().path_from_teb[i].theta));
                            // }
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Teb Optimized Path Received.");
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_from_teb.size: %d", Owner().path_from_teb.size());
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "end_idx_from_teb:   %d", Owner().end_idx_from_teb);
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "RX_TEB_PATH end  pose:  %f  %f  %f", Owner().path_from_teb.back().pt.x, Owner().path_from_teb.back().pt.y, rad2deg(Owner().path_from_teb.back().theta));
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "end_idx_from_teb pose:  %f  %f  %f", Owner().path_points_mpc[Owner().end_idx_from_teb].pt.x, Owner().path_points_mpc[Owner().end_idx_from_teb].pt.y, rad2deg(Owner().path_points_mpc[Owner().end_idx_from_teb].theta));
                        }
                    }
                    if (TYPE_EQUALS(evt_type, EvGTebPlannerError))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Global Teb Planner Infeasible ++++++++++++");
                        teb_infeasible_flag = 1;
                        if (teb_infeasible_timer_flag == 0)
                        {
                            teb_infeasible_timer_flag = 1;
                            teb_infeasible_timer_start_ts = Timer::getSystemTimestampUS();
                        }
                    }
                    if (TYPE_EQUALS(evt_type, EvTebRotateReq))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ Receive EscapeRequest_Teb ++++++++++++");
                        EvTebRotateReqPtr escape_request_evt_teb = dynamic_pointer_cast<EvTebRotateReq>(evt);
                        if (escape_request_evt_teb->ts < ts_ev_escape_request_teb)
                        {
                            ts_ev_escape_request_teb = escape_request_evt_teb->ts;
                            rotate_request_left_dir = escape_request_evt_teb->dir;
                            rotate_request_target_angle = escape_request_evt_teb->target_yaw;
                            rotate_request_flag = 1;
                            teb_rotate_request_flag = 1;
                        }
                    }
                    // ********** mpc_planner
                    if (TYPE_EQUALS(evt_type, EvCurIndexInCurTask_MPC))
                    {
                        EvCurIndexInCurTask_MPCPtr cur_index_type_evt_mpc = dynamic_pointer_cast<EvCurIndexInCurTask_MPC>(evt);
                        PATHFOLLOW_PLANNER_DEBUG_LOG("cur_index_type_evt_mpc->ts: %lu", cur_index_type_evt_mpc->ts);
                        PATHFOLLOW_PLANNER_DEBUG_LOG("ts_ev_cur_index_mpc:        %lu", ts_ev_cur_index_mpc);
                        if (cur_index_type_evt_mpc->ts > ts_ev_cur_index_mpc)
                        {
                            ts_ev_cur_index_mpc = cur_index_type_evt_mpc->ts;
                            Owner().mpc_return_index = cur_index_type_evt_mpc->cur_index_in_cur_task;
                            Owner().mpc_path_points_states = cur_index_type_evt_mpc->path_points_states;
                            if (cur_index_type_evt_mpc->global_path_index >= 0)
                            {
                                mpc_rx_flag = 1;
                                Owner().goal_index_mpc_last = (size_t)(cur_index_type_evt_mpc->global_path_index);
                            }
                            mpc_infeasible_flag = 0;
                        }
                        // PATHFOLLOW_PLANNER_INFO_LOG("cur_index_type_evt_mpc->global_path_index: %lu", cur_index_type_evt_mpc->global_path_index);
                    }
                    if (TYPE_EQUALS(evt_type, EvTaskPathInfeasible_MPC))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ MPC Task Path Infeasible ++++++++++++");
                        EvTaskPathInfeasible_MPCPtr mpc_infeasible_evt = dynamic_pointer_cast<EvTaskPathInfeasible_MPC>(evt);
                        mpc_infeasible_flag = mpc_infeasible_evt->id;
                        // mpc_infeasible_flag = 1; // when active, use pp to follow
                    }
                    // ********** pure_pursuit_planner
                    if (TYPE_EQUALS(evt_type, EvCurIndexInCurTask_PurePursuit))
                    {
                        EvCurIndexInCurTask_PurePursuitPtr cur_index_type_evt_purepursuit = dynamic_pointer_cast<EvCurIndexInCurTask_PurePursuit>(evt);
                        if (cur_index_type_evt_purepursuit->ts > ts_ev_cur_index_purepursuit)
                        {
                            ts_ev_cur_index_purepursuit = cur_index_type_evt_purepursuit->ts;
                            Owner().pure_pursuit_return_index = cur_index_type_evt_purepursuit->cur_index_in_cur_task;
                            Owner().goal_index_pure_pursuit_last = cur_index_type_evt_purepursuit->global_path_index;
                            Owner().pure_pursuit_path_points_states = cur_index_type_evt_purepursuit->path_points_states;
                        }
                    }
                    if (TYPE_EQUALS(evt_type, EvTaskPathInfeasible_PurePursuit))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ PurePursuit Task Path Infeasible ++++++++++++");
                        pp_infeasible_flag = 1; // unused
                    }
                    if (TYPE_EQUALS(evt_type, EvEscapeRequest_PurePursuit))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ Receive EscapeRequest_PurePursuit ++++++++++++");
                        EvEscapeRequest_PurePursuitPtr escape_request_evt_purepursuit = dynamic_pointer_cast<EvEscapeRequest_PurePursuit>(evt);
                        if (escape_request_evt_purepursuit->ts < ts_ev_escape_request_purepursuit)
                        {
                            ts_ev_escape_request_purepursuit = escape_request_evt_purepursuit->ts;
                            rotate_request_left_dir = escape_request_evt_purepursuit->left_dir;
                            rotate_request_target_angle = escape_request_evt_purepursuit->rotate_target_angle;
                            rotate_request_flag = 1;
                        }
                    }
                    // ********** teb_planner
                    // if (TYPE_EQUALS(evt_type, EvCurIndexInCurTask_TEB))
                    // {
                    //     EvCurIndexInCurTask_TEBPtr cur_index_type_evt_teb = dynamic_pointer_cast<EvCurIndexInCurTask_TEB>(evt);
                    //     if (cur_index_type_evt_teb->ts > ts_ev_cur_index_teb)
                    //     {
                    //         ts_ev_cur_index_teb = cur_index_type_evt_teb->ts;
                    //         Owner().teb_return_index = cur_index_type_evt_teb->cur_index_in_cur_task;
                    //         Owner().goal_index_teb_last = cur_index_type_evt_teb->global_path_index;
                    //         Owner().teb_path_points_states = cur_index_type_evt_teb->path_points_states;
                    //     }
                    // }
                    // ********** rotate_planner
                    if (TYPE_EQUALS(evt_type, EvRotationStuck))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ Rotation Stuck ++++++++++++");
                        last_rotate_task_finished = true;
                        rotation_stuck_flag = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvRotationStandby))
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ Rotation Standby ++++++++++++");
                        last_rotate_task_finished = true;
                        // rotation_standby_flag = true;
                    }
                    // ********** Task Finished
                    if (TYPE_EQUALS(evt_type, EvTaskFinished))
                    {
                        EvTaskFinishedPtr ev_task_finished = dynamic_pointer_cast<EvTaskFinished>(evt);
                        // rotate_planner finished
                        if (ev_task_finished->task_id == rotate_task_id)
                        {
                            last_rotate_task_finished = true;
                            PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ Rotate Task Finished ++++++++++++");
                            PATHFOLLOW_PLANNER_INFO_LOG("last_rotate_task_finished: true");
                        }
                        if (ev_task_finished->task_id == ending_rotate_task_id)
                        {
                            ending_rotate_task_finished = true;
                            PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ Ending Rotate Task Finished ++++++++++++");
                        }
                        // mpc_planner finished
                        if (ev_task_finished->task_id == mpc_task_id)
                        {
                            mpc_finished = 1; // unused
                            PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ MPC Task Finished ++++++++++++");
                        }
                        // pure_pursuit_planner finished
                        if (ev_task_finished->task_id == purepursuit_task_id)
                        {
                            pp_finished = 1; // unused
                            PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ PurePursuit Task Finished ++++++++++++");
                        }
                    }
                }
                // for ultrasonic/local_map emergency events
                for (auto &evt_us : evt_list_for_us)
                {
                    std::type_index evt_type = evt_us->getType();
                    // // US Lasting Events
                    // if (TYPE_EQUALS(evt_type, EvUSFrontLasting))
                    // {
                    //     us_lasting_front = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_front = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvUSFrontLeftLasting))
                    // {
                    //     us_lasting_front_left = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_front_left = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvUSFrontRightLasting))
                    // {
                    //     us_lasting_front_right = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_front_right = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvUSLeftLasting))
                    // {
                    //     us_lasting_left = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_left = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvUSRightLasting))
                    // {
                    //     us_lasting_right = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ us_lasting_right = true ++++++++++++");
                    // }
                    // // LM Lasting Events
                    // if (TYPE_EQUALS(evt_type, EvLMFrontLasting))
                    // {
                    //     lm_lasting_front = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_front = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvLMFrontLeftLasting))
                    // {
                    //     lm_lasting_front_left = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_front_left = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvLMFrontRightLasting))
                    // {
                    //     lm_lasting_front_right = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_front_right = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvLMLeftLasting))
                    // {
                    //     lm_lasting_left = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_left = true ++++++++++++");
                    // }
                    // if (TYPE_EQUALS(evt_type, EvLMRightLasting))
                    // {
                    //     lm_lasting_right = true;
                    //     // PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ lm_lasting_right = true ++++++++++++");
                    // }

                    // Single US Sensor Events
                    // if (TYPE_EQUALS(evt_type, EvEmergencyUSFront))
                    // {
                    //     emergency_us_front = true;
                    // }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSFrontLeft))
                    {
                        emergency_us_front_left = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSFrontRight))
                    {
                        emergency_us_front_right = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSLeft))
                    {
                        emergency_us_left = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSRight))
                    {
                        emergency_us_right = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSLeftPure))
                    {
                        emergency_us_left_pure = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSRightPure))
                    {
                        emergency_us_right_pure = true;
                    }
                    // Single LM Events
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMFrontLeft))
                    {
                        emergency_lm_front_left = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMFrontRight))
                    {
                        emergency_lm_front_right = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMLeft))
                    {
                        emergency_lm_left = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMRight))
                    {
                        emergency_lm_right = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMLeftPure))
                    {
                        emergency_lm_left_pure = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMRightPure))
                    {
                        emergency_lm_right_pure = true;
                    }
                    // freeze
                    if (TYPE_EQUALS(evt_type, EvLMFreezeStop))
                    {
                        freeze_stop_flag = true;
                    }
                }

                // 一些接收事件的后处理
                if (teb_rotate_request_flag == 1 || Owner().m_rx_teb_path == 1)
                {
                    teb_infeasible_timer_flag = 0;
                    freezing_replanning_req_to_teb = false;
                    teb_infeasible_ending_timer = 0;
                }

                if (mpc_rx_flag == 1)
                {
                    print_mpc_return_next_index = Owner().mpc_return_index + Owner().goal_index_mpc_last;
                    print_ppp_return_next_index = Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last;
                    print_teb_return_next_index = Owner().teb_return_index + Owner().goal_index_teb_last;
                }

                // collision_lasting
                if (Owner().m_set_collis_lasting_off == 1)
                {
                    if (Owner().m_collision_flag && collision_lasting_timer == 0)
                    {
                        collision_lasting_timer = 1;
                        collision_lasting_start_ts = Timer::getSystemTimestampUS();
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "collision detected, collision lasting timer: ON");
                    }
                    else if (Owner().m_collision_flag && collision_lasting_timer == 1)
                    {
                        uint64_t collision_lasting_cur_ts = Timer::getSystemTimestampUS();
                        float collision_lasting_period = float(collision_lasting_cur_ts - collision_lasting_start_ts) / 1000000;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "collision_lasting_period:  %f/%f", collision_lasting_period, Owner().m_collision_lasting_threshold);
                        if (collision_lasting_period < Owner().m_collision_lasting_threshold)
                        {
                            freeze_stop_flag = true;
                            CREATE_EVENT(EvPFPCollisionFreeze, ev_pfp_coll_freeze);
                            g_ec.pushEvent(ev_pfp_coll_freeze);
                            // freezing_replanning_req_to_teb = true;
                            // teb_is_planning = 0;
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "collision detected, robot freezing...");
                        }
                        else
                        {
                            Owner().m_collision_lasting_flag = true;
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "collision lasting detected...");
                        }
                    }
                    else if (!Owner().m_collision_flag)
                    {
                        collision_lasting_timer = 0;
                        Owner().m_collision_lasting_flag = false;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "collision not detected, collision_lasting_flag reset, collision lasting timer: OFF");
                    }
                }

                // ******************************* 判断运行模式 *******************************

#ifdef OD_SWITCH
                if (Owner().m_small_obstacle_flag == true)
                {
                    mode_obstacle_avoidance = 1;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Chang Mode: obstacle_avoidance [ON] ++++++++++++");
                }
                else if (rad2deg(Owner().m_control_speed.angular_steer) * float(obstacle_dir_in_far_circle) > 15 && Owner().m_far_circle_obstacle_flag == true)
                {
                    mode_obstacle_avoidance = 1;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Chang Mode: obstacle_avoidance [ON] ++++++++++++");
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "judge criterion:  %f  > 15", rad2deg(Owner().m_control_speed.angular_steer) * float(obstacle_dir_in_far_circle));
                }
                // if (Owner().m_small_obstacle_flag == false && Owner().m_far_circle_obstacle_flag == false)
                // if (Owner().m_far_circle_obstacle_flag == false)
                // else if (Owner().m_small_obstacle_flag == false && Owner().m_far_circle_obstacle_flag == false && Owner().m_near_circle_obstacle_flag == false)
                else if (Owner().m_small_obstacle_flag == false && Owner().m_near_circle_obstacle_flag == false)
                {
                    mode_obstacle_avoidance = 0;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "++++++++++++ Chang Mode: obstacle_avoidance [OFF] ++++++++++++");
                }
#else
                if (Owner().m_collision_lasting_flag)
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "Owner().m_collision_lasting_flag = true");
#ifdef USE_PURE_PURSUIT
                    mode_obstacle_avoidance = 1;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Chang Mode: obstacle_avoidance [ON], Pure_Pursuit ++++++++++++");
#else
                    // if (Owner().m_rx_teb_path == 0 && teb_is_planning == 0)
                    if (last_mode_obstacle_avoidance == 0)
                    {
                        mode_obstacle_avoidance = 2; // 刚切到OA模式
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "++++++++++++ Chang Mode: obstacle_avoidance [ON], OA Mode 2 ++++++++++++");
                    }
                    // else if (Owner().m_rx_teb_path == 1 && teb_is_planning == 0)
                    // {
                    //     mode_obstacle_avoidance = 3; // 得到了teb路径的OA模式
                    //     PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "++++++++++++ Chang Mode: obstacle_avoidance [ON], OA Mode 3 ++++++++++++");
                    // }
#endif
                }
                // 特殊情况强行切OA2时，保证过渡到OA3
                if (Owner().m_rx_teb_path == 1 && teb_is_planning == 0)
                {
                    mode_obstacle_avoidance = 3; // 得到了teb路径的OA模式
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "++++++++++++ Chang Mode: obstacle_avoidance [ON], OA Mode 3 ++++++++++++");
                }
                if (!Owner().m_collision_lasting_flag) // only when !m_collision_lasting_flag && robot_pose close to mpc_path, OA set to OFF.
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "Owner().m_collision_lasting_flag = false");
#ifdef USE_PURE_PURSUIT
                    // pure_pursuit
                    RobotPose returned_pp_path_point = Owner().path_points_pure_pursuit[Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last];
                    float rtn_diff_x = returned_pp_path_point.pt.x - Owner().m_slam_data.pose.pt.x;
                    float rtn_diff_y = returned_pp_path_point.pt.y - Owner().m_slam_data.pose.pt.y;
                    float rtn_dis = hypot(rtn_diff_y, rtn_diff_x);
                    // float rtn_diff_yaw = rad2deg(abs(toNPPiAngleRangeR(atan2(rtn_diff_y, rtn_diff_x) - Owner().m_slam_data.pose.theta)));
                    if (rtn_dis < Owner().oa_switch_radius)
                    {
                        mode_obstacle_avoidance = 0;
                        // PATHFOLLOW_PLANNER_WARN_LOG("pp_index:  %d   rtn_dis:  %f", Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last, rtn_dis);
                        // PATHFOLLOW_PLANNER_WARN_LOG("rtn_pp_point:  %f  %f  %f", returned_pp_path_point.pt.x, returned_pp_path_point.pt.y, returned_pp_path_point.theta);
                        // PATHFOLLOW_PLANNER_WARN_LOG("m_slam_data:   %f  %f  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, Owner().m_slam_data.pose.theta);
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "++++++++++++ Chang Mode: obstacle_avoidance [OFF] ++++++++++++");
                    }
#else
                    // teb
                    if (Owner().end_idx_from_teb != -1)
                    {
                        // for (size_t i = Owner().goal_index_mpc; i <= (size_t)Owner().end_idx_from_teb; i++)
                        // {
                        //     RobotPose teb_end = Owner().path_points_mpc[i];
                        //     float rtn_diff_x = teb_end.pt.x - Owner().m_slam_data.pose.pt.x;
                        //     float rtn_diff_y = teb_end.pt.y - Owner().m_slam_data.pose.pt.y;
                        //     float rtn_dis = hypot(rtn_diff_y, rtn_diff_x);
                        //     // float rtn_diff_yaw = rad2deg(fabs(toNPPiAngleRangeR(atan2(rtn_diff_y, rtn_diff_x) - Owner().m_slam_data.pose.theta)));
                        //     float rtn_pose_diff_yaw = rad2deg(toNPPiAngleRangeR(teb_end.theta - Owner().m_slam_data.pose.theta));
                        //     // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "rtn_dis: %f/%f", rtn_dis, Owner().oa_switch_radius);
                        //     // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "end_idx_from_teb pose:  %f  %f  %f", teb_end.pt.x, teb_end.pt.y, teb_end.theta);
                        //     // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "current_robot    pose:  %f  %f  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, Owner().m_slam_data.pose.theta);
                        //     if (rtn_dis < Owner().oa_switch_radius)
                        //     {
                        //         mode_obstacle_avoidance = 0;
                        //         teb_is_planning = 0;
                        //         oa_switched_off_idx = (int)i;
                        //         CREATE_EVENT(EvPFPtebRelax, ev_teb_relax);
                        //         g_ec.pushEvent(ev_teb_relax);
                        //         // PATHFOLLOW_PLANNER_WARN_LOG("pp_index:  %d   rtn_dis:  %f", Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last, rtn_dis);
                        //         // PATHFOLLOW_PLANNER_WARN_LOG("rtn_pp_point:  %f  %f  %f", returned_pp_path_point.pt.x, returned_pp_path_point.pt.y, returned_pp_path_point.theta);
                        //         // PATHFOLLOW_PLANNER_WARN_LOG("m_slam_data:   %f  %f  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, Owner().m_slam_data.pose.theta);
                        //         PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "++++++++++++ Chang Mode: obstacle_avoidance [OFF] ++++++++++++");
                        //         PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "oa_switched_off_idx:  %d", oa_switched_off_idx);
                        //         PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "rtn_pose_diff_yaw:    %f", rtn_pose_diff_yaw);
                        //         // if not (rtn_diff_yaw < 60 && abs(rtn_pose_diff_yaw) < 60)
                        //         if (fabs(rtn_pose_diff_yaw) > 45)
                        //         {
                        //             // 若需要旋转, 下周期会收到mpc的点信息，目前不会覆盖path_points_mpc_situation，但是以后能改则改
                        //             rotate_request_flag = 1;
                        //             rotate_request_target_angle = teb_end.theta;
                        //             if (rtn_pose_diff_yaw > 0)
                        //             {
                        //                 rotate_request_left_dir = true;
                        //             }
                        //             else
                        //             {
                        //                 rotate_request_left_dir = false;
                        //             }
                        //         }
                        //         break;
                        //     }
                        // }
                        RobotPose teb_end = Owner().path_points_mpc[Owner().end_idx_from_teb];
                        float rtn_diff_x = teb_end.pt.x - Owner().m_slam_data.pose.pt.x;
                        float rtn_diff_y = teb_end.pt.y - Owner().m_slam_data.pose.pt.y;
                        float rtn_dis = hypot(rtn_diff_y, rtn_diff_x);
                        // float rtn_diff_yaw = rad2deg(fabs(toNPPiAngleRangeR(atan2(rtn_diff_y, rtn_diff_x) - Owner().m_slam_data.pose.theta)));
                        float rtn_pose_diff_yaw = rad2deg(toNPPiAngleRangeR(teb_end.theta - Owner().m_slam_data.pose.theta));
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "rtn_dis: %f/%f", rtn_dis, Owner().oa_switch_radius);
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "end_idx_from_teb pose:  %f  %f  %f", teb_end.pt.x, teb_end.pt.y, teb_end.theta);
                        // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "current_robot    pose:  %f  %f  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, Owner().m_slam_data.pose.theta);
                        if (rtn_dis < Owner().oa_switch_radius)
                        {
                            mode_obstacle_avoidance = 0;
                            teb_infeasible_timer_flag = 0; // 若切回时收到teb_infeasible,则关掉timer不启动infeasible计时
                            teb_is_planning = 0;
                            oa_switched_off_idx = Owner().end_idx_from_teb;
                            CREATE_EVENT(EvPFPtebRelax, ev_teb_relax);
                            g_ec.pushEvent(ev_teb_relax);
                            // PATHFOLLOW_PLANNER_WARN_LOG("pp_index:  %d   rtn_dis:  %f", Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last, rtn_dis);
                            // PATHFOLLOW_PLANNER_WARN_LOG("rtn_pp_point:  %f  %f  %f", returned_pp_path_point.pt.x, returned_pp_path_point.pt.y, returned_pp_path_point.theta);
                            // PATHFOLLOW_PLANNER_WARN_LOG("m_slam_data:   %f  %f  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, Owner().m_slam_data.pose.theta);
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "++++++++++++ Chang Mode: obstacle_avoidance [OFF] ++++++++++++");
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "oa_switched_off_idx:  %d", oa_switched_off_idx);
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "rtn_pose_diff_yaw:    %f", rtn_pose_diff_yaw);
                            // if not (rtn_diff_yaw < 60 && abs(rtn_pose_diff_yaw) < 60)
                            if (fabs(rtn_pose_diff_yaw) > 45)
                            {
                                // 若需要旋转, 下周期会收到mpc的点信息，目前不会覆盖path_points_mpc_situation，但是以后能改则改
                                rotate_request_flag = 1;
                                rotate_request_target_angle = teb_end.theta;
                                if (rtn_pose_diff_yaw > 0)
                                {
                                    rotate_request_left_dir = true;
                                }
                                else
                                {
                                    rotate_request_left_dir = false;
                                }
                            }
                        }
                    }
#endif
                }

#endif

                // if (mpc_infeasible_flag == 1 && Owner().m_rx_teb_path != 1) // mpc infeasible时用pp/teb跟线
                if (mpc_infeasible_flag != 0 && Owner().m_rx_teb_path != 1 && last_mode_obstacle_avoidance == 0) // mpc infeasible时用pp/teb跟线
                {
#ifdef USE_PURE_PURSUIT
                    mode_obstacle_avoidance = 1;
#else
                    mode_obstacle_avoidance = 2; // 刚切到OA模式
#endif
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Chang Mode: obstacle_avoidance [ON], mpc_infeasible_flag ++++++++++++");
                }

                if (Owner().m_start_deviation == 1) // 起点偏离，用pp
                {
#ifdef USE_PURE_PURSUIT
                    mode_obstacle_avoidance = 1;
#else
                    mode_obstacle_avoidance = 2; // 刚切到OA模式
#endif
                    // Owner().m_start_deviation = 0;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Chang Mode: obstacle_avoidance [ON], start deviation ++++++++++++");
                }

                // 坐标跳变
                if (slam_coord_jumped)
                {
                    // mode_obstacle_avoidance = 0; // 坐标跳变时：用mpc强制刷掉infeasible的点，以后也有可能改成变为FREEZE,等坐标稳定再动
                    freeze_stop_flag = true;
                    // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ Chang Mode: obstacle_avoidance [OFF], slam_coord_jumped ++++++++++++");
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "++++++++++++ slam_coord_jumped ++++++++++++");
                }

                // 强行切OA = 0, 因为补点补到终点了
                if (force_to_OA_0)
                {
                    mode_obstacle_avoidance = 0;
                    teb_is_planning = 0; // 切回来要记得置0,否则下次进OA2不会给teb发送请求
                    oa_switched_off_idx = Owner().end_idx_from_teb;
                    force_to_OA_0 = false;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "++++++++++++ Chang Mode: obstacle_avoidance [OFF], force_to_OA_0 ++++++++++++");
                }

                // set OA mode for test
                if (Owner().m_set_oa_off == 1)
                {
                    mode_obstacle_avoidance = 0;
                }
                else if (Owner().m_set_oa_off == 2)
                {
                    // #ifdef USE_PURE_PURSUIT
                    //                     mode_obstacle_avoidance = 1;
                    // #else
                    //                     mode_obstacle_avoidance = 2;
                    // #endif
                }
                last_mode_obstacle_avoidance = mode_obstacle_avoidance;
                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "Current OA Mode:  %d  Last OA Mode:  %d", mode_obstacle_avoidance, last_mode_obstacle_avoidance);

                // ******************************* Send Event of OA_MODE *******************************
                CREATE_EVENT(EvPFPOAMode, ev_pfp_oa_mode);
                if (mode_obstacle_avoidance == 0)
                {
                    ev_pfp_oa_mode->oa_is_on = false;
                }
                else
                {
                    ev_pfp_oa_mode->oa_is_on = true;
                }
                g_ec.pushEvent(ev_pfp_oa_mode);

                // ******************************* 不同模式的额外预处理，使轨迹同步 *******************************
                if (mode_obstacle_avoidance == 0 || mode_obstacle_avoidance == 2)
                {
                    if (oa_switched_off_idx == -1)
                    {
                        // 从pure pursuit切到mpc
                        while (Owner().pair_pp_index_to_mpc_index[Owner().pure_pursuit_return_next_sparse_index].second > Owner().mpc_return_next_dense_index)
                        {
                            PATHFOLLOW_PLANNER_INFO_LOG("mode_obstacle_avoidance = 0, switch to mpc from pure pursuit");
                            PATHFOLLOW_PLANNER_INFO_LOG("sparse: %u   dense: %u  mpc_return_next_dense_index: %u", Owner().pure_pursuit_return_next_sparse_index, Owner().pair_pp_index_to_mpc_index[Owner().pure_pursuit_return_next_sparse_index].second, Owner().mpc_return_next_dense_index);

                            float tmp_diff_y = Owner().path_points_mpc[Owner().mpc_return_next_dense_index].pt.y - Owner().m_slam_data.pose.pt.y;
                            float tmp_diff_x = Owner().path_points_mpc[Owner().mpc_return_next_dense_index].pt.x - Owner().m_slam_data.pose.pt.x;
                            float tmp_dis = hypot(tmp_diff_y, tmp_diff_x);
                            float tmp_diff_yaw = rad2deg(abs(toNPPiAngleRangeR(atan2(tmp_diff_y, tmp_diff_x) - Owner().m_slam_data.pose.theta)));
                            if (tmp_dis < Owner().max_jmp_r_pp2mpc && tmp_diff_yaw < Owner().jmp_tol_angle)
                            {
                                break;
                            }
                            else
                            {
                                Owner().path_points_mpc_situation[Owner().mpc_return_next_dense_index] = 2;
                                Owner().mpc_return_next_dense_index++;
                            }
                        }
                    }
                    else
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("Switch from teb to mpc, changing path_points_mpc_situation");
                        // 从teb切到mpc, 根据其返回的end_idx, 把相应的path_points_mpc_situation更改
                        for (int i = 0; i <= oa_switched_off_idx; i++)
                        {
                            if (Owner().path_points_mpc_situation[i] == 0)
                            {
                                Owner().path_points_mpc_situation[i] = 4;
                            }
                        }
                    }
                }
                if (mode_obstacle_avoidance == 1)
                {
                    // 从mpc切到pure pursuit
                    // for (size_t i = 0; i < Owner().pair_pp_index_to_mpc_index.size(); i++)
                    for (size_t i = Owner().pure_pursuit_return_next_sparse_index; i < Owner().pair_pp_index_to_mpc_index.size(); i++)
                    {
                        Owner().pure_pursuit_return_next_sparse_index = Owner().pair_pp_index_to_mpc_index[i].first;
                        if (Owner().mpc_return_next_dense_index > Owner().pair_pp_index_to_mpc_index[i].second)
                        {
                            // Owner().path_points_sparse_skipped[Owner().pure_pursuit_return_next_sparse_index] = true;
                            Owner().path_points_pure_pursuit_situation[Owner().pure_pursuit_return_next_sparse_index] = 2;
                        }
                        else
                        {
                            size_t tmp_idx1 = Owner().pair_pp_index_to_mpc_index[i].second;
                            size_t tmp_idx2 = Owner().mpc_return_next_dense_index;
                            float tmp_distance = hypot(Owner().path_points_mpc[tmp_idx1].pt.x - Owner().path_points_mpc[tmp_idx2].pt.x, Owner().path_points_mpc[tmp_idx1].pt.y - Owner().path_points_mpc[tmp_idx2].pt.y);
                            if (tmp_distance < Owner().jmp_r_mpc2pp)
                            {
                                // Owner().path_points_sparse_skipped[Owner().pure_pursuit_return_next_sparse_index] = true;
                                Owner().path_points_pure_pursuit_situation[Owner().pure_pursuit_return_next_sparse_index] = 2;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                if (mode_obstacle_avoidance == 3)
                {
                    // if (Owner().m_rx_teb_path == 1)
                    // {
                    //     收到teb path则根据其返回的end_idx, 把相应的path_points_mpc_situation更改
                    //     for (int i = 0; i <= Owner().end_idx_from_teb; i++)
                    //     {
                    //         if (Owner().path_points_mpc_situation[i] == 0)
                    //         {
                    //             Owner().path_points_mpc_situation[i] = 4;
                    //         }
                    //     }
                    // }
                }

                // ******************************* update next index *******************************
                if (mode_obstacle_avoidance == 0 || mode_obstacle_avoidance == 2)
                {
                    if (oa_switched_off_idx == -1 && mpc_rx_flag == 1) // 刚切回来的这个周期内，不做这些操作
                    {
                        // Owner().mpc_return_next_dense_index = max(Owner().mpc_return_index + Owner().goal_index_mpc_last, Owner().mpc_return_next_dense_index);
                        if (Owner().mpc_return_next_dense_index > Owner().mpc_return_index + Owner().goal_index_mpc_last)
                        {
                            index_jumped = 1;
                        }
                        else
                        {
                            Owner().mpc_return_next_dense_index = Owner().mpc_return_index + Owner().goal_index_mpc_last;
                        }
                        PATHFOLLOW_PLANNER_INFO_LOG("mpc_return_next_dense_index:  %u  goal_index_mpc_last:  %u  mpc_return_index:  %u", Owner().mpc_return_next_dense_index, Owner().goal_index_mpc_last, Owner().mpc_return_index);
                        // PATHFOLLOW_PLANNER_INFO_LOG("path_points_dense_reached.size():  %u  path_points_dense_skipped.size():  %u", Owner().path_points_dense_reached.size(), Owner().path_points_dense_skipped.size());
                    }
                    else
                    {
                        if (mode_obstacle_avoidance == 0)
                        {
                            Owner().end_idx_from_teb = -1; // 切回MPC, Owner().end_idx_from_teb 完成使命重置, 只有OA2重置
                            PATHFOLLOW_PLANNER_INFO_LOG("Owner().end_idx_from_teb = -1");
                        }
                    }
                }
                if (mode_obstacle_avoidance == 1)
                {
                    Owner().pure_pursuit_return_next_sparse_index = max(Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last, Owner().pure_pursuit_return_next_sparse_index);
                    if (Owner().pure_pursuit_return_next_sparse_index > Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last)
                    {
                        index_jumped = 1;
                    }
                    else
                    {
                        Owner().pure_pursuit_return_next_sparse_index = Owner().pure_pursuit_return_index + Owner().goal_index_pure_pursuit_last;
                    }
                    PATHFOLLOW_PLANNER_INFO_LOG("pp_return_next_sparse_index:  %u  goal_index_pure_pursuit_last:  %u  pp_return_index:  %u", Owner().pure_pursuit_return_next_sparse_index, Owner().goal_index_pure_pursuit_last, Owner().pure_pursuit_return_index);
                    // PATHFOLLOW_PLANNER_INFO_LOG("path_points_sparse_reached.size():  %u  path_points_sparse_skipped.size():  %u", Owner().path_points_sparse_reached.size(), Owner().path_points_sparse_skipped.size());
                }
                if (mode_obstacle_avoidance == 3)
                {
                }

                // ******************************* update & prepare for next task *******************************
                if (mode_obstacle_avoidance != 3 && index_jumped == 0 && oa_switched_off_idx == -1 && mpc_rx_flag == 1) // 刚切回来的这个周期内，不做这些操作
                {
                    Owner().update_path_index_and_reached_skipped();
                }
                if (mode_obstacle_avoidance == 0 || mode_obstacle_avoidance == 1 || mode_obstacle_avoidance == 2)
                {
                    Owner().prepare_for_next_task();
                    PATHFOLLOW_PLANNER_INFO_LOG("Get Robot Current Pose:  %f  %f  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y, rad2deg(Owner().m_slam_data.pose.theta));
                    Owner().op_local_planner_sparse_and_dense();

                    // 如果未来path_to_mpc在车身后方，可以请求rotate
                    if (mode_obstacle_avoidance == 0 && Owner().path_to_mpc.size() >= 5)
                    {
                        // 构造从路径头指向路径尾的vector
                        float vector_yaw = atan2(Owner().path_to_mpc[4].pt.y - Owner().path_to_mpc[0].pt.y, Owner().path_to_mpc[4].pt.x - Owner().path_to_mpc[0].pt.x);
                        float yaw_diff = rad2deg(toNPPiAngleRangeR(vector_yaw - Owner().m_slam_data.pose.theta));
                        PATHFOLLOW_PLANNER_INFO_LOG("vector path_to_mpc:  vector_yaw:  %f    yaw_diff:  %f", rad2deg(vector_yaw), yaw_diff);
                        if (fabs(yaw_diff) >= 90)
                        {
                            rotate_request_flag = 1;
                            rotate_request_target_angle = vector_yaw;
                            if (yaw_diff > 0)
                            {
                                rotate_request_left_dir = true;
                                PATHFOLLOW_PLANNER_INFO_LOG("vector path_to_mpc requests left rotation");
                            }
                            else
                            {
                                rotate_request_left_dir = false;
                                PATHFOLLOW_PLANNER_INFO_LOG("vector path_to_mpc requests right rotation");
                            }
                        }
                    }
                }
                if (mode_obstacle_avoidance == 2 && teb_is_planning == 0)
                {
                    // Owner().path_to_teb_states.clear();
                    // 给teb预备路径
                    Owner().path_to_teb_prepare.clear();
                    size_t upper_point_index = ((Owner().goal_index_mpc + Owner().m_first_to_teb_num) > Owner().path_points_mpc.size()) ? Owner().path_points_mpc.size() : (Owner().goal_index_mpc + Owner().m_first_to_teb_num);
                    for (size_t i = Owner().goal_index_mpc; i < upper_point_index; i++)
                    {
                        Owner().path_to_teb_prepare.add(Owner().path_points_mpc[i]);
                    }
                    // for (size_t i = 0; i < Owner().path_to_teb_prepare.size(); i++)
                    // {
                    // Owner().path_to_teb_states.push_back(0);
                    // }
                }
                // next path point direction, for front lasting escape rotation direction
                float tmp_yaw_next_path_point = 0;
                if (mode_obstacle_avoidance == 0) // mpc
                {
                    size_t tmp_idx_mpc = Owner().path_to_mpc.size() - 1;
                    tmp_yaw_next_path_point = toNPPiAngleRangeR(atan2((Owner().path_to_mpc[tmp_idx_mpc].pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_mpc[tmp_idx_mpc].pt.x - Owner().m_slam_data.pose.pt.x)) - Owner().m_slam_data.pose.theta);
                }
                else if (mode_obstacle_avoidance == 1) // pure pursuit
                {
                    size_t tmp_idx_pp = Owner().path_to_pure_pursuit.size() - 1;
                    tmp_yaw_next_path_point = toNPPiAngleRangeR(atan2((Owner().path_to_pure_pursuit[tmp_idx_pp].pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_pure_pursuit[tmp_idx_pp].pt.x - Owner().m_slam_data.pose.pt.x)) - Owner().m_slam_data.pose.theta);
                }
                else if (mode_obstacle_avoidance == 2) // 刚切到OA模式
                {
                    size_t tmp_idx_oa2 = Owner().path_buffer.size() - 1;
                    tmp_yaw_next_path_point = toNPPiAngleRangeR(atan2((Owner().path_to_mpc[tmp_idx_oa2].pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_mpc[tmp_idx_oa2].pt.x - Owner().m_slam_data.pose.pt.x)) - Owner().m_slam_data.pose.theta);
                }
                else if (mode_obstacle_avoidance == 3)
                {
                    size_t tmp_idx_oa3 = Owner().path_buffer.size() - 1;
                    tmp_yaw_next_path_point = toNPPiAngleRangeR(atan2((Owner().path_to_mpc[tmp_idx_oa3].pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_mpc[tmp_idx_oa3].pt.x - Owner().m_slam_data.pose.pt.x)) - Owner().m_slam_data.pose.theta);
                }
                PATHFOLLOW_PLANNER_INFO_LOG("tmp_yaw_next_path_point: %f", tmp_yaw_next_path_point);

                // ******************************* compute speed according to curvature *******************************
                Owner().m_planning_velocity = Owner().m_min_velocity;
                if (mode_obstacle_avoidance == 0 && oa_switched_off_idx == -1) // 刚切回mpc仍低速
                {
                    float path_avg_curv = Owner().m_max_curvature;
                    uint64_t curvature_start_ts = Timer::getTimestampUS();
                    path_avg_curv = Owner().getPathAverageCurvature(Owner().path_to_mpc);
                    // float tmp_curv = Owner().getPathAverageCurvature(Owner().path_to_mpc);
                    // if (tmp_curv >= last_curvature) // 缓加速
                    // {
                    //     path_avg_curv = 0.01 * tmp_curv + 0.99 * last_curvature;
                    // }
                    // else // 急减速
                    // {
                    //     path_avg_curv = 0.8 * tmp_curv + 0.2 * last_curvature;
                    // }
                    // last_curvature = path_avg_curv;
                    uint64_t curvature_end_ts = Timer::getTimestampUS();
                    float curvature_period = float(curvature_end_ts - curvature_start_ts) / 1000000;
                    // v = q*m + p; m = sqrt(1/k);
                    float q = (Owner().m_max_velocity - Owner().m_min_velocity) / (sqrt(1 / Owner().m_min_curvature) - sqrt(1 / Owner().m_max_curvature));
                    float p = Owner().m_max_velocity - q * sqrt(1 / Owner().m_min_curvature);
                    // Owner().m_planning_velocity = q * sqrt(1 / path_avg_curv) + p;
                    float tmp_velocity = q * sqrt(1 / path_avg_curv) + p;
                    if (tmp_velocity >= last_velocity) // 缓加速
                    {
                        Owner().m_planning_velocity = 0.1 * tmp_velocity + 0.9 * last_velocity;
                    }
                    else // 急减速
                    {
                        Owner().m_planning_velocity = 0.8 * tmp_velocity + 0.2 * last_velocity;
                    }
                    // 限辐
                    if (Owner().m_planning_velocity > Owner().m_max_velocity)
                    {
                        Owner().m_planning_velocity = Owner().m_max_velocity;
                    }
                    else if (Owner().m_planning_velocity < Owner().m_min_velocity)
                    {
                        Owner().m_planning_velocity = Owner().m_min_velocity;
                    }
                    last_velocity = Owner().m_planning_velocity;
                    uint64_t compute_v_end_ts = Timer::getTimestampUS();
                    float compute_v_period = float(compute_v_end_ts - curvature_start_ts) / 1000000;
                    // PATHFOLLOW_PLANNER_WARN_LOG("curvature_period:  %f   compute_v_period:  %f", curvature_period, compute_v_period);
                    // PATHFOLLOW_PLANNER_WARN_LOG("path_avg_curv:  %f", path_avg_curv);
                    PATHFOLLOW_PLANNER_WARN_LOG("mpc_velocity:   %f", Owner().m_planning_velocity);
                    // 是否设置定速模式(mpc的速度)
                    if (Owner().m_const_velocity == 1)
                    {
                        Owner().m_planning_velocity = Owner().m_test_velocity;
                    }
                    // close to final point need to slowdown
                    if ((int)Owner().path_to_mpc.size() < Owner().m_points_num_to_mpc)
                    {
                        Owner().m_planning_velocity = Owner().m_min_velocity;
                    }
                }

                // ******************************* generate path to teb if needed *******************************
                if (((mode_obstacle_avoidance == 2 && teb_is_planning == 1) || mode_obstacle_avoidance == 3) && teb_infeasible_ending_timer != 1)
                {
                    if (!teb_path_is_robot_as_start)
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Generating Path to teb: mode2&isPlanning or mode3");
                        PosePath tmp_path;
                        tmp_path.clear();
                        for (int i = (int)Owner().mpc_return_index; i < (int)(Owner().path_buffer.size()) - 1; i++)
                        {
                            tmp_path.add(Owner().path_buffer[i]);
                        }
                        Owner().path_buffer.clear();
                        Owner().path_buffer = tmp_path;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "update path_buffer.size: %d", Owner().path_buffer.size());
                        // Owner().goal_index_mpc = Owner().mpc_return_index + Owner().goal_index_mpc_last;
                    }
                    else
                    {
                        // 若teb_path_is_robot_as_start则清空buffer,否则会积累
                        Owner().path_buffer.clear();
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "teb_path_is_robot_as_start, path_buffer clear");
                    }
                }
                else if (mode_obstacle_avoidance == 2 && teb_is_planning == 0)
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Generating Path to teb: mode2&notPlanning");
                    // get start index of path_to_teb & fill path_to_teb with path_to_teb_prepare
                    size_t start_idx_path_to_teb = Owner().goal_index_mpc;
                    bool got_start_idx = false;
                    Owner().path_to_teb.clear();
                    Owner().path_buffer.clear();
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "(First) path_to_teb_prepare.size: %d", Owner().path_to_teb_prepare.size());
                    for (size_t i = 0; i < Owner().path_to_teb_prepare.size(); i++)
                    {
                        float dis = hypot((Owner().path_to_teb_prepare[i].pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_teb_prepare[i].pt.x - Owner().m_slam_data.pose.pt.x));
                        if (dis > Owner().m_planning_velocity / Owner().m_running_freq)
                        {
                            if (!got_start_idx)
                            {
                                start_idx_path_to_teb = i + Owner().goal_index_mpc;
                                got_start_idx = true;
                            }
                            Owner().path_to_teb.add(Owner().path_to_teb_prepare[i]);
                        }
                        else
                        {
                            Owner().path_buffer.add(Owner().path_to_teb_prepare[i]);
                        }
                    }
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "(First) path_buffer.size: %d", Owner().path_buffer.size());
                    bool check_finished = false;
                    bool no_need_to_append = false;
                    for (size_t i = start_idx_path_to_teb + Owner().path_to_teb.size() - 1; i < Owner().path_points_mpc.size() - 1; i++)
                    {
                        for (int p = 0; p < Owner().m_local_map_size_x; p++)
                        {
                            for (int q = 0; q < Owner().m_local_map_size_y; q++)
                            {
                                if (Owner().m_simple_local_map.grid[p][q] != 0)
                                {
                                    if (Owner().check_path_point_collision(p, q, i))
                                    {
                                        Owner().path_to_teb.add(Owner().path_points_mpc[i + 1]);
                                        check_finished = true;
                                        break;
                                    }
                                }
                                if (p == Owner().m_local_map_size_x - 1 && q == Owner().m_local_map_size_y - 1)
                                {
                                    no_need_to_append = true;
                                }
                            }
                            if (check_finished)
                            {
                                check_finished = false;
                                break;
                            }
                        }
                        if (no_need_to_append)
                            break;
                    }
                    // creat teb planning request event
                    if (!Owner().path_to_teb.empty())
                    {
                        // get end index of path_to_teb
                        size_t end_idx_path_to_teb = start_idx_path_to_teb + Owner().path_to_teb.size() - 1;
                        // send teb planning request event
                        CREATE_EVENT(EvPFPtebPlanningReq, evt_teb_planning_req);
                        // evt_teb_planning_req->start_idx = start_idx_path_to_teb;
                        evt_teb_planning_req->end_idx = end_idx_path_to_teb;
                        std::shared_ptr<PosePath> tmp_teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());
                        for (size_t i = 0; i < Owner().path_to_teb.size(); i++)
                        {
                            tmp_teb_path_ptr->add(Owner().path_to_teb[i]);
                        }
                        evt_teb_planning_req->path_to_teb_ptr = tmp_teb_path_ptr;
                        if (end_idx_path_to_teb == Owner().path_points_mpc.size() - 1)
                        {
                            evt_teb_planning_req->is_end = true;
                            Owner().end_idx_from_teb = (int)end_idx_path_to_teb;
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Owner().end_idx_from_teb = Owner().path_points_mpc.size() - 1");
                        }
                        if (Owner().m_start_deviation == 1 || mpc_infeasible_flag != 0 || freezing_replanning_req_to_teb)
                        {
                            evt_teb_planning_req->robot_pose_as_start = true;
                            Owner().m_start_deviation = 0;
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "evt_teb_planning_req->robot_pose_as_start = true");
                        }
                        // first time request
                        evt_teb_planning_req->is_first = true;
                        g_ec.pushEvent(evt_teb_planning_req);
                        teb_is_planning = 1;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "TebPlanningReq(First) sent, path_to_teb.size: %d", Owner().path_to_teb.size());
                    }
                }

                // if (mode_obstacle_avoidance == 3 && (Owner().m_rx_teb_path == 1 || teb_infeasible_flag == 1))
                if ((mode_obstacle_avoidance == 3 && Owner().m_rx_teb_path == 1) || teb_infeasible_flag == 1 || teb_infeasible_ending_timer == 1)
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Generating Path to teb: mode3&RxTebPath or teb infeasible or teb infeasible ending timer");
                    // 拼起path_to_teb_prepare
                    int added_points_num_1 = 0;
                    if (mode_obstacle_avoidance == 3 && Owner().m_rx_teb_path == 1) // 若OA = 2且teb_infeasible_flag = 1, 则不用清Owner().path_to_teb_prepare
                    {
                        Owner().path_to_teb_prepare.clear();
                        for (size_t i = 0; i < Owner().path_buffer.size(); i++)
                        {
                            Owner().path_to_teb_prepare.add(Owner().path_buffer[i]);
                        }
                        for (size_t i = 0; i < Owner().path_from_teb.size(); i++)
                        {
                            Owner().path_to_teb_prepare.add(Owner().path_from_teb[i]);
                        }
                    }
                    float teb_end_dis = 0;
                    teb_end_dis = hypot((Owner().path_to_teb_prepare.back().pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_teb_prepare.back().pt.x - Owner().m_slam_data.pose.pt.x));
                    // for (size_t i = 1; i < Owner().path_to_teb_prepare.size(); i++)
                    // {
                    //     teb_end_dis += hypot((Owner().path_to_teb_prepare[i].pt.y - Owner().path_to_teb_prepare[i - 1].pt.y), (Owner().path_to_teb_prepare[i].pt.x - Owner().path_to_teb_prepare[i - 1].pt.x));
                    // }
                    if ((teb_end_dis < 2 && Owner().m_collision_lasting_flag) || teb_infeasible_flag == 1) // 且未来有碰撞, 或teb infeasible
                    {
                        int need_to_fill_point_num = 20;
                        size_t upper_point_index = (((size_t)Owner().end_idx_from_teb + 1 + (size_t)need_to_fill_point_num) > Owner().path_points_mpc.size()) ? Owner().path_points_mpc.size() : ((size_t)Owner().end_idx_from_teb + 1 + (size_t)need_to_fill_point_num);
                        for (size_t i = (size_t)Owner().end_idx_from_teb + 1; i < upper_point_index; i++)
                        {
                            Owner().path_to_teb_prepare.add(Owner().path_points_mpc[i]);
                            added_points_num_1++;
                        }
                        Owner().end_idx_from_teb += added_points_num_1;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "teb_end_dis < 2 && Owner().m_collision_lasting_flag, or teb infeasible, added_points_num_1: %d", added_points_num_1);
                    }
                    // if (Owner().path_from_teb.size() < 20)
                    // {
                    //     int need_to_fill_point_num = 20;
                    //     size_t upper_point_index = (((size_t)Owner().end_idx_from_teb + (size_t)need_to_fill_point_num) > Owner().path_points_mpc.size()) ? Owner().path_points_mpc.size() : ((size_t)Owner().end_idx_from_teb + (size_t)need_to_fill_point_num);
                    //     for (size_t i = (size_t)Owner().end_idx_from_teb; i < upper_point_index; i++)
                    //     {
                    //         Owner().path_to_teb_prepare.add(Owner().path_points_mpc[i]);
                    //         added_points_num++;
                    //     }
                    //     PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_from_teb.size() < 20, added_points_num: %d", added_points_num);
                    // }
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_to_teb_prepare.size: %d", Owner().path_to_teb_prepare.size());
                    // get start index of path_to_teb & fill path_to_teb with path_to_teb_prepare
                    // size_t start_idx_path_to_teb = 0;
                    // bool got_start_idx = false;
                    Owner().path_to_teb.clear();
                    Owner().path_buffer.clear();
                    for (size_t i = 0; i < Owner().path_to_teb_prepare.size(); i++)
                    {
                        float dis = hypot((Owner().path_to_teb_prepare[i].pt.y - Owner().m_slam_data.pose.pt.y), (Owner().path_to_teb_prepare[i].pt.x - Owner().m_slam_data.pose.pt.x));
                        if (dis > Owner().m_necessary_T_num_for_buffer * Owner().m_planning_velocity / Owner().m_running_freq)
                        {
                            // if (!got_start_idx)
                            // {
                            //     start_idx_path_to_teb = i + Owner().goal_index_mpc;
                            //     got_start_idx = true;
                            // }
                            Owner().path_to_teb.add(Owner().path_to_teb_prepare[i]);
                        }
                        else
                        {
                            Owner().path_buffer.add(Owner().path_to_teb_prepare[i]);
                        }
                    }
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_buffer.size: %d", Owner().path_buffer.size());
                    // check if the final point of path_to_teb on obstacle
                    bool check_finished = false;
                    bool no_need_to_append = false;
                    int added_points_num_2 = 0;
                    for (size_t i = (size_t)Owner().end_idx_from_teb; i < Owner().path_points_mpc.size() - 1; i++)
                    {
                        for (int p = 0; p < Owner().m_local_map_size_x; p++)
                        {
                            for (int q = 0; q < Owner().m_local_map_size_y; q++)
                            {
                                if (Owner().m_simple_local_map.grid[p][q] != 0)
                                {
                                    if (Owner().check_path_point_collision(p, q, i))
                                    {
                                        Owner().path_to_teb.add(Owner().path_points_mpc[i + 1]);
                                        added_points_num_2++;
                                        check_finished = true;
                                        break;
                                    }
                                }
                                if (p == Owner().m_local_map_size_x - 1 && q == Owner().m_local_map_size_y - 1)
                                {
                                    no_need_to_append = true;
                                }
                            }
                            if (check_finished)
                            {
                                check_finished = false;
                                break;
                            }
                        }
                        if (no_need_to_append)
                            break;
                    }
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "added_points_num_2: %d", added_points_num_2);
                    // creat teb planning request event
                    if (!Owner().path_to_teb.empty())
                    {
                        if (Owner().end_idx_from_teb == Owner().path_points_mpc.size() - 1 && teb_infeasible_flag == 1)
                        {
                            freeze_stop_flag = true;
                            PATHFOLLOW_PLANNER_INFO_LOG("teb_infeasible_ending...");
                            if (teb_infeasible_ending_timer == 0)
                            {
                                teb_infeasible_ending_timer_start_ts = Timer::getSystemTimestampUS();
                                teb_infeasible_ending_timer = 1;
                                PATHFOLLOW_PLANNER_INFO_LOG("teb_infeasible_ending_timer = 1");
                            }
                        }
                        else
                        {
                            // get end index of path_to_teb
                            size_t end_idx_path_to_teb = Owner().end_idx_from_teb + added_points_num_2;
                            // send teb planning request event
                            CREATE_EVENT(EvPFPtebPlanningReq, evt_teb_planning_req);
                            // evt_teb_planning_req->start_idx = start_idx_path_to_teb;
                            evt_teb_planning_req->end_idx = end_idx_path_to_teb;
                            std::shared_ptr<PosePath> tmp_teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());
                            for (size_t i = 0; i < Owner().path_to_teb.size(); i++)
                            {
                                tmp_teb_path_ptr->add(Owner().path_to_teb[i]);
                            }
                            evt_teb_planning_req->path_to_teb_ptr = tmp_teb_path_ptr;
                            if (end_idx_path_to_teb == Owner().path_points_mpc.size() - 1)
                            {
                                evt_teb_planning_req->is_end = true;
                                Owner().end_idx_from_teb = (int)end_idx_path_to_teb;
                                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Owner().end_idx_from_teb = Owner().path_points_mpc.size() - 1");
                            }
                            if (mpc_infeasible_flag != 0 || freezing_replanning_req_to_teb)
                            {
                                evt_teb_planning_req->robot_pose_as_start = true;
                                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "mpc infeasible or freezing replanning, evt_teb_planning_req->robot_pose_as_start = true");
                            }
                            g_ec.pushEvent(evt_teb_planning_req);
                            teb_is_planning = 1;
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "TebPlanningReq sent, path_to_teb.size: %d", Owner().path_to_teb.size());
                            if (Owner().path_to_teb.size() == 2)
                            {
                                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_to_teb.size[0]:  %f  %f  %f", Owner().path_to_teb[0].pt.x, Owner().path_to_teb[0].pt.y, rad2deg(Owner().path_to_teb[0].theta));
                                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_to_teb.size[1]:  %f  %f  %f", Owner().path_to_teb[1].pt.x, Owner().path_to_teb[1].pt.y, rad2deg(Owner().path_to_teb[1].theta));
                            }
                        }
                    }
                    else // Owner().path_to_teb为空
                    {
                        if (Owner().end_idx_from_teb == Owner().path_points_mpc.size() - 1) // 补点补到终点了
                        {
                            // 强行切OA=0, 下一次update会自动跳转finished
                            force_to_OA_0 = true;
                            PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Owner().path_to_teb.empty(), force_to_OA_0 = true");
                        }
                    }
                }
                else if (mode_obstacle_avoidance == 3 && Owner().m_rx_teb_path == 0)
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "Generating Path to teb: mode3&notRxTebPath");
                    // 拼起path_to_teb_prepare
                    Owner().path_to_teb_prepare.clear();
                    for (size_t i = 0; i < Owner().path_buffer.size(); i++)
                    {
                        Owner().path_to_teb_prepare.add(Owner().path_buffer[i]);
                    }
                    for (size_t i = 0; i < Owner().path_to_teb.size(); i++)
                    {
                        Owner().path_to_teb_prepare.add(Owner().path_to_teb[i]);
                    }

                    if (Owner().path_buffer.empty())
                    {
                        force_to_OA_0 = true;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_CYAN "path_buffer.empty, force_to_OA_0");
                    }
                }

#ifndef OD_SWITCH
                // ******************************* update param for every detector *******************************
                // collision_detector
                path_points_to_cd.clear();
                if (mode_obstacle_avoidance == 0 || mode_obstacle_avoidance == 2)
                {
                    for (size_t i = Owner().goal_index_mpc; i < Owner().path_points_mpc.size(); i++)
                    {
                        path_points_to_cd.add(Owner().path_points_mpc[i]);
                    }
                }
                else if (mode_obstacle_avoidance == 1)
                {
                    for (size_t i = Owner().pair_pp_index_to_mpc_index[Owner().goal_index_pure_pursuit].second; i < Owner().path_points_mpc.size(); i++)
                    {
                        path_points_to_cd.add(Owner().path_points_mpc[i]);
                    }
                }
                else if (mode_obstacle_avoidance == 3)
                {
                    // 选出最近的点作为起点
                    int start_idx_selected = -1;
                    float min_distance = 1000;
                    for (size_t i = Owner().goal_index_mpc; i <= (size_t)Owner().end_idx_from_teb; i++)
                    {
                        float diff_x = Owner().path_points_mpc[i].pt.x - Owner().m_slam_data.pose.pt.x;
                        float diff_y = Owner().path_points_mpc[i].pt.y - Owner().m_slam_data.pose.pt.y;
                        float tmp_dist = hypot(diff_x, diff_y);
                        if (tmp_dist < min_distance)
                        {
                            min_distance = tmp_dist;
                            start_idx_selected = (int)i;
                        }
                    }
                    for (size_t i = (size_t)start_idx_selected; i < Owner().path_points_mpc.size(); i++)
                    {
                        path_points_to_cd.add(Owner().path_points_mpc[i]);
                    }
                }
                g_collision_detector.stopDetect();
                if (!path_points_to_cd.empty())
                {
                    CREATE_DETECTOR_PARAM(CollisionDetectorParam, collision_detector_param);
                    collision_detector_param->current_path_points = path_points_to_cd;
                    collision_detector_param->vehicle_velocity = Owner().m_planning_velocity;
                    collision_detector_param->path_points_resolution = 0.05;
                    g_collision_detector.startDetect(collision_detector_param);
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_PURPLE "path_points_to_cd.size: %lu", path_points_to_cd.size());
                }

#endif

                // ******************************* check teb_infeasible_timer *******************************
                if (teb_infeasible_timer_flag == 1 && teb_infeasible_ending_timer == 0)
                {
                    uint64_t teb_infeasible_timer_cur_ts = Timer::getSystemTimestampUS();
                    float infeasible_period = float(teb_infeasible_timer_cur_ts - teb_infeasible_timer_start_ts) / 1000000;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "infeasible_period:  %f", infeasible_period);
                    if (infeasible_period > 5 & infeasible_period <= 180)
                    {
                        // Owner().m_path_follow_error = 5;
                        // teb_infeasible_timer_flag = 0;
                        freeze_stop_flag = true;
                        freezing_replanning_req_to_teb = true;
                        teb_is_planning = 0;
                        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "teb infeasible, robot freezing & send replanning_req_to_teb...");
                    }
                    else if (infeasible_period > 180)
                    {
                        Owner().m_path_follow_error = 5;
                        teb_infeasible_timer_flag = 0;
                    }
                }
                else if (teb_infeasible_ending_timer == 1)
                {
                    freeze_stop_flag = true;
                    freezing_replanning_req_to_teb = true;
                    teb_is_planning = 0;
                    uint64_t teb_infeasible_ending_timer_cur_ts = Timer::getSystemTimestampUS();
                    float teb_infeasible_ending_period = float(teb_infeasible_ending_timer_cur_ts - teb_infeasible_ending_timer_start_ts) / 1000000;
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "teb_infeasible_ending_period:  %f", teb_infeasible_ending_period);
                    if (teb_infeasible_ending_period > 30)
                    {
                        teb_infeasible_ending_timer = 0;
                        teb_infeasible_timer_flag = 0;
                        force_to_OA_0 = true;
                    }
                }

                // ******************************* check Path Follow finish *******************************
                size_t available_dense_index = Owner().path_points_mpc.size();
                for (size_t i = 0; i < Owner().path_points_mpc.size(); i++)
                {
                    // if (Owner().path_points_dense_reached[i] == false && Owner().path_points_dense_skipped[i] == false)
                    if (Owner().path_points_mpc_situation[i] == 0)
                    {
                        available_dense_index = i;
                        break;
                    }
                }
                size_t available_sparse_index = Owner().path_points_pure_pursuit.size();
                for (size_t i = 0; i < Owner().path_points_pure_pursuit.size(); i++)
                {
                    // if (Owner().path_points_sparse_reached[i] == false && Owner().path_points_sparse_skipped[i] == false)
                    if (Owner().path_points_pure_pursuit_situation[i] == 0)
                    {
                        available_sparse_index = i;
                        break;
                    }
                }
                PATHFOLLOW_PLANNER_INFO_LOG("available_dense_index:   %d/%d", available_dense_index, Owner().path_points_mpc.size() - 1);
                PATHFOLLOW_PLANNER_INFO_LOG("available_sparse_index:  %d/%d", available_sparse_index, Owner().path_points_pure_pursuit.size() - 1);
                if (available_dense_index == Owner().path_points_mpc.size() && (mpc_finished == 1 || mpc_infeasible_flag != 0 || Owner().path_to_mpc.empty()))
                {
                    Owner().m_path_follow_finished = 1;
                    if (mpc_finished == 1 || mpc_infeasible_flag == 2)
                    {
                        ending_type = 1; // mpc finished
                    }
                    else
                    {
                        ending_type = 3; // mpc infeasible/empty/ending_rotate
                    }
                    PATHFOLLOW_PLANNER_INFO_LOG("Owner().m_path_follow_finished = 1, ending with mpc");
                }
                else if (available_sparse_index >= Owner().path_points_pure_pursuit.size() && (pp_finished == 1 || pp_infeasible_flag == 1))
                {
                    Owner().m_path_follow_finished = 1;
                    ending_type = 2; // pur_pursuit
                    PATHFOLLOW_PLANNER_INFO_LOG("Owner().m_path_follow_finished = 1, ending with pure_pursuit");
                }
                PATHFOLLOW_PLANNER_INFO_LOG("ending_type = %d", ending_type);
                // ******************************* check US Events & if US Escape *******************************
                Owner().check_escape_mode_with_us_and_local_map(us_lasting_front, us_lasting_front_left, us_lasting_front_right, us_lasting_left, us_lasting_right,
                                                                lm_lasting_front, lm_lasting_front_left, lm_lasting_front_right, lm_lasting_right, lm_lasting_left,
                                                                emergency_us_front_left, emergency_us_front_right, emergency_us_left_pure, emergency_us_right_pure,
                                                                emergency_lm_front_left, emergency_lm_front_right, emergency_lm_right_pure, emergency_lm_left_pure,
                                                                escape_fl_rotate_right, escape_l_rotate_right, escape_fr_rotate_left, escape_r_rotate_left, escape_f_rotate_right, escape_f_rotate_left,
                                                                rotate_request_left_dir, rotate_request_flag, ignore_rotate_request_flag, tmp_yaw_next_path_point, last_rotate_dir);

                // distribute task
                PATHFOLLOW_PLANNER_INFO_LOG("============= Distribute Task =============");
                if (Owner().m_path_follow_finished == 0 && !freeze_stop_flag)
                {
                    // check US Escape & rotation stuck
                    if ((escape_fl_rotate_right || escape_l_rotate_right || escape_fr_rotate_left || escape_r_rotate_left || escape_f_rotate_left || escape_f_rotate_right) && last_rotate_task_finished && !rotation_stuck_flag)
                    {
                        PATHFOLLOW_PLANNER_INFO_LOG("Enter escape rotate...");
                        if (escape_fl_rotate_right || escape_l_rotate_right || escape_f_rotate_right)
                        {
                            // right rotate task
                            last_rotate_task_finished = false;
                            PATHFOLLOW_PLANNER_INFO_LOG("Send Task US Escape Rotate Right");
                            float rotate_target_angle = Owner().m_slam_data.pose.theta - delta_rotate_angle;
                            CREATE_TASK(TaskRotate, test_rotate);
                            test_rotate->preempt_type = PREEMPT_DELETE;
                            test_rotate->type = TaskRotate::TARGET_ANGLE;
                            test_rotate->dir = TaskRotate::ROTATE_RIGHT;
                            test_rotate->rotate_vel = Owner().m_rotate_vel_default;
                            test_rotate->target_angle = rotate_target_angle;
                            if (escape_fl_rotate_right)
                            {
                                test_rotate->reason = TaskRotate::ESCAPE_FRONT_LEFT;
                                PATHFOLLOW_PLANNER_INFO_LOG("Reason: ESCAPE_FRONT_LEFT");
                            }
                            else if (escape_l_rotate_right)
                            {
                                test_rotate->reason = TaskRotate::ESCAPE_LEFT;
                                PATHFOLLOW_PLANNER_INFO_LOG("Reason: ESCAPE_LEFT");
                            }
                            else if (escape_f_rotate_right)
                            {
                                test_rotate->reason = TaskRotate::ESCAPE_FRONT;
                                PATHFOLLOW_PLANNER_INFO_LOG("Reason: ESCAPE_FRONT");
                            }
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_min_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            rotate_task_id = g_tm.addTask(test_rotate);
                            // rotate_remain_angle = g_speed_controller.SetRotate(false, rotate_target_angle);
                            // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "right_rotate_remain_angle(deg) :  %f ", rad2deg(rotate_remain_angle));
                        }
                        else if (escape_fr_rotate_left || escape_r_rotate_left || escape_f_rotate_left)
                        {
                            // left rotate task
                            last_rotate_task_finished = false;
                            PATHFOLLOW_PLANNER_INFO_LOG("Send Task US Escape Rotate Left");
                            float rotate_target_angle = Owner().m_slam_data.pose.theta + delta_rotate_angle;
                            CREATE_TASK(TaskRotate, test_rotate);
                            test_rotate->preempt_type = PREEMPT_DELETE;
                            test_rotate->type = TaskRotate::TARGET_ANGLE;
                            test_rotate->dir = TaskRotate::ROTATE_LEFT;
                            test_rotate->rotate_vel = Owner().m_rotate_vel_default;
                            test_rotate->target_angle = rotate_target_angle;
                            if (escape_fr_rotate_left)
                            {
                                test_rotate->reason = TaskRotate::ESCAPE_FRONT_RIGHT;
                                PATHFOLLOW_PLANNER_INFO_LOG("Reason: ESCAPE_FRONT_RIGHT");
                            }
                            else if (escape_r_rotate_left)
                            {
                                test_rotate->reason = TaskRotate::ESCAPE_RIGHT;
                                PATHFOLLOW_PLANNER_INFO_LOG("Reason: ESCAPE_RIGHT");
                            }
                            else if (escape_f_rotate_left)
                            {
                                test_rotate->reason = TaskRotate::ESCAPE_FRONT;
                                PATHFOLLOW_PLANNER_INFO_LOG("Reason: ESCAPE_FRONT");
                            }
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_min_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            rotate_task_id = g_tm.addTask(test_rotate);
                            // rotate_remain_angle = g_speed_controller.SetRotate(true, rotate_target_angle);
                            // PATHFOLLOW_PLANNER_INFO_LOG(COLOR_WHITE "left_rotate_remain_angle(deg) :  %f ", rad2deg(rotate_remain_angle));
                        }
                    }
                    else if (!last_rotate_task_finished && !rotation_stuck_flag)
                    {
                        // do nothing, waiting for rotate task to be finished...
                        PATHFOLLOW_PLANNER_INFO_LOG("Do nothing, waiting for rotate task to be finished");
                    }
                    else
                    {
                        if (mode_obstacle_avoidance == 0 && rotate_request_flag == 0)
                        {
                            PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task MPC");
                            CREATE_TASK(TaskMPC, test_mpc);
                            test_mpc->preempt_type = PREEMPT_DELETE;
                            test_mpc->type = TaskMPC::MPC_FOLLOW;
                            test_mpc->mpc_ref_path = Owner().path_to_mpc;
                            test_mpc->start_segment_index = 0;
                            test_mpc->global_path_index = Owner().goal_index_mpc;
                            test_mpc->velocity = Owner().m_planning_velocity;
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_planning_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            mpc_task_id = g_tm.addTask(test_mpc);
                        }
                        else if (mode_obstacle_avoidance == 2 && rotate_request_flag == 0)
                        {
                            PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task MPC, OAMode2, Teb Planning Request Sent");
                            CREATE_TASK(TaskMPC, test_mpc);
                            test_mpc->preempt_type = PREEMPT_DELETE;
                            test_mpc->type = TaskMPC::MPC_FOLLOW;
                            test_mpc->mpc_ref_path = Owner().path_buffer;
                            test_mpc->start_segment_index = 0;
                            test_mpc->global_path_index = Owner().goal_index_mpc;
                            test_mpc->velocity = Owner().m_planning_velocity;
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_planning_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            mpc_task_id = g_tm.addTask(test_mpc);
                        }
                        else if (mode_obstacle_avoidance == 3 && Owner().m_rx_teb_path == 1 && rotate_request_flag == 0)
                        {
                            PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task MPC, OAMode3, Teb Path");
                            CREATE_TASK(TaskMPC, test_mpc);
                            test_mpc->preempt_type = PREEMPT_DELETE;
                            test_mpc->type = TaskMPC::MPC_FOLLOW;
                            test_mpc->mpc_ref_path = Owner().path_to_teb_prepare;
                            test_mpc->start_segment_index = 0;
                            test_mpc->global_path_index = -1;
                            test_mpc->velocity = Owner().m_planning_velocity;
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_planning_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            mpc_task_id = g_tm.addTask(test_mpc);
                        }
                        else if (mode_obstacle_avoidance == 3 && Owner().m_rx_teb_path == 0 && rotate_request_flag == 0 && teb_infeasible_flag == 0)
                        {
                            PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task MPC, OAMode3 not RxTebPath, Teb Path Buffer");
                            CREATE_TASK(TaskMPC, test_mpc);
                            test_mpc->preempt_type = PREEMPT_DELETE;
                            test_mpc->type = TaskMPC::MPC_FOLLOW;
                            test_mpc->mpc_ref_path = Owner().path_buffer;
                            test_mpc->start_segment_index = 0;
                            test_mpc->global_path_index = -1;
                            test_mpc->velocity = Owner().m_planning_velocity;
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_planning_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            mpc_task_id = g_tm.addTask(test_mpc);
                        }
                        else if (mode_obstacle_avoidance == 1 && rotate_request_flag == 0)
                        {
                            PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_BLUE "Send Task PurePursuit");
                            CREATE_TASK(TaskPurepursuit, test_purepursuit);
                            test_purepursuit->preempt_type = PREEMPT_DELETE;
                            test_purepursuit->type = TaskPurepursuit::PUREPURSUIT_FOLLOW;
                            test_purepursuit->goal_path_points = Owner().path_to_pure_pursuit;
                            test_purepursuit->global_path_index = Owner().goal_index_pure_pursuit;
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_min_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            purepursuit_task_id = g_tm.addTask(test_purepursuit);
                        }
                        // else if (mode_obstacle_avoidance == 1 && rotate_request_flag == 1 && ignore_rotate_request_flag == 0)
                        else if (rotate_request_flag == 1 && ignore_rotate_request_flag == 0)
                        {
                            if (mode_obstacle_avoidance == 0)
                            {
                                PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task Rotate from MPC request");
                            }
                            else if (mode_obstacle_avoidance == 1)
                            {
                                PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_BLUE "Send Task Rotate from PurePursuit request");
                            }
                            else
                            {
                                PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_CYAN "Send Task Rotate from Teb request");
                            }
                            last_rotate_task_finished = false;
                            CREATE_TASK(TaskRotate, request_rotate);
                            request_rotate->preempt_type = PREEMPT_DELETE;
                            request_rotate->type = TaskRotate::TARGET_ANGLE;
                            if (rotate_request_left_dir == true)
                            {
                                request_rotate->dir = TaskRotate::ROTATE_LEFT;
                            }
                            else
                            {
                                request_rotate->dir = TaskRotate::ROTATE_RIGHT;
                            }
                            request_rotate->rotate_vel = Owner().m_rotate_vel_default;
                            request_rotate->target_angle = rotate_request_target_angle;
                            // update intend_velocity for detectors
                            ev_det_intend_vel->intend_velocity = Owner().m_min_velocity;
                            g_ec.pushEvent(ev_det_intend_vel);
                            rotate_task_id = g_tm.addTask(request_rotate);
                        }
                        else
                        {
                            PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task: No task is sent");
                        }
                    }
                }
                else if (!freeze_stop_flag && !ending_rotate_task_sent) // path follow结束，摆正yaw
                {
                    RobotPose ending_path_point;
                    bool ending_rotate_dir = false;
                    bool no_rotation = false;
                    if (ending_type == 1) // mpc finished
                    {
                        ending_path_point = Owner().path_points_mpc.back();
                    }
                    else if (ending_type == 2) // pure_pursuit
                    {
                        ending_path_point = Owner().path_points_pure_pursuit.back();
                        Owner().m_path_follow_error = 2;
                        no_rotation = true;
                    }
                    else if (ending_type == 3) // mpc infeasible/empty/ending_rotate
                    {
                        no_rotation = true;
                    }

                    float tmp_diff_yaw = toNPPiAngleRangeR(ending_path_point.theta - Owner().m_slam_data.pose.theta);
                    PATHFOLLOW_PLANNER_WARN_LOG("tmp_diff_yaw: %f, ending_path_point.theta: %f, m_slam_data.pose.theta: %f", rad2deg(tmp_diff_yaw), rad2deg(ending_path_point.theta), rad2deg(Owner().m_slam_data.pose.theta));
                    if (tmp_diff_yaw > 0)
                    {
                        ending_rotate_dir = true;
                    }
                    else if (tmp_diff_yaw == 0)
                    {
                        no_rotation = true;
                    }
                    else
                    {
                    }

                    if (!no_rotation)
                    {
                        CREATE_TASK(TaskRotate, ending_rotate);
                        ending_rotate->preempt_type = PREEMPT_DELETE;
                        ending_rotate->type = TaskRotate::TARGET_ANGLE;
                        if (ending_rotate_dir == true)
                        {
                            ending_rotate->dir = TaskRotate::ROTATE_LEFT;
                        }
                        else
                        {
                            ending_rotate->dir = TaskRotate::ROTATE_RIGHT;
                        }
                        ending_rotate->rotate_vel = Owner().m_rotate_vel_default;
                        ending_rotate->target_angle = ending_path_point.theta;
                        // update intend_velocity for detectors
                        ev_det_intend_vel->intend_velocity = Owner().m_min_velocity;
                        g_ec.pushEvent(ev_det_intend_vel);
                        ending_rotate_task_id = g_tm.addTask(ending_rotate);
                        ending_rotate_task_sent = true;
                        PATHFOLLOW_PLANNER_INFO_LOG("The Ending Rotation...");
                    }
                    else
                    {
                        ending_rotate_task_finished = true;
                    }
                }
                else if (freeze_stop_flag)
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "=== FREEZE === FREEZE === FREEZE === FREEZE === FREEZE === FREEZE ");
                    PATHFOLLOW_PLANNER_WARN_LOG(COLOR_L_GREEN "Send Task MPC FREEZE");
                    last_rotate_task_finished = true;
                    CREATE_TASK(TaskMPC, test_mpc);
                    test_mpc->preempt_type = PREEMPT_DELETE;
                    test_mpc->type = TaskMPC::MPC_FOLLOW;
                    test_mpc->start_segment_index = 0;
                    test_mpc->global_path_index = -1;
                    test_mpc->velocity = Owner().m_planning_velocity;
                    mpc_task_id = g_tm.addTask(test_mpc);
                }
                PATHFOLLOW_PLANNER_DEBUG_LOG("=========== Distribute Task End ===========");

                uint64_t update_end_ts = Timer::getTimestampUS();
                float update_period = float(update_end_ts - update_start_ts) / 1000000;
                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "update_period:  %f", update_period);
                if (update_period > max_period)
                {
                    max_period = update_period;
                }
                PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "max_period:     %f", max_period);

#ifdef LOG_OUTPUT
                PATH_FOLLOW_DEBUG_LOG_OUTPUT.open(filename, ios::out | ios::app);
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << mode_obstacle_avoidance << " " << Owner().path_points_mpc.size() << " " << Owner().path_points_pure_pursuit.size() << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << print_mpc_return_next_index << " " << mpc_infeasible_flag << " " << Owner().goal_index_mpc << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << print_ppp_return_next_index << " " << pp_infeasible_flag << " " << Owner().goal_index_pure_pursuit << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << Owner().m_slam_data.pose.pt.x << " " << Owner().m_slam_data.pose.pt.y << " " << rad2deg(Owner().m_slam_data.pose.theta) << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << Owner().path_to_mpc[0].pt.x << " " << Owner().path_to_mpc[0].pt.y << " " << rad2deg(Owner().path_to_mpc[0].theta) << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << Owner().path_to_pure_pursuit[0].pt.x << " " << Owner().path_to_pure_pursuit[0].pt.y << " " << rad2deg(Owner().path_to_pure_pursuit[0].theta) << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << rotate_request_flag << " " << rad2deg(rotate_request_target_angle) << " " << ignore_rotate_request_flag << " ";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT << "\n";
                PATH_FOLLOW_DEBUG_LOG_OUTPUT.close();
#endif
            }
        };

        struct Finished : BaseState
        {
            virtual void OnEnter()
            {
                PATHFOLLOW_PLANNER_INFO_LOG("Enter Finished...");
                Owner().finishPlanning();

                PATHFOLLOW_PLANNER_DEBUG_LOG("set stop flag, change to Idle");
                Owner().m_stop_flag = true;
            }

            virtual void OnExit()
            {
                PATHFOLLOW_PLANNER_INFO_LOG("Exit Finished");
            }

            virtual Transition GetTransition()
            {
                return NoTransition();
            }
        };

        struct Error : BaseState
        {
            virtual void OnEnter()
            {
                PATHFOLLOW_PLANNER_ERROR_LOG("Enter Error...");
                if (Owner().m_path_follow_error == 1)
                {
                    PATHFOLLOW_PLANNER_ERROR_LOG("Input no path points");
                }
                else if (Owner().m_path_follow_error == 2)
                {
                    PATHFOLLOW_PLANNER_ERROR_LOG("Path_Follow_Planner ending with pure_pursuit");
                }
                // else if (Owner().m_path_follow_error == 3)
                // {
                // }
                else if (Owner().m_path_follow_error == 5)
                {
                    PATHFOLLOW_PLANNER_ERROR_LOG("Global teb planner infeasible");
                }
                CREATE_EVENT(EvPathFollowPlannerError, ev_pfp_error);
                ev_pfp_error->id = Owner().m_path_follow_error;
                g_ec.pushEvent(ev_pfp_error);

                PATHFOLLOW_PLANNER_ERROR_LOG("set stop flag, change to Idle");
                Owner().m_stop_flag = true;
            }

            virtual void OnExit()
            {
                PATHFOLLOW_PLANNER_ERROR_LOG("Exit Error...");
            }

            virtual Transition GetTransition()
            {
                return NoTransition();
            }
        };
    };
} // namespace planning_planner

ContextPtr PathFollowPlanner::saveContext()
{
}

Transition PathFollowPlanner::restoreContext(ContextPtr ctx)
{
}

DEFINE_CONFIG_TYPE(CONFIG_PATH_FOLLOW_PLANNER, PathFollowPlanner);

PathFollowPlanner &g_path_follow_planner = PathFollowPlanner::getInstance();

float PathFollowPlanner::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigPathFollowPlanner *cfg_pfp = dynamic_cast<ConfigPathFollowPlanner *>(
        cfg_mgr.GetSubConfig(CONFIG_PATH_FOLLOW_PLANNER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    ConfigRobot *cfg_robot = dynamic_cast<ConfigRobot *>(
        cfg_mgr.GetSubConfig(CONFIG_ROBOT));

    m_vehicle_longitude_size = cfg_robot->vehicle_longitude_size;
    m_vehicle_lateral_size = cfg_robot->vehicle_lateral_size;
    m_local_map_size_x = cfg_robot->local_map_size_x;
    m_local_map_size_y = cfg_robot->local_map_size_x;
    m_robot_pose_in_local_map_x_default = cfg_robot->robot_pose_in_local_map_x_default;
    m_robot_pose_in_local_map_y_default = cfg_robot->robot_pose_in_local_map_y_default;
    m_local_map_resolution = cfg_robot->local_map_resolution;
    m_rotate_vel_default = cfg_robot->rotate_vel_default;

    cfg_pfp->log_path = cfg_planning->log_path;
    m_interval_mpc = cfg_pfp->interval_dense;
    m_interval_pure_pursuit = cfg_pfp->interval_sparse;
    m_ev_deduction_period_for_us = cfg_pfp->ev_deduction_period_for_us;
    m_set_oa_off = cfg_pfp->set_oa_off;
    m_set_collis_lasting_off = cfg_pfp->set_collis_lasting_off;
    m_collision_lasting_threshold = cfg_pfp->collision_lasting_threshold;
    m_delta_rotate_angle = cfg_pfp->delta_rotate_angle;
    m_max_velocity = cfg_pfp->max_velocity;
    m_min_velocity = cfg_pfp->min_velocity;
    m_max_curvature = cfg_pfp->max_curvature;
    m_min_curvature = cfg_pfp->min_curvature;
    m_const_velocity = cfg_pfp->const_velocity;
    m_test_velocity = cfg_pfp->test_velocity;
    m_running_freq = cfg_pfp->planning_frequency;

    CREATE_LOG(PlainText, LOG_PATHFOLLOW_PLANNER_FLAG, LOG_PATHFOLLOW_PLANNER,
               cfg_pfp->log_name, cfg_pfp->log_path,
               cfg_pfp->log_extension, cfg_pfp->log_ts_mask,
               cfg_pfp->log_print_to_console,
               (cfg_pfp->log_max_file_size_mb)MB + (cfg_pfp->log_max_file_size_kb)KB,
               cfg_pfp->log_max_file_cnt, cfg_pfp->log_level);

    return cfg_pfp->planning_frequency;
}

void PathFollowPlanner::initRunSM()
{
    m_run_sm.Initialize<PathFollowPlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("PathFollowPlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

bool PathFollowPlanner::handleInput(const GlobalPlannerInputPtr input)
{
    PathFollowPlannerInputPtr pathfollow_input = std::dynamic_pointer_cast<PathFollowPlannerInput>(input);
    m_need_path_insert = pathfollow_input->need_path_insert;
    m_input_idx = pathfollow_input->input_idx;
    m_feedback_path_points_states = pathfollow_input->feedback_path_points_states;
    global_path_points = pathfollow_input->global_path;
    return true;
}

void PathFollowPlanner::reset()
{
}

void PathFollowPlanner::getData()
{
    g_dc.getData<DataSlam>(m_slam_data);
    g_dc.getData<DataControlSpeed>(m_control_speed);
    g_dc.getData<DataLocalMap>(m_simple_local_map);
}

void PathFollowPlanner::finishPlanning()
{
    PATHFOLLOW_PLANNER_DEBUG_LOG("finish path_follow planner, send planner finished event, input id = %ld",
                                 m_input_id);
    CREATE_EVENT(EvPlannerFinished, ev_path_follow_finished);
    ev_path_follow_finished->id = m_input_id;
    g_ec.pushEvent(ev_path_follow_finished);
    g_speed_controller.SetStop(true);
}

void PathFollowPlanner::preprocess_global_path_points(void)
{
    cout << "before preprocess: global_path_points.size(): " << global_path_points.size() << endl;
    // 从上一个规划器发出来的全局轨迹赋值
    PosePath goal_point_tmp_1;
    // RobotPose start_Pose{m_slam_data.pose.pt.x, m_slam_data.pose.pt.x, m_slam_data.pose.theta};
    // RobotPose start_Pose{0,0,0};
    //  加入起点
    // goal_point_tmp_1.add(start_Pose);

    // global_path_points 在handleInput中赋值
    for (size_t j = 0; j < global_path_points.size(); j++)
    {
        RobotPose point_tmp;
        point_tmp.pt.x = global_path_points[j].pt.x;
        point_tmp.pt.y = global_path_points[j].pt.y;
        point_tmp.theta = global_path_points[j].theta;
        goal_point_tmp_1.add(point_tmp);
    }

    float interval_dense = m_interval_mpc;

    size_t start_index_dense = 0;
    path_points_mpc.add(goal_point_tmp_1[start_index_dense]);
    for (size_t i = 1; i < goal_point_tmp_1.size(); i++)
    {
        RobotPose pose_single_i;
        RobotPose pose_single_ii;
        pose_single_i.pt.x = goal_point_tmp_1[i].pt.x;
        pose_single_i.pt.y = goal_point_tmp_1[i].pt.y;
        pose_single_i.theta = goal_point_tmp_1[i].theta;

        pose_single_ii.pt.x = goal_point_tmp_1[start_index_dense].pt.x;
        pose_single_ii.pt.y = goal_point_tmp_1[start_index_dense].pt.y;
        pose_single_ii.theta = goal_point_tmp_1[start_index_dense].theta;
        float distance = hypot((pose_single_i.pt.x - pose_single_ii.pt.x), (pose_single_i.pt.y - pose_single_ii.pt.y));
        if (distance >= interval_dense || fabs(pose_single_i.theta - pose_single_ii.theta) > deg2rad(30))
        {
            path_points_mpc.add(pose_single_i);
            start_index_dense = i;
        }
    }
    if (path_points_mpc.back() == goal_point_tmp_1.back())
    {
        // do nothing
    }
    else
    {
        path_points_mpc.add(goal_point_tmp_1.back());
    }
    cout << "interval_dense: " << interval_dense << "  path_points_mpc.size(): " << path_points_mpc.size() << endl;
    // 初始化 path_points_dense_reached / skipped
    for (size_t i = 0; i < path_points_mpc.size(); i++)
    {
        // path_points_dense_reached.push_back(false);
        // path_points_dense_skipped.push_back(false);
        path_points_mpc_situation.push_back(0);
    }

    float interval_sparse = m_interval_pure_pursuit;
    size_t start_index = 0;
    path_points_pure_pursuit.add(path_points_mpc[start_index]);
    // pp_index_correspond_mpc_index.push_back(start_index);
    pair_pp_index_to_mpc_index.push_back(make_pair(start_index, start_index));
    for (size_t i = 1; i < path_points_mpc.size(); i++)
    {
        RobotPose pose_single_i;
        RobotPose pose_single_ii;
        pose_single_i.pt.x = path_points_mpc[i].pt.x;
        pose_single_i.pt.y = path_points_mpc[i].pt.y;
        pose_single_i.theta = path_points_mpc[i].theta;

        pose_single_ii.pt.x = path_points_mpc[start_index].pt.x;
        pose_single_ii.pt.y = path_points_mpc[start_index].pt.y;
        float distance = hypot((pose_single_i.pt.x - pose_single_ii.pt.x), (pose_single_i.pt.y - pose_single_ii.pt.y));
        if (distance >= interval_sparse)
        {
            path_points_pure_pursuit.add(pose_single_i);
            // pp_index_correspond_mpc_index.push_back(i);
            pair_pp_index_to_mpc_index.push_back(make_pair(pair_pp_index_to_mpc_index.size(), i));
            start_index = i;
        }
    }
    path_points_pure_pursuit.add(path_points_mpc.back());
    pair_pp_index_to_mpc_index.push_back(make_pair(path_points_pure_pursuit.size() - 1, path_points_mpc.size() - 1));
    cout << "interval_sparse: " << interval_sparse << "  path_points_pure_pursuit.size(): " << path_points_pure_pursuit.size() << endl;
    // 初始化 path_points_sparse_reached / skipped
    for (size_t i = 0; i < path_points_pure_pursuit.size(); i++)
    {
        // path_points_sparse_reached.push_back(false);
        // path_points_sparse_skipped.push_back(false);
        path_points_pure_pursuit_situation.push_back(0);
    }

}

void PathFollowPlanner::update_path_index_and_reached_skipped()
{
    // for mpc
    // mpc_return_next_dense_index = max(mpc_return_index + goal_index_mpc_last, mpc_return_next_dense_index);
    // PATHFOLLOW_PLANNER_INFO_LOG("mpc_return_next_dense_index:  %u  goal_index_mpc_last:  %u  mpc_return_index:  %u", mpc_return_next_dense_index, goal_index_mpc_last, mpc_return_index);
    // PATHFOLLOW_PLANNER_INFO_LOG("path_points_dense_reached.size():  %u  path_points_dense_skipped.size():  %u", path_points_dense_reached.size(), path_points_dense_skipped.size());
    if (mpc_return_next_dense_index < path_points_mpc.size())
    {
        // if (!mpc_reached_points.empty() && !mpc_skipped_points.empty())
        if (!mpc_path_points_states.empty())
        {
            for (size_t i = 0; i < mpc_return_index + 1; i++)
            {
                // path_points_dense_reached[goal_index_mpc_last + i] = mpc_reached_points[i];
                // path_points_dense_skipped[goal_index_mpc_last + i] = mpc_skipped_points[i];
                // if (mpc_reached_points[i] == 1)
                // {
                //     path_points_mpc_situation[goal_index_mpc_last + i] = 1;
                // }
                // else if (mpc_skipped_points[i] == 1)
                // {
                //     path_points_mpc_situation[goal_index_mpc_last + i] = 2;
                // }
                if (path_points_mpc_situation[goal_index_mpc_last + i] == 0) // 若该点从没标记过，则可标记，否则不能被重复标记
                {
                    path_points_mpc_situation[goal_index_mpc_last + i] = mpc_path_points_states[i];
                }
                if (mpc_path_points_states[i] > 3)
                {
                    PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "mpc_path_points_states > 3 : %d", mpc_path_points_states[i]);
                }
            }
        }
    }
    else
    {
        // reached_final_goal = true;
        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "mpc_return_next_dense_index overflow");
    }

    // for pure pursuit
    // 用 pure_pursuit 返回的下一个要去的 goal_index_pure_pursuit 更新 path_points_sparse_reached
    // pure_pursuit_return_next_sparse_index = max(pure_pursuit_return_index + goal_index_pure_pursuit_last, pure_pursuit_return_next_sparse_index);
    // PATHFOLLOW_PLANNER_INFO_LOG("pure_pursuit_return_next_sparse_index:  %u  goal_index_pure_pursuit_last:  %u  pure_pursuit_return_index:  %u", pure_pursuit_return_next_sparse_index, goal_index_pure_pursuit_last, pure_pursuit_return_index);
    // PATHFOLLOW_PLANNER_INFO_LOG("path_points_sparse_reached.size():  %u  path_points_sparse_skipped.size():  %u", path_points_sparse_reached.size(), path_points_sparse_skipped.size());
    if (pure_pursuit_return_next_sparse_index < path_points_pure_pursuit.size())
    {
        // if (!pure_pursuit_reached_points.empty() && !pure_pursuit_skipped_points.empty())
        if (!pure_pursuit_path_points_states.empty())
        {
            for (size_t i = 0; i < pure_pursuit_return_index + 1; i++)
            {
                // path_points_sparse_reached[goal_index_pure_pursuit_last + i] = pure_pursuit_reached_points[i];
                // path_points_sparse_skipped[goal_index_pure_pursuit_last + i] = pure_pursuit_skipped_points[i];
                // if (pure_pursuit_reached_points[i] == 1)
                // {
                //     path_points_pure_pursuit_situation[goal_index_pure_pursuit_last + i] = 1;
                // }
                // else if (pure_pursuit_skipped_points[i] == 1)
                // {
                //     path_points_pure_pursuit_situation[goal_index_pure_pursuit_last + i] = 2;
                // }
                path_points_pure_pursuit_situation[goal_index_pure_pursuit_last + i] = pure_pursuit_path_points_states[i];
            }
        }
    }
    else
    {
        // reached_final_goal = true;
        PATHFOLLOW_PLANNER_INFO_LOG(COLOR_L_RED "pure_pursuit_return_next_sparse_index overflow");
    }
}

void PathFollowPlanner::prepare_for_next_task()
{
    // for mpc
    goal_index_mpc = path_points_mpc.size();
    for (size_t i = mpc_return_next_dense_index; i < path_points_mpc.size(); i++)
    {
        // if (path_points_dense_reached[i] == false && path_points_dense_skipped[i] == false)
        if (path_points_mpc_situation[i] == 0)
        {
            goal_index_mpc = i;
            // goal_index_mpc_last = goal_index_mpc;
            PATHFOLLOW_PLANNER_INFO_LOG("prepare_for_next_task: goal_index_mpc = %u", goal_index_mpc);
            break;
        }
        // else if (i == path_points_mpc.size() - 1) // 到终点了
        // {
        //     goal_index_mpc = i;
        //     // goal_index_mpc_last = goal_index_mpc;
        //     PATHFOLLOW_PLANNER_INFO_LOG("prepare_for_next_task: goal_index_mpc = %u", goal_index_mpc);
        // }
    }

    // for pure_pursuit
    goal_index_pure_pursuit = path_points_pure_pursuit.size();
    for (size_t i = pure_pursuit_return_next_sparse_index; i < path_points_pure_pursuit.size(); i++)
    {
        // if (path_points_sparse_reached[i] == false && path_points_sparse_skipped[i] == false)
        if (path_points_pure_pursuit_situation[i] == 0)
        {
            goal_index_pure_pursuit = i;
            // goal_index_pure_pursuit_last = goal_index_pure_pursuit;
            PATHFOLLOW_PLANNER_INFO_LOG("prepare_for_next_task: goal_index_pure_pursuit = %u", goal_index_pure_pursuit);
            break;
        }
        // else if (i == path_points_pure_pursuit.size() - 1) // 到终点了
        // {
        //     goal_index_pure_pursuit = i;
        //     // goal_index_pure_pursuit_last = goal_index_pure_pursuit;
        //     PATHFOLLOW_PLANNER_INFO_LOG("prepare_for_next_task: goal_index_pure_pursuit = %u", goal_index_pure_pursuit);
        // }
    }
}

void PathFollowPlanner::path_follow_main_loop()
{
}

void PathFollowPlanner::insert_ellipse_between_clean_lines(const PosePath &input_path, PosePath &output_path)
{
    // std::cout << "input_path.size(): " << input_path.size() << std::endl;
    // for ccpp path points no_minor_direction_insert_ellipse only
    // PosePath output_path;
    float switch_line_distance = 0.5;
    // ellipse parameters
    float b = 0.5;   // adaptive to environment
    float res = 0.1; // insert resolution

    for (size_t i = 0; i < input_path.size(); i++)
    {
        float pose_x = input_path[i].pt.x;
        float pose_y = input_path[i].pt.y;
        float pose_theta = input_path[i].theta;

        // #ifdef WZ_TEST
        //         // coordinate transformation for wrong coordiante
        //         if (pose_theta > 1.57 - 0.01 && pose_theta < 1.57 + 0.01)
        //         {
        //             pose_theta = 0;
        //         }
        //         else if (pose_theta > 0 - 0.01 && pose_theta < 0 + 0.01)
        //         {
        //             pose_theta = 1.57;
        //         }
        //         else if (pose_theta > -1.57 - 0.01 && pose_theta < -1.57 + 0.01)
        //         {
        //             pose_theta = 3.14;
        //         }
        //         else
        //         {
        //             pose_theta = -1.57;
        //         }
        // #endif

        RobotPose current_point;
        current_point.pt.x = pose_x;
        current_point.pt.y = pose_y;
        current_point.theta = pose_theta;
        // cout << "show current point: " << i << "   " << pose_x << " " << pose_y << " " << pose_theta << endl;
        // cout << "show current point: " << i << "   " << current_point.x << " " << current_point.y << " " << current_point.theta << endl;

        // if i + 1 is not the last point
        if (i + 1 <= input_path.size() - 1)
        {
            // check whether insert ellipse
            float next_pose_x = input_path[i + 1].pt.x;
            float next_pose_y = input_path[i + 1].pt.y;
            float next_pose_theta = input_path[i + 1].theta;
            float distance = hypot((next_pose_x - pose_x), (next_pose_y - pose_y));
            // if (distance > switch_line_distance && next_pose_theta != pose_theta) // here means another clean line
            if (distance > switch_line_distance && fabs(next_pose_theta - pose_theta) > deg2rad(45)) // here means another clean line
            {
                // insert ellipse
                float a = distance / 2;
                int N_ellipse = b / res;
                float local_coord_theta = toNPPiAngleRangeR(atan2(next_pose_y - pose_y, next_pose_x - pose_x) + M_PI_2);
                // insert the first point on ellipse
                output_path.add(current_point);
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
                    ellipse_global_1.pt.x = pose_x + ellipse_local_x * cos(local_coord_theta) - ellipse_local_y_1 * sin(local_coord_theta);
                    ellipse_global_1.pt.y = pose_y + ellipse_local_y_1 * cos(local_coord_theta) + ellipse_local_x * sin(local_coord_theta);
                    ellipse_global_1.theta = toNPPiAngleRangeR(ellipse_local_yaw_1 + local_coord_theta);
                    // insert
                    output_path.add(ellipse_global_1);
                }
                // insert the top point on ellipse
                RobotPose top_ellipse_point_local;
                RobotPose top_ellipse_point_global;
                top_ellipse_point_local.pt.x = b;
                top_ellipse_point_local.pt.y = a;
                top_ellipse_point_local.theta = M_PI / 2;
                top_ellipse_point_global.pt.x = pose_x + top_ellipse_point_local.pt.x * cos(local_coord_theta) - top_ellipse_point_local.pt.y * sin(local_coord_theta);
                top_ellipse_point_global.pt.y = pose_y + top_ellipse_point_local.pt.y * cos(local_coord_theta) + top_ellipse_point_local.pt.x * sin(local_coord_theta);
                top_ellipse_point_global.theta = toNPPiAngleRangeR(top_ellipse_point_local.theta + local_coord_theta);
                output_path.add(top_ellipse_point_global);
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
                    ellipse_global_2.pt.x = pose_x + ellipse_local_x * cos(local_coord_theta) - ellipse_local_y_2 * sin(local_coord_theta);
                    ellipse_global_2.pt.y = pose_y + ellipse_local_y_2 * cos(local_coord_theta) + ellipse_local_x * sin(local_coord_theta);
                    ellipse_global_2.theta = toNPPiAngleRangeR(ellipse_local_yaw_2 + local_coord_theta);
                    // insert
                    output_path.add(ellipse_global_2);
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
                output_path.add(current_point);
            }
        }
        else
        {
            output_path.add(current_point);
        }
    }
    // std::cout << "output_path.size(): " << output_path.size() << std::endl;
}

void PathFollowPlanner::insert_straight_line_between_clean_lines(const PosePath &input_path, PosePath &output_path)
{
    float switch_line_distance = 0.5;
    float res = 0.05; // insert resolution

    for (size_t i = 0; i < input_path.size(); i++)
    {
        RobotPose current_pose = input_path[i];

        // if i is not the last point
        if (i + 1 <= input_path.size() - 1)
        {
            // check whether insert ellipse
            RobotPose next_pose = input_path[i + 1];
            float distance = hypot((next_pose.pt.y - current_pose.pt.y), (next_pose.pt.x - current_pose.pt.x));
            if (distance > switch_line_distance && fabs(next_pose.theta - current_pose.theta) > deg2rad(45)) // here means another clean line
            {
                output_path.add(current_pose);
                int N_ellipse = distance / res;
                float insert_line_yaw = atan2(next_pose.pt.y - current_pose.pt.y, next_pose.pt.x - current_pose.pt.x);
                for (int i = 1; i < N_ellipse - 1; i++)
                {
                    RobotPose tmp_pose;
                    tmp_pose.pt.x = i * res * cos(insert_line_yaw);
                    tmp_pose.pt.y = i * res * sin(insert_line_yaw);
                    tmp_pose.theta = insert_line_yaw;
                    output_path.add(tmp_pose);
                }
            }
            else
            {
                output_path.add(current_pose);
            }
        }
        else
        {
            output_path.add(current_pose);
        }
    }
    // std::cout << "output_path.size(): " << output_path.size() << std::endl;
}

void PathFollowPlanner::insert_bezier_between_clean_lines(const PosePath &input_path, PosePath &output_path)
{
    float switch_line_distance = 0.5;
    float res = 0.05;               // insert resolution
    float anchor_point_shoot = 0.5; 

    for (size_t i = 0; i < input_path.size(); i++)
    {
        RobotPose current_pose = input_path[i];

        // if i is not the last point
        if (i + 1 <= input_path.size() - 1)
        {
            // check whether insert or not
            RobotPose next_pose = input_path[i + 1];
            float distance = hypot((next_pose.pt.y - current_pose.pt.y), (next_pose.pt.x - current_pose.pt.x));
            if (distance > switch_line_distance) // here means another clean line
            {
                int N_insert = (int)(distance / res);
                PATHFOLLOW_PLANNER_WARN_LOG("distance:  %f   N_insert:  %d", distance, N_insert);
                // PATHFOLLOW_PLANNER_WARN_LOG("curr:  x:  %f  y:  %f   theta:  %f", current_pose.pt.x, current_pose.pt.y, current_pose.theta);
                // PATHFOLLOW_PLANNER_WARN_LOG("next:  x:  %f  y:  %f   theta:  %f", next_pose.pt.x, next_pose.pt.y, next_pose.theta);
                RobotPose anchor_0 = current_pose;
                RobotPose anchor_1;
                RobotPose anchor_2;
                RobotPose anchor_3 = next_pose;

                anchor_1.pt.x = anchor_0.pt.x + anchor_point_shoot * cos(anchor_0.theta);
                anchor_1.pt.y = anchor_0.pt.y + anchor_point_shoot * sin(anchor_0.theta);
                anchor_1.theta = anchor_0.theta;

                anchor_2.pt.x = anchor_3.pt.x - anchor_point_shoot * cos(anchor_3.theta);
                anchor_2.pt.y = anchor_3.pt.y - anchor_point_shoot * sin(anchor_3.theta);
                anchor_2.theta = anchor_3.theta;

                for (int i = 0; i < N_insert; i++)
                {
                    float t = (float)i / (float)(N_insert - 1);
                    RobotPose tmp_pose;
                    tmp_pose.pt.x = pow((1 - t), 3) * anchor_0.pt.x + 3 * t * pow((1 - t), 2) * anchor_1.pt.x + 3 * pow(t, 2) * (1 - t) * anchor_2.pt.x + pow(t, 3) * anchor_3.pt.x;
                    tmp_pose.pt.y = pow((1 - t), 3) * anchor_0.pt.y + 3 * t * pow((1 - t), 2) * anchor_1.pt.y + 3 * pow(t, 2) * (1 - t) * anchor_2.pt.y + pow(t, 3) * anchor_3.pt.y;
                    float dx = -anchor_0.pt.x * 3 * pow((1 - t), 2) + anchor_1.pt.x * 3 * (1 + 3 * t * t - 4 * t) + anchor_2.pt.x * 3 * (2 * t - 3 * t * t) + anchor_3.pt.x * 3 * t * t;
                    float dy = -anchor_0.pt.y * 3 * pow((1 - t), 2) + anchor_1.pt.y * 3 * (1 + 3 * t * t - 4 * t) + anchor_2.pt.y * 3 * (2 * t - 3 * t * t) + anchor_3.pt.y * 3 * t * t;
                    tmp_pose.theta = atan2(dy, dx);
                    output_path.add(tmp_pose);
                    // PATHFOLLOW_PLANNER_WARN_LOG("tmp :  x:  %f  y:  %f   theta:  %f", tmp_pose.pt.x, tmp_pose.pt.y, tmp_pose.theta);
                }
            }
            else
            {
                output_path.add(current_pose);
            }
        }
        else
        {
            output_path.add(current_pose);
        }
    }
    // std::cout << "output_path.size(): " << output_path.size() << std::endl;
}

void PathFollowPlanner::op_local_planner_sparse_and_dense()
{
    // for mpc
    {
        path_to_mpc.clear();
        size_t upper_point_index = ((goal_index_mpc + m_points_num_to_mpc) > path_points_mpc.size()) ? path_points_mpc.size() : (goal_index_mpc + m_points_num_to_mpc);
        for (size_t i = goal_index_mpc; i < upper_point_index; i++)
        {
            // PATHFOLLOW_PLANNER_INFO_LOG("path_points_mpc[%u]:  %f  %f  %f", i, path_points_mpc[i].pt.x, path_points_mpc[i].pt.y, path_points_mpc[i].theta);
            path_to_mpc.add(path_points_mpc[i]);
        }
        for (size_t i = 0; i < path_to_mpc.size(); i++)
        {
            // PATHFOLLOW_PLANNER_INFO_LOG("path_to_mpc[%u]:           %f  %f  %f", i, path_to_mpc[i].pt.x, path_to_mpc[i].pt.y, path_to_mpc[i].theta);
        }
        PATHFOLLOW_PLANNER_INFO_LOG("path_to_mpc.size():  %u", path_to_mpc.size());
    }

    // for pure pursuit
    {
        path_to_pure_pursuit.clear();
        // goal_path_points_pure_pursuit.clear();
        size_t upper_point_index_sparse = ((goal_index_pure_pursuit + m_points_num_to_pp) > path_points_pure_pursuit.size()) ? path_points_pure_pursuit.size() : (goal_index_pure_pursuit + m_points_num_to_pp);
        for (size_t i = goal_index_pure_pursuit; i < upper_point_index_sparse; i++)
        {
            // PATHFOLLOW_PLANNER_INFO_LOG("path_points_pure_pursuit[%u]:  %f  %f  %f", i, path_points_pure_pursuit[i].pt.x, path_points_pure_pursuit[i].pt.y, path_points_pure_pursuit[i].theta);
            path_to_pure_pursuit.add(path_points_pure_pursuit[i]);
        }
        for (size_t i = 0; i < path_to_pure_pursuit.size(); i++)
        {
            // PATHFOLLOW_PLANNER_INFO_LOG("path_to_pure_pursuit[%u]:  %f  %f  %f", i, path_to_pure_pursuit[i].pt.x, path_to_pure_pursuit[i].pt.y, path_to_pure_pursuit[i].theta);
        }
        // std::cout << "path_to_pure_pursuit.size()  " << path_to_pure_pursuit.size() << std::endl;
        PATHFOLLOW_PLANNER_INFO_LOG("path_to_pure_pursuit.size():  %u", path_to_pure_pursuit.size());
    }
}

bool PathFollowPlanner::check_path_point_collision(const int &p_, const int &q_, const int &i_, bool use_rect_shape_)
{
    float g_obs_x = (p_ - m_robot_pose_in_local_map_x_default) * m_local_map_resolution + m_simple_local_map.x;
    float g_obs_y = (q_ - m_robot_pose_in_local_map_y_default) * m_local_map_resolution + m_simple_local_map.y;
    float dist = hypot(g_obs_x - path_points_mpc[i_].pt.x, g_obs_y - path_points_mpc[i_].pt.y);
    float diff_yaw = toNPPiAngleRangeR(atan2(g_obs_y - path_points_mpc[i_].pt.y, g_obs_x - path_points_mpc[i_].pt.x) - path_points_mpc[i_].theta); // rad

    if (!use_rect_shape_)
    {
        // simple detection
        if (dist <= 1.0)
        {
            return true;
        }
    }
    else
    {
        // rect-shape detection
        float proj_longitude = dist * cos(diff_yaw);
        float proj_lateral = fabs(dist * sin(diff_yaw));
        // for small obstacle
        if (proj_longitude <= m_vehicle_longitude_size + m_inflate_longitude_param && proj_longitude >= 0 && proj_lateral <= m_vehicle_lateral_size / 2 + m_inflate_lateral_param)
        {
            return true;
        }
    }

    return false;
}

float PathFollowPlanner::getCurvatureFrom3points(const RobotPose &pose1, const RobotPose &pose2, const RobotPose &pose3)
{
    float a = hypot(pose1.pt.x - pose2.pt.x, pose1.pt.y - pose2.pt.y);
    float b = hypot(pose1.pt.x - pose3.pt.x, pose1.pt.y - pose3.pt.y);
    float c = hypot(pose3.pt.x - pose2.pt.x, pose3.pt.y - pose2.pt.y);

    if (a == 0 || b == 0 || c == 0 || b == a + c || a == b + c || c == a + b)
        return 0.0f;

    float k = sqrt((a + b - c) * (a - b + c) * (b + c - a) * (a + b + c)) / (a * b * c);

    // PATHFOLLOW_PLANNER_WARN_LOG("=======k:  %f", k);
    // PATHFOLLOW_PLANNER_WARN_LOG("pose1:  %f  %f  %f", pose1.pt.x, pose1.pt.y, pose1.theta);
    // PATHFOLLOW_PLANNER_WARN_LOG("pose2:  %f  %f  %f", pose2.pt.x, pose2.pt.y, pose2.theta);
    // PATHFOLLOW_PLANNER_WARN_LOG("pose3:  %f  %f  %f", pose3.pt.x, pose3.pt.y, pose3.theta);

    return k;
}

float PathFollowPlanner::getPathAverageCurvature(const PosePath &input_path)
{
    float sum_k = 0;
    int cnt = 0;
    float average_k = 0;

    if (input_path.size() < 3)
        return 0.0f;

    for (size_t i = 0; i < input_path.size() - 2; i++)
    {
        float k = getCurvatureFrom3points(input_path[i], input_path[i + 1], input_path[i + 2]);
        sum_k += k;
        cnt += 1;
    }

    if (isnan(sum_k)) 
    {
        // if (sum_k == FLT_MAX)
        // {
        //     PATHFOLLOW_PLANNER_WARN_LOG("sum_k == FLT_MAX");
        // }
        // else if (sum_k == FLT_MIN)
        // {
        //     PATHFOLLOW_PLANNER_WARN_LOG("sum_k == FLT_MIN");
        // }
        sum_k = 0;
        // PATHFOLLOW_PLANNER_WARN_LOG("sum_k is nan");
    }
    // PATHFOLLOW_PLANNER_WARN_LOG("sum_k:  %f  cnt:  %u", sum_k, cnt);

    average_k = sum_k / cnt;
    return average_k;
}

void PathFollowPlanner::check_escape_mode_with_us_and_local_map(const bool &us_lasting_front_, const bool &us_lasting_front_left_, const bool &us_lasting_front_right_, const bool &us_lasting_left_, const bool &us_lasting_right_,
                                                                const bool &lm_lasting_front_, const bool &lm_lasting_front_left_, const bool &lm_lasting_front_right_, const bool &lm_lasting_right_, const bool &lm_lasting_left_,
                                                                const bool &emergency_us_front_left_, const bool &emergency_us_front_right_, const bool &emergency_us_left_pure_, const bool &emergency_us_right_pure_,
                                                                const bool &emergency_lm_front_left_, const bool &emergency_lm_front_right_, const bool &emergency_lm_right_pure_, const bool &emergency_lm_left_pure_,
                                                                bool &escape_fl_rotate_right_, bool &escape_l_rotate_right_, bool &escape_fr_rotate_left_, bool &escape_r_rotate_left_, bool &escape_f_rotate_right_, bool &escape_f_rotate_left_,
                                                                bool &rotate_request_left_dir_, int &rotate_request_flag_, int &ignore_rotate_request_flag_, const float &tmp_yaw_next_path_point_, int &last_rotate_dir_)
{
    int escape_fl_rotate_right_int = 0;
    int escape_l_rotate_right_int = 0;
    int escape_fr_rotate_left_int = 0;
    int escape_r_rotate_left_int = 0;
    int escape_f_rotate_right_int = 0;
    int escape_f_rotate_left_int = 0;

    // fusion handle
    if ((us_lasting_front_left_ || lm_lasting_front_left_) && !emergency_us_front_right_ && !emergency_us_right_pure_ && !emergency_lm_right_pure_)
    {
        escape_fl_rotate_right_ = true;
        escape_fl_rotate_right_int = 1;
        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_fl_rotate_right_ = true ++++++++++++");
    }
    else if ((us_lasting_left_ || lm_lasting_left_) && !emergency_us_front_right_ && !emergency_us_right_pure_ && !emergency_lm_right_pure_)
    {
        escape_l_rotate_right_ = true;
        escape_l_rotate_right_int = 1;
        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_l_rotate_right_ = true ++++++++++++");
    }
    else if ((us_lasting_front_right_ || lm_lasting_front_right_) && !emergency_us_front_left_ && !emergency_us_left_pure_ && !emergency_lm_left_pure_)
    {
        escape_fr_rotate_left_ = true;
        escape_fr_rotate_left_int = 1;
        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_fr_rotate_left_ = true ++++++++++++");
    }
    else if ((us_lasting_right_ || lm_lasting_right_) && !emergency_us_front_left_ && !emergency_us_left_pure_ && !emergency_lm_left_pure_)
    {
        escape_r_rotate_left_ = true;
        escape_r_rotate_left_int = 1;
        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_r_rotate_left_ = true ++++++++++++");
    }
    else if ((us_lasting_front_ || lm_lasting_front_) && last_rotate_dir_ == 0 && !emergency_us_front_left_ && !emergency_us_left_pure_ && !emergency_lm_left_pure_ && !emergency_us_front_right_ && !emergency_us_right_pure_ && !emergency_lm_right_pure_)
    {
        if (tmp_yaw_next_path_point_ >= 0) // 目标点在左侧
        {
            escape_f_rotate_left_ = true;
            escape_f_rotate_left_int = 1;
            PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_f_rotate_left_ = true, target on the left ++++++++++++");
            last_rotate_dir_ = 1;
        }
        else if (tmp_yaw_next_path_point_ < 0) // 目标点在右侧
        {
            escape_f_rotate_right_ = true;
            escape_f_rotate_right_int = 1;
            PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_f_rotate_right_ = true, target on the right ++++++++++++");
            last_rotate_dir_ = -1;
        }
    }
    else if ((us_lasting_front_ || lm_lasting_front_) && (last_rotate_dir_ == 0 || last_rotate_dir_ == 1) && !emergency_us_front_left_ && !emergency_us_left_pure_ && !emergency_lm_left_pure_)
    {
        escape_f_rotate_left_ = true;
        escape_f_rotate_left_int = 1;
        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_f_rotate_left_ = true ++++++++++++");
        last_rotate_dir_ = 1;
    }
    else if ((us_lasting_front_ || lm_lasting_front_) && (last_rotate_dir_ == 0 || last_rotate_dir_ == -1) && !emergency_us_front_right_ && !emergency_us_right_pure_ && !emergency_lm_right_pure_)
    {
        escape_f_rotate_right_ = true;
        escape_f_rotate_right_int = 1;
        PATHFOLLOW_PLANNER_INFO_LOG("++++++++++++ escape_f_rotate_right_ = true ++++++++++++");
        last_rotate_dir_ = -1;
    }
    else
    {
        last_rotate_dir_ = 0;
    }

    if (rotate_request_flag_ == 1)
    {
        if (rotate_request_left_dir_)
        {
            // turn left
            if (emergency_us_front_left_ || emergency_us_left_pure_ || emergency_lm_left_pure_)
            {
                ignore_rotate_request_flag_ = 1;
                PATHFOLLOW_PLANNER_INFO_LOG("turn left: ignore_rotate_request_flag = 1");
            }
        }
        else
        {
            // turn right
            if (emergency_us_front_right_ || emergency_us_right_pure_ || emergency_lm_right_pure_)
            {
                ignore_rotate_request_flag_ = 1;
                PATHFOLLOW_PLANNER_INFO_LOG("turn right: ignore_rotate_request_flag = 1");
            }
        }
    }

#ifdef LOG_OUTPUT
    PATH_FOLLOW_DEBUG_LOG_OUTPUT.open(filename, ios::out | ios::app);
    PATH_FOLLOW_DEBUG_LOG_OUTPUT << escape_l_rotate_right_int << " ";
    PATH_FOLLOW_DEBUG_LOG_OUTPUT << escape_fl_rotate_right_int << " ";
    PATH_FOLLOW_DEBUG_LOG_OUTPUT << escape_f_rotate_right_int << " ";
    PATH_FOLLOW_DEBUG_LOG_OUTPUT << escape_f_rotate_left_int << " ";
    PATH_FOLLOW_DEBUG_LOG_OUTPUT << escape_fr_rotate_left_int << " ";
    PATH_FOLLOW_DEBUG_LOG_OUTPUT << escape_r_rotate_left_int << " ";
    // PATH_FOLLOW_DEBUG_LOG_OUTPUT << have_obstacle_[0] << have_obstacle_[1] << have_obstacle_[2] << have_obstacle_[3] << have_obstacle_[4] << have_obstacle_[5] << "|" << have_obstacle_[6] << "|" << have_obstacle_[7] << have_obstacle_[8] << have_obstacle_[9] << have_obstacle_[10] << have_obstacle_[11] << " ";
    PATH_FOLLOW_DEBUG_LOG_OUTPUT.close();
#endif
}

void PathFollowPlanner::print_reached_and_skipped()
{
    // printf("dense-reached:            ");
    // for (size_t i = 0; i < path_points_dense_reached.size(); i++)
    // {
    //     if (path_points_dense_reached[i] == false)
    //     {
    //         printf("%d", 0);
    //     }
    //     else
    //     {
    //         printf("%d", 1);
    //     }
    // }
    // printf("\n");
    // printf("dense-skipped:            ");
    // for (size_t i = 0; i < path_points_dense_skipped.size(); i++)
    // {
    //     if (path_points_dense_skipped[i] == false)
    //     {
    //         printf("%d", 0);
    //     }
    //     else
    //     {
    //         printf("%d", 1);
    //     }
    // }
    // printf("\n");
    printf("dense-situation:          ");
    for (size_t i = 0; i < path_points_mpc_situation.size(); i++)
    {
        printf("%d", path_points_mpc_situation[i]);
    }
    printf("\n");
    // printf("sparse-reached:            ");
    // for (size_t i = 0; i < path_points_sparse_reached.size(); i++)
    // {
    //     if (path_points_sparse_reached[i] == false)
    //     {
    //         printf("%d", 0);
    //     }
    //     else
    //     {
    //         printf("%d", 1);
    //     }
    // }
    // printf("\n");
    // printf("sparse-skipped:            ");
    // for (size_t i = 0; i < path_points_sparse_skipped.size(); i++)
    // {
    //     if (path_points_sparse_skipped[i] == false)
    //     {
    //         printf("%d", 0);
    //     }
    //     else
    //     {
    //         printf("%d", 1);
    //     }
    // }
    // printf("\n");
    printf("sparse-situation:          ");
    for (size_t i = 0; i < path_points_pure_pursuit_situation.size(); i++)
    {
        printf("%d", path_points_pure_pursuit_situation[i]);
    }
    printf("\n");
    printf("%lu", path_points_mpc_situation.size());
    printf("\n");
    printf("%lu", path_points_pure_pursuit_situation.size());
    printf("\n");

    // DEBUG_LOG: reached & skipped
}

std::vector<std::string> PathFollowPlanner::Split(const std::string &s, const std::string &delim)
{
    std::vector<std::string> v;
    if (!delim.empty())
    {
        auto b = s.begin();
        auto i = s.end();
        while ((i = std::search(b, s.end(), delim.begin(), delim.end())) != s.end())
        {
            if (i - b > 0) // token can't be empty string
            {
                v.emplace_back(std::string(b, i));
            }

            b = i + delim.length();
        }

        if (b != s.end())
        {
            v.emplace_back(std::string(b, s.end()));
        }
    }

    return v;
}