#include "obstacle_detector.h"
#include "obstacle_detector_log.h"
#include "obstacle_detector_config.h"

#include "path_follow_planner/path_follow_planner_events.h"

#include "geometry/geometry_func.h"
#include "misc/planning_common_config.h"
#include "misc/robot_config.h"

using namespace planning_detector;
using namespace planning_utils;

#define RunOnceEvery(_prescaler, _code) \
    {                                   \
        static uint16_t prescaler = 0;  \
        prescaler++;                    \
        if (prescaler >= _prescaler)    \
        {                               \
            prescaler = 0;              \
            _code;                      \
        }                               \
    }

// #define FULL_DIRECTION_SEG

#ifdef FULL_DIRECTION_SEG
const static size_t segmentation_num = 12;
#else
const static size_t segmentation_num = 7; // 0:left  1:front left  2:front(near)  3:front right  4:right  5: center zone  6:front far
#endif

namespace planning_detector
{
    struct ObstacleDetectorStates
    {
        struct BaseState : StateWithOwner<ObstacleDetector>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter()
            {
                OBSTACLE_DET_DEBUG_LOG("Enter Disable...");
            }

            virtual void OnExit()
            {
                OBSTACLE_DET_DEBUG_LOG("Exit Disable...");
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
                OBSTACLE_DET_DEBUG_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                OBSTACLE_DET_DEBUG_LOG("Enter Enable...");
                Owner().dispatchThread();
            }

            virtual void OnExit()
            {
                OBSTACLE_DET_DEBUG_LOG("Exit Enable...");
                Owner().stopThread();
            }

            virtual Transition GetTransition()
            {
                ContextPtr &ctx = Owner().m_context;
                if (!Owner().m_run_loop_enabled)
                {
                    ctx = Owner().saveContext();
                    return SiblingTransition<Disable>();
                }

                if (ctx == nullptr)
                {
                    // 正常开始
                    OBSTACLE_DET_DEBUG_LOG("Enable -> Obstacle_Init...");
                    return InnerEntryTransition<ObstacleDet_Init>();
                }
                else
                {
                    // 恢复context
                    OBSTACLE_DET_DEBUG_LOG("Obstacle recover context...");
                    Transition trans = Owner().restoreContext(ctx);
                    ctx = nullptr;
                    return trans;
                }

                return NoTransition();
            }

            virtual void Update()
            {
                OBSTACLE_DET_DEBUG_LOG("Enable...");
                // Owner().getData();
            }
        };

        struct ObstacleDet_Init : BaseState
        {
            virtual void OnEnter()
            {
                OBSTACLE_DET_INFO_LOG("Enter ObstacleDet_Init...");
            }

            virtual void OnExit()
            {
                OBSTACLE_DET_DEBUG_LOG("Exit ObstacleDet_Init...");
            }

            virtual Transition GetTransition()
            {
                OBSTACLE_DET_DEBUG_LOG("ObstacleDet_Init -- GetTransition()...");

                return SiblingTransition<ObstacleDet_Running>();

                // OBSTACLE_DET_INFO_LOG("ObstacleDet_Init -- GetTransition() end...");
                // return NoTransition();
            }

            virtual void Update()
            {
                OBSTACLE_DET_DEBUG_LOG("ObstacleDet_Init -- update()...");
            }
        };

        struct ObstacleDet_Running : BaseState
        {
            uint64_t last_ts = Timer::getSystemTimestampUS();

            // float feedback_speed = 0;
            float feedback_steering = 0;
            float rotation_detectable_angle = 15; // degree
            float k;
            float k_for_far_circle;
            float F;
            float F_for_far_circle;
            float k_for_near_circle;

            // for LM Lasting check
            uint64_t lm_f_start_ts;
            uint64_t lm_f_reset_start_ts;
            bool lm_f_timing = false;
            bool lm_lasting_front = false;
            uint64_t lm_l_start_ts;
            uint64_t lm_l_reset_start_ts;
            bool lm_l_timing = false;
            bool lm_lasting_left = false;
            uint64_t lm_fl_start_ts;
            uint64_t lm_fl_reset_start_ts;
            bool lm_fl_timing = false;
            bool lm_lasting_front_left = false;
            uint64_t lm_r_start_ts;
            uint64_t lm_r_reset_start_ts;
            bool lm_r_timing = false;
            bool lm_lasting_right = false;
            uint64_t lm_fr_start_ts;
            uint64_t lm_fr_reset_start_ts;
            bool lm_fr_timing = false;
            bool lm_lasting_front_right = false;

            float lasting_period_for_lm;
            float reset_lasting_period_for_lm;

            // vector<RobotGrid> grid_with_obstacle_in_GOI;
            vector<vector<float>> F_l;
            vector<vector<float>> F_r;
            vector<vector<float>> F_n;
            vector<vector<float>> F_f;
            vector<float> k_longitude;
            vector<float> k_lateral;
            vector<float> bias_longitude_l_in_function;
            vector<float> bias_longitude_r_in_function;
            vector<float> bias_lateral_in_function;
            vector<float> bias_lateral_bottom_in_function;
            vector<float> angles_of_field;

            vector<int> have_obstacle;

            bool small_obstacle_flag;
            bool far_circle_obstacle_flag;
            bool near_circle_obstacle_flag;
            int obstacle_dir_in_far_circle;
            int obstacle_dir_in_near_circle;

            virtual void OnEnter()
            {
                OBSTACLE_DET_INFO_LOG("Enter ObstacleDet_Running...");
                lasting_period_for_lm = Owner().m_lasting_period;
                reset_lasting_period_for_lm = Owner().m_reset_lasting_period;
            }

            virtual void OnExit()
            {
                OBSTACLE_DET_INFO_LOG("Exit ObstacleDet_Running...");
            }

            virtual Transition GetTransition()
            {
                OBSTACLE_DET_DEBUG_LOG("ObstacleDet_Running -- GetTransition()...");

                return NoTransition();
            }

            virtual void Update()
            {
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "ObstacleDet_Running -- update()...");
                uint64_t update_start_ts = Timer::getTimestampUS();
                small_obstacle_flag = false;
                far_circle_obstacle_flag = false;
                near_circle_obstacle_flag = false;
                obstacle_dir_in_far_circle = 0;
                obstacle_dir_in_near_circle = 0;
                Owner().grid_of_interest.clear();
                Owner().grid_of_far_circle.clear();
                Owner().grid_of_near_circle.clear();

                F_l.clear();
                F_r.clear();
                F_n.clear();
                F_f.clear();
                k_longitude.clear();
                k_lateral.clear();
                bias_longitude_l_in_function.clear();
                bias_longitude_r_in_function.clear();
                bias_lateral_in_function.clear();
                bias_lateral_bottom_in_function.clear();

                bool emergency_stop_flag_lm = false;
                bool emergency_slowdown_flag_lm = false;
                bool freeze_stop_flag_lm = false;

                bool emergency_lm_front = false;
                bool emergency_lm_left = false;
                bool emergency_lm_front_left = false;
                bool emergency_lm_right = false;
                bool emergency_lm_front_right = false;
                bool emergency_lm_left_pure = false;
                bool emergency_lm_right_pure = false;

                // feedback_speed = Owner().m_control_speed.linear_vel;
                feedback_steering = rad2deg(Owner().m_control_speed.angular_steer);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Feedback_Speed    = %f", feedback_speed);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Feedback_Steering = %f", feedback_steering);

                // check events
                float max_break_distance_feedback;
                float max_break_distance_intended;
                EvtList evt_list;
                evt_list.clear();
                uint64_t cur_ts = Timer::getSystemTimestampUS();
                // uint64_t delta_t = cur_ts - last_ts;
                // OBSTACLE_DET_DEBUG_LOG("last_ts:  %u    cur_ts:  %u    delta_t:  %u", last_ts, cur_ts, delta_t);
                g_ec.eventDeduction(last_ts, cur_ts, evt_list);
                last_ts = cur_ts;
                for (auto &evt : evt_list)
                {
                    std::type_index evt_type = evt->getType();
                    if (TYPE_EQUALS(evt_type, EvPFPDetIntendVel))
                    {
                        EvPFPDetIntendVelPtr det_intend_vel_evt = dynamic_pointer_cast<EvPFPDetIntendVel>(evt);
                        Owner().m_intended_velocity = det_intend_vel_evt->intend_velocity;
                    }
                }

                // update safety related params according to intended velocity
                if (Owner().m_max_break_acc == 0)
                    Owner().m_max_break_acc = 0.3;

                max_break_distance_feedback = 0.5 * Owner().m_feedback_control.l_encoder * 0.001 * Owner().m_feedback_control.l_encoder * 0.001 / Owner().m_max_break_acc;
                Owner().m_forward_safe_distance = max_break_distance_feedback + Owner().m_forward_safe_distance_default;
                Owner().m_front_side_safe_distance = max_break_distance_feedback + Owner().m_front_side_safe_distance_default;

                max_break_distance_intended = 0.5 * Owner().m_intended_velocity * Owner().m_intended_velocity / Owner().m_max_break_acc;
                Owner().m_grid_of_interest_radius = max_break_distance_intended + Owner().m_grid_of_interest_radius_default;
                Owner().m_grid_of_far_circle_radius = max_break_distance_intended + Owner().m_grid_of_far_circle_radius_default;

#ifdef FULL_DIRECTION_SEG
                bias_longitude = m_vehicle_lateral_size / 2;
                bias_lateral_near = m_vehicle_longitude_size + m_side_safe_distance;
                bias_lateral = m_grid_of_interest_radius;
#else
                Owner().bias_longitude = Owner().m_vehicle_lateral_size / 2 + Owner().m_side_safe_distance;
                Owner().bias_lateral_near = Owner().m_vehicle_longitude_size;
                Owner().bias_lateral = Owner().m_grid_of_interest_radius;
                Owner().front_seg_bottom = Owner().m_vehicle_longitude_size;
#endif
                OBSTACLE_DET_DEBUG_LOG("fb/itd_vel:  %.1f/%.1f [%d]    fb/itd_dis:  %.2f/%.2f [%d]", Owner().m_feedback_control.l_encoder * 0.001, Owner().m_intended_velocity, (int)(Owner().m_feedback_control.l_encoder * 0.1 / Owner().m_intended_velocity), max_break_distance_feedback, max_break_distance_intended, (int)(100 * max_break_distance_feedback / max_break_distance_intended));

                // ******************************* update robot_pose_in_local_map *******************************
                float dx_float = (Owner().m_slam_data.pose.pt.x - Owner().m_simple_local_map.x) / Owner().m_local_map_resolution;
                float dy_float = (Owner().m_slam_data.pose.pt.y - Owner().m_simple_local_map.y) / Owner().m_local_map_resolution;
                int dx_int = (int)round(dx_float);
                int dy_int = (int)round(dy_float);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "slam_data_x:  %f    slam_data_y:  %f", Owner().m_slam_data.pose.pt.x, Owner().m_slam_data.pose.pt.y);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "local_map_x:  %f    local_map_y:  %f", Owner().m_simple_local_map.x, Owner().m_simple_local_map.y);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "dx_float:  %f    dy_float:  %f", dx_float, dy_float);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "dx_int:    %d    dy_int:    %d", dx_int, dy_int);
                Owner().m_robot_pose_in_local_map_x = Owner().m_robot_pose_in_local_map_x_default + dx_int;
                Owner().m_robot_pose_in_local_map_y = Owner().m_robot_pose_in_local_map_y_default + dy_int;
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "m_robot_pose_in_local_map_x:   %d   m_robot_pose_in_local_map_y:   %d", Owner().m_robot_pose_in_local_map_x, Owner().m_robot_pose_in_local_map_y);

                // ******************************* 搜索grid_of_interest_radius内的grid *******************************
                for (int i = 0; i < Owner().m_local_map_size_x; i++)
                {
                    for (int j = 0; j < Owner().m_local_map_size_y; j++)
                    {
                        // circle shape
                        float grid_distance = hypot((i - Owner().m_robot_pose_in_local_map_x), (j - Owner().m_robot_pose_in_local_map_y));
                        if (grid_distance <= Owner().m_grid_of_interest_radius / Owner().m_local_map_resolution)
                        {
                            RobotGrid grid_tmp;
                            grid_tmp.x = i;
                            grid_tmp.y = j;
                            Owner().grid_of_interest.push_back(grid_tmp);
                        }
                    }
                }
                // OBSTACLE_DET_INFO_LOG("grid_of_interest size:  %u", Owner().grid_of_interest.size());

                // ******************************* update have_obstacle *******************************
                have_obstacle.clear();
                for (size_t i = 0; i < segmentation_num; i++)
                {
                    have_obstacle.push_back(0);
                }
                // clear every Update
                // grid_with_obstacle_in_GOI.clear();
                // update segmentations
                Owner().update_segmentations(segmentation_num, angles_of_field,
                                             k_longitude, k_lateral,
                                             bias_longitude_l_in_function, bias_longitude_r_in_function, bias_lateral_in_function, bias_lateral_bottom_in_function,
                                             F_l, F_r, F_n, F_f);
                // record grid_of_interest as pic
                // Owner().record_grid_of_interest_pic(angles_of_field, F_l, F_r, F_n, F_f);

                // ******************************* 搜索 grid_of_interest 内的grid, 给出have_obstacle *******************************
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "update have_obstalce");
                for (size_t i = 0; i < Owner().grid_of_interest.size(); i++)
                {
                    if (Owner().m_simple_local_map.grid[Owner().grid_of_interest[i].x][Owner().grid_of_interest[i].y] != 0)
                    {
                        // grid_with_obstacle_in_GOI.push_back(Owner().grid_of_interest[i]);
                        for (size_t j = 0; j < segmentation_num; j++)
                        {
                            // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "%f | %f %f %f %f", angles_of_field[j], F_l[i][j], F_r[i][j], F_n[i][j], F_f[i][j]);
                            if (angles_of_field[j] == 180 && F_l[i][j] > 0 && F_r[i][j] < 0 && F_n[i][j] < 0 && F_f[i][j] > 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] > 90 && angles_of_field[j] < 180 && F_l[i][j] > 0 && F_r[i][j] < 0 && F_n[i][j] > 0 && F_f[i][j] < 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] == 90 && F_l[i][j] > 0 && F_r[i][j] < 0 && F_n[i][j] > 0 && F_f[i][j] < 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] > 0 && angles_of_field[j] < 90 && F_l[i][j] < 0 && F_r[i][j] > 0 && F_n[i][j] > 0 && F_f[i][j] < 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] == 0 && F_l[i][j] < 0 && F_r[i][j] > 0 && F_n[i][j] > 0 && F_f[i][j] < 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] > -90 && angles_of_field[j] < 0 && F_l[i][j] < 0 && F_r[i][j] > 0 && F_n[i][j] < 0 && F_f[i][j] > 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] == -90 && F_l[i][j] < 0 && F_r[i][j] > 0 && F_n[i][j] < 0 && F_f[i][j] > 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                            if (angles_of_field[j] > -180 && angles_of_field[j] < -90 && F_l[i][j] > 0 && F_r[i][j] < 0 && F_n[i][j] < 0 && F_f[i][j] > 0)
                            {
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "==================In segment: %u", j);
                                have_obstacle[j] = 1;
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "j: %d  grid_of_interest[i].x: %d  grid_of_interest[i].y: %d", j, Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                            }
                        }
                    }
                }

// print have_obstacle
#ifdef FULL_DIRECTION_SEG
                OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "have_obstalce: %d%d%d%d%d%d|%d|%d%d%d%d%d", have_obstacle[0], have_obstacle[1], have_obstacle[2], have_obstacle[3], have_obstacle[4], have_obstacle[5], have_obstacle[6], have_obstacle[7], have_obstacle[8], have_obstacle[9], have_obstacle[10], have_obstacle[11]);
#else
                //   OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "have_obstalce: %d%d|%d|%d%d [%d] [%d]", have_obstacle[0], have_obstacle[1], have_obstacle[2], have_obstacle[3], have_obstacle[4], have_obstacle[5], have_obstacle[6]);
#endif
                // ******************************* 搜索 grid_of_far_circle 内的grid *******************************
                // 根据robot航向角确定身前
                if (Owner().m_slam_data.pose.theta == 0)
                {
                    k_for_far_circle = 0;
                }
                else
                {
                    k_for_far_circle = -1 / tan(Owner().m_slam_data.pose.theta); // 分界线: f = y-80 - k*(x-80)
                }
                k_for_near_circle = k_for_far_circle;

                // 搜索 grid_of_far_circle 前半区内的grid, 以及 grid_of_near_circle 内的
                for (int i = 0; i < Owner().m_local_map_size_x; i++)
                {
                    for (int j = 0; j < Owner().m_local_map_size_y; j++)
                    {
                        float grid_distance = hypot((i - Owner().m_robot_pose_in_local_map_x), (j - Owner().m_robot_pose_in_local_map_y));
                        // grid_of_far_circle
                        if (grid_distance <= Owner().m_grid_of_far_circle_radius / Owner().m_local_map_resolution)
                        {
                            RobotGrid grid_tmp;
                            grid_tmp.x = i;
                            grid_tmp.y = j;
                            // 判断是否在身前
                            if (k_for_far_circle == 0 && i >= Owner().m_robot_pose_in_local_map_x)
                            {
                                Owner().grid_of_far_circle.push_back(grid_tmp);
                                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_interest: x  %d  y  %d   \n", i, j);
                            }
                            else
                            {
                                F_for_far_circle = j - Owner().m_robot_pose_in_local_map_y - k_for_far_circle * (i - Owner().m_robot_pose_in_local_map_x);
                                if ((F_for_far_circle > 0 && Owner().m_slam_data.pose.theta > 0 && Owner().m_slam_data.pose.theta <= M_PI) || (F_for_far_circle < 0 && Owner().m_slam_data.pose.theta >= -M_PI && Owner().m_slam_data.pose.theta < 0)) // 身前
                                {
                                    Owner().grid_of_far_circle.push_back(grid_tmp);
                                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_far_circle: x  %d  y  %d  F_for_far_circle  %f \n", i, j, F_for_far_circle);
                                }
                            }
                        }
                        // grid_of_near_circle
                        if (grid_distance <= Owner().m_grid_of_near_circle_radius / Owner().m_local_map_resolution)
                        {
                            RobotGrid grid_tmp;
                            grid_tmp.x = i;
                            grid_tmp.y = j;
                            Owner().grid_of_near_circle.push_back(grid_tmp);
                        }
                    }
                }

                // ******************************* 探测障碍物 *******************************
                // 正前方
                if (have_obstacle[6] != 0)
                {
                    small_obstacle_flag = true;
                }
                if (have_obstacle[5] != 0)
                {
                    emergency_stop_flag_lm = true;
                    freeze_stop_flag_lm = true;
                }

                for (size_t i = 0; i < Owner().grid_of_far_circle.size(); i++)
                {
                    if (Owner().m_simple_local_map.grid[Owner().grid_of_far_circle[i].x][Owner().grid_of_far_circle[i].y] != 0)
                    {
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_far_circle_with_obstacle: x  %d  y  %d  \n", Owner().grid_of_far_circle[i].x, Owner().grid_of_far_circle[i].y);
                        far_circle_obstacle_flag = true;
                        // 判断左右
                        float relative_angle = atan2(Owner().grid_of_far_circle[i].y - Owner().m_robot_pose_in_local_map_y, Owner().grid_of_far_circle[i].x - Owner().m_robot_pose_in_local_map_x);
                        relative_angle = toNPPiAngleRangeR(relative_angle - Owner().m_slam_data.pose.theta);
                        if (relative_angle > 0)
                        {
                            obstacle_dir_in_far_circle = 1; // left
                            // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_far_circle_with_obstacle: dir  %d  x  %d  y  %d  \n", obstacle_dir_in_far_circle, Owner().grid_of_far_circle[i].x, Owner().grid_of_far_circle[i].y);
                        }
                        else if (relative_angle < 0)
                        {
                            obstacle_dir_in_far_circle = -1; // right
                            // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_far_circle_with_obstacle: dir  %d  x  %d  y  %d  \n", obstacle_dir_in_far_circle, Owner().grid_of_far_circle[i].x, Owner().grid_of_far_circle[i].y);
                        }
                        break;
                    }
                }

                for (size_t i = 0; i < Owner().grid_of_near_circle.size(); i++)
                {
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "here...");
                    if (Owner().m_simple_local_map.grid[Owner().grid_of_near_circle[i].x][Owner().grid_of_near_circle[i].y] != 0)
                    {
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_interest_with_obstacle: x  %d  y  %d  \n", Owner().grid_of_interest[i].x, Owner().grid_of_interest[i].y);
                        near_circle_obstacle_flag = true;
                        // 判断左右
                        float relative_angle = atan2(Owner().grid_of_near_circle[i].y - Owner().m_robot_pose_in_local_map_y, Owner().grid_of_near_circle[i].x - Owner().m_robot_pose_in_local_map_x);
                        relative_angle = toNPPiAngleRangeR(relative_angle - Owner().m_slam_data.pose.theta);
                        if (relative_angle > 0)
                        {
                            obstacle_dir_in_near_circle = 1; // left
                            // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_near_circle_with_obstacle: dir  %d  x  %d  y  %d  \n", obstacle_dir_in_near_circle, Owner().grid_of_near_circle[i].x, Owner().grid_of_near_circle[i].y);
                        }
                        else if (relative_angle < 0)
                        {
                            obstacle_dir_in_near_circle = -1; // right
                            // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "grid_of_near_circle_with_obstacle: dir  %d  x  %d  y  %d  \n", obstacle_dir_in_near_circle, Owner().grid_of_near_circle[i].x, Owner().grid_of_near_circle[i].y);
                        }
                        break;
                    }
                }

                // ******************************* LM感知与运动意图fusion *******************************
                if (have_obstacle[2] != 0) // front
                {
                    emergency_lm_front = true;
                    emergency_stop_flag_lm = true;
                    OBSTACLE_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ LM: emergency_stop_flag front, threshold[m]:  %.2f", Owner().m_forward_safe_distance);
                }
                if (have_obstacle[0] != 0) // left
                {
                    emergency_lm_left_pure = true;
                    emergency_slowdown_flag_lm = true;
                    if (feedback_steering > rotation_detectable_angle)
                    {
                        emergency_stop_flag_lm = true;
                        emergency_lm_left = true;
                        OBSTACLE_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ LM: emergency_stop_flag left, threshold[m]:  %.2f", Owner().m_side_safe_distance);
                    }
                }
                if (have_obstacle[1] != 0) // front left
                {
                    if (feedback_steering > rotation_detectable_angle)
                    {
                        emergency_lm_front_left = true;
                        emergency_stop_flag_lm = true;
                        OBSTACLE_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ LM: emergency_stop_flag front left, threshold[m]:  %.2f  %.2f", Owner().m_front_side_safe_distance, Owner().m_side_safe_distance);
                    }
                }
                if (have_obstacle[4] != 0) // right
                {
                    emergency_lm_right_pure = true;
                    emergency_slowdown_flag_lm = true;
                    if (feedback_steering < -rotation_detectable_angle)
                    {
                        emergency_stop_flag_lm = true;
                        emergency_lm_right = true;
                        OBSTACLE_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ LM: emergency_stop_flag right, threshold[m]:  %.2f", Owner().m_side_safe_distance);
                    }
                }
                if (have_obstacle[3] != 0) // front right
                {
                    if (feedback_steering < -rotation_detectable_angle)
                    {
                        emergency_lm_front_right = true;
                        emergency_stop_flag_lm = true;
                        OBSTACLE_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ LM: emergency_stop_flag front right, threshold[m]:  %.2f  %.2f", Owner().m_front_side_safe_distance, Owner().m_side_safe_distance);
                    }
                }

                // ******************************* LM Lasting check *******************************
                // front
                if (emergency_lm_front && lm_f_timing == false && !freeze_stop_flag_lm)
                {
                    // start timing...
                    lm_f_start_ts = Timer::getTimestampUS();
                    lm_f_reset_start_ts = Timer::getTimestampUS();
                    lm_f_timing = true;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front start timing...");
                }
                else if (emergency_lm_front && lm_f_timing == true)
                {
                    lm_f_reset_start_ts = Timer::getTimestampUS();
                    uint64_t lm_f_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(lm_f_update_ts - lm_f_start_ts) / 1000000;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_lm) // default 3 seconds
                    {
                        lm_lasting_front = true;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_front true ==========");
                    }
                }
                else if (!emergency_lm_front && lm_f_timing == true)
                {
                    uint64_t lm_f_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(lm_f_reset_ts - lm_f_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_lm)
                    {
                        // reset
                        lm_f_timing = false;
                        lm_lasting_front = false;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_front reset ==========");
                    }
                }
                // left
                if (emergency_lm_left && lm_l_timing == false && !freeze_stop_flag_lm)
                {
                    // start timing...
                    lm_l_start_ts = Timer::getTimestampUS();
                    lm_l_reset_start_ts = Timer::getTimestampUS();
                    lm_l_timing = true;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Left start timing...");
                }
                else if (emergency_lm_left && lm_l_timing == true)
                {
                    lm_l_reset_start_ts = Timer::getTimestampUS();
                    uint64_t lm_l_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(lm_l_update_ts - lm_l_start_ts) / 1000000;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_lm) // default 3 seconds
                    {
                        lm_lasting_left = true;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_left true ==========");
                    }
                }
                else if (!emergency_lm_left && lm_l_timing == true)
                {
                    uint64_t lm_l_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(lm_l_reset_ts - lm_l_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_lm)
                    {
                        // reset
                        lm_l_timing = false;
                        lm_lasting_left = false;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_left reset ==========");
                    }
                }
                // front left
                if (emergency_lm_front_left && lm_fl_timing == false && !freeze_stop_flag_lm)
                {
                    // start timing...
                    lm_fl_start_ts = Timer::getTimestampUS();
                    lm_fl_reset_start_ts = Timer::getTimestampUS();
                    lm_fl_timing = true;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Left start timing...");
                }
                else if (emergency_lm_front_left && lm_fl_timing == true)
                {
                    lm_fl_reset_start_ts = Timer::getTimestampUS();
                    uint64_t lm_fl_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(lm_fl_update_ts - lm_fl_start_ts) / 1000000;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_lm) // default 3 seconds
                    {
                        lm_lasting_front_left = true;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_front left true ==========");
                    }
                }
                else if (!emergency_lm_front_left && lm_fl_timing == true)
                {
                    uint64_t lm_fl_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(lm_fl_reset_ts - lm_fl_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_lm)
                    {
                        // reset
                        lm_fl_timing = false;
                        lm_lasting_front_left = false;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_front_left reset ==========");
                    }
                }
                // right
                if (emergency_lm_right && lm_r_timing == false && !freeze_stop_flag_lm)
                {
                    // start timing...
                    lm_r_start_ts = Timer::getTimestampUS();
                    lm_r_reset_start_ts = Timer::getTimestampUS();
                    lm_r_timing = true;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Right start timing...");
                }
                else if (emergency_lm_right && lm_r_timing == true)
                {
                    lm_r_reset_start_ts = Timer::getTimestampUS();
                    uint64_t lm_r_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(lm_r_update_ts - lm_r_start_ts) / 1000000;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_lm) // default 3 seconds
                    {
                        lm_lasting_right = true;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_right true ==========");
                    }
                }
                else if (!emergency_lm_right && lm_r_timing == true)
                {
                    uint64_t lm_r_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(lm_r_reset_ts - lm_r_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_lm)
                    {
                        // reset
                        lm_r_timing = false;
                        lm_lasting_right = false;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_right reset ==========");
                    }
                }
                // front right
                if (emergency_lm_front_right && lm_fr_timing == false && !freeze_stop_flag_lm)
                {
                    // start timing...
                    lm_fr_start_ts = Timer::getTimestampUS();
                    lm_fr_reset_start_ts = Timer::getTimestampUS();
                    lm_fr_timing = true;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Right start timing...");
                }
                else if (emergency_lm_front_right && lm_fr_timing == true)
                {
                    lm_fr_reset_start_ts = Timer::getTimestampUS();
                    uint64_t lm_fr_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(lm_fr_update_ts - lm_fr_start_ts) / 1000000;
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Right lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_lm) // default 3 seconds
                    {
                        lm_lasting_front_right = true;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_front_right true ==========");
                    }
                }
                else if (!emergency_lm_front_right && lm_fr_timing == true)
                {
                    uint64_t lm_fr_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(lm_fr_reset_ts - lm_fr_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_lm)
                    {
                        // reset
                        lm_fr_timing = false;
                        lm_lasting_front_right = false;
                        // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "========== lm_lasting_front_right reset ==========");
                    }
                }

                // ******************************* PUSH Events *******************************
                // SmallObstacle
                if (small_obstacle_flag == true)
                {
                    CREATE_EVENT(EvSmallObstacle, ev_small_obstacle);
                    g_ec.pushEvent(ev_small_obstacle);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "+++++++++++++ Small Obstacle Event pushed +++++++++++++");
                }

                if (far_circle_obstacle_flag == true)
                {
                    CREATE_EVENT(EvFarCircleObstacle, ev_far_circle_obstacle);
                    ev_far_circle_obstacle->obstacle_dir = obstacle_dir_in_far_circle;
                    g_ec.pushEvent(ev_far_circle_obstacle);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "++++++++++ Far Circle Obstacle Event pushed ++++++++++");
                }

                if (near_circle_obstacle_flag == true)
                {
                    CREATE_EVENT(EvNearCircleObstacle, ev_near_circle_obstacle);
                    ev_near_circle_obstacle->obstacle_dir = obstacle_dir_in_near_circle;
                    g_ec.pushEvent(ev_near_circle_obstacle);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "++++++++++ Near Circle Obstacle Event pushed ++++++++++");
                }

                // ******************************* stop/slowdown/freeze Event *******************************
                if (freeze_stop_flag_lm)
                {
                    // Freeze Stop
                    CREATE_EVENT(EvLMFreezeStop, ev_lm_freeze_stop);
                    g_ec.pushEvent(ev_lm_freeze_stop);
                    OBSTACLE_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ LM Freeze Stop Event pushed");
                }
                if (emergency_stop_flag_lm)
                {
                    // Emergency Stop
                    CREATE_EVENT(EvLMEmergencyStop, ev_lm_emergency_stop);
                    g_ec.pushEvent(ev_lm_emergency_stop);
                    OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "+++++++++++++++ LM Emergency Stop Event pushed");
                }
                else if (!emergency_stop_flag_lm && emergency_slowdown_flag_lm)
                {
                    // Emergency Slowdown
                    CREATE_EVENT(EvLMEmergencySlowdown, ev_lm_emergency_slowdown);
                    g_ec.pushEvent(ev_lm_emergency_slowdown);
                    OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "+++++++++++++++ LM Emergency Slowdown Event pushed");
                }

                // ******************************* Single LM Lasting Event *******************************
                if (lm_lasting_front)
                {
                    CREATE_EVENT(EvLMFrontLasting, ev_lm_front_lasting);
                    g_ec.pushEvent(ev_lm_front_lasting);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Left Lasting Event pushed +++++++++++++++");
                }
                if (lm_lasting_left)
                {
                    CREATE_EVENT(EvLMLeftLasting, ev_lm_left_lasting);
                    g_ec.pushEvent(ev_lm_left_lasting);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Left Lasting Event pushed +++++++++++++++");
                }
                if (lm_lasting_front_left)
                {
                    CREATE_EVENT(EvLMFrontLeftLasting, ev_lm_front_left_lasting);
                    g_ec.pushEvent(ev_lm_front_left_lasting);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front Left Lasting Event pushed +++++++++++++++");
                }
                if (lm_lasting_right)
                {
                    CREATE_EVENT(EvLMRightLasting, ev_lm_right_lasting);
                    g_ec.pushEvent(ev_lm_right_lasting);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Right Lasting Event pushed +++++++++++++++");
                }
                if (lm_lasting_front_right)
                {
                    CREATE_EVENT(EvLMFrontRightLasting, ev_lm_front_right_lasting);
                    g_ec.pushEvent(ev_lm_front_right_lasting);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "LM Front_Right Lasting Event pushed +++++++++++++++");
                }

                // ******************************* Single LM Event *******************************
                if (emergency_lm_front)
                {
                    CREATE_EVENT(EvEmergencyLMFront, ev_emergency_lm_front);
                    g_ec.pushEvent(ev_emergency_lm_front);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Front Event pushed +++++++++++++++");
                }
                if (emergency_lm_left)
                {
                    CREATE_EVENT(EvEmergencyLMLeft, ev_emergency_lm_left);
                    g_ec.pushEvent(ev_emergency_lm_left);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Left Event pushed +++++++++++++++");
                }
                if (emergency_lm_front_left)
                {
                    CREATE_EVENT(EvEmergencyLMFrontLeft, ev_emergency_lm_front_left);
                    g_ec.pushEvent(ev_emergency_lm_front_left);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Front Left Event pushed +++++++++++++++");
                }
                if (emergency_lm_right)
                {
                    CREATE_EVENT(EvEmergencyLMRight, ev_emergency_lm_right);
                    g_ec.pushEvent(ev_emergency_lm_right);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Right Event pushed +++++++++++++++");
                }
                if (emergency_lm_front_right)
                {
                    CREATE_EVENT(EvEmergencyLMFrontRight, ev_emergency_lm_front_right);
                    g_ec.pushEvent(ev_emergency_lm_front_right);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Front Right Event pushed +++++++++++++++");
                }
                if (emergency_lm_left_pure)
                {
                    CREATE_EVENT(EvEmergencyLMLeftPure, ev_emergency_lm_left_pure);
                    g_ec.pushEvent(ev_emergency_lm_left_pure);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Left Pure Event pushed +++++++++++++++");
                }
                if (emergency_lm_right_pure)
                {
                    CREATE_EVENT(EvEmergencyLMRightPure, ev_emergency_lm_right_pure);
                    g_ec.pushEvent(ev_emergency_lm_right_pure);
                    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "Emergency LM Right Pure Event pushed +++++++++++++++");
                }

                CREATE_EVENT(EvHaveObstacle, ev_have_obstacle);
                ev_have_obstacle->have_obstacle = have_obstacle;
                g_ec.pushEvent(ev_have_obstacle);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "++++++++++ Have Obstacle Event pushed ++++++++++");

                // // LargeObstacle
                // CREATE_EVENT(EvLargeObstacle, ev_large_obstacle);
                // g_ec.pushEvent(ev_large_obstacle);
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "+++++++++++++Large Obstacle Event pushed+++++++++++++");

                uint64_t update_end_ts = Timer::getTimestampUS();
                float update_period = float(update_end_ts - update_start_ts) / 1000000;
                // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "update_period:  %f", update_period);
            }
        };
    };
} // namespace planning_detector

DEFINE_CONFIG_TYPE(CONFIG_OBSTACLE_DETECTOR, ObstacleDetector);

ObstacleDetector &g_obstacle_detector = ObstacleDetector::getInstance(); //定义Obstacle检测器

ObstacleDetector &ObstacleDetector::getInstance()
{
    static ObstacleDetector instance;
    return instance;
}

ObstacleDetector::ObstacleDetector()
    : Detector()
{
    grid_of_interest.clear();
    grid_of_far_circle.clear();
    // m_must_stop = false;
}

float ObstacleDetector::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigObstacleDetector *cfg_od = dynamic_cast<ConfigObstacleDetector *>(
        cfg_mgr.GetSubConfig(CONFIG_OBSTACLE_DETECTOR));
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
    m_max_break_acc = cfg_robot->max_break_acc;

    m_lasting_period = cfg_od->lasting_period;
    m_reset_lasting_period = cfg_od->reset_lasting_period;
    m_vehicle_lateral_inflate_size = cfg_od->vehicle_lateral_inflate_size;
    m_forward_safe_distance_default = cfg_od->forward_safe_distance_default;
    m_front_side_safe_distance_default = cfg_od->front_side_safe_distance_default;
    m_side_safe_distance = cfg_od->side_safe_distance;
    m_left_safe_dis_bias = cfg_od->left_safe_dis_bias;
    m_right_safe_dis_bias = cfg_od->right_safe_dis_bias;
    m_grid_of_interest_radius_default = cfg_od->grid_of_interest_radius_default;
    m_grid_of_far_circle_radius_default = cfg_od->grid_of_far_circle_radius_default;
    m_grid_of_near_circle_radius = cfg_od->grid_of_near_circle_radius;

    cfg_od->log_path = cfg_planning->log_path;
    CREATE_LOG(PlainText, LOG_OBSTACLE_DETECTOR_FLAG, LOG_OBSTACLE_DETECTOR,
               cfg_od->log_name, cfg_od->log_path,
               cfg_od->log_extension, cfg_od->log_ts_mask,
               cfg_od->log_print_to_console,
               (cfg_od->log_max_file_size_mb)MB + (cfg_od->log_max_file_size_kb)KB,
               cfg_od->log_max_file_cnt, cfg_od->log_level);

    m_log_name = LOG_OBSTACLE_DETECTOR;

    OBSTACLE_DET_INFO_LOG("obstacle detector config...\n");
    return cfg_od->polling_frequency;
}

void ObstacleDetector::initRunSM()
{
    m_run_sm.Initialize<ObstacleDetectorStates::Disable>(this);
    m_run_sm.SetDebugInfo("ObstacleDetector", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void ObstacleDetector::reset()
{
    grid_of_interest.clear();
    grid_of_far_circle.clear();
    // m_must_stop = false;
}

void ObstacleDetector::getData()
{
    g_dc.getData<DataSlam>(m_slam_data);
    // g_dc.getData<DataUltrasonic>(m_ultrasonic_data);
    g_dc.getData<DataControlSpeed>(m_control_speed);
    g_dc.getData<DataLocalMap>(m_simple_local_map);
    g_dc.getData<DataFeedbackSpeed>(m_feedback_control);
}

bool ObstacleDetector::handleParam(DetectorParamPtr param)
{
    OBSTACLE_DET_INFO_LOG("obstacle det start...");

    ObstacleDetectorParamPtr obstacle_det_param = std::dynamic_pointer_cast<ObstacleDetectorParam>(param);
    m_intended_velocity = obstacle_det_param->vehicle_velocity;

    // OBSTACLE_DET_INFO_LOG("handle obstalee det...");
    return true;
}

void ObstacleDetector::finishDetect()
{
    // OBSTACLE_DET_INFO_LOG("generate obstacle detector finished evt id = %ld", m_input_id);
    // CREATE_EVENT(EvPlannerFinished, ev_obstacle_det_finished);
    // ev_obstacle_det_finished->id = m_input_id;
    // g_ec.pushEvent(ev_obstacle_det_finished);
}

ContextPtr ObstacleDetector::saveContext()
{
    OBSTACLE_DET_INFO_LOG("save ObstacleDetector context...");
    CREATE_DETECTOR_CONTEXT(ContextObstacleDetector, od_ctx);
    //TODO: 需要保存的变量、信息内容

    return od_ctx;
}

Transition ObstacleDetector::restoreContext(ContextPtr ctx)
{
    OBSTACLE_DET_INFO_LOG("restore ObstacleDetector context...");

    return NoTransition();
}

void ObstacleDetector::update_segmentations(const size_t &segmentation_num_, vector<float> &angles_of_field_,
                                            vector<float> &k_longitude_, vector<float> &k_lateral_,
                                            vector<float> &bias_longitude_l_in_function_, vector<float> &bias_longitude_r_in_function_,
                                            vector<float> &bias_lateral_in_function_, vector<float> &bias_lateral_bottom_in_function_,
                                            vector<vector<float>> &F_l_, vector<vector<float>> &F_r_,
                                            vector<vector<float>> &F_n_, vector<vector<float>> &F_f_)
{
    k_longitude_.clear();
    k_lateral_.clear();
    bias_longitude_l_in_function_.clear();
    bias_longitude_r_in_function_.clear();
    bias_lateral_in_function_.clear();
    bias_lateral_bottom_in_function_.clear();
    angles_of_field_.clear();

    /* description
    bias_longitude = m_vehicle_lateral_size / 2 + m_side_safe_distance; 用于左右感知区
    bias_lateral_near = m_vehicle_longitude_size;　　　　　　　　　　　　　　用于左右感知区
    bias_lateral = m_grid_of_interest_radius;　　　　　　　　　用于正前方(远)探测区
    front_seg_bottom = m_vehicle_longitude_size;      用于正前方(远/近)探测区
    SEG_3: 车身尺寸SEG
    SEG_4: 车前类US感知区
    */

#ifdef FULL_DIRECTION_SEG
    float cake_angle = 2 * M_PI / segmentation_num_;

    // define segmentation
    for (size_t i = 0; i < segmentation_num_; i++)
    {
        angles_of_field_.push_back(rad2deg(toNPPiAngleRangeR(M_PI - cake_angle * i + m_slam_data.pose.theta)));
        // printf("new segmentation: %f \n", angles_of_field_[i]);
    }

    // ****************segmetation of local map, parameters setting
    for (size_t i = 0; i < segmentation_num_; i++)
    {
        if (angles_of_field_[i] != 180 && angles_of_field_[i] != 90 && angles_of_field_[i] != 0 && angles_of_field_[i] != -90)
        {
            k_longitude_.push_back(tan(deg2rad(angles_of_field_[i])));
            k_lateral_.push_back(-1 / k_longitude_[i]);
            bias_longitude_in_function_.push_back(bias_longitude * sqrt(1 + k_longitude_[i] * k_longitude_[i]));
            bias_lateral_in_function_.push_back(bias_lateral_near * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
        }
        else
        {
            k_longitude_.push_back(0);
            k_lateral_.push_back(0);
            bias_longitude_in_function_.push_back(0);
            bias_lateral_in_function_.push_back(bias_lateral_near);
        }

        // 除了正中的seg，其他seg范围都是车长，正中seg的范围是m_grid_of_interest_radius
        if (i == segmentation_num_ / 2)
        {
            bias_lateral_in_function_[i] = bias_lateral * sqrt(1 + k_lateral_[i] * k_lateral_[i]);
        }
    }
    // printf("size of k: %d %d %d %d \n", k_longitude_.size(), k_lateral_.size(), bias_longitude_in_function_.size(), bias_lateral_in_function_.size());
#else
    for (size_t i = 0; i < segmentation_num_; i++)
    {
        angles_of_field_.push_back(rad2deg(m_slam_data.pose.theta));
        if (angles_of_field_[0] != 180 && angles_of_field_[0] != 90 && angles_of_field_[0] != 0 && angles_of_field_[0] != -90)
        {
            k_longitude_.push_back(tan(deg2rad(angles_of_field_[0])));
            k_lateral_.push_back(-1 / k_longitude_[0]);
            if (i == 0) // left
            {
                bias_longitude_l_in_function_.push_back(bias_longitude * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(-m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back(bias_lateral_near * sqrt(1 + k_lateral_[0] * k_lateral_[0]));
                bias_lateral_bottom_in_function_.push_back(0);
            }
            if (i == 4) // right
            {
                bias_longitude_l_in_function_.push_back(-m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(bias_longitude * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back(bias_lateral_near * sqrt(1 + k_lateral_[0] * k_lateral_[0]));
                bias_lateral_bottom_in_function_.push_back(0);
            }
            if (i == 1) // front left
            {
                bias_longitude_l_in_function_.push_back(bias_longitude * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(-m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back((m_vehicle_longitude_size + m_front_side_safe_distance) * sqrt(1 + k_lateral_[0] * k_lateral_[0]));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
            }
            if (i == 3) // front right
            {
                bias_longitude_l_in_function_.push_back(-m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(bias_longitude * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back((m_vehicle_longitude_size + m_front_side_safe_distance) * sqrt(1 + k_lateral_[0] * k_lateral_[0]));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
            }
            if (i == 6) // front far
            {
                bias_longitude_l_in_function_.push_back(m_vehicle_lateral_inflate_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(m_vehicle_lateral_inflate_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back(bias_lateral * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
            }
            if (i == 5) // center zone
            {
                bias_longitude_l_in_function_.push_back(m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back(m_vehicle_longitude_size * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
                bias_lateral_bottom_in_function_.push_back(0);
            }
            if (i == 2) // front near
            {
                bias_longitude_l_in_function_.push_back(m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_longitude_r_in_function_.push_back(m_vehicle_lateral_size * 0.5 * sqrt(1 + k_longitude_[0] * k_longitude_[0]));
                bias_lateral_in_function_.push_back((m_vehicle_longitude_size + m_forward_safe_distance) * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom * sqrt(1 + k_lateral_[i] * k_lateral_[i]));
            }
        }
        else
        {
            k_longitude_.push_back(0);
            k_lateral_.push_back(0);
            if (i == 0) // left
            {
                bias_longitude_l_in_function_.push_back(bias_longitude);
                bias_longitude_r_in_function_.push_back(-m_vehicle_lateral_size * 0.5);
                bias_lateral_in_function_.push_back(bias_lateral_near);
                bias_lateral_bottom_in_function_.push_back(0);
            }
            if (i == 4) // right
            {
                bias_longitude_l_in_function_.push_back(-m_vehicle_lateral_size * 0.5);
                bias_longitude_r_in_function_.push_back(bias_longitude);
                bias_lateral_in_function_.push_back(bias_lateral_near);
                bias_lateral_bottom_in_function_.push_back(0);
            }
            if (i == 1) // front left
            {
                bias_longitude_l_in_function_.push_back(bias_longitude);
                bias_longitude_r_in_function_.push_back(-m_vehicle_lateral_size * 0.5);
                bias_lateral_in_function_.push_back((m_vehicle_longitude_size + m_front_side_safe_distance));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom);
            }
            if (i == 3) // front right
            {
                bias_longitude_l_in_function_.push_back(-m_vehicle_lateral_size * 0.5);
                bias_longitude_r_in_function_.push_back(bias_longitude);
                bias_lateral_in_function_.push_back((m_vehicle_longitude_size + m_front_side_safe_distance));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom);
            }
            if (i == 6) // front far
            {
                bias_longitude_l_in_function_.push_back(m_vehicle_lateral_inflate_size * 0.5);
                bias_longitude_r_in_function_.push_back(m_vehicle_lateral_inflate_size * 0.5);
                bias_lateral_in_function_.push_back(bias_lateral);
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom);
            }
            if (i == 5) // center zone
            {
                bias_longitude_l_in_function_.push_back(m_vehicle_lateral_size * 0.5);
                bias_longitude_r_in_function_.push_back(m_vehicle_lateral_size * 0.5);
                bias_lateral_in_function_.push_back(m_vehicle_longitude_size);
                bias_lateral_bottom_in_function_.push_back(0);
            }
            if (i == 2) // front near
            {
                bias_longitude_l_in_function_.push_back(m_vehicle_lateral_size * 0.5);
                bias_longitude_r_in_function_.push_back(m_vehicle_lateral_size * 0.5);
                bias_lateral_in_function_.push_back((m_vehicle_longitude_size + m_forward_safe_distance));
                bias_lateral_bottom_in_function_.push_back(front_seg_bottom);
            }
        }
    }
#endif
    // segmentation of grid_of_interest
    vector<float> F_l_each_grid;
    vector<float> F_r_each_grid;
    vector<float> F_n_each_grid;
    vector<float> F_f_each_grid;
    F_l_.clear();
    F_r_.clear();
    F_n_.clear();
    F_f_.clear();

#ifdef FULL_DIRECTION_SEG
    // OBSTACLE_DET_INFO_LOG(COLOR_L_PURPLE "m_robot_pose_in_local_map: x= %d  y= %d", m_robot_pose_in_local_map_x, m_robot_pose_in_local_map_y);
    for (size_t i = 0; i < grid_of_interest.size(); i++)
    {
        F_l_each_grid.clear();
        F_r_each_grid.clear();
        F_n_each_grid.clear();
        F_f_each_grid.clear();
        for (size_t j = 0; j < segmentation_num_; j++)
        {
            if (angles_of_field_[j] == 180)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_longitude);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_longitude);
                F_n_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution);
                F_f_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > 90 && angles_of_field_[j] < 180)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] == 90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_longitude);
                F_r_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_longitude);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > 0 && angles_of_field_[j] < 90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] == 0)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_longitude);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_longitude);
                F_n_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution);
                F_f_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > -90 && angles_of_field_[j] < 0)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] + bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] == -90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_longitude);
                F_r_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_longitude);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > -180 && angles_of_field_[j] < -90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] + bias_lateral_in_function_[j]);
            }
        }
        F_l_.push_back(F_l_each_grid);
        F_r_.push_back(F_r_each_grid);
        F_n_.push_back(F_n_each_grid);
        F_f_.push_back(F_f_each_grid);
    }
#else
    for (size_t i = 0; i < grid_of_interest.size(); i++)
    {
        F_l_each_grid.clear();
        F_r_each_grid.clear();
        F_n_each_grid.clear();
        F_f_each_grid.clear();
        for (size_t j = 0; j < segmentation_num_; j++)
        {
            if (angles_of_field_[j] == 180)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > 90 && angles_of_field_[j] < 180)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] - bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] == 90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > 0 && angles_of_field_[j] < 90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] - bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] == 0)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > -90 && angles_of_field_[j] < 0)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] + bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] + bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] == -90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution - bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].x - m_robot_pose_in_local_map_x) * m_local_map_resolution + bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution + bias_lateral_in_function_[j]);
            }
            else if (angles_of_field_[j] > -180 && angles_of_field_[j] < -90)
            {
                F_l_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] + bias_longitude_l_in_function_[j]);
                F_r_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_longitude_[j] - bias_longitude_r_in_function_[j]);
                F_n_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] + bias_lateral_bottom_in_function_[j]);
                F_f_each_grid.push_back((grid_of_interest[i].y - m_robot_pose_in_local_map_y) * m_local_map_resolution - m_local_map_resolution * (grid_of_interest[i].x - m_robot_pose_in_local_map_x) * k_lateral_[j] + bias_lateral_in_function_[j]);
            }
        }
        F_l_.push_back(F_l_each_grid);
        F_r_.push_back(F_r_each_grid);
        F_n_.push_back(F_n_each_grid);
        F_f_.push_back(F_f_each_grid);
    }
#endif
}

// void ObstacleDetector::record_grid_of_interest_pic()
// {
//     cv::Mat GOI_in_OD_im(m_local_map_size_x, m_local_map_size_y, CV_8UC3);
//     GOI_in_OD_im = GOI_in_OD_im * 0;
//     cv::Vec3b colorGOI(0x00, 0x00, 0xFF);
//     cv::Vec3b colorBot(0xFF, 0x00, 0x00);

//     for (size_t i = 0; i < grid_of_interest.size(); i++)
//     {
//         GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorGOI;
//     }

//     GOI_in_OD_im.at<cv::Vec3b>(m_robot_pose_in_local_map_x, m_robot_pose_in_local_map_y) = colorBot;
//     std::string filename = "/tmp/local_map_pms/GOI/" + std::to_string(m_slam_data.ts) + ".jpg";
//     cv::imwrite(filename, GOI_in_OD_im);
// }

void ObstacleDetector::record_grid_of_interest_pic(const vector<float> &angles_of_field_,
                                                   const vector<vector<float>> &F_l_, const vector<vector<float>> &F_r_,
                                                   const vector<vector<float>> &F_n_, const vector<vector<float>> &F_f_)
{
    cv::Mat GOI_in_OD_im(m_local_map_size_x, m_local_map_size_y, CV_8UC3);
    GOI_in_OD_im = GOI_in_OD_im * 0;
#ifdef FULL_DIRECTION_SEG
    cv::Vec3b colorSEG0(0x7F, 0x00, 0x7F);
    cv::Vec3b colorSEG1(0x7F, 0x00, 0x00);
    cv::Vec3b colorSEG2(0x9F, 0x00, 0x00);
    cv::Vec3b colorSEG3(0xBF, 0x00, 0x00);
    cv::Vec3b colorSEG4(0xDF, 0x00, 0x00);
    cv::Vec3b colorSEG5(0xFF, 0x00, 0x00);
    cv::Vec3b colorSEG6(0xFF, 0x00, 0xFF);
    cv::Vec3b colorSEG7(0x00, 0x00, 0xFF);
    cv::Vec3b colorSEG8(0x00, 0x00, 0xDF);
    cv::Vec3b colorSEG9(0x00, 0x00, 0xBF);
    cv::Vec3b colorSEG10(0x00, 0x00, 0x9F);
    cv::Vec3b colorSEG11(0x00, 0x00, 0x7F);
    std::vector<cv::Vec3b> colorSEG;
    colorSEG.push_back(colorSEG0);
    colorSEG.push_back(colorSEG1);
    colorSEG.push_back(colorSEG2);
    colorSEG.push_back(colorSEG3);
    colorSEG.push_back(colorSEG4);
    colorSEG.push_back(colorSEG5);
    colorSEG.push_back(colorSEG6);
    colorSEG.push_back(colorSEG7);
    colorSEG.push_back(colorSEG8);
    colorSEG.push_back(colorSEG9);
    colorSEG.push_back(colorSEG10);
    colorSEG.push_back(colorSEG11);
#else
    cv::Vec3b colorSEG0(0xFF, 0x00, 0x00);
    cv::Vec3b colorSEG1(0x7F, 0x00, 0x00);
    cv::Vec3b colorSEG2(0xFF, 0x00, 0xFF);
    cv::Vec3b colorSEG3(0x00, 0x00, 0x7F);
    cv::Vec3b colorSEG4(0x00, 0x00, 0xFF);
    cv::Vec3b colorSEG5(0x00, 0x7F, 0x7F);
    cv::Vec3b colorSEG6(0x00, 0xFF, 0x00);
    std::vector<cv::Vec3b> colorSEG;
    colorSEG.push_back(colorSEG0);
    colorSEG.push_back(colorSEG1);
    colorSEG.push_back(colorSEG2);
    colorSEG.push_back(colorSEG3);
    colorSEG.push_back(colorSEG4);
    colorSEG.push_back(colorSEG5);
    colorSEG.push_back(colorSEG6);
#endif

    cv::Vec3b colorObs(0xFF, 0xFF, 0x00);
    cv::Vec3b colorBot(0x00, 0xFF, 0xFF);

    for (size_t i = 0; i < grid_of_interest.size(); i++)
    {
        {
            size_t j = 6;
            if (angles_of_field_[j] == 180 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > 90 && angles_of_field_[j] < 180 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == 90 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > 0 && angles_of_field_[j] < 90 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == 0 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > -90 && angles_of_field_[j] < 0 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == -90 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > -180 && angles_of_field_[j] < -90 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
        }

        for (size_t j = 0; j < 5; j++)
        {
            if (angles_of_field_[j] == 180 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > 90 && angles_of_field_[j] < 180 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == 90 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > 0 && angles_of_field_[j] < 90 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == 0 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > -90 && angles_of_field_[j] < 0 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == -90 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > -180 && angles_of_field_[j] < -90 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
        }

        {
            size_t j = 5;
            if (angles_of_field_[j] == 180 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > 90 && angles_of_field_[j] < 180 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == 90 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > 0 && angles_of_field_[j] < 90 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == 0 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] > 0 && F_f_[i][j] < 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > -90 && angles_of_field_[j] < 0 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] == -90 && F_l_[i][j] < 0 && F_r_[i][j] > 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
            if (angles_of_field_[j] > -180 && angles_of_field_[j] < -90 && F_l_[i][j] > 0 && F_r_[i][j] < 0 && F_n_[i][j] < 0 && F_f_[i][j] > 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(grid_of_interest[i].x, grid_of_interest[i].y) = colorSEG[j];
            }
        }
    }

    for (int i = 0; i < m_local_map_size_x; i++)
    {
        for (int j = 0; j < m_local_map_size_y; j++)
        {
            if (m_simple_local_map.grid[i][j] != 0)
            {
                GOI_in_OD_im.at<cv::Vec3b>(i, j) = colorObs;
            }
        }
    }

    GOI_in_OD_im.at<cv::Vec3b>(m_robot_pose_in_local_map_x, m_robot_pose_in_local_map_y) = colorBot;
    struct timespec curtime;
    clock_gettime(CLOCK_MONOTONIC, &curtime);
    uint64_t record_ts = curtime.tv_sec * 1e6 + curtime.tv_nsec / 1e3;
    std::string filename = "/tmp/GOI_ppp/" + std::to_string(record_ts) + ".jpg";
    // std::string filename = "/tmp/GOI_ppp.jpg";
    cv::imwrite(filename, GOI_in_OD_im);
}