#include "emergency_stop_detector.h"
#include "emergency_stop_detector_log.h"
#include "emergency_stop_detector_config.h"

#include "path_follow_planner/path_follow_planner_events.h"

#include "geometry/geometry_func.h"
#include "misc/planning_common_config.h"
#include "misc/robot_config.h"
#include "iostream"

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

namespace planning_detector
{
    struct EmergencyStopDetectorStates
    {
        struct BaseState : StateWithOwner<EmergencyStopDetector>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Enter Disable...");
            }

            virtual void OnExit()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Exit Disable...");
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
                EMERGENCY_STOP_DET_DEBUG_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Enter Enable...");
                Owner().dispatchThread();
            }

            virtual void OnExit()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Exit Enable...");
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
                    EMERGENCY_STOP_DET_DEBUG_LOG("Enable -> EmergencyStop_Init...");
                    return InnerEntryTransition<EmergencyStopDet_Init>();
                }
                else
                {
                    // 恢复context
                    EMERGENCY_STOP_DET_DEBUG_LOG("EmergencyStop recover context...");
                    Transition trans = Owner().restoreContext(ctx);
                    ctx = nullptr;
                    return trans;
                }

                return NoTransition();
            }

            virtual void Update()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Enable...");
                // Owner().getData();
            }
        };

        struct EmergencyStopDet_Init : BaseState
        {
            virtual void OnEnter()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Enter EmergencyStopDet_Init...");
            }

            virtual void OnExit()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Exit EmergencyStopDet_Init...");
            }

            virtual Transition GetTransition()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("EmergencyStopDet_Init -- GetTransition()...");

                return SiblingTransition<EmergencyStopDet_Running>();

                // EMERGENCY_STOP_DET_DEBUG_LOG("EmergencyStopDet_Init -- GetTransition() end...");
                // return NoTransition();
            }

            virtual void Update()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("EmergencyStopDet_Init -- update()...");
            }
        };

        struct EmergencyStopDet_Running : BaseState
        {
            uint64_t last_ts = Timer::getSystemTimestampUS();

            float feedback_speed = 0;
            float feedback_steering = 0;

            float rotation_detectable_angle = 15; // degree

            // for US Lasting check
            uint64_t us_f_start_ts;
            uint64_t us_f_reset_start_ts;
            bool us_f_timing = false;
            bool us_lasting_front = false;
            uint64_t us_fl_start_ts;
            uint64_t us_fl_reset_start_ts;
            bool us_fl_timing = false;
            bool us_lasting_front_left = false;
            uint64_t us_fr_start_ts;
            uint64_t us_fr_reset_start_ts;
            bool us_fr_timing = false;
            bool us_lasting_front_right = false;
            uint64_t us_l_start_ts;
            uint64_t us_l_reset_start_ts;
            bool us_l_timing = false;
            bool us_lasting_left = false;
            uint64_t us_r_start_ts;
            uint64_t us_r_reset_start_ts;
            bool us_r_timing = false;
            bool us_lasting_right = false;

            float lasting_period_for_US;
            float reset_lasting_period_for_US;

            // 坐标跳变相关
            int coord_jumped = 0;
            DataSlam m_slam_data_last;
            float dis_last_slam_data = 0;
            int m_slam_data_last_init = 0;
            float max_slam_dis_diff = 0;
            // for lasting check
            uint64_t cj_start_ts;
            bool cj_timing = false;
            float lasting_period_for_cj = 2;
            int print_coord_jumped = 0;

            // SMS/BMS alive check
            int print_sms_not_ready_flag = 0;
            int print_bms_not_ready_flag = 0;

            // US threshold define
            uint16_t threshold_ultrasonic_front;
            uint16_t threshold_ultrasonic_front_left;
            uint16_t threshold_ultrasonic_front_right;
            uint16_t threshold_ultrasonic_left_down_1;
            uint16_t threshold_ultrasonic_left_down_2;
            uint16_t threshold_ultrasonic_left_up_1;
            uint16_t threshold_ultrasonic_left_up_2;
            uint16_t threshold_ultrasonic_right_down_1;
            uint16_t threshold_ultrasonic_right_down_2;
            uint16_t threshold_ultrasonic_right_up_1;
            uint16_t threshold_ultrasonic_right_up_2;

            // slowdown
            float discount;

            virtual void OnEnter()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Enter EmergencyStopDet_Running...");
                lasting_period_for_US = Owner().m_lasting_period;
                // US threshold define
                threshold_ultrasonic_left_down_1 = Owner().m_threshold_ultrasonic_left_down_1; // akslam3: 23  akslam4: 25 [30]
                threshold_ultrasonic_left_down_2 = Owner().m_threshold_ultrasonic_left_down_2;
                threshold_ultrasonic_left_up_1 = 255;
                threshold_ultrasonic_left_up_2 = 255;
                threshold_ultrasonic_right_down_1 = Owner().m_threshold_ultrasonic_right_down_1; // akslam3: 25  akslam4: 23 [30]
                threshold_ultrasonic_right_down_2 = Owner().m_threshold_ultrasonic_right_down_2;
                threshold_ultrasonic_right_up_1 = 255;
                threshold_ultrasonic_right_up_2 = 255;
            }

            virtual void OnExit()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("Exit EmergencyStopDet_Running...");
            }

            virtual Transition GetTransition()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("EmergencyStopDet_Running -- GetTransition()...");

                return NoTransition();
            }

            virtual void Update()
            {
                EMERGENCY_STOP_DET_DEBUG_LOG("EmergencyStopDet_Running -- update()...");

                // check events
                float max_break_distance_feedback;
                float max_break_distance_intended;
                EvtList evt_list;
                evt_list.clear();
                uint64_t cur_ts = Timer::getSystemTimestampUS();
                // uint64_t delta_t = cur_ts - last_ts;
                // EMERGENCY_STOP_DET_DEBUG_LOG("last_ts:  %u    cur_ts:  %u    delta_t:  %u", last_ts, cur_ts, delta_t);
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
                max_break_distance_feedback = max_break_distance_feedback * 100; //  m to cm
                Owner().m_threshold_ultrasonic_front = max_break_distance_feedback + Owner().m_threshold_ultrasonic_front_default;
                Owner().m_threshold_ultrasonic_front_left = max_break_distance_feedback + Owner().m_threshold_ultrasonic_front_left_default;
                Owner().m_threshold_ultrasonic_front_right = max_break_distance_feedback + Owner().m_threshold_ultrasonic_front_right_default;

                max_break_distance_intended = 0.5 * Owner().m_intended_velocity * Owner().m_intended_velocity / Owner().m_max_break_acc;
                max_break_distance_intended = max_break_distance_intended * 100; //  m to cm
                Owner().m_threshold_slowdown_front = max_break_distance_intended;
                Owner().m_threshold_slowdown_side = max_break_distance_intended;
                discount = 1.0;
                EMERGENCY_STOP_DET_INFO_LOG("fb/itd_vel:  %.1f/%.1f [%d]    fb/itd_dis:  %.0f/%.0f [%d]", Owner().m_feedback_control.l_encoder * 0.001, Owner().m_intended_velocity, (int)(Owner().m_feedback_control.l_encoder * 0.1 / Owner().m_intended_velocity), max_break_distance_feedback, max_break_distance_intended, (int)(100 * max_break_distance_feedback / max_break_distance_intended));

                feedback_speed = Owner().m_control_speed.linear_vel;
                feedback_steering = rad2deg(Owner().m_control_speed.angular_steer);

                threshold_ultrasonic_front = Owner().m_threshold_ultrasonic_front;             // akslam3: 25  akslam4: 25 [30]
                threshold_ultrasonic_front_left = Owner().m_threshold_ultrasonic_front_left;   // akslam3: 20  akslam4: 25 [30]
                threshold_ultrasonic_front_right = Owner().m_threshold_ultrasonic_front_right; // akslam3: 20  akslam4: 24 [30]

                // 判断BMS和SMS是否alive
                // BMS
                uint64_t current_ts = Timer::getTimestampUS();
                float alive_lasting = float(current_ts - Owner().m_local_map_ready.ts) / 1000000;
                int local_map_ready_flag = Owner().m_local_map_ready.flag;
                // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "alive_lasting = %f", alive_lasting);
                if (alive_lasting > 1.5) // 1s
                {
                    local_map_ready_flag = 0;
                }
                else
                {
                    print_bms_not_ready_flag = 0;
                }
                // SMS 5
                int sms_ready_flag = 0;
                if (Owner().m_slam_data.pose_status == 5)
                {
                    sms_ready_flag = 1;
                    print_sms_not_ready_flag = 0;
                }
                EMERGENCY_STOP_DET_DEBUG_LOG("check_alive finished...");

                // 坐标跳变相关
                if (m_slam_data_last_init == 0)
                {
                    m_slam_data_last = Owner().m_slam_data;
                    m_slam_data_last_init = 1;
                }
                else
                {
                    dis_last_slam_data = hypot((Owner().m_slam_data.pose.pt.x - m_slam_data_last.pose.pt.x), (Owner().m_slam_data.pose.pt.y - m_slam_data_last.pose.pt.y));
                    if (dis_last_slam_data >= Owner().m_jumped_dist_threshold)
                    {
                        // 坐标跳了
                        coord_jumped = 1;
                        EMERGENCY_STOP_DET_DEBUG_LOG("=== coord_jumped === coord_jumped === coord_jumped ===");
                    }
                    else
                    {
                        // 坐标没跳，更新 m_slam_data_last
                        m_slam_data_last = Owner().m_slam_data;
                    }

                    if (dis_last_slam_data > max_slam_dis_diff)
                    {
                        max_slam_dis_diff = dis_last_slam_data;
                    }
                    EMERGENCY_STOP_DET_DEBUG_LOG(COLOR_L_RED "dis_last_slam_data = %f", dis_last_slam_data);
                    EMERGENCY_STOP_DET_DEBUG_LOG(COLOR_L_RED "max_slam_dis_diff  = %f", max_slam_dis_diff);
                }

                // 坐标跳变 lasting check
                if (coord_jumped == 1 && cj_timing == false)
                {
                    // start timing...
                    cj_start_ts = Timer::getTimestampUS();
                    cj_timing = true;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Coord Jumped start timing...");
                }
                else if (coord_jumped == 1 && cj_timing == true)
                {
                    uint64_t cj_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(cj_update_ts - cj_start_ts) / 1000000;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Coord Jumped lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_cj && dis_last_slam_data < Owner().m_jumped_dist_threshold) // default 2 seconds
                    {
                        if (sms_ready_flag == 1)
                        {
                            coord_jumped = 0;
                            cj_timing = false;
                            print_coord_jumped = 0;
                            EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "========== coord_jumped reset ==========");
                        }
                    }
                    else if (lasting_period > lasting_period_for_cj && dis_last_slam_data >= Owner().m_jumped_dist_threshold)
                    {
                        EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "coord_jumped not reset, dis_last_slam_data = %f", dis_last_slam_data);
                    }
                }
                EMERGENCY_STOP_DET_DEBUG_LOG("coord jumped finished...");

                // ******************************* US sensor define *******************************
                uint16_t ultrasonic_front = Owner().m_ultrasonic_data.ultrasonic_04;
                uint16_t ultrasonic_front_left = Owner().m_ultrasonic_data.ultrasonic_03;
                uint16_t ultrasonic_front_right = Owner().m_ultrasonic_data.ultrasonic_05;
                uint16_t ultrasonic_left_down_1 = Owner().m_ultrasonic_data.ultrasonic_01;
                uint16_t ultrasonic_left_down_2 = Owner().m_ultrasonic_data.ultrasonic_02;
                uint16_t ultrasonic_left_up_1 = 255;
                uint16_t ultrasonic_left_up_2 = 255;
                uint16_t ultrasonic_right_down_1 = Owner().m_ultrasonic_data.ultrasonic_07;
                uint16_t ultrasonic_right_down_2 = Owner().m_ultrasonic_data.ultrasonic_06;
                uint16_t ultrasonic_right_up_1 = 255;
                uint16_t ultrasonic_right_up_2 = 255;

                if (Owner().m_print_us == 1)
                {
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_left_down_1 = %d", ultrasonic_left_down_1);   // min akslam3:18 akslam4:18
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_left_down_2 = %d", ultrasonic_left_down_2);   // min akslam3:18 akslam4:18
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_front_left = %d", ultrasonic_front_left);     // min akslam3:13 akslam4:18
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_front = %d", ultrasonic_front);               // min akslam3:13 akslam4:18
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_front_right = %d", ultrasonic_front_right);   // min akslam3:18 akslam4:13
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_right_down_2 = %d", ultrasonic_right_down_2); // min akslam3:13 akslam4:13
                    EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "ultrasonic_right_down_1 = %d", ultrasonic_right_down_1); // min akslam3:13 akslam4:13
                }

                // calib: [30] 23, 22, 25, 23, 25
                // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Feedback_Speed    = %f", feedback_speed);
                // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Feedback_Steering = %f", feedback_steering);

                bool emergency_stop_flag_ultrasonic = false;
                bool emergency_slowdown_flag_ultrasonic = false;

                bool emergency_us_front = false;
                bool emergency_us_front_left = false;
                bool emergency_us_front_right = false;
                bool emergency_us_left = false;
                bool emergency_us_right = false;
                bool emergency_us_left_pure = false;
                bool emergency_us_right_pure = false;

                bool escape_fl_rotate_right = false;
                bool escape_l_rotate_right = false;
                bool escape_fr_rotate_left = false;
                bool escape_r_rotate_left = false;
                bool escape_f_rotate_left = false;
                bool escape_f_rotate_right = false;

                // ******************************* US active check *******************************
                // 前方slowdown
                if (ultrasonic_front <= Owner().m_threshold_slowdown_front) // 前向
                {
                    emergency_slowdown_flag_ultrasonic = true;
                    float tmp_discount = sqrtf((float)ultrasonic_front / (float)Owner().m_threshold_slowdown_front);
                    if (tmp_discount < discount)
                    {
                        discount = tmp_discount;
                    }
                    EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic front, value/threshold[cm]:  %d / %0.1f", ultrasonic_front, Owner().m_threshold_slowdown_front);
                }
                if (ultrasonic_front_right <= Owner().m_threshold_slowdown_front) //　右前方
                {
                    emergency_slowdown_flag_ultrasonic = true;
                    float tmp_discount = sqrtf((float)ultrasonic_front_right / (float)Owner().m_threshold_slowdown_front);
                    if (tmp_discount < discount)
                    {
                        discount = tmp_discount;
                    }
                    EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic front right, value/threshold[cm]:  %d / %0.1f", ultrasonic_front_right, Owner().m_threshold_slowdown_front);
                }
                if (ultrasonic_front_left <= Owner().m_threshold_slowdown_front) //　左前方
                {
                    emergency_slowdown_flag_ultrasonic = true;
                    float tmp_discount = sqrtf((float)ultrasonic_front_left / (float)Owner().m_threshold_slowdown_front);
                    if (tmp_discount < discount)
                    {
                        discount = tmp_discount;
                    }
                    EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic front left, value/threshold[cm]:  %d / %0.1f", ultrasonic_front_left, Owner().m_threshold_slowdown_front);
                }
                // 前方stop
                if (ultrasonic_front <= threshold_ultrasonic_front) // 前向
                {
                    emergency_stop_flag_ultrasonic = true;
                    emergency_us_front = true;
                    EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ US: emergency_stop_flag front, value/threshold[cm]:  %d / %d", ultrasonic_front, threshold_ultrasonic_front);
                }
                if (ultrasonic_front_right <= threshold_ultrasonic_front_right) //　右前方
                {
                    emergency_stop_flag_ultrasonic = true;
                    emergency_us_front_right = true;
                    EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ US: emergency_stop_flag front right, value/threshold[cm]:  %d / %d", ultrasonic_front_right, threshold_ultrasonic_front_right);
                }
                if (ultrasonic_front_left <= threshold_ultrasonic_front_left) //　左前方
                {
                    emergency_stop_flag_ultrasonic = true;
                    emergency_us_front_left = true;
                    EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ US: emergency_stop_flag front left, value/threshold[cm]:  %d / %d", ultrasonic_front_left, threshold_ultrasonic_front_left);
                }
                // 等新US装好，还要再加侧前方极近时的stop
                // 侧向slowdown
                if (ultrasonic_left_down_1 < Owner().m_threshold_slowdown_side || ultrasonic_left_down_2 < Owner().m_threshold_slowdown_side || ultrasonic_left_up_1 < Owner().m_threshold_slowdown_side || ultrasonic_left_up_2 < Owner().m_threshold_slowdown_side) // left
                {
                    // left
                    if (feedback_steering > rotation_detectable_angle)
                    {
                        emergency_slowdown_flag_ultrasonic = true;
                        uint16_t min_value = min(min(ultrasonic_left_down_1, ultrasonic_left_down_2), min(ultrasonic_left_up_1, ultrasonic_left_up_2));
                        float tmp_discount = sqrtf((float)min_value / (float)Owner().m_threshold_slowdown_side);
                        if (tmp_discount < discount)
                        {
                            discount = tmp_discount;
                        }
                        EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic left, threshold[cm]:  %0.1f", Owner().m_threshold_slowdown_side);
                    }
                }
                if (ultrasonic_right_down_1 < Owner().m_threshold_slowdown_side || ultrasonic_right_down_2 < Owner().m_threshold_slowdown_side || ultrasonic_right_up_1 < Owner().m_threshold_slowdown_side || ultrasonic_right_up_2 < Owner().m_threshold_slowdown_side) // right
                {
                    // right
                    if (feedback_steering < -rotation_detectable_angle)
                    {
                        emergency_slowdown_flag_ultrasonic = true;
                        uint16_t min_value = min(min(ultrasonic_right_down_1, ultrasonic_right_down_2), min(ultrasonic_right_up_1, ultrasonic_right_up_2));
                        float tmp_discount = sqrtf((float)min_value / (float)Owner().m_threshold_slowdown_side);
                        if (tmp_discount < discount)
                        {
                            discount = tmp_discount;
                        }
                        EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic right, threshold[cm]:  %0.1f", Owner().m_threshold_slowdown_side);
                    }
                }

                // 侧向stop
                if (ultrasonic_left_down_1 < threshold_ultrasonic_left_down_1 || ultrasonic_left_down_2 < threshold_ultrasonic_left_down_2 || ultrasonic_left_up_1 < threshold_ultrasonic_left_up_1 || ultrasonic_left_up_2 < threshold_ultrasonic_left_up_2) // left
                {
                    emergency_us_left_pure = true;
                    emergency_slowdown_flag_ultrasonic = true;
                    float tmp_discount = 0.3;
                    if (tmp_discount < discount)
                    {
                        discount = tmp_discount;
                    }
                    EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic left near");
                    if (feedback_steering > rotation_detectable_angle)
                    {
                        emergency_stop_flag_ultrasonic = true;
                        emergency_us_left = true;
                        EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ US: emergency_stop_flag left");
                    }
                }
                if (ultrasonic_right_down_1 < threshold_ultrasonic_right_down_1 || ultrasonic_right_down_2 < threshold_ultrasonic_right_down_2 || ultrasonic_right_up_1 < threshold_ultrasonic_right_up_1 || ultrasonic_right_up_2 < threshold_ultrasonic_right_up_2) // right
                {
                    emergency_us_right_pure = true;
                    emergency_slowdown_flag_ultrasonic = true;
                    float tmp_discount = 0.3;
                    if (tmp_discount < discount)
                    {
                        discount = tmp_discount;
                    }
                    EMERGENCY_STOP_DET_WARN_LOG("+++++++++++++++ US: emergency_slowdown_flag_ultrasonic right near");
                    if (feedback_steering < -rotation_detectable_angle)
                    {
                        emergency_stop_flag_ultrasonic = true;
                        emergency_us_right = true;
                        EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "+++++++++++++++ US: emergency_stop_flag right");
                    }
                }

                // ******************************* US Lasting check *******************************
                // front
                if (emergency_us_front && us_f_timing == false)
                {
                    // start timing...
                    us_f_start_ts = Timer::getTimestampUS();
                    us_f_reset_start_ts = Timer::getTimestampUS();
                    us_f_timing = true;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front start timing...");
                }
                else if (emergency_us_front && us_f_timing == true)
                {
                    us_f_reset_start_ts = Timer::getTimestampUS();
                    uint64_t us_f_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(us_f_update_ts - us_f_start_ts) / 1000000;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_US) // default 3 seconds
                    {
                        us_lasting_front = true;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_front true ==========");
                    }
                }
                else if (!emergency_us_front && us_f_timing == true)
                {
                    uint64_t us_f_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(us_f_reset_ts - us_f_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_US)
                    {
                        // reset
                        us_f_timing = false;
                        us_lasting_front = false;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_front reset ==========");
                    }
                }
                // front left
                if (emergency_us_front_left && us_fl_timing == false)
                {
                    // start timing...
                    us_fl_start_ts = Timer::getTimestampUS();
                    us_fl_reset_start_ts = Timer::getTimestampUS();
                    us_fl_timing = true;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Left start timing...");
                }
                else if (emergency_us_front_left && us_fl_timing == true)
                {
                    us_fl_reset_start_ts = Timer::getTimestampUS();
                    uint64_t us_fl_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(us_fl_update_ts - us_fl_start_ts) / 1000000;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_US) // default 3 seconds
                    {
                        us_lasting_front_left = true;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_front_left true ==========");
                    }
                }
                else if (!emergency_us_front_left && us_fl_timing == true)
                {
                    uint64_t us_fl_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(us_fl_reset_ts - us_fl_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_US)
                    {
                        // reset
                        us_fl_timing = false;
                        us_lasting_front_left = false;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_front_left reset ==========");
                    }
                }
                // front right
                if (emergency_us_front_right && us_fr_timing == false)
                {
                    // start timing...
                    us_fr_start_ts = Timer::getTimestampUS();
                    us_fr_reset_start_ts = Timer::getTimestampUS();
                    us_fr_timing = true;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Right start timing...");
                }
                else if (emergency_us_front_right && us_fr_timing == true)
                {
                    us_fr_reset_start_ts = Timer::getTimestampUS();
                    uint64_t us_fr_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(us_fr_update_ts - us_fr_start_ts) / 1000000;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Right lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_US) // default 3 seconds
                    {
                        us_lasting_front_right = true;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_front_right true ==========");
                    }
                }
                else if (!emergency_us_front_right && us_fr_timing == true)
                {
                    uint64_t us_fr_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(us_fr_reset_ts - us_fr_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_US)
                    {
                        // reset
                        us_fr_timing = false;
                        us_lasting_front_right = false;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_front_right reset ==========");
                    }
                }
                // left
                if (emergency_us_left && us_l_timing == false)
                {
                    // start timing...
                    us_l_start_ts = Timer::getTimestampUS();
                    us_l_reset_start_ts = Timer::getTimestampUS();
                    us_l_timing = true;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Left start timing...");
                }
                else if (emergency_us_left && us_l_timing == true)
                {
                    us_l_reset_start_ts = Timer::getTimestampUS();
                    uint64_t us_l_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(us_l_update_ts - us_l_start_ts) / 1000000;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_US) // default 3 seconds
                    {
                        us_lasting_left = true;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_left true ==========");
                    }
                }
                else if (!emergency_us_left && us_l_timing == true)
                {
                    uint64_t us_l_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(us_l_reset_ts - us_l_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_US)
                    {
                        // reset
                        us_l_timing = false;
                        us_lasting_left = false;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_left reset ==========");
                    }
                }
                // right
                if (emergency_us_right && us_r_timing == false)
                {
                    // start timing...
                    us_r_start_ts = Timer::getTimestampUS();
                    us_r_reset_start_ts = Timer::getTimestampUS();
                    us_r_timing = true;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Right start timing...");
                }
                else if (emergency_us_right && us_r_timing == true)
                {
                    us_r_reset_start_ts = Timer::getTimestampUS();
                    uint64_t us_r_update_ts = Timer::getTimestampUS();
                    float lasting_period = float(us_r_update_ts - us_r_start_ts) / 1000000;
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Left lasting:  %f", lasting_period);
                    if (lasting_period > lasting_period_for_US) // default 3 seconds
                    {
                        us_lasting_right = true;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_right true ==========");
                    }
                }
                else if (!emergency_us_right && us_r_timing == true)
                {
                    uint64_t us_r_reset_ts = Timer::getTimestampUS();
                    float lasting_reset_period = float(us_r_reset_ts - us_r_reset_start_ts) / 1000000;
                    if (lasting_reset_period > reset_lasting_period_for_US)
                    {
                        // reset
                        us_r_timing = false;
                        us_lasting_right = false;
                        // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "========== us_lasting_right reset ==========");
                    }
                }

                EMERGENCY_STOP_DET_DEBUG_LOG("US check finished...");

                // ******************************* stop/slowdown Event *******************************
                if (emergency_stop_flag_ultrasonic || coord_jumped == 1 || local_map_ready_flag == 0 || sms_ready_flag == 0)
                {
                    // Emergency Stop
                    CREATE_EVENT(EvEmergencyStop, ev_emergency_stop);
                    g_ec.pushEvent(ev_emergency_stop);
                    if (sms_ready_flag == 0 && print_sms_not_ready_flag == 0)
                    {
                        // RunOnceEvery(20, {EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency Stop Event pushed, sms not ready +++++++++++++++");});
                        EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "Emergency Stop Event pushed, sms not ready +++++++++++++++");
                        print_sms_not_ready_flag = 1;
                    }
                    if (local_map_ready_flag == 0 && print_bms_not_ready_flag == 0)
                    {
                        // RunOnceEvery(20, {EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency Stop Event pushed, bms not ready +++++++++++++++");});
                        EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "Emergency Stop Event pushed, bms not ready +++++++++++++++");
                        print_bms_not_ready_flag = 1;
                    }
                    if (coord_jumped == 1 && print_coord_jumped == 0)
                    {
                        // RunOnceEvery(20, {EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency Stop Event pushed, coord jumped +++++++++++++++");});
                        EMERGENCY_STOP_DET_WARN_LOG(COLOR_L_RED "Emergency Stop Event pushed, coord jumped +++++++++++++++");
                        print_coord_jumped = 1;
                    }
                    if (emergency_stop_flag_ultrasonic)
                    {
                        EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency Stop Event pushed, emergency_stop_flag +++++++++++++++");
                    }
                }
                else if (!emergency_stop_flag_ultrasonic && emergency_slowdown_flag_ultrasonic)
                {
                    // Emergency Slowdown
                    CREATE_EVENT(EvEmergencySlowdown, ev_emergency_slowdown);
                    ev_emergency_slowdown->discount = discount;
                    g_ec.pushEvent(ev_emergency_slowdown);
                    EMERGENCY_STOP_DET_INFO_LOG("Emergency Slowdown Event pushed +++++++++++++");
                }
                EMERGENCY_STOP_DET_DEBUG_LOG("stop/slowdown event finished...");

                // ******************************* coord jumped Event *******************************
                if (coord_jumped == 1)
                {
                    CREATE_EVENT(EvCoordJumped, ev_coord_jumped);
                    g_ec.pushEvent(ev_coord_jumped);
                    // RunOnceEvery(100, { EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Coord Jumped Event pushed +++++++++++++++"); });
                    // EMERGENCY_STOP_DET_INFO_LOG("Coord Jumped Event pushed +++++++++++++++");
                }
                EMERGENCY_STOP_DET_DEBUG_LOG("coord jumped event finished...");

                // ******************************* Single US Sensor Lasting Event *******************************
                if (us_lasting_front)
                {
                    CREATE_EVENT(EvUSFrontLasting, ev_us_front_lasting);
                    g_ec.pushEvent(ev_us_front_lasting);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Lasting Event pushed +++++++++++++++");
                }
                if (us_lasting_front_left)
                {
                    CREATE_EVENT(EvUSFrontLeftLasting, ev_us_front_lasting);
                    g_ec.pushEvent(ev_us_front_lasting);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Left Lasting Event pushed +++++++++++++++");
                }
                if (us_lasting_front_right)
                {
                    CREATE_EVENT(EvUSFrontRightLasting, ev_us_front_lasting);
                    g_ec.pushEvent(ev_us_front_lasting);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Front Right Lasting Event pushed +++++++++++++++");
                }
                if (us_lasting_left)
                {
                    CREATE_EVENT(EvUSLeftLasting, ev_us_front_lasting);
                    g_ec.pushEvent(ev_us_front_lasting);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Left Lasting Event pushed +++++++++++++++");
                }
                if (us_lasting_right)
                {
                    CREATE_EVENT(EvUSRightLasting, ev_us_front_lasting);
                    g_ec.pushEvent(ev_us_front_lasting);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "US Right Lasting Event pushed +++++++++++++++");
                }

                // ******************************* Single US Sensor Event *******************************
                if (emergency_us_front)
                {
                    CREATE_EVENT(EvEmergencyUSFront, ev_emergency_us_front);
                    g_ec.pushEvent(ev_emergency_us_front);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Front Event pushed +++++++++++++++");
                }
                if (emergency_us_front_left)
                {
                    CREATE_EVENT(EvEmergencyUSFrontLeft, ev_emergency_us_front_left);
                    g_ec.pushEvent(ev_emergency_us_front_left);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Front Left Event pushed +++++++++++++++");
                }
                if (emergency_us_front_right)
                {
                    CREATE_EVENT(EvEmergencyUSFrontRight, ev_emergency_us_front_right);
                    g_ec.pushEvent(ev_emergency_us_front_right);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Front Right Event pushed +++++++++++++++");
                }
                if (emergency_us_left)
                {
                    CREATE_EVENT(EvEmergencyUSLeft, ev_emergency_us_left);
                    g_ec.pushEvent(ev_emergency_us_left);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Left Event pushed +++++++++++++++");
                }
                if (emergency_us_right)
                {
                    CREATE_EVENT(EvEmergencyUSRight, ev_emergency_us_right);
                    g_ec.pushEvent(ev_emergency_us_right);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Right Event pushed +++++++++++++++");
                }
                if (emergency_us_left_pure)
                {
                    CREATE_EVENT(EvEmergencyUSLeftPure, ev_emergency_us_left_pure);
                    g_ec.pushEvent(ev_emergency_us_left_pure);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Left Pure Event pushed +++++++++++++++");
                }
                if (emergency_us_right_pure)
                {
                    CREATE_EVENT(EvEmergencyUSRightPure, ev_emergency_us_right_pure);
                    g_ec.pushEvent(ev_emergency_us_right_pure);
                    // EMERGENCY_STOP_DET_INFO_LOG(COLOR_L_RED "Emergency US Right Pure Event pushed +++++++++++++++");
                }
            }
        };
    };
} // namespace planning_detector

DEFINE_CONFIG_TYPE(CONFIG_EMERGENCY_STOP_DETECTOR, EmergencyStopDetector);

EmergencyStopDetector &g_emergency_stop_detector = EmergencyStopDetector::getInstance(); //定义EmergencyStop检测器

EmergencyStopDetector &EmergencyStopDetector::getInstance()
{
    static EmergencyStopDetector instance;
    return instance;
}

EmergencyStopDetector::EmergencyStopDetector()
    : Detector()
{
    m_ultrasonic_data.ultrasonic_01 = 255;
    m_ultrasonic_data.ultrasonic_02 = 255;
    m_ultrasonic_data.ultrasonic_03 = 255;
    m_ultrasonic_data.ultrasonic_04 = 255;
    m_ultrasonic_data.ultrasonic_05 = 255;
    m_ultrasonic_data.ultrasonic_06 = 255;
    m_ultrasonic_data.ultrasonic_07 = 255;
    m_must_stop = false;
}

float EmergencyStopDetector::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigEmergencyStopDetector *cfg_esd = dynamic_cast<ConfigEmergencyStopDetector *>(
        cfg_mgr.GetSubConfig(CONFIG_EMERGENCY_STOP_DETECTOR));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    ConfigRobot *cfg_robot = dynamic_cast<ConfigRobot *>(
        cfg_mgr.GetSubConfig(CONFIG_ROBOT));

    cfg_esd->log_path = cfg_planning->log_path;

    m_max_break_acc = cfg_robot->max_break_acc;

    m_lasting_period = cfg_esd->lasting_period;
    m_reset_lasting_period = cfg_esd->reset_lasting_period;
    m_jumped_dist_threshold = cfg_esd->jumped_dist_threshold;
    m_threshold_ultrasonic_front_default = cfg_esd->threshold_ultrasonic_front_default;
    m_threshold_ultrasonic_front_left_default = cfg_esd->threshold_ultrasonic_front_left_default;
    m_threshold_ultrasonic_front_right_default = cfg_esd->threshold_ultrasonic_front_right_default;
    m_threshold_ultrasonic_left_down_1 = cfg_esd->threshold_ultrasonic_left_down_1;
    m_threshold_ultrasonic_right_down_1 = cfg_esd->threshold_ultrasonic_right_down_1;
    m_print_us = cfg_esd->print_us;

    CREATE_LOG(PlainText, LOG_EMERGENCY_STOP_DETECTOR_FLAG, LOG_EMERGENCY_STOP_DETECTOR,
               cfg_esd->log_name, cfg_esd->log_path,
               cfg_esd->log_extension, cfg_esd->log_ts_mask,
               cfg_esd->log_print_to_console,
               (cfg_esd->log_max_file_size_mb)MB + (cfg_esd->log_max_file_size_kb)KB,
               cfg_esd->log_max_file_cnt, cfg_esd->log_level);

    m_log_name = LOG_EMERGENCY_STOP_DETECTOR;

    EMERGENCY_STOP_DET_DEBUG_LOG("emergency stop detector config...\n");
    return cfg_esd->polling_frequency;
}

void EmergencyStopDetector::initRunSM()
{
    m_run_sm.Initialize<EmergencyStopDetectorStates::Disable>(this);
    m_run_sm.SetDebugInfo("EmergencyStopDetector", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void EmergencyStopDetector::reset()
{
    m_ultrasonic_data.ultrasonic_01 = 255;
    m_ultrasonic_data.ultrasonic_02 = 255;
    m_ultrasonic_data.ultrasonic_03 = 255;
    m_ultrasonic_data.ultrasonic_04 = 255;
    m_ultrasonic_data.ultrasonic_05 = 255;
    m_ultrasonic_data.ultrasonic_06 = 255;
    m_ultrasonic_data.ultrasonic_07 = 255;
    m_must_stop = false;
}

void EmergencyStopDetector::getData()
{
    g_dc.getData<DataSlam>(m_slam_data);
    g_dc.getData<DataUltrasonic>(m_ultrasonic_data);
    g_dc.getData<DataControlSpeed>(m_control_speed);
    g_dc.getData<DataLocalMap>(m_simple_local_map);
    g_dc.getData<DataLocalMapReady>(m_local_map_ready);
    g_dc.getData<DataFeedbackSpeed>(m_feedback_control);
}

bool EmergencyStopDetector::handleParam(DetectorParamPtr param)
{
    EMERGENCY_STOP_DET_DEBUG_LOG("emergency stop det start...");
    EmergencyStopDetectorParamPtr emergency_stop_det_param = std::dynamic_pointer_cast<EmergencyStopDetectorParam>(param);

    m_intended_velocity = emergency_stop_det_param->vehicle_velocity;

    // EMERGENCY_STOP_DET_DEBUG_LOG("handle emergency stop det...");
    // EMERGENCY_STOP_DET_DEBUG_LOG("m_emergency_stop_dir:%d", m_emergency_stop_dir);
    // EMERGENCY_STOP_DET_DEBUG_LOG("m_start_pose:{%f, %f, %f}", m_start_pose.pt.x, m_start_pose.pt.y, m_start_pose.theta);
    // EMERGENCY_STOP_DET_DEBUG_LOG("m_start_time:%d", m_start_time);
    // EMERGENCY_STOP_DET_DEBUG_LOG("");
    return true;
}

void EmergencyStopDetector::finishDetect()
{
    // EMERGENCY_STOP_DET_DEBUG_LOG("generate emergency stop detector finished evt id = %ld", m_input_id);
    // CREATE_EVENT(EvPlannerFinished, ev_emergency_stop_finished);
    // ev_emergency_stop_finished->id = m_input_id;
    // g_ec.pushEvent(ev_emergency_stop_finished);
}

ContextPtr EmergencyStopDetector::saveContext()
{
    EMERGENCY_STOP_DET_DEBUG_LOG("save EmergencyStopDetector context...");
    CREATE_DETECTOR_CONTEXT(ContextEmergencyStopDetector, esd_ctx);
    //TODO: 需要保存的变量、信息内容

    return esd_ctx;
}

Transition EmergencyStopDetector::restoreContext(ContextPtr ctx)
{
    EMERGENCY_STOP_DET_DEBUG_LOG("restore EmergencyStopDetector context...");

    return NoTransition();
}