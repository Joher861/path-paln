#include "rotate_planner.h"
#include "rotate_planner_log.h"
#include "rotate_planner_config.h"

#include "local_planner/local_planner_events.h"
#include "path_follow_planner/path_follow_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "speed_controller/speed_controller.h"
#include "misc/planning_common_config.h"

#include "emergency_stop_detector/emergency_stop_detector.h"
#include "obstacle_detector/obstacle_detector.h"

using namespace planning_controller;
using namespace planning_utils;
using namespace std;

namespace planning_controller
{
    struct RotatePlannerStates
    {
        struct BaseState : StateWithOwner<RotatePlanner>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter()
            {
                // ROTATE_DEBUG_LOG("Enter Disable...");
            }

            virtual void OnExit()
            {
                // ROTATE_DEBUG_LOG("Exit Disable...");
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
                // ROTATE_DEBUG_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            uint64_t last_ts_enable = Timer::getSystemTimestampUS();
            bool path_follow_go_exit_flag = false;

            virtual void OnEnter()
            {
                ROTATE_DEBUG_LOG("Enter Enable...");
                Owner().dispatchThread();
            }

            virtual void OnExit()
            {
                ROTATE_DEBUG_LOG("Exit Enable...");
                if (path_follow_go_exit_flag) // 因为pfp停了退出的话，要把速度置0
                {
                    // g_speed_controller.SetStopKeepSteering(true);
                    g_speed_controller.SetStop(true);
                    ROTATE_DEBUG_LOG("path_follow_go_exit_flag: SetStop");
                }
                Owner().stopThread();
            }

            virtual Transition GetTransition()
            {
                // if (!Owner().m_run_loop_enabled || (path_follow_go_exit_flag && IsInInnerState<Finished>()))
                if (!Owner().m_run_loop_enabled || path_follow_go_exit_flag)
                {
                    return SiblingTransition<Disable>();
                }
                if (Owner().m_type == TaskRotate::TARGET_ANGLE)
                {
                    return InnerEntryTransition<Rotate_target_angle>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                // ROTATE_DEBUG_LOG("Enable...");
                // path_follow_go_exit_flag = false;
                // check events
                EvtList evt_list_enable;
                evt_list_enable.clear();
                uint64_t cur_ts_enable = Timer::getSystemTimestampUS();
                // uint64_t delta_t_enable = cur_ts_enable - last_ts_enable;
                // ROTATE_DEBUG_LOG("last_ts_enable:  %u    cur_ts_enable:  %u    delta_t_enable:  %u", last_ts_enable, cur_ts_enable, delta_t_enable);
                g_ec.eventDeduction(last_ts_enable, cur_ts_enable, evt_list_enable);
                last_ts_enable = cur_ts_enable;
                for (auto &evt : evt_list_enable)
                {
                    std::type_index evt_type = evt->getType();
                    if (TYPE_EQUALS(evt_type, EvPathFollowGoExit)) // Path_Follow_Go Exit
                    {
                        ROTATE_DEBUG_LOG("=============Path_Follow_Go Exit=============");
                        path_follow_go_exit_flag = true;
                    }
                }
            }
        };

        struct Rotate_target_angle : BaseState
        {
            float min_deg = 5;
            uint64_t last_ts = Timer::getSystemTimestampUS();

            // for stuck
            uint64_t cur_ts_for_us_stop_left_dir;
            uint64_t start_ts_for_us_stop_left_dir;
            uint64_t cur_ts_for_us_stop_right_dir;
            uint64_t start_ts_for_us_stop_right_dir;

            // for standby
            uint64_t cur_ts_for_standby_left_dir;
            uint64_t start_ts_for_standby_left_dir;
            uint64_t cur_ts_for_standby_right_dir;
            uint64_t start_ts_for_standby_right_dir;

            bool left_dir_stuck_timing = false;
            bool right_dir_stuck_timing = false;
            bool left_dir_standby_timing = false;
            bool right_dir_standby_timing = false;

            float lasting_period_stuck = 5;   // seconds
            float lasting_period_standby = 5; // seconds

            virtual void OnEnter()
            {
                ROTATE_DEBUG_LOG("Enter Rotate_target_angle...");
            }

            virtual void OnExit()
            {
                ROTATE_DEBUG_LOG("Exit Rotate_target_angle...");
                Owner().m_type = TaskRotate::MAX_ROTATE_TYPE;
            }

            virtual Transition GetTransition()
            {
                if (Owner().rotation_finished == true)
                {
                    return SiblingTransition<Finished>();
                }
                if (Owner().stuck_flag == true)
                {
                    return SiblingTransition<Rotation_Stuck>();
                }
                if (Owner().standby_flag == true)
                {
                    return SiblingTransition<Rotation_Standby>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                ROTATE_DEBUG_LOG("Rotate_target_angle...");
                // bool emergency_us_front = false;
                bool emergency_us_front_left = false;
                bool emergency_us_front_right = false;
                // bool emergency_us_left = false;
                // bool emergency_us_right = false;
                bool emergency_us_left_pure = false;
                bool emergency_us_right_pure = false;

                bool emergency_lm_front_left = false;
                bool emergency_lm_front_right = false;
                // bool emergency_lm_left = false;
                // bool emergency_lm_right = false;
                bool emergency_lm_left_pure = false;
                bool emergency_lm_right_pure = false;

                bool freeze_stop_flag_lm = false;

                // check events
                EvtList evt_list;
                evt_list.clear();
                uint64_t cur_ts = Timer::getSystemTimestampUS();
                last_ts = cur_ts - 1000000; // 1s
                g_ec.eventDeduction(last_ts, cur_ts, evt_list);
                
                for (auto &evt : evt_list)
                {
                    std::type_index evt_type = evt->getType();
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
                    // if (TYPE_EQUALS(evt_type, EvEmergencyUSLeft))
                    // {
                    //     emergency_us_left = true;
                    // }
                    // if (TYPE_EQUALS(evt_type, EvEmergencyUSRight))
                    // {
                    //     emergency_us_right = true;
                    // }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSLeftPure))
                    {
                        emergency_us_left_pure = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyUSRightPure))
                    {
                        emergency_us_right_pure = true;
                    }

                    if (TYPE_EQUALS(evt_type, EvEmergencyLMFrontLeft))
                    {
                        emergency_lm_front_left = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMFrontRight))
                    {
                        emergency_lm_front_right = true;
                    }
                    // if (TYPE_EQUALS(evt_type, EvEmergencyLMLeft))
                    // {
                    //     emergency_lm_left = true;
                    // }
                    // if (TYPE_EQUALS(evt_type, EvEmergencyLMRight))
                    // {
                    //     emergency_lm_right = true;
                    // }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMLeftPure))
                    {
                        emergency_lm_left_pure = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvEmergencyLMRightPure))
                    {
                        emergency_lm_right_pure = true;
                    }
                    if (TYPE_EQUALS(evt_type, EvLMFreezeStop))
                    {
                        freeze_stop_flag_lm = true;
                        ROTATE_WARN_LOG(COLOR_L_RED "=== FREEZE === FREEZE === FREEZE === FREEZE === FREEZE === FREEZE ");
                    }
                }

                if (Owner().left_dir)
                {
                    ROTATE_DEBUG_LOG("rotating direction:   left");
                    if (emergency_us_left_pure || emergency_us_front_left || emergency_lm_left_pure || emergency_lm_front_left || freeze_stop_flag_lm)
                    {
                        // 左前方或左侧不为空
                        ROTATE_DEBUG_LOG("front left or left not empty");
                        g_speed_controller.m_rotate_vel = 0.0;
                        if (Owner().m_reason == TaskRotate::ESCAPE_RIGHT && emergency_us_left_pure && emergency_lm_left_pure && !emergency_us_front_left && !emergency_lm_front_left && !freeze_stop_flag_lm) // stuck
                        {
                            // 由于右侧触发的左转，且左前方为空，左侧不为空，标记为stuck，即之后会尝试脱困
                            ROTATE_INFO_LOG("stuck timing...");
                            if (left_dir_stuck_timing == false)
                            {
                                start_ts_for_us_stop_left_dir = Timer::getTimestampUS();
                                left_dir_stuck_timing = true;
                            }
                            else
                            {
                                cur_ts_for_us_stop_left_dir = Timer::getTimestampUS();
                                float lasting_period = float(cur_ts_for_us_stop_left_dir - start_ts_for_us_stop_left_dir) / 1000000;
                                if (lasting_period > lasting_period_stuck)
                                {
                                    Owner().stuck_flag = true;
                                }
                            }
                        }
                        else // standby
                        {
                            ROTATE_INFO_LOG("standby timing...");
                            if (left_dir_standby_timing == false)
                            {
                                start_ts_for_standby_left_dir = Timer::getTimestampUS();
                                left_dir_standby_timing = true;
                            }
                            else
                            {
                                cur_ts_for_standby_left_dir = Timer::getTimestampUS();
                                float lasting_period = float(cur_ts_for_standby_left_dir - start_ts_for_standby_left_dir) / 1000000;
                                if (lasting_period > lasting_period_standby)
                                {
                                    Owner().standby_flag = true;
                                }
                            }
                        }
                    }
                    else
                    {
                        ROTATE_DEBUG_LOG("g_speed_controller.m_rotate_vel = %f", Owner().m_rotate_vel);
                        g_speed_controller.m_rotate_vel = Owner().m_rotate_vel;
                    }
                }
                else
                {
                    ROTATE_DEBUG_LOG("rotating direction:   right");
                    if (emergency_us_right_pure || emergency_us_front_right || emergency_lm_right_pure || emergency_lm_front_right || freeze_stop_flag_lm)
                    {
                        // 右前方或右侧不为空
                        ROTATE_DEBUG_LOG("front right or right not empty");
                        g_speed_controller.m_rotate_vel = 0.0;
                        if (Owner().m_reason == TaskRotate::ESCAPE_LEFT && emergency_us_right_pure && emergency_lm_right_pure && !emergency_us_front_right && !emergency_lm_front_right && !freeze_stop_flag_lm) // stuck
                        {
                            // 由于左侧触发的左转，且右前方为空，右侧不为空，标记为stuck，即之后会尝试脱困
                            ROTATE_INFO_LOG("stuck timing...");
                            if (right_dir_stuck_timing == false)
                            {
                                start_ts_for_us_stop_right_dir = Timer::getTimestampUS();
                                right_dir_stuck_timing = true;
                            }
                            else
                            {
                                cur_ts_for_us_stop_right_dir = Timer::getTimestampUS();
                                float lasting_period = float(cur_ts_for_us_stop_right_dir - start_ts_for_us_stop_right_dir) / 1000000;
                                if (lasting_period > lasting_period_stuck)
                                {
                                    Owner().stuck_flag = true;
                                }
                            }
                        }
                        else // standby
                        {
                            ROTATE_INFO_LOG("standby timing...");
                            if (right_dir_standby_timing == false)
                            {
                                start_ts_for_standby_right_dir = Timer::getTimestampUS();
                                right_dir_standby_timing = true;
                            }
                            else
                            {
                                cur_ts_for_standby_right_dir = Timer::getTimestampUS();
                                float lasting_period = float(cur_ts_for_standby_right_dir - start_ts_for_standby_right_dir) / 1000000;
                                if (lasting_period > lasting_period_standby)
                                {
                                    Owner().standby_flag = true;
                                }
                            }
                        }
                    }
                    else
                    {
                        ROTATE_DEBUG_LOG("g_speed_controller.m_rotate_vel = Owner().m_rotate_vel;");
                        g_speed_controller.m_rotate_vel = Owner().m_rotate_vel;
                    }
                }

                // control
                ROTATE_DEBUG_LOG("Send Rotate Controll");
                Owner().m_remained_angle = g_speed_controller.SetRotate(Owner().left_dir, Owner().m_target_angle);
                ROTATE_INFO_LOG("remained_angle(deg):  %f", rad2deg(Owner().m_remained_angle));
                if (Owner().m_remained_angle < deg2rad(min_deg)) // default 5 deg
                {
                    Owner().rotation_finished = true;
                }
            }
        };

        struct Rotation_Stuck : BaseState
        {
            virtual void OnEnter()
            {
                // ROTATE_DEBUG_LOG("Enter Rotation_Stuck...");
                CREATE_EVENT(EvRotationStuck, ev_rotation_stuck);
                g_ec.pushEvent(ev_rotation_stuck);
                ROTATE_INFO_LOG("Rotation Stuck Event pushed +++++++++++++++");

                Owner().stuck_flag = false;
            }

            virtual void OnExit()
            {
                // ROTATE_DEBUG_LOG("Exit Rotation_Stuck...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_type == TaskRotate::TARGET_ANGLE)
                {
                    return SiblingTransition<Rotate_target_angle>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                // ROTATE_DEBUG_LOG("Enable...");
            }
        };

        struct Rotation_Standby : BaseState
        {
            virtual void OnEnter()
            {
                // ROTATE_DEBUG_LOG("Enter Rotation_Standby...");
                CREATE_EVENT(EvRotationStandby, ev_rotation_standby);
                g_ec.pushEvent(ev_rotation_standby);
                ROTATE_INFO_LOG("Rotation Standby Event pushed +++++++++++++++");

                Owner().standby_flag = false;
            }

            virtual void OnExit()
            {
                // ROTATE_DEBUG_LOG("Exit Rotation_Standby...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_type == TaskRotate::TARGET_ANGLE)
                {
                    return SiblingTransition<Rotate_target_angle>();
                }

                return NoTransition();
            }

            virtual void Update()
            {
                // ROTATE_DEBUG_LOG("Enable...");
            }
        };

        struct Finished : BaseState
        {
            virtual void OnEnter()
            {
                ROTATE_INFO_LOG("Enter Finished, Rotate task finished");
                Owner().finishTask();

                ROTATE_DEBUG_LOG("set stop flag, change to Idle");
                Owner().m_stop_flag = true;

                Owner().rotation_finished = false;
            }

            virtual void OnExit()
            {
                ROTATE_DEBUG_LOG("Exit Finished");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_type == TaskRotate::TARGET_ANGLE)
                {
                    return SiblingTransition<Rotate_target_angle>();
                }

                return NoTransition();
            }
        };
    };
} // namespace planning_controller

DEFINE_CONFIG_TYPE(CONFIG_ROTATE_PLANNER, RotatePlanner);

RotatePlannerPtr &RotatePlanner::getInstance(ThreadPool *pool)
{
    static RotatePlannerPtr instance = RotatePlannerPtr(new RotatePlanner(pool));
    return instance;
}

RotatePlanner::RotatePlanner(ThreadPool *pool)
    : LocalPlanner(pool)
{
    m_log_name = LOG_ROTATE_PLANNER;
    m_type = TaskRotate::MAX_ROTATE_TYPE;
    m_dir = TaskRotate::MAX_ROTATE_DIRECTION;
    m_reason = TaskRotate::MAX_ROTATE_REASON;
    m_rotate_vel = 0.0f;
    m_target_angle = 0.0f;
    // m_delta_angle = 0.0f;
    m_remained_angle = 0.0f;
    left_dir = true;
    rotation_finished = false;
    stuck_flag = false;
    standby_flag = false;
}

float RotatePlanner::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigRotatePlanner *cfg_rp = dynamic_cast<ConfigRotatePlanner *>(
        cfg_mgr.GetSubConfig(CONFIG_ROTATE_PLANNER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));

    cfg_rp->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_ROTATE_PLANNER_FLAG, LOG_ROTATE_PLANNER,
               cfg_rp->log_name, cfg_rp->log_path,
               cfg_rp->log_extension, cfg_rp->log_ts_mask,
               cfg_rp->log_print_to_console,
               (cfg_rp->log_max_file_size_mb)MB + (cfg_rp->log_max_file_size_kb)KB,
               cfg_rp->log_max_file_cnt, cfg_rp->log_level);

    return cfg_rp->planning_frequency;
}

void RotatePlanner::initRunSM()
{
    m_run_sm.Initialize<RotatePlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("RotatePlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void RotatePlanner::reset()
{
    m_type = TaskRotate::MAX_ROTATE_TYPE;
    m_dir = TaskRotate::MAX_ROTATE_DIRECTION;
    m_reason = TaskRotate::MAX_ROTATE_REASON;
    m_rotate_vel = 0.0f;
    m_target_angle = 0.0f;
    // m_delta_angle = 0.0f;
    m_remained_angle = 0.0f;
    left_dir = true;
    rotation_finished = false;
    stuck_flag = false;
    standby_flag = false;
}

void RotatePlanner::getData()
{
    g_dc.getData<DataSlam>(m_slam_data);
}

bool RotatePlanner::handleTask(const Task &task)
{
    const TaskRotate &rotate_task = dynamic_cast<const TaskRotate &>(task);

    ROTATE_INFO_LOG("receive rotate task, type = %d, dir = %d, "
                    "rotate_vel = %f, target_angle = %f, reason = %d",
                    rotate_task.type, rotate_task.dir,
                    rotate_task.rotate_vel, rad2deg(rotate_task.target_angle), rotate_task.reason);

    if (rotate_task.type >= TaskRotate::MAX_ROTATE_TYPE)
        return false;
    if (rotate_task.dir >= TaskRotate::MAX_ROTATE_DIRECTION)
        return false;

    m_rotate_vel = rotate_task.rotate_vel;
    m_type = rotate_task.type;
    m_dir = rotate_task.dir;
    m_reason = rotate_task.reason;
    if (m_type == TaskRotate::TARGET_ANGLE)
    {
        if (m_dir == TaskRotate::ROTATE_LEFT)
        {
            left_dir = true;
            ROTATE_INFO_LOG("handle task: direction left");
        }
        else if (m_dir == TaskRotate::ROTATE_RIGHT)
        {
            left_dir = false;
            ROTATE_INFO_LOG("handle task: direction right");
        }
        m_target_angle = rotate_task.target_angle;
    }
    // else if (m_type == TaskRotate::DELTA_ANGLE)
    // {
    // if (m_dir == TaskRotate::ROTATE_LEFT)
    // {
    //     left_dir = true;
    // }
    // else if (m_dir == TaskRotate::ROTATE_RIGHT)
    // {
    //     left_dir = false;
    // }
    // m_delta_angle = rotate_task.delta_angle;
    // }
    // else if (m_type == TaskRotate::DELTA_TIME)
    // {
    // }

    return true;
}

void RotatePlanner::finishTask()
{
    ROTATE_DEBUG_LOG("generate rotate task finished evt id = %ld", m_task_id);

    g_speed_controller.SetStopKeepSteering(true);

    CREATE_EVENT(EvTaskFinished, ev_rotate_finished);
    ev_rotate_finished->task_id = m_task_id;
    g_ec.pushEvent(ev_rotate_finished);
}