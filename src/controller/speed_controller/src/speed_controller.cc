#include "speed_controller.h"
#include "speed_controller_log.h"
#include "speed_controller_config.h"

#include "misc/robot_config.h"
#include "misc/planning_common_config.h"
#include "data_center/data_center.h"
#include "data/chassis_data.h"

using namespace std;
using namespace planning_controller;
using namespace planning_utils;
using namespace planning_data;

DEFINE_CONFIG_TYPE(CONFIG_SPEED_CONTROLLER, SpeedController);
SpeedControl g_speed_controller;

static DataFeedbackSpeed m_feed_back_speed; //速度反馈

SpeedControl::SpeedControl()
    : m_set_chassis_speed_polling_thread(nullptr),
      m_initialized(false)
// : m_set_chassis_speed_polling_thread(&SpeedControl::setChassisSpeedPolling, this)  //: linear_vel[VEL_ARR_LEN]({0}), angular_steer[VEL_ARR_LEN]({0}), m_vel_arr_pointer(0)
{
}

SpeedControl::~SpeedControl()
{
    if (m_set_chassis_speed_polling_thread)
    {
        delete m_set_chassis_speed_polling_thread;
    }
}

void SpeedControl::init(ConfigManager &cfg_mgr,
                        const std::function<void(float, float)> &send_speed_func)
{
    ConfigSpeedController *cfg_sc = dynamic_cast<ConfigSpeedController *>(
        cfg_mgr.GetSubConfig(CONFIG_SPEED_CONTROLLER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    ConfigRobot *cfg_robot = dynamic_cast<ConfigRobot *>(
        cfg_mgr.GetSubConfig(CONFIG_ROBOT));

    m_max_linear_vel = cfg_robot->max_linear_vel; // unit: m/s
    m_min_linear_vel = cfg_robot->min_linear_vel;
    m_max_angular_steer = cfg_sc->max_angular_steer * M_PI / 180; // from degree to rad
    m_min_angular_steer = cfg_sc->min_angular_steer * M_PI / 180;
    m_set_chassis_speed_period = (int)1000 / cfg_sc->sending_speed_frequency; // 1000/50 ms = 20ms
    m_rotate_vel = cfg_sc->rotate_vel;                                        // unit: m/s
    m_steer_bias = cfg_sc->steer_bias;                                        // unit: degree

    m_control_speed.linear_vel = 0.0;
    m_control_speed.angular_steer = 0.0;

    // bias correction
    m_max_angular_steer = m_max_angular_steer - deg2rad(m_steer_bias);
    m_min_angular_steer = m_min_angular_steer - deg2rad(m_steer_bias);

    cfg_sc->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_SPEED_CONTROLLER_FLAG, LOG_SPEED_CONTROLLER,
               cfg_sc->log_name, cfg_sc->log_path,
               cfg_sc->log_extension, cfg_sc->log_ts_mask,
               cfg_sc->log_print_to_console,
               (cfg_sc->log_max_file_size_mb)MB + (cfg_sc->log_max_file_size_kb)KB,
               cfg_sc->log_max_file_cnt, cfg_sc->log_level);

    m_send_speed_func = send_speed_func;

    m_set_chassis_speed_polling_thread = new std::thread(&SpeedControl::setChassisSpeedPolling, this);
    m_set_chassis_speed_polling_thread->detach();
    m_initialized = true;
}

void SpeedControl::SpeedSmoothHandle(float v_curr, float delta_curr, float v, float delta, int smooth_time)
{
    unique_lock<mutex> lock(m_vel_mtx);
    if (v < m_min_linear_vel)
        v = m_min_linear_vel;
    if (v > m_max_linear_vel)
        v = m_max_linear_vel;
    if (delta < m_min_angular_steer)
        delta = m_min_angular_steer;
    if (delta > m_max_angular_steer)
        delta = m_max_angular_steer;
    if (smooth_time < m_set_chassis_speed_period)
        smooth_time = m_set_chassis_speed_period;
    if (smooth_time > (m_set_chassis_speed_period * VEL_ARR_LEN))
        smooth_time = m_set_chassis_speed_period * VEL_ARR_LEN; // 20ms * VEL_ARR_LEN

    int smooth_speed_cnt = smooth_time / m_set_chassis_speed_period;
    if (smooth_speed_cnt <= 0)
        smooth_speed_cnt = 1;
    if (smooth_speed_cnt > VEL_ARR_LEN)
        smooth_speed_cnt = VEL_ARR_LEN;

    float v_diff_gap = (v - v_curr) / smooth_speed_cnt;
    // float delta_diff_gap = (delta - delta_curr) / smooth_speed_cnt;

    for (int i = 0; i < smooth_speed_cnt; i++)
    {
        linear_vel[i] = 1000 * (v_curr + v_diff_gap * (i + 1));
        // angular_steer[i] = -1000 * (delta_curr + delta_diff_gap * (i + 1));
        angular_steer[i] = -1000 * delta;
    }
    for (int i = smooth_speed_cnt; i < VEL_ARR_LEN; i++)
    {
        linear_vel[i] = 1000 * v;
        angular_steer[i] = -1000 * delta;
    }

    m_vel_arr_pointer = 0;

    // exp-form smooth
    // float time_to_stop = 0.1; // unit: s
    // float cnt = ceil(1000 * time_to_stop / m_set_chassis_speed_period);
    // float min_linear_vel_for_stop = 100; // default 0.1m/s

    // float slowdown_factor = pow(linear_vel / min_linear_vel_for_stop, -cnt);

    // if (slowdown_factor <= 1)
    // {
    //     SetChassisControl(0.0, 0.0);
    // }
    // else
    // {
    //     for (int i = 0; i < (int)cnt; i++)
    //     {
    //         linear_vel /= slowdown_factor;
    //         SetChassisControl(linear_vel / 1000, angular_steer / (-1000));
    //     }
    // }
}

void SpeedControl::NoSpeedSmoothHandle(float v, float delta)
{
    unique_lock<mutex> lock(m_vel_mtx);
    if (v < m_min_linear_vel)
        v = m_min_linear_vel;
    if (v > m_max_linear_vel)
        v = m_max_linear_vel;
    if (delta < m_min_angular_steer)
        delta = m_min_angular_steer;
    if (delta > m_max_angular_steer)
        delta = m_max_angular_steer;

    for (int i = 0; i < VEL_ARR_LEN; i++)
    {
        linear_vel[i] = 1000 * v;
        angular_steer[i] = -1000 * delta;
    }

    m_vel_arr_pointer = VEL_ARR_LEN - 1;
}

void SpeedControl::SetChassisControl(float v, float delta, bool smooth, int smooth_time) // unit: m, rad
{
    if (smooth)
    {
        if (fabs(v - m_control_speed.linear_vel) >= 0.01) //如果速度过小，没必要做速度平滑操作, unit: m/s
        {
            SpeedSmoothHandle(m_control_speed.linear_vel, m_control_speed.angular_steer, v, delta, smooth_time);
        }
        else
        {
            NoSpeedSmoothHandle(v, delta);
        }
    }
    else
    {
        NoSpeedSmoothHandle(v, delta);
    }
    SPEED_CONTROLLER_DEBUG_LOG("SetChassisControl:m_control_speed.angular_steer(deg): %f", rad2deg(m_control_speed.angular_steer));
    g_dc.getData<DataFeedbackSpeed>(m_feed_back_speed);
    SPEED_CONTROLLER_DEBUG_LOG("SetChassisControl:m_feed_back_speed.r_encoder(deg):   %f", (float)m_feed_back_speed.r_encoder / 100);
}

void SpeedControl::SetStop(bool smooth, int smooth_time)
{
    SetChassisControl(0.0, 0.0, smooth, smooth_time);
}

void SpeedControl::SetStopKeepSteering(bool smooth, int smooth_time)
{
    g_dc.getData<DataFeedbackSpeed>(m_feed_back_speed);
    SetChassisControl(0.0, m_control_speed.angular_steer, smooth, smooth_time);
}

float SpeedControl::SetRotate(const bool &left_dir, const float &target_yaw_angle)
{
    g_dc.getData<DataSlam>(m_slam_data);
    g_dc.getData<DataFeedbackSpeed>(m_feed_back_speed);
    float remian_angle = 0;
    if (left_dir)
    {
        remain_angle = toStdAngleRangeR(target_yaw_angle - m_slam_data.pose.theta);
        SPEED_CONTROLLER_DEBUG_LOG("SetRotate: left_dir remain_angle: %f, target_yaw_angle: %f, m_slam_data.pose.theta: %f", rad2deg(remain_angle), rad2deg(target_yaw_angle), rad2deg(m_slam_data.pose.theta));
    }
    else
    {
        remain_angle = toStdAngleRangeR(-target_yaw_angle + m_slam_data.pose.theta);
        SPEED_CONTROLLER_DEBUG_LOG("SetRotate: right_dir remain_angle: %f, target_yaw_angle: %f, m_slam_data.pose.theta: %f", rad2deg(remain_angle), rad2deg(target_yaw_angle), rad2deg(m_slam_data.pose.theta));
    }
    float min_deg = 5;
    float steer90_tol_deg = 6;
    if (remain_angle > deg2rad(min_deg)) // default 5 deg
    {
        if (left_dir)
        {
            if (abs(90.0 - (float)m_feed_back_speed.r_encoder / 100.0) >= steer90_tol_deg) // 这里应该是反馈上来的steering，否则无法确保已经转到位
            {
                SPEED_CONTROLLER_DEBUG_LOG("SetRotate: only rotation [left], velocity not given ==========");
                SetChassisControl(0.0, M_PI_2, false);
                g_dc.getData<DataFeedbackSpeed>(m_feed_back_speed);
                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            else
            {
                SetChassisControl(m_rotate_vel, M_PI_2, true); // default 0.1m/s, pi/2 left rotation
                SPEED_CONTROLLER_DEBUG_LOG("SetRotate: rotation [left], velocity given ==========");
            }
        }
        else
        {
            if (abs(-90.0 - (float)m_feed_back_speed.r_encoder / 100.0) >= steer90_tol_deg) // 这里应该是反馈上来的steering，否则无法确保已经转到位
            {
                SPEED_CONTROLLER_DEBUG_LOG("SetRotate: only rotation [right], velocity not given ==========");
                SetChassisControl(0.0, -M_PI_2, false);
                g_dc.getData<DataFeedbackSpeed>(m_feed_back_speed);
                // std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            else
            {
                SetChassisControl(m_rotate_vel, -M_PI_2, true); // default 0.1m/s, pi/2 right rotation
                SPEED_CONTROLLER_DEBUG_LOG("SetRotate: rotation [right], velocity given ==========");
            }
        }
    }
    else
    {
        // SetStop();
        SetStopKeepSteering(true);
    }
    // SPEED_CONTROLLER_INFO_LOG("remain_angle(deg):  %f", deg2rad(remain_angle));
    return remain_angle;
}

int SpeedControl::SendChassisControl(float linear_vel_, float angular_steer_)
{

    if (linear_vel_ == 0 && angular_steer_ == 0)
    {
        move_set_status = MOVE_SET_STOP;
    }
    else if (linear_vel_ != 0 && angular_steer_ != 0)
    {
        move_set_status = MOVE_SET_MOVING;
    }
    else if (linear_vel_ == 0 && angular_steer_ != 0)
    {
        move_set_status = MOVE_SET_STEERING_ONLY;
    }

    // bias correction
    angular_steer_ = angular_steer_ - deg2rad(m_steer_bias) * (-1000);

    m_send_speed_func(linear_vel_, angular_steer_);

    // std::cout << "Enter SpeedControl::SendChassisControl..." << std::endl;

    return 0;
}

void SpeedControl::setChassisSpeedPolling()
{
    while (true)
    {
        if (!m_initialized)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(m_set_chassis_speed_period));
            continue;
        }

        // SPEED_CONTROLLER_DEBUG_LOG("send_control v: %f, delta: %f \n", linear_vel, angular_steer);
        unique_lock<mutex> lock(m_vel_mtx);

        SendChassisControl(linear_vel[m_vel_arr_pointer], angular_steer[m_vel_arr_pointer]);

        m_control_speed.linear_vel = linear_vel[m_vel_arr_pointer] / 1000;
        m_control_speed.angular_steer = angular_steer[m_vel_arr_pointer] / (-1000);
        SPEED_CONTROLLER_INFO_LOG("data:send_control v: %f, delta: %f", m_control_speed.linear_vel, rad2deg(m_control_speed.angular_steer));
        g_dc.updateData<DataControlSpeed>(&m_control_speed);

        if (m_vel_arr_pointer < VEL_ARR_LEN - 1)
        {
            m_vel_arr_pointer++;
        }

        lock.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(m_set_chassis_speed_period));
        // std::cout << "Enter SpeedControl::setChassisSpeedPolling..." << std::endl;
    }
}