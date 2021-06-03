#include "collision_detector.h"
#include "collision_detector_log.h"
#include "collision_detector_config.h"

#include "geometry/geometry_func.h"
#include "misc/planning_common_config.h"
#include "misc/robot_config.h"

using namespace planning_detector;
using namespace planning_utils;

// #define SIMPLE_DET

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
    struct CollisionDetectorStates
    {
        struct BaseState : StateWithOwner<CollisionDetector>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter()
            {
                COLLISION_DET_DEBUG_LOG("Enter Disable...");
            }

            virtual void OnExit()
            {
                COLLISION_DET_DEBUG_LOG("Exit Disable...");
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
                COLLISION_DET_DEBUG_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                COLLISION_DET_DEBUG_LOG("Enter Enable...");
                Owner().dispatchThread();
            }

            virtual void OnExit()
            {
                COLLISION_DET_DEBUG_LOG("Exit Enable...");
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
                    COLLISION_DET_DEBUG_LOG("Enable -> Collision_Init...");
                    return InnerEntryTransition<CollisionDet_Init>();
                }
                else
                {
                    // 恢复context
                    COLLISION_DET_DEBUG_LOG("Collision recover context...");
                    Transition trans = Owner().restoreContext(ctx);
                    ctx = nullptr;
                    return trans;
                }

                return NoTransition();
            }

            virtual void Update()
            {
                COLLISION_DET_DEBUG_LOG("Enable...");
                // Owner().getData();
            }
        };

        struct CollisionDet_Init : BaseState
        {
            virtual void OnEnter()
            {
                COLLISION_DET_DEBUG_LOG("Enter CollisionDet_Init...");
            }

            virtual void OnExit()
            {
                COLLISION_DET_DEBUG_LOG("Exit CollisionDet_Init...");
            }

            virtual Transition GetTransition()
            {
                COLLISION_DET_DEBUG_LOG("CollisionDet_Init -- GetTransition()...");

                return SiblingTransition<CollisionDet_Running>();

                // COLLISION_DET_DEBUG_LOG("CollisionDet_Init -- GetTransition() end...");
                // return NoTransition();
            }

            virtual void Update()
            {
                COLLISION_DET_DEBUG_LOG("CollisionDet_Init -- update()...");
            }
        };

        struct CollisionDet_Running : BaseState
        {

            virtual void OnEnter()
            {
                COLLISION_DET_DEBUG_LOG("Enter CollisionDet_Running...");
            }

            virtual void OnExit()
            {
                COLLISION_DET_DEBUG_LOG("Exit CollisionDet_Running...");
            }

            virtual Transition GetTransition()
            {
                COLLISION_DET_DEBUG_LOG("CollisionDet_Running -- GetTransition()...");

                return NoTransition();
            }

            virtual void Update()
            {
                // COLLISION_DET_DEBUG_LOG(COLOR_L_PURPLE "CollisionDet_Running -- update()...");
                uint64_t update_start_ts = Timer::getTimestampUS();
                int collision_flag = 0;

                // compute max/min coordinates according to local map
                int diff_x_int_max = Owner().m_local_map_size_x - 1 - Owner().m_robot_pose_in_local_map_x_default;
                int diff_x_int_min = 0 - Owner().m_robot_pose_in_local_map_x_default;
                int diff_y_int_max = Owner().m_local_map_size_y - 1 - Owner().m_robot_pose_in_local_map_y_default;
                int diff_y_int_min = 0 - Owner().m_robot_pose_in_local_map_y_default;
                float diff_x_max = diff_x_int_max * Owner().m_local_map_resolution;
                float diff_x_min = diff_x_int_min * Owner().m_local_map_resolution;
                float diff_y_max = diff_y_int_max * Owner().m_local_map_resolution;
                float diff_y_min = diff_y_int_min * Owner().m_local_map_resolution;
                float local_map_x_max = diff_x_max + Owner().m_simple_local_map.x;
                float local_map_x_min = diff_x_min + Owner().m_simple_local_map.x;
                float local_map_y_max = diff_y_max + Owner().m_simple_local_map.y;
                float local_map_y_min = diff_y_min + Owner().m_simple_local_map.y;

                // compute m_current_path_points_in_local_map
                Owner().m_current_path_points_in_local_map.clear();
                if (!Owner().m_current_path_points.empty())
                {
                    size_t max_detect_size = (size_t)Owner().m_max_det_path_points_num < Owner().m_current_path_points.size() ? (size_t)Owner().m_max_det_path_points_num : Owner().m_current_path_points.size();
                    for (size_t i = 0; i < max_detect_size; i++)
                    {
                        if (Owner().m_current_path_points[i].pt.x <= local_map_x_max &&
                            Owner().m_current_path_points[i].pt.x >= local_map_x_min &&
                            Owner().m_current_path_points[i].pt.y <= local_map_y_max &&
                            Owner().m_current_path_points[i].pt.y >= local_map_y_min)
                        {
                            Owner().m_current_path_points_in_local_map.add(Owner().m_current_path_points[i]);
                        }
                        else
                        {
                            // 一旦有路径出了Local Map范围，就截止。
                            break;
                        }
                    }
                }
                else
                {
                    COLLISION_DET_WARN_LOG(COLOR_L_PURPLE "Owner().m_current_path_points.empty() == true");
                }

                // collision detection
                if (!Owner().m_current_path_points_in_local_map.empty())
                {
                    for (int i = 0; i < Owner().m_local_map_size_x; i++)
                    {
                        for (int j = 0; j < Owner().m_local_map_size_y; j++)
                        {
                            if (Owner().m_simple_local_map.grid[i][j] != 0)
                            {
                                for (size_t k = 0; k < Owner().m_current_path_points_in_local_map.size(); k++)
                                {
                                    float g_obs_x = (i - Owner().m_robot_pose_in_local_map_x_default) * Owner().m_local_map_resolution + Owner().m_simple_local_map.x;
                                    float g_obs_y = (j - Owner().m_robot_pose_in_local_map_y_default) * Owner().m_local_map_resolution + Owner().m_simple_local_map.y;
                                    float dist = hypot(g_obs_x - Owner().m_current_path_points_in_local_map[k].pt.x, g_obs_y - Owner().m_current_path_points_in_local_map[k].pt.y);
                                    float diff_yaw = toNPPiAngleRangeR(atan2(g_obs_y - Owner().m_current_path_points_in_local_map[k].pt.y, g_obs_x - Owner().m_current_path_points_in_local_map[k].pt.x) - Owner().m_current_path_points_in_local_map[k].theta); // rad
#ifdef SIMPLE_DET
                                    // simple detection
                                    if (dist <= Owner().m_obstacle_radius)
                                    {
                                        collision_flag = 1;
                                        Owner().record_collision_detected_pic(i, j, k);
                                        break;
                                    }
#else
                                    // rect-shape detection
                                    float proj_longitude = dist * cos(diff_yaw);
                                    float proj_lateral = fabs(dist * sin(diff_yaw));
                                    // for small obstacle
                                    if (Owner().m_simple_local_map.grid[i][j] != 0)
                                    {
                                        if (proj_longitude <= Owner().m_vehicle_longitude_size + Owner().m_inflate_longitude_dist_small && proj_longitude >= 0 && proj_lateral <= Owner().m_vehicle_lateral_size / 2 + Owner().m_inflate_lateral_dist_small)
                                        {
                                            collision_flag = 1;
                                            Owner().record_collision_detected_pic(i, j, k);
                                            break;
                                        }
                                    }
                                    // for large obstacle
                                    // else if (Owner().m_simple_local_map.grid[i][j] == 2)
                                    // {
                                    //     if (proj_longitude <= Owner().m_vehicle_longitude_size + Owner().m_inflate_longitude_dist_large && proj_longitude >= 0 && proj_lateral <= Owner().m_vehicle_lateral_size / 2 + Owner().m_inflate_lateral_dist_large)
                                    //     {
                                    //         collision_flag = 1;
                                    //         Owner().record_collision_detected_pic(i, j, k);
                                    //         break;
                                    //     }
                                    // }
#endif
                                }
                            }

                            if (collision_flag == 1)
                            {
                                break;
                            }
                        }

                        if (collision_flag == 1)
                        {
                            break;
                        }
                    }
                }
                else
                {
                    COLLISION_DET_WARN_LOG(COLOR_L_PURPLE "Owner().m_current_path_points_in_local_map.empty() == true");
                }

                // create events
                if (collision_flag == 1)
                {
                    CREATE_EVENT(EvCollisionTrue, ev_collision_true);
                    g_ec.pushEvent(ev_collision_true);
                    COLLISION_DET_WARN_LOG(COLOR_L_PURPLE "========== EvCollisionTrue pushed ==========");
                }

                // show pic
                Owner().record_collision_pic();

                // computation period
                uint64_t update_end_ts = Timer::getTimestampUS();
                float update_period = float(update_end_ts - update_start_ts) / 1000000;
                if (update_period > 0.05)
                {
                    COLLISION_DET_WARN_LOG(COLOR_L_PURPLE "update_period:  %f/0.05", update_period);
                }
            }
        };
    };
} // namespace planning_detector

DEFINE_CONFIG_TYPE(CONFIG_COLLISION_DETECTOR, CollisionDetector);

CollisionDetector &g_collision_detector = CollisionDetector::getInstance(); //定义Collision检测器

CollisionDetector &CollisionDetector::getInstance()
{
    static CollisionDetector instance;
    return instance;
}

CollisionDetector::CollisionDetector()
    : Detector()
{
}

float CollisionDetector::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigCollisionDetector *cfg_cd = dynamic_cast<ConfigCollisionDetector *>(
        cfg_mgr.GetSubConfig(CONFIG_COLLISION_DETECTOR));
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

    m_obstacle_radius = cfg_cd->obstacle_radius;
    m_inflate_longitude_dist_small = cfg_cd->inflate_longitude_dist_small;
    m_inflate_lateral_dist_small = cfg_cd->inflate_lateral_dist_small;
    m_inflate_longitude_dist_large = cfg_cd->inflate_longitude_dist_large;
    m_inflate_lateral_dist_large = cfg_cd->inflate_lateral_dist_large;
    m_max_det_path_points_num_default = cfg_cd->max_det_path_points_num_default;
    m_predict_horizon = cfg_cd->predict_horizon;
    m_max_det_path_points_num = m_max_det_path_points_num_default;

    cfg_cd->log_path = cfg_planning->log_path;
    CREATE_LOG(PlainText, LOG_COLLISION_DETECTOR_FLAG, LOG_COLLISION_DETECTOR,
               cfg_cd->log_name, cfg_cd->log_path,
               cfg_cd->log_extension, cfg_cd->log_ts_mask,
               cfg_cd->log_print_to_console,
               (cfg_cd->log_max_file_size_mb)MB + (cfg_cd->log_max_file_size_kb)KB,
               cfg_cd->log_max_file_cnt, cfg_cd->log_level);

    m_log_name = LOG_COLLISION_DETECTOR;

    COLLISION_DET_DEBUG_LOG("collision detector config...\n");
    return cfg_cd->polling_frequency;
}

void CollisionDetector::initRunSM()
{
    m_run_sm.Initialize<CollisionDetectorStates::Disable>(this);
    m_run_sm.SetDebugInfo("CollisionDetector", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void CollisionDetector::reset()
{
}

void CollisionDetector::getData()
{
    // g_dc.getData<DataSlam>(m_slam_data);
    // g_dc.getData<DataUltrasonic>(m_ultrasonic_data);
    // g_dc.getData<DataControlSpeed>(m_control_speed);
    g_dc.getData<DataLocalMap>(m_simple_local_map);
}

bool CollisionDetector::handleParam(DetectorParamPtr param)
{
    CollisionDetectorParamPtr collision_det_param = std::dynamic_pointer_cast<CollisionDetectorParam>(param);

    COLLISION_DET_INFO_LOG("collision det start, m_max_det_path_points_num:  %d", m_max_det_path_points_num);

    m_current_path_points = collision_det_param->current_path_points;
    m_max_det_path_points_num = collision_det_param->vehicle_velocity * m_predict_horizon / collision_det_param->path_points_resolution;

    return true;
}

void CollisionDetector::finishDetect()
{
    // COLLISION_DET_DEBUG_LOG("generate collision detector finished evt id = %ld", m_input_id);
    // CREATE_EVENT(EvPlannerFinished, ev_collision_det_finished);
    // ev_collision_det_finished->id = m_input_id;
    // g_ec.pushEvent(ev_collision_det_finished);
}

ContextPtr CollisionDetector::saveContext()
{
    COLLISION_DET_DEBUG_LOG("save CollisionDetector context...");
    CREATE_DETECTOR_CONTEXT(ContextCollisionDetector, cd_ctx);
    //TODO: 需要保存的变量、信息内容

    return cd_ctx;
}

Transition CollisionDetector::restoreContext(ContextPtr ctx)
{
    COLLISION_DET_DEBUG_LOG("restore CollisionDetector context...");

    return NoTransition();
}

void CollisionDetector::record_collision_pic()
{
    cv::Mat collision_im(m_local_map_size_x, m_local_map_size_y, CV_8UC3);
    collision_im = collision_im * 0;
    cv::Vec3b colorObs(0x00, 0x00, 0xFF);
    cv::Vec3b colorPath(0xFF, 0x00, 0x00);
    cv::Vec3b colorBot(0x00, 0xFF, 0x00);

    // show obstacle
    for (int i = 0; i < m_local_map_size_x; i++)
    {
        for (int j = 0; j < m_local_map_size_y; j++)
        {
            if (m_simple_local_map.grid[i][j] != 0)
            {
                collision_im.at<cv::Vec3b>(i, j) = colorObs;
            }
        }
    }

    // show path
    for (size_t i = 0; i < m_current_path_points_in_local_map.size(); i++)
    {
        int path_point_grid_x = (int)round((m_current_path_points_in_local_map[i].pt.x - m_simple_local_map.x) / m_local_map_resolution) + m_robot_pose_in_local_map_x_default;
        int path_point_grid_y = (int)round((m_current_path_points_in_local_map[i].pt.y - m_simple_local_map.y) / m_local_map_resolution) + m_robot_pose_in_local_map_y_default;
        collision_im.at<cv::Vec3b>(path_point_grid_x, path_point_grid_y) = colorPath;
    }

    collision_im.at<cv::Vec3b>(m_robot_pose_in_local_map_x_default, m_robot_pose_in_local_map_y_default) = colorBot;
    std::string filename = "/tmp/collision_detector/" + std::to_string(m_simple_local_map.ts) + ".jpg";
    cv::imwrite(filename, collision_im);
}

void CollisionDetector::record_collision_detected_pic(const int &i_, const int &j_, const size_t &k_)
{
    cv::Mat collision_detected_im(m_local_map_size_x, m_local_map_size_y, CV_8UC3);
    collision_detected_im = collision_detected_im * 0;
    cv::Vec3b colorObs(0x00, 0x00, 0x7F);
    cv::Vec3b colorPath(0x7F, 0x00, 0x00);
    cv::Vec3b colorObs_detected(0x00, 0x00, 0xFF);
    cv::Vec3b colorPath_collision(0xFF, 0x00, 0x00);
    cv::Vec3b colorBot(0x00, 0xFF, 0x00);

    // show obstacle
    for (int i = 0; i < m_local_map_size_x; i++)
    {
        for (int j = 0; j < m_local_map_size_y; j++)
        {
            if (m_simple_local_map.grid[i][j] != 0)
            {
                collision_detected_im.at<cv::Vec3b>(i, j) = colorObs;
            }
        }
    }

    // show path
    for (size_t i = 0; i < m_current_path_points_in_local_map.size(); i++)
    {
        int path_point_grid_x = (int)round((m_current_path_points_in_local_map[i].pt.x - m_simple_local_map.x) / m_local_map_resolution) + m_robot_pose_in_local_map_x_default;
        int path_point_grid_y = (int)round((m_current_path_points_in_local_map[i].pt.y - m_simple_local_map.y) / m_local_map_resolution) + m_robot_pose_in_local_map_y_default;
        if (i == k_)
        {
            collision_detected_im.at<cv::Vec3b>(path_point_grid_x, path_point_grid_y) = colorPath_collision;
        }
        else
        {
            collision_detected_im.at<cv::Vec3b>(path_point_grid_x, path_point_grid_y) = colorPath;
        }
    }

    // show detected obstacle
    collision_detected_im.at<cv::Vec3b>(i_, j_) = colorObs_detected;

    collision_detected_im.at<cv::Vec3b>(m_robot_pose_in_local_map_x_default, m_robot_pose_in_local_map_y_default) = colorBot;
    std::string filename = "/tmp/collision_detector_detected/" + std::to_string(m_simple_local_map.ts) + ".jpg";
    cv::imwrite(filename, collision_detected_im);
}
