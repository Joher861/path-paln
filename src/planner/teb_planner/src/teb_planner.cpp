#include <teb_planner.h>


#include <boost/algorithm/string.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include <teb_local_planner/teb_log.h>

#include <costmap_2d/footprint.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <boost/make_shared.hpp>

#include <teb_local_planner/teb_log.h>

#include "local_planner/local_planner_events.h"
#include "path_follow_planner/path_follow_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "timer/sleep_timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "speed_controller/speed_controller.h"
#include "misc/planning_common_config.h"

#include "emergency_stop_detector/emergency_stop_detector.h"
#include "obstacle_detector/obstacle_detector.h"

using namespace planning_planner;
using namespace planning_utils;
using namespace planning_map;
using namespace std;

//定义全局的teb规划器    
planning_planner::GTebPlanner g_teb_planner;
namespace planning_planner
{
  

GTebPlanner::GTebPlanner() : tf_(NULL), costmap_model_(NULL),
                           custom_via_points_active_(false), goal_reached_(false),
                           no_infeasible_plans_(0),
                           last_preferred_rotdir_(RotType::none), initialized_(false)
{
}

void GTebPlanner::setThreadPool(ThreadPool *pool)
{
  pool_ptr_ = pool;
}

GTebPlanner::~GTebPlanner()
{
  if(turnPID_)
  {
    delete turnPID_;
    turnPID_ = nullptr;
  }

}

void GTebPlanner::reconfigureCB(TebConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  current_max_vel_x_ = cfg_.robot.max_vel_x;    
}

float GTebPlanner::loadConfig(ConfigManager &cfg_mgr)
{
  ConfigTebPlanner *cfg_tebp = dynamic_cast<ConfigTebPlanner *>(cfg_mgr.GetSubConfig(CONFIG_TEB_PLANNER));

  ConfigTeblog *cfg_teblog = dynamic_cast<ConfigTeblog *>(cfg_mgr.GetSubConfig(CONFIG_TEBLOG));

  ConfigTebTurnPid *cfg_tebturnpid = dynamic_cast<ConfigTebTurnPid *>(cfg_mgr.GetSubConfig(CONFIG_TEBTURNPID));

  ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(cfg_mgr.GetSubConfig(CONFIG_PLANNING));

  m_ev_deduction_period_for_us = cfg_tebp->ev_deduction_period_for_us;

  if(cfg_tebturnpid)
  {
    turnPID_ = new SimPID(cfg_tebturnpid->turn_p,cfg_tebturnpid->turn_i,
        cfg_tebturnpid->turn_d,cfg_tebturnpid->turn_scale,cfg_tebturnpid->turn_min_error);

    turnPID_->setMaxOutput(cfg_tebturnpid->turn_max_output);
    turnPID_->setMinOutput(cfg_tebturnpid->turn_min_output);
    turnPID_->setErrorIncrement(cfg_tebturnpid->turn_error_inc);
    turnPID_->setContinuousAngle(cfg_tebturnpid->turn_continuous_angle);
    turnPID_->setDesiredValue(0);
    turnPID_->reset();
    // printf("cfg_tebturnpid->turn_p=%f\r\n",cfg_tebturnpid->turn_p);
  }
  else
  {
		turnPID_ = new SimPID(60,2,0,0,5);
				
		turnPID_->setMaxOutput(50);
		turnPID_->setMinOutput(5);
		turnPID_->setErrorIncrement(10);//设置为10相当于没有用这个增量限制功能
		turnPID_->setContinuousAngle(1);
    turnPID_->setDesiredValue(0);
    turnPID_->reset();

    printf("default turn PID configuration.");
  }
  
  cfg_teblog->log_path = cfg_planning->log_path;
	//配置log参数
	CREATE_LOG(PlainText, LOG_TEB_PLANNER_FLAG, LOG_TEB_PLANNER,
		cfg_teblog->log_name, cfg_teblog->log_path, cfg_teblog->log_extension,
		 cfg_teblog->log_ts_mask, cfg_teblog->log_print_to_console,
		(cfg_teblog->log_max_file_size_mb) MB + (cfg_teblog->log_max_file_size_kb) KB,
		 cfg_teblog->log_max_file_cnt,cfg_teblog->log_level);

  m_log_name = LOG_TEB_PLANNER;
  
  cfg_.init(cfg_mgr);

  controller_frequency_ = cfg_teblog->planning_frequency;
  initialize();//根据参数初始化teb
  
  return cfg_teblog->planning_frequency;
}



void GTebPlanner::initialize()
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	

    current_max_vel_x_ = cfg_.robot.max_vel_x;    
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);

    if(cfg_.hcp.use_astar)
    {
      a_star_.reset(new AStar);
      a_star_->initGridMap(Eigen::Vector2i(75, 75));
    }
    // create visualization instance
    visualization_ = TebVisualizationPtr(new TebVisualization(cfg_)); 
    // TODO:string 参数配置不对，字符串显示不出，里面没有内容
    // create robot footprint/contour model for optimization
    costmap_2d::makeFootprintFromString(cfg_.footprint.foot_str, footprint_spec_);

    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    
    printf("footprint:%s footprint_spec size = %d\r\n",cfg_.footprint.foot_str.c_str(),footprint_spec_.size());
    printf("footprint_spec{(%f,%f),(%f,%f),(%f,%f),(%f,%f)} robot_inscribed_radius_=%f robot_circumscribed_radius=%f\r\n",footprint_spec_[0].x,footprint_spec_[0].y,
      footprint_spec_[1].x,footprint_spec_[1].y,footprint_spec_[2].x,footprint_spec_[2].y,
      footprint_spec_[3].x,footprint_spec_[3].y,robot_inscribed_radius_,robot_circumscribed_radius);

    RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer();//剔除了其他类型的footprintModel
    
    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_,pool_ptr_));
      // TEB_INFO_LOG("Parallel planning in distinctive topologies enabled.");
      printf("Parallel planning in distinctive topologies enabled.\r\n");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
      // TEB_INFO_LOG("Parallel planning in distinctive topologies disabled.");
      printf("Parallel planning in distinctive topologies disabled.\r\n");
    }
    printf("enable_homotopy_class_planning=%d\r\n",cfg_.hcp.enable_homotopy_class_planning);
    // init other variables
    // tf_ = tf;
    // costmap_ros_ = costmap_ros;
    // costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.  //TODO:替换costmap接口
    
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(teb_local_map_);

    // global_frame_ = costmap_ros_->getGlobalFrameID();
    global_frame_ = std::string("/map");
    cfg_.map_frame = global_frame_; // TODO
    // robot_base_frame_ = costmap_ros_->getBaseFrameID();
    robot_base_frame_ = std::string("base_link");
   
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    // std::string footstr("[[0.17, 0.17], [-0.17, 0.17], [-0.17,-0.17],[0.17,-0.17]]");//TODO:人为设置只为了测试

    // footprint_spec_ = costmap_ros_->getRobotFootprint();
    //用获取的footprint来计算robot_inscribed_radius_->轮廓内圈半径和robot_circumscribed_radius->轮廓外圈半径
    
    // setup dynamic reconfigure  //TODO:动态参数配置后面自己重新实现
    // dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    // dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&GTebPlanner::reconfigureCB, this, _1, _2);
    // dynamic_recfg_->setCallback(cb);
    
    // validate optimization footprint and costmap footprint
    validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    // custom_obst_sub_ = nh.subscribe("obstacles", 1, &GTebPlanner::customObstacleCB, this);

    // setup callback for custom via-points
    // via_points_sub_ = nh.subscribe("via_points", 1, &GTebPlanner::customViaPointsCB, this);//自定义路标点，这些点是机器人会尽量通过的点
    
    // initialize failure detector
    // nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency_));
    

    currentWheelTurnSet = 0;
    rotate_to_global_plan_ = true;

    pool_ptr_->enqueue([&]{
      SleepTimer t(10);
      double dist;
      try_astar_path_ = false;
      uint8_t collide_count = 0;
      uint8_t retry_rotate_count = 2;

      while(true)
      {
        if(!start_path_collide_check_ || !initialized_ ) //|| !is_global_path_valid_
        {
          t.sleep();
          continue;
        }
        
        // //检测参考路径的事件
        // for (auto evt : m_evt_list)
        // {
        //   TEB_DEBUG_LOG("got EvPFPtebPlanningReq event00.");
        //   std::type_index type = evt->getType();
        //   TEB_DEBUG_LOG("got EvPFPtebPlanningReq event001.");
          
        //   if (TYPE_EQUALS(type, EvPFPtebPlanningReq))
        //   {
        //       TEB_DEBUG_LOG("got EvPFPtebPlanningReq event.");
        //       EvPFPtebPlanningReqPtr event_ptr = std::dynamic_pointer_cast<EvPFPtebPlanningReq>(evt);
        //       TEB_DEBUG_LOG("got EvPFPtebPlanningReq event1.");
        //       setPlan(event_ptr->path_to_teb_ptr);
        //       TEB_DEBUG_LOG("got EvPFPtebPlanningReq event2.");
        //       global_path_index_ = event_ptr->end_idx;
        //       global_path_first_ = event_ptr->is_first;
        //       planner_->clearPlanner();//每次都要重置一下，冷启动
        //       is_global_path_valid_ = 1;

        //       last_valid_control_ = Timer::getSystemTimestampMS();//Controlling 状态中，记录上次有效控制控制的时间
        //       planning_retries_ = 0;//记录请求全局规划的次数

 
        //   }
        // }

        const TimedElasticBand* time_elasticband = planner_->selectedTeb();
        if(!time_elasticband)
        {
          continue;
        }
        LocalMap::InfeasiableDir infeasiable_side;
        bool   safe = time_elasticband->isTimedElasticBandFeasible(dist, infeasiable_side);//这里是对局部路径的collision check
        if (!safe) {
          if (dist > 0.5) {  //0.5
            TEB_WARN_LOG("current traj %lf m to collision", dist);
            ++collide_count;
          } else { //离障碍物非常近的情况
            if(dist < 0.1)
            {

              DataSlam slam_data;
              int8_t ok = g_dc.getData<DataSlam>(slam_data);
              if (ok != 0)
              {
                continue;
              }

              double dir_angle;
              switch (infeasiable_side)
              {
                case LocalMap::FRONT :
                  //停止
                  break;
                case LocalMap::LEFT :
                  //左边干涉就右转一点
                  dir_angle = normalizeAngle(slam_data.pose.theta - deg2rad(15));
                  break;
                case LocalMap::RIGHT :
                  //右边干涉就左转一点
                  dir_angle = normalizeAngle(slam_data.pose.theta + deg2rad(15));

                  break;
                case LocalMap::BACK :

                  break;
              }
              
              // bool feasible = g_local_map.checkRotateFeasible(slam_data.pose.pt.x, slam_data.pose.pt.x,
              //     slam_data.pose.theta, dir_angle, infeasiable_side);
              // if(feasible)//可以旋转
              // {
              //   {
              //     std::unique_lock<shared_recursive_mutex> rotation_lock(rotation_mtx_);
              //     //collect self-rotate parameters
              //     rotation_info_.self_rotation_enable = true;
              //     // rotation_info_.self_rotation_dir = true;
              //     rotation_info_.target_angle = dir_angle;
              //   }
              // }
            }

            TEB_ERROR_LOG("current traj %lf m to collision, emergency stop!", dist);
          }
        } else {
          TEB_WARN_LOG("current traj is collision free");
          local_path_collide_ = false;
          try_astar_path_ = false;
          collide_count = 0;
        }

        if(collide_count > 3)
        {
          collide_count = 0;
          try_astar_path_ = true;
          local_path_collide_ = true;
        }

        //后退速度的检测
        // {
        //   std::unique_lock<shared_recursive_mutex> velocity_lock(velocity_mtx_);

        //   if(velocity_info_.is_valid && velocity_info_.vx < 0 && is_global_path_end_ == false)
        //   {
        //     int pose_index;
        //     PoseSE2 pose;
        //     velocity_info_.is_valid = false;
        //     pose_index = time_elasticband->sizePoses() / retry_rotate_count;
        //     pose = time_elasticband->Pose(pose_index);

        //     DataSlam slam_data;
        //     if(g_dc.getData<DataSlam>(slam_data) < 0)
        //     {
        //       continue;
        //     }

        //     //取优化后轨迹中的位置，计算出当前位置到轨迹点的朝向
        //     double dir_angle = atan2(pose.y() - slam_data.pose.pt.y, pose.x() - slam_data.pose.pt.x);

        //     TEB_WARN_LOG("backward trajectory. We have to rotate in place. dir_angle=%f",dir_angle);
        //     teb_controller_state_ = BACKWARD_TRAJECTORY;
        //     LocalMap::InfeasiableDir infeasiable_side;
        //     bool feasible = g_local_map.checkRotateFeasible(slam_data.pose.pt.x, slam_data.pose.pt.y, slam_data.pose.theta, dir_angle, infeasiable_side);
        //     if(feasible)//可以旋转
        //     {
        //       {
        //         std::unique_lock<shared_recursive_mutex> rotation_lock(rotation_mtx_);
        //         //collect self-rotate parameters
        //         rotation_info_.self_rotation_enable = true;
        //         rotation_info_.target_angle = dir_angle;
        //       }
        //       retry_rotate_count = 2;
        //     }
        //     else
        //     {
        //       ++retry_rotate_count;
        //       if(retry_rotate_count > 4)
        //       {
        //         TEB_WARN_LOG("Can't rotate in place. ROBOT_STUCK. target rotate angle = %f. Retry = %d",dir_angle);
        //         teb_controller_state_ = ROBOT_STUCK;
        //         retry_rotate_count = 2;
        //       }
        //     }
        //   }
        //   else if(velocity_info_.is_valid && velocity_info_.vx >= 0)
        //   {
        //     retry_rotate_count = 2;
        //   }
        // }

        t.sleep();
      }
    });

    // set initialized flag
    initialized_ = true;

    // TEB_DEBUG_LOG("teb_local_planner plugin initialized.");
    // printf("teb_local_planner plugin initialized.\r\n");
  }
  else
  {
    // TEB_WARN_LOG("teb_local_planner has already been initialized, doing nothing.");
    printf("teb_local_planner has already been initialized, doing nothing.\r\n");
  }
}



bool GTebPlanner::setPlan(const PosePathPtr orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    TEB_ERROR_LOG("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  auto form_line = [](RobotPose start,RobotPose end,float resolution) -> PosePath
  {
    PosePath waypoints;
    RobotPose pt;
    pt.theta = 0;
    //get ready for parameterization
    float U = 0;
    int I = 0;
    int size = (int)(hypot(start.pt.x-end.pt.x,start.pt.y-end.pt.y)/resolution);
    double dir_angle = atan2(end.pt.y - start.pt.y, end.pt.x - start.pt.x);

    //fill in the points of the path  
    while(I < size){
      //fill in the point
      pt.pt.x = start.pt.x + ((end.pt.x - start.pt.x) * U);
      pt.pt.y = start.pt.y + ((end.pt.y - start.pt.y) * U);
      if(I == 0)
      {
        pt.theta = start.theta;
      }
      // else if(I == size - 1)
      // {
      //   pt.theta = end.theta;
      // }
      else
      {
        pt.theta = dir_angle;
      }
    
      //go to next point
      U += 1.f/(size-1);
      I++;

      waypoints.add(pt);
    }
    return waypoints;
  };

  // store the global plan
  global_plan_.clear();
  // PosePath path = form_line(orig_global_plan->front().pt, orig_global_plan->back().pt, 0.08);

  // TEB_INFO_LOG("start{%f,%f} end{%f,%f}",orig_global_plan->front().pt.x,orig_global_plan->front().pt.y,
  //   orig_global_plan->back().pt.x,orig_global_plan->back().pt.y);

  // for (uint32_t i=0;i < path.size();++i)
  // {

  //   geometry_msgs::PoseStamped goal_copy;
  //   goal_copy.header.stamp = Timer::getSystemTimestampMS();
  //   geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(path.at(i).theta);
  //   goal_copy.header.frame_id = "map";
  //   goal_copy.pose.position.x = path.at(i).pt.x;
  //   goal_copy.pose.position.y = path.at(i).pt.y;
  //   goal_copy.pose.position.z = 0.0;
  //   goal_copy.pose.orientation.x = q.x;
  //   goal_copy.pose.orientation.y = q.y;
  //   goal_copy.pose.orientation.z = q.z;
  //   goal_copy.pose.orientation.w = q.w;

  //   global_plan_.push_back(goal_copy);
  //   TEB_INFO_LOG("global plan[%d]=(%f,%f,%f)",i,path.at(i).pt.x,path.at(i).pt.y,path.at(i).theta);
      
  // }

  for (uint32_t i=0;i < orig_global_plan->size();++i)
  {

  //   geometry_msgs::PoseStamped goal_copy;
  //   goal_copy.header.stamp = Timer::getSystemTimestampMS();
  //   geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(orig_global_plan->at(i).theta);
  //   goal_copy.header.frame_id = "map";
  //   goal_copy.pose.position.x = orig_global_plan->at(i).pt.x;
  //   goal_copy.pose.position.y = orig_global_plan->at(i).pt.y;
  //   goal_copy.pose.position.z = 0.0;
  //   goal_copy.pose.orientation.x = q.x;
  //   goal_copy.pose.orientation.y = q.y;
  //   goal_copy.pose.orientation.z = q.z;

  //   goal_copy.pose.orientation.w = q.w;

  //   global_plan_.push_back(goal_copy);
    TEB_INFO_LOG("global plan[%d]=(%f,%f,%f)",i,orig_global_plan->at(i).pt.x,
      orig_global_plan->at(i).pt.y,orig_global_plan->at(i).theta);
  }

  geometry_msgs::PoseStamped goal_copy;
  goal_copy.header.stamp = Timer::getSystemTimestampMS();
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(orig_global_plan->front().theta);
  goal_copy.header.frame_id = "map";
  goal_copy.pose.position.x = orig_global_plan->front().pt.x;
  goal_copy.pose.position.y = orig_global_plan->front().pt.y;
  goal_copy.pose.position.z = 0.0;
  goal_copy.pose.orientation.x = q.x;
  goal_copy.pose.orientation.y = q.y;
  goal_copy.pose.orientation.z = q.z;
  goal_copy.pose.orientation.w = q.w;

  global_plan_.push_back(goal_copy);

  goal_copy.header.stamp = Timer::getSystemTimestampMS();
  q = tf::createQuaternionMsgFromYaw(orig_global_plan->back().theta);
  goal_copy.header.frame_id = "map";
  goal_copy.pose.position.x = orig_global_plan->back().pt.x;
  goal_copy.pose.position.y = orig_global_plan->back().pt.y;
  goal_copy.pose.position.z = 0.0;
  goal_copy.pose.orientation.x = q.x;
  goal_copy.pose.orientation.y = q.y;
  goal_copy.pose.orientation.z = q.z;
  goal_copy.pose.orientation.w = q.w;

  global_plan_.push_back(goal_copy);
  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
            
  // reset goal_reached_ flag
  goal_reached_ = false;
  TEB_INFO_LOG("receive an global plan.size=%d",global_plan_.size());
  
  return true;
}


//旋转到目标角度
bool GTebPlanner::rotateToGoal(
    double ang_diff,
    geometry_msgs::Twist& cmd_vel,
    double sim_period,
    double vel_yaw,
    const TebConfig& cfg) {
  bool ret = true;

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
      
  double v_theta_samp = std::min(cfg.robot.max_vel_theta, std::max(cfg.robot.min_vel_theta, fabs(ang_diff)));//最小旋转速度0.05

  //take the acceleration limits of the robot into account
  double max_acc_vel = fabs(vel_yaw) + cfg.robot.acc_lim_theta * sim_period;
  double min_acc_vel = fabs(vel_yaw) - cfg.robot.acc_lim_theta * sim_period;

  v_theta_samp = std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

  //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
  double max_speed_to_stop = sqrt(2 * cfg.robot.acc_lim_theta * fabs(ang_diff));
  v_theta_samp = std::min(max_speed_to_stop, fabs(v_theta_samp));

  v_theta_samp = std::min(cfg.robot.max_vel_theta, std::max(cfg.robot.min_vel_theta, v_theta_samp));//最小旋转速度0.05

  if (ang_diff < 0) {
    v_theta_samp = - v_theta_samp;
  }

  if (fabs(ang_diff) < cfg.goal_tolerance.yaw_goal_tolerance) {
    v_theta_samp = 0;
    ret = false;
  }
  //TODO:添加判断旋转是否会碰到东西
  //we still want to lay down the footprint of the robot and check if the action is legal
  // bool valid_cmd = obstacle_check(Eigen::Vector3f(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), yaw),
  //     Eigen::Vector3f(robot_vel.getOrigin().getX(), robot_vel.getOrigin().getY(), vel_yaw),
  //     Eigen::Vector3f( 0.0, 0.0, v_theta_samp));

  // if (valid_cmd) {
    TEB_DEBUG_LOG("[rotate]Moving to desired goal orientation, th cmd: %.2f", v_theta_samp);
    cmd_vel.angular.z = v_theta_samp;
    return ret;
  // }
  // TEB_WARN_LOG("Rotation cmd in collision");
  // cmd_vel.angular.z = 0.0;
  // return false;

}




bool GTebPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // check if plugin initialized
  if(!initialized_)
  {
    TEB_ERROR_LOG("teb_local_planner has not been initialized, please call initialize() before using this planner");
    teb_controller_state_ = NOT_INITIALIZED;
    return false;
  }

  static uint8_t retry_rotate_count = 2;
  static uint8_t optimize_count = 0;

  cmd_vel.linear.x = 0;
  cmd_vel.linear.y = 0;
  cmd_vel.angular.z = 0;
  goal_reached_ = false;  

  current_max_vel_x_ = cfg_.robot.max_vel_x;
  //获取机器人当前位置
  robot_pose_ = PoseSE2(m_slam_data.pose.pt.x,m_slam_data.pose.pt.y,m_slam_data.pose.theta);
  
  //发布机器人位置
  // visualization_->publishRobotPose(robot_pose_); 

  // Get robot velocity   目前反馈的速度不正确
  robot_vel_.linear.x = m_speed_data.v;
  robot_vel_.linear.y = 0;
  robot_vel_.angular.z = m_speed_data.w;
  TEB_DEBUG_LOG("get robot pose[%f,%f,%f]",robot_pose_.x(),robot_pose_.y(),robot_pose_.theta());
  TEB_DEBUG_LOG("get robot vel[%f,%f]",robot_vel_.linear.x,robot_vel_.linear.z);

  static int goal_idx = 1;
  static int start_idx;
  // prune global plan to cut off parts of the past (spatially before the robot)
  // pruneGlobalPlan(m_slam_data.pose, global_plan_,&goal_idx, cfg_.trajectory.global_plan_prune_distance);//清除全局路径中已经走过的部分
  TEB_DEBUG_LOG("finish pruneGlobalPlan.max_global_plan_lookahead_dist=%f global_plan size=%d",cfg_.trajectory.max_global_plan_lookahead_dist,global_plan_.size());
  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  // tf::StampedTransform tf_plan_to_global;
  // if (!transformGlobalPlan(global_plan_, m_slam_data.pose, *costmap_, cfg_.trajectory.max_global_plan_lookahead_dist, 
  //                          transformed_plan, &goal_idx, &start_idx))//提取局部地图内的全局规划。提取后的计划所在坐标系仍然为global_frame.
  // {
  //   TEB_WARN_LOG("Could not transform the global plan to the frame of the controller");
  //   teb_controller_state_ = INTERNAL_ERROR;
  //   return false;
  // }

  transformed_plan.clear();
  transformed_plan.push_back(global_plan_.front());
  transformed_plan.push_back(global_plan_.back());

  TEB_DEBUG_LOG("finish transformGlobalPlan.max_global_plan_lookahead_dist=%f goal_idx=%d",cfg_.trajectory.max_global_plan_lookahead_dist,goal_idx);


  // create event for upper planner
  // CREATE_EVENT(EvCurIndexInCurTask_TEB, ev_cur_index_in_cur_task);               
  // ev_cur_index_in_cur_task->cur_index_in_cur_task = start_idx;
  // ev_cur_index_in_cur_task->global_path_index = global_path_index_;
  // // ev_cur_index_in_cur_task->path_points_states = path_points_states;
  // g_ec.pushEvent(ev_cur_index_in_cur_task);
  // TEB_INFO_LOG(COLOR_L_BLUE "CurIndexInCurTask_TEB Event pushed: %u", ev_cur_index_in_cur_task->cur_index_in_cur_task);

  // update via-points container
  // if (!custom_via_points_active_)//优化时可以让优化的路径尽量靠近viapoint   这里判断了是否使用自定义的 via_point
  //   updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);//在截取的路径上每隔global_plan_viapoint_sep米选取一个viapoint。
  

  // TEB_DEBUG_LOG("finish updateViaPointsContainer.goal_idx=%d global_plan_viapoint_sep=%f",goal_idx,cfg_.trajectory.global_plan_viapoint_sep);

  // check if global goal is reached
//   tf::Stamped<tf::Pose> global_goal;  
//   tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
//   global_goal.setData( tf_plan_to_global * global_goal );
//   double dx = global_goal.getOrigin().getX() - robot_pose_.x();
//   double dy = global_goal.getOrigin().getY() - robot_pose_.y();

  //获取距离差值
  double dx = global_plan_.back().pose.position.x - robot_pose_.x();
  double dy = global_plan_.back().pose.position.y - robot_pose_.y();
  //获取目标角度与当前机器人角度之间的差值
//   double delta_orient = g2o::normalize_theta( tf::getYaw(global_goal.getRotation()) - robot_pose_.theta() );

  tf::Quaternion global_orientation;
  tf::quaternionMsgToTF(global_plan_.back().pose.orientation, global_orientation);
  double delta_orient = g2o::normalize_theta( tf::getYaw(global_orientation) - robot_pose_.theta() );

  double goal_distance = fabs(std::sqrt(dx*dx+dy*dy));

  if(goal_distance < cfg_.goal_tolerance.xy_goal_tolerance
    // && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0))
  {
    if(need_theta_ == false || fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance){
      goal_reached_ = true;
      rotate_to_global_plan_ = true;
      currentWheelTurnSet = 0;
      isFinish_ = 1;//标记任务完成
      TEB_DEBUG_LOG("teb reach the goal.distance error[%.3lf] theta error[%.3lf]", \
        goal_distance,fabs(delta_orient));
    }
    else{

      rotateToGoal(
        delta_orient,
        cmd_vel,
        1.0/controller_frequency_,  //velocity simulation period(the unit is second)
        robot_vel_.angular.z,
        cfg_);


      planner_->visualize();
      // visualization_->publishObstacles(obstacles_);
      // visualization_->publishViaPoints(via_points_);
      // visualization_->publishGlobalPlan(global_plan_);


    }
    teb_controller_state_ = SUCCESS;
    return true;
  }
  
  
  // check if we should enter any backup mode and apply settings
  // configureBackupModes(transformed_plan, goal_idx);//缩小局部路径长度和是否进行震荡恢复
  
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    TEB_WARN_LOG("Transformed plan is empty. Cannot determine a local plan.");
    teb_controller_state_ = INVALID_PATH;
    return false;

  }
              
  // Get current goal point (last point of the transformed plan)
  tf::Stamped<tf::Pose> goal_point;
  tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);//是否覆盖掉全局路径点的方位角
  // robot_goal_.x() = goal_point.getOrigin().getX();
  // robot_goal_.y() = goal_point.getOrigin().getY();      
  // if (cfg_.trajectory.global_plan_overwrite_orientation)
  // {
  //   robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx);//用当前点和后边几个的角度球平均，应该是一种平滑处理，然后作为局部路径目标点的方向
  //   // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
  //   transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());//注意替换的是局部目标点的方位角
  //   TEB_DEBUG_LOG("robot_goal_(%f,%f,%f).",robot_goal_.x(),robot_goal_.y(),robot_goal_.theta());
  // }  
  // else
  // {
  //   robot_goal_.theta() = tf::getYaw(goal_point.getRotation());
  // }


  //robot_pose_ 为当前机器人位置   goal_point 为局部目标位置
  double goal_angle = atan2(goal_point.getOrigin().getY() - robot_pose_.y(), goal_point.getOrigin().getX() - robot_pose_.x());
  // double delta_rotate_orient = normalize_theta( goal_angle - robot_pose_.theta()); //g2o::normalize_theta报错用normalize_theta替换


  // if( rotate_to_global_plan_) {

  //     // rotate_to_global_plan_ =rotateToGoal(
  //     //   delta_rotate_orient,
  //     //   cmd_vel,
  //     //   1.0/controller_frequency_,  //velocity simulation period(the unit is second)
  //     //   robot_vel_.angular.z,
  //     //   cfg_);
  //     // TEB_DEBUG_LOG("before navigating, rotate to goal's direction.delta_angle=%f goal_angle=%f",
  //     //   delta_rotate_orient,goal_angle);
  //     // last_cmd_ = cmd_vel;

  //     //collect self-rotate parameters
  //     rotation_info_.self_rotation_enable = true;
  //     rotation_info_.target_angle = goal_angle;


  //     planner_->visualize();
  //     visualization_->publishObstacles(obstacles_);
  //     // visualization_->publishViaPoints(via_points_);
  //     // visualization_->publishGlobalPlan(global_plan_);
  //     return true;
  // }

  //replace the goal angle
  if(goal_idx >= global_plan_.size() - 1){//it means we reach the global goal point
    //save the global point angle
    // gobal_angle_backup_ = tf::getYaw(global_goal.getRotation());
    gobal_angle_backup_ = tf::getYaw(transformed_plan.back().pose.orientation);         
    transformed_plan.back().pose.orientation =  tf::createQuaternionMsgFromYaw(goal_angle);//replace the final goal theta
    TEB_DEBUG_LOG("goal_idx >= global_plan_.size() - 1");
  }


  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = robot_pose_.x();
    pose.pose.position.y = robot_pose_.y();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose_.theta());
    transformed_plan.insert(transformed_plan.begin(), pose); // insert start (not yet initialized)
    TEB_DEBUG_LOG("transformed_plan.size()==1");
  }
  
  // tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); 
  // update start   注释则使用原始路径的起点
  // float attempt_end = last_valid_control_ + cfg_.goal_tolerance.controller_patience * 500.0;//取参数的一半
  // || Timer::getSystemTimestampMS() > attempt_end
  //check if we've tried to find a valid control for longer than our time limit
  // double dis = hypot(robot_pose_.x()-transformed_plan.front().pose.position.x,robot_pose_.y()-transformed_plan.front().pose.position.y);
  // double delta_robot = g2o::normalize_theta( tf::getYaw(transformed_plan.front().pose.orientation) - robot_pose_.theta() );
  // if(abs(delta_robot) > deg2rad(65) )//底盘当前位置与起始点方向差别很大，使用机器人当前位置作为起点
  // {
  //   is_robot_as_start_ = true;
  // }
  
  if(is_robot_as_start_ )
  {
    TEB_DEBUG_LOG("is_robot_as_start_= %d",is_robot_as_start_);
    transformed_plan.front().pose.position.x = robot_pose_.x();
    transformed_plan.front().pose.position.y = robot_pose_.y();
    transformed_plan.front().pose.orientation = tf::createQuaternionMsgFromYaw(robot_pose_.theta());
  }

  // clear currently existing obstacles
  obstacles_.clear();
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  // if (costmap_converter_)
  //   updateObstacleContainerWithCostmapConverter();//将障碍物转换成多边形，然后存入到障碍物列表里面
  // else
    //TODO:需要将local map中的障碍物加入到障碍物容器中。
    updateObstacleContainerWithCostmap();//将costmap中每个障碍物的cell点作为单独的障碍物，这样障碍物的数量会很多，极大影响计算效率
  
  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  // updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  // boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());
  // double decrease_scale = 0.5;
  // if( goal_distance < cfg_.trajectory.max_global_plan_lookahead_dist/2.0)
  // {
  //     double last_max_vel = decrease_scale * 2 * cfg_.robot.max_vel_x * goal_distance/cfg_.trajectory.max_global_plan_lookahead_dist;//按比例减少最大速度的约束
  //     if ( last_max_vel < cfg_.robot.max_vel_x )
  //       current_max_vel_x_ = last_max_vel;

  //     if( current_max_vel_x_ < cfg_.robot.min_vel_x )
  //     {
  //         current_max_vel_x_ = cfg_.robot.min_vel_x;
  //     }
  //     TEB_DEBUG_LOG("goal_distance=%lf current_max_vel_x_=%lf",goal_distance,current_max_vel_x_);
  // }
  //TODL:障碍物距离min_obs_dis_目前一直是0，会导致在近处速度突然加快。
  // else if(min_obs_dis_ < cfg_.trajectory.max_global_plan_lookahead_dist/2.0)
  // {
  //     double last_max_vel = 2* cfg_.robot.max_vel_x * min_obs_dis_/cfg_.trajectory.max_global_plan_lookahead_dist;//按比例减少最大速度的约束
  //     if ( last_max_vel < cfg_.robot.max_vel_x )
  //       current_max_vel_x_ = last_max_vel;

  //     if( current_max_vel_x_ < cfg_.robot.min_vel_x ){
  //         current_max_vel_x_ = cfg_.robot.min_vel_x;
  //     }
  //     TEB_DEBUG_LOG("min_obs_dis_=%lf current_max_vel_x_=%lf",min_obs_dis_,current_max_vel_x_);

  // }

  for(uint16_t i = 0;i < transformed_plan.size();i++)
  {
    TEB_DEBUG_LOG("transformed_plan[%d]=(%f,%f,%f)",  \
      i,                                              \
      transformed_plan[i].pose.position.x,            \
      transformed_plan[i].pose.position.y,            \
      tf::getYaw(transformed_plan[i].pose.orientation));
  }
  
  uint64_t timer1 = Timer::getSystemTimestampMS();

  bool success = false;
  //使用astar搜索一条路径  try_astar_no_feasiable_count == 100
  if(cfg_.hcp.use_astar && try_astar_path_ && (!is_global_path_end_))//连续多少次都得不到合适的轨迹就用astar搜索一下。&& try_astar_no_feasiable_count > 5
  {
    Eigen::Vector2d start_pt;
    Eigen::Vector2d end_pt;
    start_pt[0] = transformed_plan.front().pose.position.x;
    start_pt[1] = transformed_plan.front().pose.position.y;

    end_pt[0] = transformed_plan.back().pose.position.x;
    end_pt[1] = transformed_plan.back().pose.position.y;
    if(a_star_->AstarSearch(0.1, start_pt, end_pt))
    {
      std::vector<Eigen::Vector2d> astar_path = a_star_->getPath();
      // for(uint8_t i = 0;i < astar_path.size();++i)
      // {
      //   TEB_DEBUG_LOG("Astar_path[%d]{%f,%f}",i,astar_path[i].x(),astar_path[i].y());
      // }
      //更新每个点的角度
      // orientation_filter_->processPath(m_slam_data.pose, global_plan_);
      // visualization_->publishLocalPlanAndPoses(astar_path);
      success = planner_->plan(transformed_plan, astar_path , &robot_vel_, cfg_.goal_tolerance.free_goal_vel, local_path_collide_);
      TEB_DEBUG_LOG("AstarSearch successful.");
    }
    else
    {
      TEB_DEBUG_LOG("AstarSearch failed.");
      success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel, local_path_collide_);
    }
  }
  else
  {
    // Now perform the actual planning
    success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel, local_path_collide_);
  }
  uint64_t timer2 = Timer::getSystemTimestampMS();
  TEB_DEBUG_LOG("Get the local path and the whole time is %ld ms.",timer2-timer1);

  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    TEB_WARN_LOG("teb_local_planner was not able to obtain a local plan for the current setting.");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row  backup操作中会用到
    time_last_infeasible_plan_ = Timer::getSystemTimestampMS(); //last_time = Timer::getSystemTimestampMS();
    last_cmd_ = cmd_vel;
    teb_controller_state_ = INVALID_PATH;
    return false;
  }

  
  // Check feasibility (but within the first few states only)

  //局部轨迹是否可行，主要检测路径中点的footprint状态（包括朝向）是否与障碍物重叠，具体检测几个点是可设置的
  bool feasible;
  if(is_global_path_end_)
    feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  else
    feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
  if (!feasible)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;

    //不可行也显示一下
    planner_->visualize();
    // visualization_->publishObstacles(obstacles_);
    // visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);

    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    TEB_WARN_LOG("GTebPlanner: trajectory is not feasible. Resetting planner...");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = Timer::getSystemTimestampMS();
    last_cmd_ = cmd_vel;
    teb_controller_state_ = INVALID_PATH;
    optimize_count = 0;
    return false;
  }
  // planner_->filterTrajectory(20);
  TEB_DEBUG_LOG("after isTrajectoryFeasible.control_look_ahead_poses=%d",cfg_.trajectory.control_look_ahead_poses);
  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.trajectory.control_look_ahead_poses))
  {
    planner_->clearPlanner();
    TEB_WARN_LOG("GTebPlanner: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = Timer::getSystemTimestampMS();
    last_cmd_ = cmd_vel;
    teb_controller_state_ = NO_VALID_CMD;
    return false;
  }

  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
  saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
                   cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);//给速度做限制

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)//是否将速度消息转成car-like的速度执行消息
  {
    cmd_vel.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.angular.z))
    {
      cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
      last_cmd_ = cmd_vel;
      planner_->clearPlanner();
      TEB_WARN_LOG("GTebPlanner: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = Timer::getSystemTimestampMS();
      teb_controller_state_ = NO_VALID_CMD;
      return false;
    }

    {
      std::unique_lock<shared_recursive_mutex> velocity_lock(velocity_mtx_);
      velocity_info_.is_valid = true;
      velocity_info_.vx = cmd_vel.linear.x;
      velocity_info_.vw = cmd_vel.angular.z;
    }

    // if(cmd_vel.linear.x >=0)  //速度是正向的就发出路径
    // {
      static double max_jerk = 0;
      if(global_path_first_)
      {
        ++optimize_count;

        if(optimize_count > 10)//第一次要多执行一些优化再发出去
        {
          const TimedElasticBand* timed_elastic_band = planner_->selectedTeb();
          if(!timed_elastic_band)
            return false;

          //计算jerk
          double jerk = const_cast<TimedElasticBand*>(timed_elastic_band)->getJerk();
          if(jerk > max_jerk)
            max_jerk = jerk;
          TEB_DEBUG_LOG("jerk value = %f",jerk);
          
          // 反馈优化好的路径
          CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
          std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

          uint16_t i = 0;
          for(i = 0;i < timed_elastic_band->sizePoses()-1;++i) 
          {
            Eigen::Vector2d delta_dist = timed_elastic_band->Pose(i+1).position()-timed_elastic_band->Pose(i).position();
            if(delta_dist.norm() > 0.07)
            {
              int n_additional_samples = std::ceil(delta_dist.norm() / 0.07) - 1;
              PoseSE2 intermediate_pose = timed_elastic_band->Pose(i);

              teb_path_ptr->add(RobotPose{intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta()});

              for(int step = 0; step < n_additional_samples; ++step)
              {
                intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                teb_path_ptr->add(RobotPose{intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta()});
              }
            }
            else
            {
              teb_path_ptr->add(RobotPose{timed_elastic_band->Pose(i).x(),
                timed_elastic_band->Pose(i).y(),timed_elastic_band->Pose(i).theta()});
            }
          }      

          teb_path_ptr->add(RobotPose{timed_elastic_band->Pose(i).x(),
            timed_elastic_band->Pose(i).y(),timed_elastic_band->Pose(i).theta()});
          
          if(cfg_.hcp.use_astar && try_astar_path_ && (!is_global_path_end_))
          {
            teb_path_ptr->add(RobotPose{transformed_plan.back().pose.position.x,
              transformed_plan.back().pose.position.y,tf::getYaw(transformed_plan.back().pose.orientation)});
          }

          ev_teb_opt_path->opt_path_ptr = teb_path_ptr;
          ev_teb_opt_path->end_idx = global_path_index_;
          ev_teb_opt_path->is_robot_as_start = is_robot_as_start_;
          
          g_ec.pushEvent(ev_teb_opt_path);
          optimize_count = 0;
          global_path_first_ = 0;
          is_global_path_valid_ = 0;
        }
      }
      else
      {
        const TimedElasticBand* timed_elastic_band = planner_->selectedTeb();
        if(!timed_elastic_band)
          return false;

        //计算jerk
        double jerk = const_cast<TimedElasticBand*>(timed_elastic_band)->getJerk();
        if(jerk > max_jerk)
          max_jerk = jerk;
        TEB_DEBUG_LOG("jerk value = %f",jerk);

        // 反馈优化好的路径
        CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
        std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

        uint16_t i = 0;
        for(i = 0;i < timed_elastic_band->sizePoses()-1;++i) 
        {        

          Eigen::Vector2d delta_dist = timed_elastic_band->Pose(i+1).position()-timed_elastic_band->Pose(i).position();
          if(delta_dist.norm() > 0.07)
          {
            int n_additional_samples = std::ceil(delta_dist.norm() / 0.07) - 1;
            PoseSE2 intermediate_pose = timed_elastic_band->Pose(i);

            teb_path_ptr->add(RobotPose{intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta()});
            // TEB_DEBUG_LOG("OPT_TEB_POINTS{%f,%f,%f}",intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta());
            for(int step = 0; step < n_additional_samples; ++step)
            {
              intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
              teb_path_ptr->add(RobotPose{intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta()});
              // TEB_DEBUG_LOG("OPT_TEB_POINTS{%f,%f,%f}",intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta());

            }
          }
          else
          {
            teb_path_ptr->add(RobotPose{timed_elastic_band->Pose(i).x(),
              timed_elastic_band->Pose(i).y(),timed_elastic_band->Pose(i).theta()});

            // TEB_DEBUG_LOG("OPT_TEB_POINTS{%f,%f,%f}",timed_elastic_band->Pose(i).x(),
            //   timed_elastic_band->Pose(i).y(),timed_elastic_band->Pose(i).theta());

          }
        }      

        teb_path_ptr->add(RobotPose{timed_elastic_band->Pose(i).x(),
          timed_elastic_band->Pose(i).y(),timed_elastic_band->Pose(i).theta()});            
        
        if(cfg_.hcp.use_astar && try_astar_path_ && (!is_global_path_end_))
        {
          teb_path_ptr->add(RobotPose{transformed_plan.back().pose.position.x,
            transformed_plan.back().pose.position.y,tf::getYaw(transformed_plan.back().pose.orientation)});
        }

        // TEB_DEBUG_LOG("OPT_TEB_POINTS{%f,%f,%f}",timed_elastic_band->Pose(i).x(),
        //   timed_elastic_band->Pose(i).y(),timed_elastic_band->Pose(i).theta());


        ev_teb_opt_path->opt_path_ptr = teb_path_ptr;        
        ev_teb_opt_path->end_idx = global_path_index_;
        ev_teb_opt_path->is_robot_as_start = is_robot_as_start_;

        g_ec.pushEvent(ev_teb_opt_path);
        optimize_count = 0;
        is_global_path_valid_ = 0;
      }
    // }

  }
  // ros::Time time2 = Timer::getSystemTimestampMS();
  // TEB_WARN_LOG("Get the local path and the whole time is %f ms.",(time2 - time1).toSec() * 1000.0);
  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;
  
  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel;
  
  // Now visualize everything    
  planner_->visualize();
  // visualization_->publishObstacles(obstacles_);
  // visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  teb_controller_state_ = SUCCESS;
  return true;
}


bool GTebPlanner::isGoalReached()
{
  if (goal_reached_)
  {
    TEB_INFO_LOG("GOAL Reached!");
    rotate_to_global_plan_ = true;
    currentWheelTurnSet = 0;
    planner_->clearPlanner();
    return true;
  }
  return false;
}


//TODO:这里需要替换成localmap的接口
void GTebPlanner::updateObstacleContainerWithCostmap()
{  
  obstacles_.clear();//每次先清除障碍物信息
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();


    // std::unordered_map<double,std::pair<Eigen::Vector2d,double>> angle_obs_container;
    // // g_dc.getData<DataLocalMap>(local_map_data_);
    // // RobotPose origin_pose(local_map_data_.x, local_map_data_.y, local_map_data_.theta);//local_map的机器人位置
    // teb_local_map_.update();
    // // TEB_DEBUG_LOG("cosmap size x=%d y=%d",teb_local_map_.size_x(),teb_local_map_.size_y());
    // for(uint16_t i = 0;i < teb_local_map_.size_x();i++)
    // {
    //   for(uint16_t j = 0;j < teb_local_map_.size_y();j++)
    //   {
    //     uint16_t row = i;
    //     uint16_t col = j;
    //     if(teb_local_map_.local_map_data()->grid[col][row] != 0)//judge if grid is occupied.
    //     {
    //       // RobotGrid grid(col,row);//注意x,y与row，col的对应关系是反过来的。
    //       // RobotPose pose = local_map_gridToPose(grid, origin_pose);
          
    //       float world_x,world_y;
    //       teb_local_map_.map2world(row,col,&world_x,&world_y);
    //       // TEB_DEBUG_LOG("[map2world] grid_x=%d grid_y=%d world_x=%f world_y=%f",row,col,world_x,world_y);
    //       // unsigned int grid_x,grid_y;
    //       // teb_local_map_.world2map(world_x,world_y,&grid_x,&grid_y);
    //       // TEB_DEBUG_LOG("[world2map] grid_x=%d grid_y=%d world_x=%f world_y=%f",grid_x,grid_y,world_x,world_y);


    //       Eigen::Vector2d obs;
    //       obs[0] = world_x; //pose.pt.x;
    //       obs[1] = world_y; //pose.pt.y;


            
    //       // check if obstacle is interesting (e.g. not far behind the robot)
    //       Eigen::Vector2d obs_dir = obs-robot_pose_.position();

          
    //       double direction = obs_dir.dot(robot_orient);
          
    //       if(direction > 0)    //在车体前方
    //         // || (direction < 0 && obs_dis < cfg_.obstacles.costmap_obstacles_behind_robot_dist))//在车体后方一定距离内
    //       {
    //         double obs_dis = obs_dir.norm();
    //         double obs_angle = atan2(obs.coeffRef(1), obs.coeffRef(0));
    //         double delta_angel = normalize_theta( obs_angle - robot_pose_.theta());
    //         double delta_deg = delta_angel * 180.0 / M_PI;//转成degree

    //         //查找对应的角度key
    //         // double key = floor(delta_deg);
            
    //         // while(key <= delta_deg)
    //         // {
    //         //   key += cfg_.obstacles.obstacle_angle_increasment;
    //         // }
    //         // auto it = angle_obs_container.find (key);

    //         // if ( it == angle_obs_container.end() )//没找到
    //         // {
    //         //   angle_obs_container.insert({key,std::make_pair(obs,obs_dis)});//添加新的障碍物
    //         // }
    //         // else//找到了
    //         // {
    //         //   if(it->second.second > obs_dis)
    //         //   {
    //         //     it->second = std::make_pair(obs,obs_dis);//替换掉原来的障碍物
    //         //   }
    //         // }

    //         obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));

    //       }
    //     }

    //   }
    // }

    std::vector<Eigen::Vector2d>& all_obs = g_local_map.getObstacleVec();

    if(all_obs.empty())
      return;

    for(auto obs : all_obs)
    {
      // check if obstacle is interesting (e.g. not far behind the robot)
      Eigen::Vector2d obs_dir = obs-robot_pose_.position();

      double direction = obs_dir.dot(robot_orient);
      double obs_dis = obs_dir.norm();
      
      if((direction > 0) 
        || (direction < 0 && obs_dis < cfg_.obstacles.costmap_obstacles_behind_robot_dist))//在车体后方一定距离内
      {
        // double obs_angle = atan2(obs.coeffRef(1), obs.coeffRef(0));
        // double delta_angel = normalize_theta( obs_angle - robot_pose_.theta());
        // double delta_deg = delta_angel * 180.0 / M_PI;//转成degree

        obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
      }
    }
        


    // std::unordered_map<double,std::pair<Eigen::Vector2d,double>> angle_obs_container;
    
    // for (unsigned int i=0; i<g_local_map.getMapInfo().size.x-1; ++i)//TODO:替换成localmap的接口
    // {
    //   for (unsigned int j=0; j<g_local_map.getMapInfo().size.y-1; ++j)
    //   {
    //     LocalCell cell_value;
    //     g_local_map.get(std::string("local_base"),RobotGrid{i,j},cell_value); //局部地图并没有包含静态地图的障碍物
    //     if(cell_value.status == LocalBaseLayer::OCCUPIED)
    //     {

    //       Eigen::Vector2d obs;
    //       RobotPose pose = g_local_map.gridToPose(RobotGrid{i,j});
    //       obs[0] = pose.pt.x;
    //       obs[1] = pose.pt.y;
            
    //       // check if obstacle is interesting (e.g. not far behind the robot)
    //       Eigen::Vector2d obs_dir = obs-robot_pose_.position();


    //       double obs_dis = obs_dir.norm();
    //       double direction = obs_dir.dot(robot_orient);
          
    //       if(direction > 0    //在车体前方
    //         || (direction < 0 && obs_dis < cfg_.obstacles.costmap_obstacles_behind_robot_dist))//在车体后方一定距离内
    //       {
    //         double obs_angle = atan2(obs.coeffRef(1), obs.coeffRef(0));
    //         double delta_angel = normalize_theta( obs_angle - robot_pose_.theta());
    //         double delta_deg = delta_angel * 180.0 / M_PI;//转成degree

    //         //查找对应的角度key
    //         double key = floor(delta_deg);
            
    //         while(key <= delta_deg)
    //         {
    //           key += cfg_.obstacles.obstacle_angle_increasment;
    //         }
    //         auto it = angle_obs_container.find (key);

    //         if ( it == angle_obs_container.end() )//没找到
    //         {
    //           angle_obs_container.insert({key,std::make_pair(obs,obs_dis)});//添加新的障碍物
    //         }
    //         else//找到了
    //         {
    //           if(it->second.second > obs_dis)
    //           {
    //             it->second = std::make_pair(obs,obs_dis);//替换掉原来的障碍物
    //           }
    //         }
    //       }

    //     }
    //   }
    // }

    // double min_dis = 1e10;
    // Eigen::Vector2d min_obs;
    // for(auto it = angle_obs_container.begin();it != angle_obs_container.end();++it)
    // {
    //   obstacles_.push_back(ObstaclePtr(new PointObstacle(it->second.first)));
    //   if(min_dis > it->second.second
    //     || (it->first > -45 && it->first < 45))//找到角度在前方并且距离最短的障碍物
    //   {
    //     min_dis = it->second.second;
    //     min_obs = it->second.first;
    //   }
    // }
    // min_obs_dis_ = min_dis;

    // TEB_DEBUG_LOG("obs size=%d min_dis=%lf min obs position{%lf,%lf}",
    //   angle_obs_container.size(),min_dis,min_obs[0],min_obs[1]);

  }
}

// void GTebPlanner::updateObstacleContainerWithCostmapConverter()
// {
//   if (!costmap_converter_)
//     return;
    
//   //Get obstacles from costmap converter
//   costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
//   if (!obstacles)
//     return;

//   for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
//   {
//     const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
//     const geometry_msgs::Polygon* polygon = &obstacle->polygon;

//     if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
//     {
//       obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
//     }
//     else if (polygon->points.size()==1) // Point
//     {
//       obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
//     }
//     else if (polygon->points.size()==2) // Line
//     {
//       obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
//                                                         polygon->points[1].x, polygon->points[1].y )));
//     }
//     else if (polygon->points.size()>2) // Real polygon
//     {
//         PolygonObstacle* polyobst = new PolygonObstacle;
//         for (std::size_t j=0; j<polygon->points.size(); ++j)
//         {
//             polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
//         }
//         polyobst->finalizePolygon();
//         obstacles_.push_back(ObstaclePtr(polyobst));
//     }

//     // Set velocity, if obstacle is moving
//     if(!obstacles_.empty())
//       obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
//   }
// }


// void GTebPlanner::updateObstacleContainerWithCustomObstacles()
// {
//   // Add custom obstacles obtained via message
//   boost::mutex::scoped_lock l(custom_obst_mutex_);

//   if (!custom_obstacle_msg_.obstacles.empty())//添加通过消息发送的自定义障碍物
//   {
//     // We only use the global header to specify the obstacle coordinate system instead of individual ones
//     Eigen::Affine3d obstacle_to_map_eig;
//     try 
//     {
//       tf::StampedTransform obstacle_to_map;
//       tf_->waitForTransform(global_frame_, ros::Time(0),
//             custom_obstacle_msg_.header.frame_id, ros::Time(0),
//             custom_obstacle_msg_.header.frame_id, ros::Duration(0.5));
//       tf_->lookupTransform(global_frame_, ros::Time(0),
//           custom_obstacle_msg_.header.frame_id, ros::Time(0), 
//           custom_obstacle_msg_.header.frame_id, obstacle_to_map);
//       tf::transformTFToEigen(obstacle_to_map, obstacle_to_map_eig);
//     }
//     catch (tf::TransformException ex)
//     {
//       TEB_ERROR_LOG("%s",ex.what());
//       obstacle_to_map_eig.setIdentity();
//     }
    
//     for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
//     {
//       if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
//       {
//         Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
//                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
//                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
//         obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
//       }
//       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
//       {
//         Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
//                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
//                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
//         obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
//       }
//       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
//       {
//         Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
//                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
//                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
//         Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
//                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
//                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
//         obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
//                                                            (obstacle_to_map_eig * line_end).head(2) )));
//       }
//       else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
//       {
//         TEB_WARN_LOG("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
//         continue;
//       }
//       else // polygon
//       {
//         PolygonObstacle* polyobst = new PolygonObstacle;
//         for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
//         {
//           Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
//                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
//                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
//           polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
//         }
//         polyobst->finalizePolygon();
//         obstacles_.push_back(ObstaclePtr(polyobst));
//       }

//       // Set velocity, if obstacle is moving
//       if(!obstacles_.empty())
//         obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
//     }
//   }
// }

void GTebPlanner::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;

  std::size_t viapoint_forward_step = cfg_.trajectory.no_viapoint_idx;//从机器前方的点开始加viapoints，避免机器去走离它很近的点
  if(transformed_plan.size() <= viapoint_forward_step)
  {
    viapoint_forward_step = 0;
  }
  LocalMap::InfeasiableDir infeasiable_side;

  std::size_t prev_idx = transformed_plan.size() - 1;
  // for (std::size_t i=1+viapoint_forward_step; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  for (std::size_t i=transformed_plan.size()-viapoint_forward_step; i > viapoint_forward_step; --i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    if(!g_local_map.checkTrajectoryPointFeasible(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y,
        tf::getYaw(transformed_plan[i].pose.orientation), infeasiable_side))
    {
      if(via_points_.size() >= 1)
      {
        via_points_.pop_back();
      }
      break;
    }
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  TEB_DEBUG_LOG("via_points size=%d",via_points_.size());
}
      
Eigen::Vector2d GTebPlanner::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
//清理掉走过的路径
bool GTebPlanner::pruneGlobalPlan(const RobotPose& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, int* current_goal_idx, double dist_behind_robot)
{
  if (global_plan.empty() || *current_goal_idx == int(global_plan.size())-1)
    return true;
    
  double dist_threshold = 0.5 * g_local_map.getMapInfo().size.x;                    

  int nearest_idx = 0;
  double sq_dist_threshold = dist_threshold * dist_threshold;
  double sq_dist = 1e10;

  double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
  
  for(int j=*current_goal_idx; j >= 0; --j)//在global_plan上面找一个离当前机器人位置最近的一个点
  {
    double x_diff = global_pose.pt.x - global_plan[j].pose.position.x;
    double y_diff = global_pose.pt.y - global_plan[j].pose.position.y;
    double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (new_sq_dist > sq_dist_threshold)
      break;  // force stop if we have reached the costmap border

    if (new_sq_dist < sq_dist) // find closest distance
    {
      sq_dist = new_sq_dist;
      nearest_idx = j;
    }
  }
  TEB_DEBUG_LOG("current_goal_idx=%d nearest_idx=%d total_size=%d",*current_goal_idx,nearest_idx,global_plan.size());
  // iterate plan until a pose close the robot is found
  std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin() + nearest_idx;
  std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
  bool cut_flag = false;
  while (it != global_plan.begin())
  {
    double dx = global_pose.pt.x - it->pose.position.x;
    double dy = global_pose.pt.y - it->pose.position.y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq > dist_thresh_sq)
    {
        erase_end = it;
        cut_flag = true;
        break;
    }
    --it;
  }
  if (erase_end == global_plan.end())
    return false;
  
  if (erase_end != global_plan.begin() && cut_flag == true)
    global_plan.erase(global_plan.begin(), erase_end);

  return true;
}
      

bool GTebPlanner::transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const RobotPose& global_pose, const costmap_2d::Costmap2D& costmap, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, int* start_index) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  if (global_plan.empty())
  {
    TEB_ERROR_LOG("Received plan with zero length");
    *current_goal_idx = 0;
    return false;
  }


  //we'll discard points on the plan that are outside the local costmap //TODO:替换costmap的接口
  // double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
  //                                  costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
  // double dist_threshold = 3.21;                    
  // double dist_threshold = 0.5 * g_local_map.getMapInfo().size.x * g_local_map.getMapInfo().resolution;                    
  // double dist_threshold = 0.5 * teb_local_map_.size_x() * teb_local_map_.resolution(); 
  double dist_threshold = 0.5 * g_local_map.getMapInfo().size.x * g_local_map.getMapInfo().resolution;                   
  dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                          // located on the border of the local costmap
  TEB_DEBUG_LOG("local_map dist_threshold = %f global_plan.size() = %d",dist_threshold,global_plan.size());

  int i = 0;
  double sq_dist_threshold = dist_threshold * dist_threshold;
  double sq_dist = 1e10;
  
  //we need to loop to a point on the plan that is within a certain distance of the robot
  for(int j=0; j < (int)global_plan.size(); ++j)//在global_plan上面找一个离当前机器人位置最近的一个点
  {
    double x_diff = global_pose.pt.x - global_plan[j].pose.position.x;
    double y_diff = global_pose.pt.y - global_plan[j].pose.position.y;
    double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
    if (new_sq_dist > sq_dist_threshold || j > *current_goal_idx)//一般是比上次目标点多一点
      break;  // force stop if we have reached the costmap border

    if (new_sq_dist < sq_dist) // find closest distance
    {
      sq_dist = new_sq_dist;
      i = j;
    }
  }
  TEB_DEBUG_LOG("transform plan nearest id = %d sq_dist = %f",i,sq_dist);
  double plan_length = 0; // check cumulative Euclidean distance along the plan
  
  //now we'll transform until points are outside of our distance threshold
  // while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
  while(i < (int)global_plan.size()) //每次都包含所有全局路径点，实际的路径长度由path_follow来控制
  {

    transformed_plan.push_back(global_plan[i]);

    // double x_diff = global_pose.pt.x - global_plan[i].pose.position.x;
    // double y_diff = global_pose.pt.y - global_plan[i].pose.position.y;
    // sq_dist = x_diff * x_diff + y_diff * y_diff;
    
    // // caclulate distance to previous pose
    // if (i>0 && max_plan_length>0)
    // {
    //   // plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);
    //   plan_length += hypot(global_plan[i].pose.position.x - global_plan[i-1].pose.position.x,
    //     global_plan[i].pose.position.y - global_plan[i-1].pose.position.y);

    // }
      

    ++i;
  }
      
  // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
  // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
  if (transformed_plan.empty())
  {

    transformed_plan.push_back(global_plan.back());

    // Return the index of the current goal point (inside the distance threshold)
    if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
  }
  else
  {
    // Return the index of the current goal point (inside the distance threshold)
    if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
  }
    

  return true;
}

    
      
      
double GTebPlanner::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const tf::Stamped<tf::Pose>& local_goal,
                    int current_goal_idx,int moving_average_length) const
{
  int n = (int)global_plan.size();//moving_average_length默认设置为3
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf::getYaw(local_goal.getRotation());
    }
    else
    {
      tf::Quaternion global_orientation;
      tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);//离全局目标点比较近就用全局目标点的角度
    //   return  tf::getYaw(tf_plan_to_global.getRotation() *  global_orientation );
      return  tf::getYaw(global_orientation );
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  tf::Stamped<tf::Pose> tf_pose_k = local_goal;
  tf::Stamped<tf::Pose> tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    const geometry_msgs::PoseStamped& pose = global_plan.at(i+1);
    tf::poseStampedMsgToTF(pose, tf_pose_kp1);
    // tf_pose_kp1.setData(tf_plan_to_global * tf_pose_kp1);
    tf_pose_kp1.setData(tf_pose_kp1);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
              tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX() ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);//返回当前点和后边的几个点角度的平均值，这里相当于做了一个角度平滑
}
      
      
void GTebPlanner::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
{
  static uint8_t flag = 0;
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(vy / max_vel_y);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    if(flag == 0){
      flag = 1;
      TEB_WARN_LOG("GTebPlanner(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
    }
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }
}
     
     
double GTebPlanner::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  TEB_DEBUG_LOG("radius = %f, min_turning_radius = %f",radius,min_turning_radius);
  if (fabs(radius) < min_turning_radius)
    radius = double(sign(radius)) * min_turning_radius;   //g2o::sign报错，用sign替换，sign的代码从G2O里抠出来

  return std::atan(wheelbase / radius);
}
     

void GTebPlanner::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    if(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius){
      TEB_WARN_LOG("The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                    "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                    "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
    }
}
   
   
   
void GTebPlanner::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    uint64_t current_time = Timer::getSystemTimestampMS();
    
    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup &&  //当没有可行的轨迹时候，减少视野（即将局部目标点设近一点）
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_)/1000.0 < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        if(no_infeasible_plans_==1)
          TEB_INFO_LOG("Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            if(no_infeasible_plans_==10)
              TEB_INFO_LOG("Infeasible trajectory detected 10 times in a row: further reducing horizon...");

            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());//缩短了局部路径
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    

    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)//检测是否出现震荡的情况
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? current_max_vel_x_ : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, current_max_vel_x_, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (Timer::getSystemTimestampMS()-time_last_oscillation_)/1000.0 < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                TEB_WARN_LOG("GTebPlanner: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = Timer::getSystemTimestampMS();
            planner_->setPreferredTurningDir(last_preferred_rotdir_);

            // if(recovery_state_ != GTebPlannerInput::FORWARD)
            // {
            //     g_speed_controller.SetStop();
            //     //出现震荡，先尝试让底盘向前走
            //     setInitMoveInfo(m_slam_data.pose.pt.x,m_slam_data.pose.pt.y);
            //     setRecoveryState(GTebPlannerInput::FORWARD);
            //     recovery_tries_++;
            //     TEB_WARN_LOG("Forward recovery from oscillating.recovery_tries = %d.",recovery_tries_);
            // }
            // else if(recovery_state_ == GTebPlannerInput::FORWARD)
            // {
            //     g_speed_controller.SetStop();
            //     if (robot_vel_.angular.z >= 0)
            //         setRotateRecoveryRelativeAngle(90);
            //     else
            //         setRotateRecoveryRelativeAngle(-90);
            //     recovery_tries_++;
            //     setRecoveryState(GTebPlannerInput::ROTATE);
            //     TEB_WARN_LOG("Rotate recovery from oscillating.recovery_tries = %d.",recovery_tries_);
            // }
            // else if(recovery_state_ == GTebPlannerInput::ROTATE)
            // {
            //     g_speed_controller.SetStop();
            //     setClearLocalMap();
            //     recovery_tries_++;
            //     TEB_WARN_LOG("Clear localmap recovery from oscillating.recovery_tries = %d.",recovery_tries_);
            // }
            // else
            // {
            //     g_speed_controller.SetStop();
            //     isError_ = 1;
            //     TEB_WARN_LOG("Can't recovery from oscillating.recovery_tries = %d.",recovery_tries_);
            // }
            

        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            TEB_DEBUG_LOG("GTebPlanner: oscillation recovery disabled/expired.");
        }
    }

}
     
     
// void GTebPlanner::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
// {
//   boost::mutex::scoped_lock l(custom_obst_mutex_);
//   custom_obstacle_msg_ = *obst_msg;  
// }

// void GTebPlanner::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
// {
//   PMS_LOG_INFO_ONCE("Via-points received. This message is printed once.");
//   if (cfg_.trajectory.global_plan_viapoint_sep > 0)
//   {
//     TEB_WARN_LOG("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
//              "Ignoring custom via-points.");
//     custom_via_points_active_ = false;
//     return;
//   }

//   boost::mutex::scoped_lock l(via_point_mutex_);
//   via_points_.clear();
//   for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
//   {
//     via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
//   }
//   custom_via_points_active_ = !via_points_.empty();
// }
     
RobotFootprintModelPtr GTebPlanner::getRobotFootprintFromParamServer()
{
  // std::string model_name; 
  // if (!nh.getParam("footprint_model/type", model_name))
  // {
  //   TEB_INFO_LOG("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
  //   return boost::make_shared<PointRobotFootprint>();
  // }
    
  // // point  
  // if (model_name.compare("point") == 0)
  // {
    // TEB_INFO_LOG("Footprint model 'point' loaded for trajectory optimization.");
    // return boost::make_shared<PointRobotFootprint>();
  // }
  
  // // circular
  // if (model_name.compare("circular") == 0)
  // {
  //   // get radius
  //   double radius;
  //   if (!nh.getParam("footprint_model/radius", radius))
  //   {
  //     PMS_LOG_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
  //                      << "/footprint_model/radius' does not exist. Using point-model instead.");
  //     return boost::make_shared<PointRobotFootprint>();
  //   }
  //   TEB_DEBUG_LOG("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
  //   return boost::make_shared<CircularRobotFootprint>(radius);
  // }
  
  // line
  // if (model_name.compare("line") == 0)
  // {
    // check parameters
    // if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    // {
    //   PMS_LOG_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
    //                    << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
    //   return boost::make_shared<PointRobotFootprint>();
    // }
    // get line coordinates
    // printf("line footprint /r/n");

    //使用该类型会报段错误  -> 指定元素个数就正常了
    std::vector<double> line_start(2), line_end(2);
    line_start[0] = 0;
    line_start[1] = 0;
    line_end[0] = cfg_.robot.wheelbase;
    line_end[1] = 0;
    // nh.getParam("footprint_model/line_start", line_start);
    // nh.getParam("footprint_model/line_end", line_end);
    // if (line_start.size() != 2 || line_end.size() != 2)
    // {
    //   PMS_LOG_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
    //                    << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
    //   return boost::make_shared<PointRobotFootprint>();
    // }
    
    // TEB_DEBUG_LOG("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
    //                  << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
  // }
  
  // // two circles
  // if (model_name.compare("two_circles") == 0)
  // {
  //   // check parameters
  //   if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
  //       || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
  //   {
  //     PMS_LOG_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
  //                      << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
  //     return boost::make_shared<PointRobotFootprint>();
  //   }
  //   double front_offset, front_radius, rear_offset, rear_radius;
  //   nh.getParam("footprint_model/front_offset", front_offset);
  //   nh.getParam("footprint_model/front_radius", front_radius);
  //   nh.getParam("footprint_model/rear_offset", rear_offset);
  //   nh.getParam("footprint_model/rear_radius", rear_radius);
  //   TEB_DEBUG_LOG("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
  //                   << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
  //   return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  // }

  // // polygon
  // if (model_name.compare("polygon") == 0)
  // {

  //   // check parameters
  //   XmlRpc::XmlRpcValue footprint_xmlrpc;
  //   if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
  //   {
  //     PMS_LOG_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
  //                      << "/footprint_model/vertices' does not exist. Using point-model instead.");
  //     return boost::make_shared<PointRobotFootprint>();
  //   }
  //   // get vertices
  //   if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
  //   {
  //     try
  //     {
        // Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        // Point2dContainer polygon;  //用该模型，轨迹穿越障碍物时会打结
        // for(uint8_t i=0;i < footprint_spec_.size();++i)
        // {
        //   Eigen::Vector2d point;
        //   point.x() = footprint_spec_[i].x;
        //   point.y() = footprint_spec_[i].y;
        //   polygon.push_back(point);
        // }
        // TEB_DEBUG_LOG("Footprint model 'polygon' loaded for trajectory optimization.");
        // return boost::make_shared<PolygonRobotFootprint>(polygon);
  //     } 
  //     catch(const std::exception& ex)
  //     {
  //       PMS_LOG_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
  //       return boost::make_shared<PointRobotFootprint>();
  //     }
  //   }
  //   else
  //   {
  //     PMS_LOG_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
  //                      << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
  //     return boost::make_shared<PointRobotFootprint>();
  //   }
    
  // }
  
  // // otherwise
  // PMS_LOG_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  // return boost::make_shared<PointRobotFootprint>();
}


bool GTebPlanner::isRotateFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, double target_angle)
{
  double resolution_collision_check_angular = 0.262; //以15度间隔来判断

  DataSlam slam_data;
  g_dc.getData<DataSlam>(slam_data);
  //先进行一次目标角度的判断
  if ( costmap_model->footprintCost(slam_data.pose.pt.x, slam_data.pose.pt.y, target_angle,
    footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
  {
    return false;
  }

  double delta_rot = g2o::normalize_theta(g2o::normalize_theta(target_angle) -
                                          g2o::normalize_theta(slam_data.pose.theta));
  if(fabs(delta_rot) > resolution_collision_check_angular)
  {
    int n_additional_samples = std::ceil(fabs(delta_rot) / resolution_collision_check_angular) - 1;

    double intermediate_theta = slam_data.pose.theta;
    for(int step = 0; step < n_additional_samples; ++step)
    {
      intermediate_theta = g2o::normalize_theta(intermediate_theta + 
                                                        delta_rot / (n_additional_samples + 1.0));
      if ( costmap_model->footprintCost(slam_data.pose.pt.x, slam_data.pose.pt.y, intermediate_theta,
        footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
      {
        return false;
      }
    }
  }
  return true;
}





//Definations for PMS

struct TebPlannerStates
{
    struct BaseState : StateWithOwner<GTebPlanner>
    {};

    struct Disable : BaseState
    {
        virtual void OnEnter()
        {
            TEB_INFO_LOG("Enter Disable...");
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit Disable...");
        }

        virtual Transition GetTransition()
        {
            if (Owner().m_run_loop_enabled && Owner().isFinish_ == 0)
            {
                return SiblingTransition<Enable>();
            }

            return NoTransition();            
        }

        virtual void Update()
        {
            // TEB_INFO_LOG("Disable...");
        }
    };

    struct Enable : BaseState
    {
        virtual void OnEnter()
        {
            TEB_INFO_LOG("Enter Enable...");
            Owner().dispatchThread();
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit Enable...");
            Owner().stopThread();
        }

        virtual Transition GetTransition()
        {
            if (!Owner().m_run_loop_enabled 
                || IsInInnerState<Finish_Done>()
                || IsInInnerState<Error_Done>()
                || IsInInnerState<Planning_Done>()
            )
                return SiblingTransition<Disable>();

            if(Owner().inite_state_ == TebPlannerInput::ONCONTROLLING)
            {
                // TEB_INFO_LOG("switch to Controlling");
                return InnerEntryTransition<Controlling>();
                // return InnerEntryTransition<SelfRotation_Done>();
                // return InnerEntryTransition<SelfRotation>(-1.57);
            }
            //当全局规划也规划不出全局路径时，直接进入Recovery
            else if(Owner().inite_state_ == TebPlannerInput::ONRECOVERY)
            {
                TEB_INFO_LOG("switch to Recovery");
                return InnerEntryTransition<Recovery>();
            }

            //不应该执行到这里
            return NoTransition();
        }

        virtual void Update()
        {
            // ROTATE_DEBUG_LOG("Enable...");
        }
    };

    //self rotation
    struct SelfRotation : BaseState  //该状态负责旋转事件发送
    {
        float target_angle;
        bool one_flag = false;
        
        virtual void OnEnter(float _target_angle)
        {
          target_angle = _target_angle;
          
          // Owner().turnPID_->reset();
          TEB_INFO_LOG("Enter SelfRotation...");
          DataSlam slam_data;
          if(g_dc.getData<DataSlam>(slam_data) != 0)
          {
            TEB_WARN_LOG("Can't get robot pose.");
          }
          LocalMap::InfeasiableDir infeasiable_dir;
          bool feasiable_rotation_dir;
          double delta_angle;
          bool isFeasiable = g_local_map.checkRotateFeasible(slam_data.pose.pt.x, slam_data.pose.pt.y,
            slam_data.pose.theta, target_angle, infeasiable_dir, feasiable_rotation_dir, delta_angle);

          if(isFeasiable)
          {

            // float delta_rotate_orient = normalize_theta(target_angle - slam_data.pose.theta);
            //发布旋转事件
            CREATE_EVENT(EvTebRotateReq, ev_teb_rotate);
            ev_teb_rotate->dir = feasiable_rotation_dir;
            ev_teb_rotate->target_yaw = target_angle;
            TEB_DEBUG_LOG("[rotate event]teb rotate target angle %f delta angle %f dir %d",target_angle, rad2deg(delta_angle),feasiable_rotation_dir);
            g_ec.pushEvent(ev_teb_rotate);
          }
          else
          {
            TEB_DEBUG_LOG("[rotate]teb rotate infeasiable target angle %f delta angle %f dir %d",target_angle, rad2deg(delta_angle),feasiable_rotation_dir);
            //TODO:处理旋转不可行的情况
          }

          Owner().is_robot_as_start_ = 1;//旋转状态下以机器人当前位置来规划路径
          one_flag = false;
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit SelfRotation...");
        }

        virtual Transition GetTransition()
        {
            if(one_flag == true)
            {
              return SiblingTransition<SelfRotation_Move>(target_angle);
            }
            else
            {
              one_flag = true;
              return NoTransition();
            }

            //直接返回，屏蔽掉后面转轮的操作
            // return InnerEntryTransition<SelfRotation_Run>(delta_rotate_orient > 0);
            
            // if(delta_rotate_orient > 0)
            // {
            // //   g_speed_controller.SetRotate(true,target_angle);
            //     // g_speed_controller.SetChassisControl(0.05,1.57);
            //     return InnerEntryTransition<SelfRotation_TurnAngle>(true, target_angle);
            // }
            // else
            // {
            // //   g_speed_controller.SetRotate(false,target_angle);
            //     // g_speed_controller.SetChassisControl(0.05,-1.57);
            //     return InnerEntryTransition<SelfRotation_TurnAngle>(false, target_angle);
            // }

            // return NoTransition();
        }

        virtual void Update()
        {

        }

    };

    struct SelfRotation_Move : BaseState
    {
        bool dir;
        float target_angle;
        uint8_t run_count = 0;
        geometry_msgs::Twist cmd_vel;
        virtual void OnEnter(float _target_angle)
        {
            target_angle = _target_angle;
            run_count = 0;
            TEB_INFO_LOG("Enter SelfRotation_Move...");
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit SelfRotation_Move...");
        }

        virtual Transition GetTransition()
        {
            DataSlam slam_data;
            g_dc.getData<DataSlam>(slam_data);
            PoseSE2 pose = PoseSE2(slam_data.pose.pt.x,slam_data.pose.pt.y,slam_data.pose.theta);
            // Owner().visualization_->publishRobotPose(pose);
            // float delta_rotate_orient = normalize_theta(Owner().final_angle_ - slam_data.pose.theta);
            float delta_rotate_orient = normalize_theta(target_angle - slam_data.pose.theta);
          
            TEB_DEBUG_LOG("final_angle=%f robot_angle=%f delta=%f",target_angle,slam_data.pose.theta,delta_rotate_orient);
            TEB_DEBUG_LOG("pose_x=%f pose_y=%f theta=%f",slam_data.pose.pt.x,slam_data.pose.pt.y,slam_data.pose.theta);

            // if(fabs(delta_rotate_orient) < Owner().turnPID_->m_errorEpsilon)
            if(fabs(delta_rotate_orient) < 0.05)
            {
              Owner().last_valid_control_ = Timer::getSystemTimestampMS();
              // g_speed_controller.SetStop();
              TEB_DEBUG_LOG("reach angle error and error = %f",delta_rotate_orient);
              // return SiblingTransition<SelfRotation_Done>();
              return SiblingTransition<Controlling>();
            }
            else if(IsInInnerState<SelfRotation_Done>())
            {
              TEB_DEBUG_LOG("Got valid velocity cmd.");
              Owner().last_valid_control_ = Timer::getSystemTimestampMS();
              return SiblingTransition<Controlling>();
            }
            else 
            {
              if(run_count != 0)
              {
                
                if(Owner().computeVelocityCommands(cmd_vel)) //第一次进入不进行teb控制
                {
                  TEB_DEBUG_LOG("[self rotate]Got a valid command: v=%.3lf, w=%.3lf",
                              cmd_vel.linear.x, cmd_vel.angular.z );
                  //下发执行速度
                  // g_speed_controller.SetChassisControl(cmd_vel.linear.x,cmd_vel.angular.z); 
                  //每次获得一次可执行的速度就将reovery状态恢复成默认状态
                  Owner().recovery_state_ = TebPlannerInput::MAX_RECOVERY_TYPE;
                  Owner().infeasible_rotate_ = 0;
                  if(cmd_vel.linear.x >= 0)
                  {
                    Owner().last_valid_control_ = Timer::getSystemTimestampMS();
                    return SiblingTransition<Controlling>();
                  }

                }
                else
                {
                  float attempt_end = Owner().last_valid_control_ + Owner().cfg_.goal_tolerance.controller_patience * 1000.0 * 6;//旋转时把无效轨迹的超时时间加长

                  TEB_DEBUG_LOG("The TEB planner could not find a valid plan.current time %lf end time %f",Timer::getSystemTimestampMS(), attempt_end);
                  
                  //check if we've tried to find a valid control for longer than our time limit
                  if(Timer::getSystemTimestampMS() > attempt_end)
                  {
                      //TODO:获取不到有效路径的处理
                      TEB_DEBUG_LOG("[rotate Timeout]The TEB planner could not find a valid plan.");
                      //发送错误事件
                      Owner().sendErrorInfo();
                      Owner().is_global_path_valid_ = false; //让其空轮循
                      return SiblingTransition<Error>();
                  }
                }
              }
            }
            
            run_count = 1;

            // for (auto evt : Owner().m_evt_list)//Owner().m_evt_list
            // {
            //   TEB_DEBUG_LOG("[self rotate]event name{%s}",evt->getTypeName().c_str());
            // }
            // TEB_DEBUG_LOG("[self rotate]event num %d",Owner().m_evt_list.size());

            //检测参考路径的事件
            for (auto evt : Owner().m_evt_list)//Owner().m_evt_list
            {
              std::type_index type = evt->getType();
              
              if (TYPE_EQUALS(type, EvPFPtebPlanningReq))
              {
                TEB_DEBUG_LOG("[self rotate]got EvPFPtebPlanningReq event.");
                EvPFPtebPlanningReqPtr event_ptr = std::dynamic_pointer_cast<EvPFPtebPlanningReq>(evt);
                
                //判断是否需要旋转
                DataSlam slam_data;
                g_dc.getData<DataSlam>(slam_data);
                Eigen::Vector2d robot_dir(std::cos(slam_data.pose.theta), std::sin(slam_data.pose.theta));
                Eigen::Vector2d robot_pose(slam_data.pose.pt.x,slam_data.pose.pt.y);
                RobotPose pose = event_ptr->path_to_teb_ptr->back();
                Eigen::Vector2d goal_point(pose.pt.x,pose.pt.y);
                double dir = (goal_point - robot_pose).dot(robot_dir);



                Owner().setPlan(event_ptr->path_to_teb_ptr);
                Owner().global_path_index_ = event_ptr->end_idx;
                Owner().global_path_first_ = event_ptr->is_first;
                
                if(Owner().global_path_first_)
                  Owner().planner_->clearPlanner();//每次都要重置一下，冷启动

                Owner().is_global_path_valid_ = 1;
                Owner().is_global_path_end_ = event_ptr->is_end;
                Owner().is_robot_as_start_ = event_ptr->robot_pose_as_start;

                Owner().last_valid_control_ = Timer::getSystemTimestampMS();//Controlling 状态中，记录上次有效控制控制的时间
                Owner().planning_retries_ = 0;//记录请求全局规划的次数

                if(dir < 0) //说明目标点在车子的后方
                {
                  double goal_angle = atan2(goal_point.y() - robot_pose.y(), goal_point.x() - robot_pose.x());
                  TEB_DEBUG_LOG("restart self rotation");
                  return SiblingTransition<SelfRotation>(goal_angle);
                }

                if(event_ptr->path_to_teb_ptr->size() < 3)
                {
                  CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
                  // std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

                  ev_teb_opt_path->opt_path_ptr = event_ptr->path_to_teb_ptr;
                  ev_teb_opt_path->end_idx = event_ptr->end_idx;
                  ev_teb_opt_path->is_robot_as_start = event_ptr->robot_pose_as_start;
                  g_ec.pushEvent(ev_teb_opt_path);

                  Owner().is_global_path_valid_ = 0;
                  return NoTransition();
                }
                else
                {
                  double distance = (event_ptr->path_to_teb_ptr->front().pt - event_ptr->path_to_teb_ptr->back().pt).norm();
                  if(distance < 0.5)
                  {

                    CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
                    // std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

                    ev_teb_opt_path->opt_path_ptr = event_ptr->path_to_teb_ptr;
                    ev_teb_opt_path->end_idx = event_ptr->end_idx;
                    ev_teb_opt_path->is_robot_as_start = event_ptr->robot_pose_as_start;
                    g_ec.pushEvent(ev_teb_opt_path);

                    Owner().is_global_path_valid_ = 0;
                    return NoTransition();
                  }
                }


                return SiblingTransition<Controlling>();

              }
              else if(TYPE_EQUALS(type, EvPFPtebRelax)) //退出teb绕障的事件
              {
                Owner().planner_->clearPlanner();
                break;
              }
            }
            return NoTransition();
        }
    };

    struct SelfRotation_TurnAngle : BaseState
    {
        uint64_t enter_time;
        bool dir;
        float target_angle;
        virtual void OnEnter(bool _dir, float _target_angle)
        {
            enter_time = Timer::getSystemTimestampMS();
            dir = _dir;
            target_angle = _target_angle;
            if(dir == true)
                g_speed_controller.SetChassisControl(0,M_PI_2);
            else
                g_speed_controller.SetChassisControl(0,-M_PI_2);
            
            TEB_INFO_LOG("Enter SelfRotation_TurnAngle...");
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit SelfRotation_TurnAngle...");
        }

        virtual Transition GetTransition()
        {
            if(Timer::getSystemTimestampMS() - enter_time > 1500)
            {
                return SiblingTransition<SelfRotation_Run>(dir, target_angle);
            }
            return NoTransition();
        }
    };

    struct SelfRotation_Run : BaseState
    {
        uint64_t enter_time;
        uint8_t run_count = 0;
        geometry_msgs::Twist cmd_vel;
        bool dir = 0;
        float target_angle;
        virtual void OnEnter(bool _dir, float _target_angle)
        {
            dir = _dir;
            run_count = 0;
            target_angle = _target_angle;
            TEB_INFO_LOG("Enter SelfRotation_Run...");
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit SelfRotation_Run...");
        }

        virtual Transition GetTransition()
        {
            if(Owner().computeVelocityCommands(cmd_vel) && (run_count != 0)) //第一次进入不进行teb控制
            {
                TEB_DEBUG_LOG("[self rotate]Got a valid command: v=%.3lf, w=%.3lf",
                            cmd_vel.linear.x, cmd_vel.angular.z );
                Owner().last_valid_control_ = Timer::getSystemTimestampMS();
                //下发执行速度
                g_speed_controller.SetChassisControl(cmd_vel.linear.x,cmd_vel.angular.z); 
                //每次获得一次可执行的速度就将reovery状态恢复成默认状态
                Owner().recovery_state_ = TebPlannerInput::MAX_RECOVERY_TYPE;
                Owner().infeasible_rotate_ = 0;
                return SiblingTransition<SelfRotation_Done>();
            }
            ++run_count;
            return NoTransition();
        }

        virtual void Update() 
        {
        //TODO：如果一直不能旋转可能会一直卡在这里
          //更新障碍物信息
          // Owner().updateObstacleContainerWithCostmap();

          // bool feasible = Owner().isRotateFeasible(Owner().costmap_model_.get(), Owner().footprint_spec_,
          //     Owner().robot_inscribed_radius_, Owner().robot_circumscribed_radius,Owner().rotation_info_.target_angle);
          DataSlam slam_data;
          g_dc.getData<DataSlam>(slam_data);
          LocalMap::InfeasiableDir infeasiable_side;
          bool feasible = g_local_map.checkRotateFeasible(slam_data.pose.pt.x, slam_data.pose.pt.y, 
              slam_data.pose.theta, target_angle, infeasiable_side);

          if(feasible)//可以旋转
          {
            TEB_DEBUG_LOG("Can rotate dir = %d",dir);
            // g_speed_controller.SetRotate(dir,target_angle); //旧版电控驱动会卡死在这
            if(dir == true)
                g_speed_controller.SetChassisControl(0.08,M_PI_2);//左转
            else
                g_speed_controller.SetChassisControl(0.08,-M_PI_2);
          }
          else
          {
            TEB_DEBUG_LOG("Can't rotate dir = %d",dir);
            // g_speed_controller.SetStopKeepSteering();
            if(dir == true)
                g_speed_controller.SetChassisControl(0,M_PI_2);
            else
                g_speed_controller.SetChassisControl(0,-M_PI_2);
          }
        }
    };
    
    struct SelfRotation_Done : BaseState
    {
        virtual void OnEnter()
        {

            TEB_INFO_LOG("Enter SelfRotation done...");
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit SelfRotation done...");
        }

        virtual Transition GetTransition()
        {
            // return SiblingTransition<Controlling>();
            return NoTransition();
        }

        virtual void Update() 
        {
            // Owner().getData();//操作前更新数据
            // Owner().robot_pose_ = PoseSE2(Owner().m_slam_data.pose.pt.x,
            //     Owner().m_slam_data.pose.pt.y,Owner().m_slam_data.pose.theta);
            // Owner().updateObstacleContainerWithCostmap();
            // Owner().visualization_->publishRobotPose(Owner().robot_pose_); 
            // Owner().visualization_->publishObstacles(Owner().obstacles_);

        }
    };

    struct Planning : BaseState
    {
        virtual void OnEnter()
        {
            TEB_INFO_LOG("Enter Planning...");
            Owner().sendReplanEvent();

            TEB_INFO_LOG("set stop flag, change to disable");
            Owner().m_stop_flag = true;
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit Planning...");
        }

        virtual Transition GetTransition()
        {


            return SiblingTransition<Planning_Done>();            
        }

        // virtual void Update()
        // {
        //     TEB_INFO_LOG("Disable...");
        // }
    };
    struct Planning_Done : BaseState
    {
    };

    struct Controlling : BaseState
    {
      uint64_t last_ts = Timer::getSystemTimestampUS();

      virtual void OnEnter()
      {
        Owner().start_path_collide_check_ = true;
        TEB_INFO_LOG("Enter Controlling ...");
      }

      virtual void OnExit()
      {
        Owner().start_path_collide_check_ = false;
        TEB_INFO_LOG("Exit Controlling ...");
      }

      virtual Transition GetTransition()
      {
        if (Owner().isRecovery_)
        {
            Owner().isRecovery_ = 0;
            TEB_INFO_LOG("switch to Recovery");
            return SiblingTransition<Recovery>();
        }
        else if(Owner().isPlanning_)
        {
            Owner().isPlanning_ = 0;
            TEB_INFO_LOG("switch to Planning");
            return SiblingTransition<Planning>();
        }
        else if(Owner().isError_)
        {
            Owner().isError_ = 0;
            TEB_INFO_LOG("switch to Error");
            return SiblingTransition<Error>();
        }
        else if(Owner().isFinish_)
        {
            TEB_INFO_LOG("switch to Finish");
            return SiblingTransition<Finish>();
        }

        //判断是否需要进入旋转状态
        {
          std::unique_lock<shared_recursive_mutex> rotation_lock(Owner().rotation_mtx_);
          if(Owner().rotate_to_global_plan_ == true 
            && Owner().rotation_info_.self_rotation_enable == true
            || Owner().rotation_info_.self_rotation_enable == true)
          {
            Owner().rotation_info_.self_rotation_enable = false;
            Owner().rotate_to_global_plan_ = false;
            return SiblingTransition<SelfRotation>(Owner().rotation_info_.target_angle);
          }
        }


        // for (auto evt : Owner().m_evt_list)//Owner().m_evt_list
        // {
        //   TEB_DEBUG_LOG("[controlling]event name{%s}",evt->getTypeName().c_str());
        // }
        // TEB_DEBUG_LOG("[controlling]event num %d",Owner().m_evt_list.size());

        //检测参考路径的事件
        for (auto evt : Owner().m_evt_list)
        {
          // TEB_DEBUG_LOG("got EvPFPtebPlanningReq event00.");
          std::type_index type = evt->getType();
          // TEB_DEBUG_LOG("got EvPFPtebPlanningReq event001.");
          
          if (TYPE_EQUALS(type, EvPFPtebPlanningReq))
          {
            TEB_DEBUG_LOG("got EvPFPtebPlanningReq event.");
            EvPFPtebPlanningReqPtr event_ptr = std::dynamic_pointer_cast<EvPFPtebPlanningReq>(evt);

            //判断是否需要旋转
            DataSlam slam_data;
            g_dc.getData<DataSlam>(slam_data);
            Eigen::Vector2d robot_dir(std::cos(slam_data.pose.theta), std::sin(slam_data.pose.theta));
            Eigen::Vector2d robot_pose(slam_data.pose.pt.x,slam_data.pose.pt.y);
            RobotPose pose = event_ptr->path_to_teb_ptr->back();
            Eigen::Vector2d goal_point(pose.pt.x,pose.pt.y);
            double dir = (goal_point - robot_pose).dot(robot_dir);

            Owner().is_global_path_valid_ = 1;
            Owner().setPlan(event_ptr->path_to_teb_ptr);
            Owner().global_path_index_ = event_ptr->end_idx;
            Owner().global_path_first_ = event_ptr->is_first;
            

            Owner().is_global_path_end_ = event_ptr->is_end;

            if(Owner().global_path_first_ ) //|| Owner().is_robot_as_start_ != event_ptr->robot_pose_as_start
              Owner().planner_->clearPlanner();//每次都要重置一下，冷启动

            Owner().is_robot_as_start_ = event_ptr->robot_pose_as_start;
            Owner().last_valid_control_ = Timer::getSystemTimestampMS();//Controlling 状态中，记录上次有效控制控制的时间
            Owner().planning_retries_ = 0;//记录请求全局规划的次数

            if(dir < 0) //说明目标点在车子的后方
            {
              TEB_DEBUG_LOG("[control]switch to self rotation");
              double goal_angle = atan2(goal_point.y() - robot_pose.y(), goal_point.x() - robot_pose.x());
              return SiblingTransition<SelfRotation>(goal_angle);
            }
            

            if(event_ptr->path_to_teb_ptr->size() < 3)
            {
              CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
              // std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

              ev_teb_opt_path->opt_path_ptr = event_ptr->path_to_teb_ptr;
              ev_teb_opt_path->end_idx = event_ptr->end_idx;
              ev_teb_opt_path->is_robot_as_start = event_ptr->robot_pose_as_start;
              g_ec.pushEvent(ev_teb_opt_path);

              Owner().is_global_path_valid_ = 0;
              return NoTransition();
            }
            else
            {
              double distance = (event_ptr->path_to_teb_ptr->front().pt - event_ptr->path_to_teb_ptr->back().pt).norm();
              if(distance < 0.5)
              {

                CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
                // std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

                ev_teb_opt_path->opt_path_ptr = event_ptr->path_to_teb_ptr;
                ev_teb_opt_path->end_idx = event_ptr->end_idx;
                ev_teb_opt_path->is_robot_as_start = event_ptr->robot_pose_as_start;
                g_ec.pushEvent(ev_teb_opt_path);

                Owner().is_global_path_valid_ = 0;
                return NoTransition();
              }
            }



          }
          else if(TYPE_EQUALS(type, EvPFPtebRelax)) //退出teb绕障的事件
          {
            Owner().planner_->clearPlanner();
          }
        }




        return NoTransition();            
      }

      virtual void Update() //该更新频率尽量5Hz，过低机器反应慢
      {
        bool emergency_stop_flag = false;
        bool emergency_slowdown_flag = false;

        geometry_msgs::Twist cmd_vel;
        // TEB_DEBUG_LOG("Update Controlling...");
        Owner().getData(); //局部规划前先获取最新数据

        // check events
        // EvtList evt_list;
        // evt_list.clear();
        // uint64_t cur_ts = Timer::getSystemTimestampUS();
        // // uint64_t delta_t = cur_ts - last_ts;
        // // TEB_INFO_LOG("last_ts:  %u    cur_ts:  %u    delta_t:  %u", last_ts, cur_ts, delta_t);
        // g_ec.eventDeduction(last_ts, cur_ts, evt_list);
        // // last_ts = cur_ts;
        // last_ts = cur_ts - Owner().m_ev_deduction_period_for_us; // 3s

        // for (auto &evt : evt_list)
        // {
        //   std::type_index evt_type = evt->getType();
        //   if (TYPE_EQUALS(evt_type, EvEmergencyStop)) // US Emergency Stop
        //   {
        //     // TEB_INFO_LOG("===============US Emergency Stop===============");
        //     emergency_stop_flag = true;
        //   }
        //   if (TYPE_EQUALS(evt_type, EvEmergencySlowdown)) // US Emergency Slowdown
        //   {
        //     // TEB_INFO_LOG("=============US Emergency Slowdown=============");
        //     emergency_slowdown_flag = true;
        //   }
        //   if (TYPE_EQUALS(evt_type, EvLMEmergencyStop)) // LM Emergency Stop
        //   {
        //     // TEB_INFO_LOG("===============LM Emergency Stop===============");
        //     emergency_stop_flag = true;
        //   }
        //   if (TYPE_EQUALS(evt_type, EvLMEmergencySlowdown)) // LM Emergency Slowdown
        //   {
        //     // TEB_INFO_LOG("=============LM Emergency Slowdown=============");
        //     emergency_slowdown_flag = true;
        //   }
        // }
        
        if(Owner().is_global_path_valid_)
        {
          if (Owner().computeVelocityCommands(cmd_vel))
          {
            TEB_DEBUG_LOG("Got a valid command: v=%.3lf, w=%.3lf",
                          cmd_vel.linear.x, cmd_vel.angular.z);
            Owner().last_valid_control_ = Timer::getSystemTimestampMS();
            // if (emergency_stop_flag)  //TODO:后面再加急停的处理
            // {
            //   g_speed_controller.SetStopKeepSteering(true);
            //   TEB_WARN_LOG("=============== Emergency Stop ===============");
            // }
            // else if (!emergency_stop_flag && emergency_slowdown_flag)
            // {
            //   g_speed_controller.SetChassisControl(cmd_vel.linear.x * 0.5, cmd_vel.angular.z * 0.5, true);
            //   TEB_WARN_LOG("============= Emergency Slowdown =============");
            // }
            // else
            // {
              //下发执行速度
              // g_speed_controller.SetChassisControl(cmd_vel.linear.x, cmd_vel.angular.z);
              //每次获得一次可执行的速度就将reovery状态恢复成默认状态
              Owner().recovery_state_ = TebPlannerInput::MAX_RECOVERY_TYPE;
              Owner().infeasible_rotate_ = 0;
              Owner().planning_retries_ = 0;

            // }
          }
          else
          {
            // g_speed_controller.SetStopKeepSteering(true);
            // float attempt_end = Owner().last_valid_control_ + Owner().controller_patience_ * 1000.0;
            float attempt_end = Owner().last_valid_control_ + Owner().cfg_.goal_tolerance.controller_patience * 1000.0;

            TEB_DEBUG_LOG("The TEB planner could not find a valid plan.current time %lf end time %f",Timer::getSystemTimestampMS(), attempt_end);
            
            //check if we've tried to find a valid control for longer than our time limit
            if(Timer::getSystemTimestampMS() > attempt_end || Owner().is_global_path_end_) //终点是目标点，若有干涉就直接停下来
            {
                //TODO:获取不到有效路径的处理
                TEB_DEBUG_LOG("[Timeout]The TEB planner could not find a valid plan.is_end=%d",Owner().is_global_path_end_);
                // if(Owner().recovery_state_ == TebPlannerInput::MAX_RECOVERY_TYPE)
                // {
                //     Owner().setInitMoveInfo(Owner().m_slam_data.pose.pt.x,Owner().m_slam_data.pose.pt.y);
                //     Owner().setRecoveryState(TebPlannerInput::BACKWARD);
                //     Owner().recovery_tries_++;
                //     TEB_WARN_LOG("Backward recovery from infeasibale plan.recovery_tries = %d.",Owner().recovery_tries_);
                // }
                // else if (Owner().recovery_state_ == TebPlannerInput::BACKWARD)
                // {
                //     //使用当前底盘的角速度作为判断
                //     if (Owner().m_speed_data.w >= 0)
                //         Owner().setRotateRecoveryRelativeAngle(90);
                //     else
                //         Owner().setRotateRecoveryRelativeAngle(-90);
                //     Owner().recovery_tries_++;
                //     Owner().infeasible_rotate_++;
                //     Owner().setRecoveryState(TebPlannerInput::ROTATE);
                //     TEB_WARN_LOG("Rotate recovery from infeasibale plan.recovery_tries = %d.",Owner().recovery_tries_);
                // }
                // else if(Owner().recovery_state_ == TebPlannerInput::ROTATE &&
                //         Owner().infeasible_rotate_ < 4) //最多尝试3次旋转
                // {
                //     if (Owner().m_speed_data.w >= 0)
                //         Owner().setRotateRecoveryRelativeAngle(90);
                //     else
                //         Owner().setRotateRecoveryRelativeAngle(-90);
                //     Owner().setRecoveryState(TebPlannerInput::ROTATE);
                //     Owner().infeasible_rotate_++;
                //     Owner().recovery_tries_++;
                //     TEB_WARN_LOG("Rotate recovery from infeasibale plan.recovery_tries = %d.",Owner().recovery_tries_);
                // }
                // else
                // {
                //     Owner().isError_ = 1;
                //     TEB_WARN_LOG("Can't recovery from infeasibale plan.recovery_tries = %d.",Owner().recovery_tries_);
                // }   


                //发送错误事件
                Owner().sendErrorInfo();
                Owner().is_global_path_valid_ = false; //让其空轮循
                Owner().isError_ = 1;
            }	
          }
        }
      }
    };

    struct Recovery : BaseState
    {
        virtual void OnEnter()
        {
            TEB_INFO_LOG("Enter Recovery...");
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit Recovery...");
        }

        virtual Transition GetTransition()
        {
            if (Owner().isPlanning_)
            {
                Owner().isPlanning_ = 0;
                return SiblingTransition<Planning>();
            }

            return NoTransition();            
        }

        virtual void Update() 
        {
            TEB_DEBUG_LOG("Recovery...");
            // Owner().getData();//操作前更新数据
            

        }
    };

    struct Error : BaseState
    {
        bool one_flag = false;//空循环一次，为了让事件列表更新。
        virtual void OnEnter()
        {
            TEB_INFO_LOG("Enter Error...");
            one_flag = false; 
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit Error...");
        }

        virtual Transition GetTransition()
        {
          if(one_flag == false)//空循环一次，为了让事件列表更新。
          {
            one_flag = true;
            return NoTransition();
          }

          // for (auto evt : Owner().m_evt_list)//Owner().m_evt_list
          // {
          //   TEB_DEBUG_LOG("[error]event name{%s}",evt->getTypeName().c_str());
          // }
          // TEB_DEBUG_LOG("[error]event num %d",Owner().m_evt_list.size());

          //检测参考路径的事件
          for (auto evt : Owner().m_evt_list)
          {
            std::type_index type = evt->getType();
            
            if (TYPE_EQUALS(type, EvPFPtebPlanningReq))
            {
              TEB_DEBUG_LOG("[error]got EvPFPtebPlanningReq event.");

              EvPFPtebPlanningReqPtr event_ptr = std::dynamic_pointer_cast<EvPFPtebPlanningReq>(evt);
              //判断是否需要旋转
              DataSlam slam_data;
              g_dc.getData<DataSlam>(slam_data);
              Eigen::Vector2d robot_dir(std::cos(slam_data.pose.theta), std::sin(slam_data.pose.theta));
              Eigen::Vector2d robot_pose(slam_data.pose.pt.x,slam_data.pose.pt.y);
              RobotPose pose = event_ptr->path_to_teb_ptr->back();
              Eigen::Vector2d goal_point(pose.pt.x,pose.pt.y);
              double dir = (goal_point - robot_pose).dot(robot_dir);

              // TEB_DEBUG_LOG("got EvPFPtebPlanningReq event1.");
              Owner().setPlan(event_ptr->path_to_teb_ptr);
              // TEB_DEBUG_LOG("got EvPFPtebPlanningReq event2.");
              Owner().global_path_index_ = event_ptr->end_idx;
              Owner().global_path_first_ = event_ptr->is_first;
              
              // if(Owner().global_path_first_)
              //error状态下接收到的路径，直接清掉缓存的teb container  TODO：测试还会不会报velocity edge的错  还会
              Owner().planner_->clearPlanner();//每次都要重置一下，冷启动

              Owner().is_global_path_valid_ = 1;
              Owner().is_global_path_end_ = event_ptr->is_end;
              Owner().is_robot_as_start_ = event_ptr->robot_pose_as_start;

              Owner().last_valid_control_ = Timer::getSystemTimestampMS();//Controlling 状态中，记录上次有效控制控制的时间
              Owner().planning_retries_ = 0;//记录请求全局规划的次数
              
              if(dir < 0) //说明目标点在车子的后方
              {
                TEB_DEBUG_LOG("[error]switch to self rotation");
                double goal_angle = atan2(goal_point.y() - robot_pose.y(), goal_point.x() - robot_pose.x());
                return SiblingTransition<SelfRotation>(goal_angle);
              }
              
              if(event_ptr->path_to_teb_ptr->size() < 3)
              {
                CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
                // std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

                ev_teb_opt_path->opt_path_ptr = event_ptr->path_to_teb_ptr;
                ev_teb_opt_path->end_idx = event_ptr->end_idx;
                ev_teb_opt_path->is_robot_as_start = event_ptr->robot_pose_as_start;
                g_ec.pushEvent(ev_teb_opt_path);

                Owner().is_global_path_valid_ = 0;
                return NoTransition();
              }
              else
              {
                double distance = (event_ptr->path_to_teb_ptr->front().pt - event_ptr->path_to_teb_ptr->back().pt).norm();
                if(distance < 0.5)
                {

                  CREATE_EVENT(EvTebOptPath, ev_teb_opt_path);
                  // std::shared_ptr<PosePath> teb_path_ptr = std::shared_ptr<PosePath>(new PosePath());

                  ev_teb_opt_path->opt_path_ptr = event_ptr->path_to_teb_ptr;
                  ev_teb_opt_path->end_idx = event_ptr->end_idx;
                  ev_teb_opt_path->is_robot_as_start = event_ptr->robot_pose_as_start;
                  g_ec.pushEvent(ev_teb_opt_path);

                  Owner().is_global_path_valid_ = 0;
                  return NoTransition();
                }
              }



              return SiblingTransition<Controlling>();
            }
            else if(TYPE_EQUALS(type, EvPFPtebRelax)) //退出teb绕障的事件
            {
              Owner().planner_->clearPlanner();
            }
          }

          return NoTransition();          
        }

        // virtual void Update()
        // {
        //     TEB_INFO_LOG("Error...");
        // }
    };
    struct Error_Done : BaseState
    {
    };

    struct Finish : BaseState
    {
        virtual void OnEnter()
        {
            TEB_INFO_LOG("Enter Finished, teb task finished");
            Owner().finishPlanning();

            TEB_INFO_LOG("set stop flag, change to Idle");
            Owner().m_stop_flag = true;
        }

        virtual void OnExit()
        {
            TEB_INFO_LOG("Exit Finish...");
        }

        virtual Transition GetTransition()
        {
            return SiblingTransition<Finish_Done>();            
        }

        // virtual void Update()
        // {
        //     // TEB_INFO_LOG("Finish...");
        // }
    };
    struct Finish_Done : BaseState
    {
    };

};



void GTebPlanner::initRunSM()
{
    m_run_sm.Initialize<TebPlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("GTebPlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void GTebPlanner::reset()
{
  isFinish_ = 0;
  isRecovery_ = 0;
  isPlanning_ = 0;
  isError_ = 0; //异常状态
  planning_retries_ = 0;
  recovery_tries_ = 0;
  infeasible_rotate_ = 0;
  rotate_to_global_plan_ = true;
  //清掉抖动检测器里面的数据
  failure_detector_.clear();
  g_speed_controller.SetStop();
}


void GTebPlanner::getData() 
{
    g_dc.getData<DataSlam>(m_slam_data);
    g_dc.getData<DataFeedbackSpeed>(m_speed_data);
    int32_t steer_speed = m_speed_data.l_encoder;//mm/s
    int32_t steer_rad = -1 * m_speed_data.r_encoder;//mrad 右正，左负  对数据取反
    m_speed_data.v = steer_speed * 0.001 * cos(steer_rad*0.001);
    m_speed_data.w = steer_speed * 0.001 * sin(steer_rad*0.001) / cfg_.robot.wheelbase;

    // TEB_DEBUG_LOG("steer_speed=%d steer_angle=%d v=%f w=%f",steer_speed,steer_rad,m_speed_data.v,m_speed_data.w);
}

bool GTebPlanner::handleInput(const GlobalPlannerInputPtr input)
{
    // const TebPlannerInputPtr teb_input = 
    //     std::dynamic_pointer_cast<TebPlannerInput>(input);

    // inite_state_ = teb_input->state;
    // need_theta_ = teb_input->need_theta;
    // global_path_index_ = teb_input->global_path_index;

    // if(teb_input->path->size() > 0)
    // {
    //   final_angle_ = teb_input->path->back().theta;//path为空的时候不能用
    //   setPlan(teb_input->path);
    // }

    last_valid_control_ = Timer::getSystemTimestampMS();//Controlling 状态中，记录上次有效控制控制的时间
    planning_retries_ = 0;//记录请求全局规划的次数

    // TEB_DEBUG_LOG("receive teb input.id[%ld] runState:%d",m_input_id,inite_state_);
        
    return true;
}

/**
     * @brief  完成当前全局规划后调用的函数
     * @param  None
     * @retval None
     */
void GTebPlanner::finishPlanning()
{
    TEB_INFO_LOG("generate teb input finished input id = %ld", m_input_id);
    CREATE_EVENT(EvPlannerFinished, ev_teb_finished);
    ev_teb_finished->id = m_input_id;
    g_ec.pushEvent(ev_teb_finished);
}


/**
  * @brief  发送teb错误事件
  * @param  None
  * @retval None
  */
void GTebPlanner::sendErrorInfo()
{
    TEB_DEBUG_LOG("send teb task error evt id = %ld", m_input_id);
    CREATE_EVENT(EvGTebPlannerError, ev_teb_error);
    ev_teb_error->input_id = m_input_id;
    g_ec.pushEvent(ev_teb_error);
}

/**
  * @brief  发送teb错误事件
  * @param  None
  * @retval None
  */
void GTebPlanner::sendReplanEvent()
{
    TEB_DEBUG_LOG("send teb task replan evt id = %ld", m_input_id);
    CREATE_EVENT(EvGTebPlannerReplan, ev_teb_replan);
    ev_teb_replan->input_id = m_input_id;
    g_ec.pushEvent(ev_teb_replan);	
}

/**
  * @brief  旋转恢复动作
  * @param  None
  * @retval None
  */
bool GTebPlanner::runRotateRecovery()
{
    geometry_msgs::Twist cmd_vel;
    //获取目标角度与当前机器人角度之间的差值
    double delta_angel = g2o::normalize_theta(rotate_recovery_angel_ - m_slam_data.pose.theta);

    if(std::fabs(delta_angel) < cfg_.goal_tolerance.yaw_goal_tolerance)
    {

        return true;
    }
    else
    {

        rotateToGoal(
          delta_angel, //传入还有多少角度差需要旋转
          cmd_vel,
          1.0/controller_frequency_,  //velocity simulation period(the unit is second) 需根据轮循频率设置
          m_speed_data.w,//传入当前机器人的旋转速度 TODO：换成pms的速度接口
          cfg_); //配置信息，主要是关注速度限制

        // g_speed_controller.SetRotate(0,cmd_vel.angular.z);//TODO：更换运动控制接口
        // g_speed_controller.SetChassisControl(0,cmd_vel.angular.z);

    }
    return false;
}

/**
  * @brief  设置要执行的恢复动作
  * @param  state 恢复类型
  * @retval None
  */
void GTebPlanner::setRecoveryState(TebPlannerInput::RecoveryType state)
{
    recovery_state_ = state;
    isRecovery_ = 1;
}

/**
  * @brief  设定旋转恢复动作的目标角度
  * @param  target_angle 目标旋转角度（degree） 
  * @retval None
  */
void GTebPlanner::setRotateRecoveryAngle(float target_angle)
{
    rotate_recovery_angel_ = angles::from_degrees(target_angle);
}

/**
  * @brief  设定旋转恢复动作的目标角度
  * @param  relative_angle 相对旋转角度（degree） 
  * @retval None
  */
void GTebPlanner::setRotateRecoveryRelativeAngle(float relative_angle)
{
    rotate_recovery_angel_ = angles::from_degrees(relative_angle) + m_slam_data.pose.theta;
}

/**
  * @brief  设置获取清除了固定区域的localmap
  * @param  None
  * @retval None
  */
void GTebPlanner::setClearLocalMap()
{
    //TODO:设置一个标志位,表示本次获取localmap时底盘周边的区域，设定为可行区域
    get_clear_local_map_ = 1;
    isPlanning_ = 1;
}

/**
  * @brief  后退恢复操作
  * @param  dir 移动方向 1-前进 0-后退
  * @retval None
  */
bool GTebPlanner::runMoveRecovery(bool dir)
{
    float move_distance_limit = 0.3;//单位：米 设定最大可以后退距离
    float move_vel = dir ? 0.1 : -0.1;//单位:mm/s
    float sim_time = 0.2;
    uint16_t max_move_time = 3000;//最大后退时间，单位：毫秒
    float delta_x = std::fabs(init_move_pose_[0] - m_slam_data.pose.pt.x);
    float delta_y = std::fabs(init_move_pose_[1] - m_slam_data.pose.pt.y);
    if(std::hypot(delta_x,delta_y) >= move_distance_limit
      || Timer::getSystemTimestampMS() >= init_move_time_ + max_move_time)
    {
        // g_speed_controller.SetSpeed1(0,0);//TODO：更换运动控制接口
        return true; 
    }
    else
    {
        float transition = move_vel * sim_time * 0.001;
        float tmp_x = m_slam_data.pose.pt.x + transition * cos(m_slam_data.pose.theta);
        float tmp_y = m_slam_data.pose.pt.y + transition * sin(m_slam_data.pose.theta);
        //TODO:判断后退的位置是否有障碍物
        // g_speed_controller.SetSpeed1(move_vel,0);//TODO：更换运动控制接口
    }

    return false;
}

/**
  * @brief  组合移动和旋转
  * @param  None
  * @retval None
  */
 bool GTebPlanner::runMoveAndRotateRecovery(bool dir)
{
    static bool move_done = 0;
    if(move_done == 0)
    {
        if(runMoveRecovery(dir))
        {
            move_done = 1;
        }
    }
    else
    {
        if(runRotateRecovery())
        {
            move_done = 0;
            return true;
        }
        
    }
    return false;
}

/**
  * @brief  设置后退操作所需的初始状态信息
  * @param  x 底盘x坐标
  * @param  y 底盘y坐标
  * @retval None
  */
void GTebPlanner::setInitMoveInfo(float x,float y)
{
    init_move_pose_[0] = x;
    init_move_pose_[1] = y;
    init_move_time_ = Timer::getSystemTimestampMS();
}



} // end namespace teb_local_planner
// } // end namespace planning_planner

