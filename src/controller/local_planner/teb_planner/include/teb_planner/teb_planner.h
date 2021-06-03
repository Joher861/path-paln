#ifndef TEB_LOCAL_PLANNER_ROS_H_
#define TEB_LOCAL_PLANNER_ROS_H_


#include <teb_planner/base_local_planner/costmap_model.h>


// timed-elastic-band related classes
#include <teb_planner/teb_local_planner/optimal_planner.h>
#include <teb_planner/teb_local_planner/homotopy_class_planner.h>
#include <teb_planner/teb_local_planner/visualization.h>
#include <teb_planner/teb_local_planner/recovery_behaviors.h>
#include <teb_planner/teb_local_planner/misc.h>
#include "teb_planner/teb_local_planner/SimPID.h"

#include <teb_planner/teb_local_planner/obstacles.h>
#include <teb_planner/teb_local_planner/teb_config.h>
#include <teb_planner/teb_local_planner/angles.h>
#include <teb_planner/teb_planner_events.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
// message types
#include <teb_planner/nav_msgs/Path.h>
#include <teb_planner/nav_msgs/Odometry.h>
#include <teb_planner/geometry_msgs/PoseStamped.h>
#include <teb_planner/geometry_msgs/Twist.h> 
#include <teb_planner/base_local_planner/visualization_msgs/Marker.h>


// transforms  //tf需要做更改，变成对接pms获取机器人位姿的接口
#include <teb_planner/base_local_planner/tf/tf.h>
#include <teb_planner/base_local_planner/tf/transform_listener.h>
#include <teb_planner/base_local_planner/tf/transform_datatypes.h>

// costmap cosmap后面要替换成pms内部的数据格式

#include <teb_planner/costmap_2d/costmap_2d.h>
#include "teb_planner/teb_local_planner/teb_local_map.h"


// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

//eigen3
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <string>
#include <math.h>
#include <vector>
#include <memory>

#include "local_planner/local_planner.h"

#include "data/slam_data.h"
#include "data/chassis_data.h"
#include "local_planner/local_planner_events.h"
#include "misc/planning_typedefs.h"
#include "task/task.h"
#include "teb_planner/teb_task.h"
#include "misc/planning_typedefs.h"
// #include "local_map/local_map.h"
#include "data/local_map_data.h"



namespace planning_controller
{

    class TebPlanner; 
    using TebPlannerPtr = std::shared_ptr<TebPlanner>;
/**
  * @class TebLocalPlannerROS
  * @brief Implements the actual abstract navigation stack routines of the teb_local_planner plugin
  * @todo Escape behavior, more efficient obstacle handling
  */
// DEFINE_LOCAL_PLANNER(Teb)  //class name is TebPlanner, TebPlannerPtr is its shared ptr
class TebPlanner : public LocalPlanner 
{

  struct RotationInfo
  {
    bool self_rotation_enable;
    float target_angle; //rad
  };

  enum TebControllerState 
  {
    SUCCESS         = 0,
    FAILURE         = 100,
    CANCELED        = 101,
    NO_VALID_CMD    = 102,
    PAT_EXCEEDED    = 103,
    COLLISION       = 104,
    OSCILLATION     = 105,
    ROBOT_STUCK     = 106,
    MISSED_GOAL     = 107,
    MISSED_PATH     = 108,
    BLOCKED_PATH    = 109,
    INVALID_PATH    = 110,
    TF_ERROR        = 111,
    NOT_INITIALIZED = 112,
    INVALID_PLUGIN  = 113,
    INTERNAL_ERROR  = 114,
    BACKWARD_TRAJECTORY=115
  };


public:
  /**
    * @brief Default constructor of the teb plugin
    */
  TebPlanner(ThreadPool* pool);

  static TebPlannerPtr& getInstance(ThreadPool* pool);

  /**
    * @brief  Destructor of the plugin
    */
  ~TebPlanner();


  /**
    * @brief Initializes the teb plugin
    * @param name The name of the instance
    * @param tf Pointer to a transform listener
    * @param costmap_ros Cost map representing occupied and free space
    */
  void initialize();//, costmap_2d::Costmap2DROS* costmap_ros

  /**
    * @brief Set the plan that the teb local planner is following
    * @param orig_global_plan The plan to pass to the local planner
    * @return True if the plan was updated successfully, false otherwise
    */
  bool setPlan(const PosePathPtr orig_global_plan);

  /**
    * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
    * @return True if a valid trajectory was found, false otherwise
    */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
    * @brief  Check if the goal pose has been achieved
    * 
    * The actual check is performed in computeVelocityCommands(). 
    * Only the status flag is checked here.
    * @return True if achieved, false otherwise
    */
  bool isGoalReached();
  
  
    
  /** @name Public utility functions/methods */
  //@{
  
    /**
    * @brief  Transform a tf::Pose type into a Eigen::Vector2d containing the translational and angular velocities.
    * 
    * Translational velocities (x- and y-coordinates) are combined into a single translational velocity (first component).
    * @param tf_vel tf::Pose message containing a 1D or 2D translational velocity (x,y) and an angular velocity (yaw-angle)
    * @return Translational and angular velocity combined into an Eigen::Vector2d
    */
  static Eigen::Vector2d tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel);

  /**
   * @brief Get the current robot footprint/contour model
   * @param nh const reference to the local ros::NodeHandle
   * @return Robot footprint model used for optimization
   */
  RobotFootprintModelPtr getRobotFootprintFromParamServer();
  
    /** 
   * @brief Set the footprint from the given XmlRpcValue.
   * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
   * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::Point
   * @param footprint_xmlrpc should be an array of arrays, where the top-level array should have 3 or more elements, and the
   * sub-arrays should all have exactly 2 elements (x and y coordinates).
   * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came. 
   * It is used only for reporting errors. 
   * @return container of vertices describing the polygon
   */
  // static Point2dContainer makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name);
  
  /** 
   * @brief Get a number from the given XmlRpcValue.
   * @remarks This method is copied from costmap_2d/footprint.h, since it is not declared public in all ros distros
   * @remarks It is modified in order to return a container of Eigen::Vector2d instead of geometry_msgs::Point
   * @param value double value type
   * @param full_param_name this is the full name of the rosparam from which the footprint_xmlrpc value came. 
   * It is used only for reporting errors. 
   * @returns double value
   */
  // static double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name);
  
  //@}
  
protected:

  /**
    * @brief Update internal obstacle vector based on occupied costmap cells
    * @remarks All occupied cells will be added as point obstacles.
    * @remarks All previous obstacles are cleared.
    * @sa updateObstacleContainerWithCostmapConverter
    * @todo Include temporal coherence among obstacle msgs (id vector)
    * @todo Include properties for dynamic obstacles (e.g. using constant velocity model)
    */
  void updateObstacleContainerWithCostmap();
  
  /**
   * @brief Update internal obstacle vector based on polygons provided by a costmap_converter plugin
   * @remarks Requires a loaded costmap_converter plugin.
   * @remarks All previous obstacles are cleared.
   * @sa updateObstacleContainerWithCostmap
   */
  // void updateObstacleContainerWithCostmapConverter();
  
  /**
   * @brief Update internal obstacle vector based on custom messages received via subscriber
   * @remarks All previous obstacles are NOT cleared. Call this method after other update methods.
   * @sa updateObstacleContainerWithCostmap, updateObstacleContainerWithCostmapConverter
   */
  // void updateObstacleContainerWithCustomObstacles();


  /**
   * @brief Update internal via-point container based on the current reference plan
   * @remarks All previous via-points will be cleared.
   * @param transformed_plan (local) portion of the global plan (which is already transformed to the planning frame)
   * @param min_separation minimum separation between two consecutive via-points
   */
  void updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation);
  
  
  /**
    * @brief Callback for the dynamic_reconfigure node.
    * 
    * This callback allows to modify parameters dynamically at runtime without restarting the node
    * @param config Reference to the dynamic reconfigure config
    * @param level Dynamic reconfigure level
    */
  void reconfigureCB(TebConfig& config, uint32_t level);
  
  
   /**
    * @brief Callback for custom obstacles that are not obtained from the costmap 
    * @param obst_msg pointer to the message containing a list of polygon shaped obstacles
    */
  // void customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
  
   /**
    * @brief Callback for custom via-points
    * @param via_points_msg pointer to the message containing a list of via-points
    */
  // void customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg);

  /**
   * @brief rotateToOrientation
   * @param angle
   * @param cmd_vel
   * @param accuracy
   * @return
   */
  bool rotateToGoal(
      double ang_diff,
      geometry_msgs::Twist& cmd_vel,
      double sim_period,
      double vel_yaw,
      const TebConfig& cfg);

   /**
    * @brief Prune global plan such that already passed poses are cut off
    * 
    * The pose of the robot is transformed into the frame of the global plan by taking the most recent tf transform.
    * If no valid transformation can be found, the method returns \c false.
    * The global plan is pruned until the distance to the robot is at least \c dist_behind_robot.
    * If no pose within the specified treshold \c dist_behind_robot can be found,
    * nothing will be pruned and the method returns \c false.
    * @remarks Do not choose \c dist_behind_robot too small (not smaller the cellsize of the map), otherwise nothing will be pruned.
    * @param tf A reference to a transform listener
    * @param global_pose The global pose of the robot
    * @param[in,out] global_plan The plan to be transformed
    * @param dist_behind_robot Distance behind the robot that should be kept [meters]
    * @return \c true if the plan is pruned, \c false in case of a transform exception or if no pose cannot be found inside the threshold
    */
  bool pruneGlobalPlan(const RobotPose& global_pose,  
                       std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);
  
  /**
    * @brief  Transforms the global plan of the robot from the planner frame to the local frame (modified).
    * 
    * The method replaces transformGlobalPlan as defined in base_local_planner/goal_functions.h 
    * such that the index of the current goal pose is returned as well as 
    * the transformation between the global plan and the planning frame.
    * @param tf A reference to a transform listener
    * @param global_plan The plan to be transformed
    * @param global_pose The global pose of the robot
    * @param costmap A reference to the costmap being used so the window size for transforming can be computed
    * @param global_frame The frame to transform the plan to
    * @param max_plan_length Specify maximum length (cumulative Euclidean distances) of the transformed plan [if <=0: disabled; the length is also bounded by the local costmap size!]
    * @param[out] transformed_plan Populated with the transformed plan
    * @param[out] current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @return \c true if the global plan is transformed, \c false otherwise
    */
  bool transformGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                           const RobotPose& global_pose,  const costmap_2d::Costmap2D& costmap,
                           double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                           int* current_goal_idx = NULL,int* start_index = NULL) const;
    
  /**
    * @brief Estimate the orientation of a pose from the global_plan that is treated as a local goal for the local planner.
    * 
    * If the current (local) goal point is not the final one (global)
    * substitute the goal orientation by the angle of the direction vector between 
    * the local goal and the subsequent pose of the global plan. 
    * This is often helpful, if the global planner does not consider orientations. \n
    * A moving average filter is utilized to smooth the orientation.
    * @param global_plan The global plan
    * @param local_goal Current local goal
    * @param current_goal_idx Index of the current (local) goal pose in the global plan
    * @param[out] tf_plan_to_global Transformation between the global plan and the global planning frame
    * @param moving_average_length number of future poses of the global plan to be taken into account
    * @return orientation (yaw-angle) estimate
    */
  double estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const tf::Stamped<tf::Pose>& local_goal,
                                      int current_goal_idx,int moving_average_length=3) const;
        
        
  /**
   * @brief Saturate the translational and angular velocity to given limits.
   * 
   * The limit of the translational velocity for backwards driving can be changed independently.
   * Do not choose max_vel_x_backwards <= 0. If no backward driving is desired, change the optimization weight for
   * penalizing backwards driving instead.
   * @param[in,out] vx The translational velocity that should be saturated.
   * @param[in,out] vy Strafing velocity which can be nonzero for holonomic robots
   * @param[in,out] omega The angular velocity that should be saturated.
   * @param max_vel_x Maximum translational velocity for forward driving
   * @param max_vel_y Maximum strafing velocity (for holonomic robots)
   * @param max_vel_theta Maximum (absolute) angular velocity
   * @param max_vel_x_backwards Maximum translational velocity for backwards driving
   */
  void saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y,
                        double max_vel_theta, double max_vel_x_backwards) const;

  
  /**
   * @brief Convert translational and rotational velocities to a steering angle of a carlike robot
   * 
   * The conversion is based on the following equations:
   * - The turning radius is defined by \f$ R = v/omega \f$
   * - For a car like robot withe a distance L between both axles, the relation is: \f$ tan(\phi) = L/R \f$
   * - phi denotes the steering angle.
   * @remarks You might provide distances instead of velocities, since the temporal information is not required.
   * @param v translational velocity [m/s]
   * @param omega rotational velocity [rad/s]
   * @param wheelbase distance between both axles (drive shaft and steering axle), the value might be negative for back_wheeled robots
   * @param min_turning_radius Specify a lower bound on the turning radius
   * @return Resulting steering angle in [rad] inbetween [-pi/2, pi/2]
   */
  double convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius = 0) const;
  
  /**
   * @brief Validate current parameter values of the footprint for optimization, obstacle distance and the costmap footprint
   * 
   * This method prints warnings if validation fails.
   * @remarks Currently, we only validate the inscribed radius of the footprints
   * @param opt_inscribed_radius Inscribed radius of the RobotFootprintModel for optimization
   * @param costmap_inscribed_radius Inscribed radius of the footprint model used for the costmap
   * @param min_obst_dist desired distance to obstacles
   */
  void validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist);
  
  
  void configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx);

  //判断是否可旋转到目标角度
  bool isRotateFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                              double inscribed_radius, double circumscribed_radius, double target_angle);
    
private:
  // pms interface
  virtual void initRunSM() override;
  virtual void reset() override;
  virtual void getData() override;
  virtual bool handleTask(const Task& task) override;
  virtual void finishTask() override;
  float loadConfig(ConfigManager &cfg_mgr);
  friend struct TebPlannerStates; //状态结构体
  DataSlam m_slam_data;//从该数据获取位置信息
  DataFeedbackSpeed m_speed_data; //从该数据获取底盘反馈的速度
  bool isRecovery_ = 0;
  bool isPlanning_ = 0;
  bool isError_ = 0; //异常状态
  bool isFinish_ = 0;
 
  uint64_t last_valid_control_ = 0;//Controlling 状态中，记录上次有效控制控制的时间
  uint32_t planning_retries_ = 0;//记录请求全局规划的次数
  //TODO:该参数后面需要添加到配置区里
  float controller_patience_ = 10;//单位：秒  该时间段内获取不到有效执行速度，可重新尝试局部规划；
                                  //超时将请求全局规划。
  TaskTeb::RunState inite_state_ = TaskTeb::ONCONTROLLING;//状态机初始运行状态
  float rotate_recovery_angel_ = 0;//旋转恢复时，设定的目标旋转角度。单位：弧度
  
  bool get_clear_local_map_ = 0;
  uint16_t recovery_tries_ = 0;//记录尝试recovery的次数
  uint8_t infeasible_rotate_ = 0;
  TaskTeb::RecoveryType recovery_state_ = TaskTeb::MAX_RECOVERY_TYPE;
  double controller_frequency_ = 5;
  float init_move_pose_[2] = {0,0};
  uint64_t init_move_time_ = 0;
  bool need_theta_ = false;

  bool runRotateRecovery();
  void setRotateRecoveryAngle(float target_angle);
  void setRotateRecoveryRelativeAngle(float relative_angle);
  void setClearLocalMap();
  bool runMoveRecovery(bool dir);
  void setInitMoveInfo(float x,float y);
  void setRecoveryState(TaskTeb::RecoveryType state);
  bool runMoveAndRotateRecovery(bool dir);
  void sendReplanEvent();
  void sendErrorInfo();

  // external objects (store weak pointers)
  // costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
  costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  tf::TransformListener* tf_; //!< pointer to Transform Listener
    
  // internal objects (memory management owned)
  PlannerInterfacePtr planner_; //!< Instance of the underlying optimal planner class
  ObstContainer obstacles_; //!< Obstacle vector that should be considered during local trajectory optimization
  ViaPointContainer via_points_; //!< Container of via-points that should be considered during local trajectory optimization
  TebVisualizationPtr visualization_; //!< Instance of the visualization class (local/global plan, obstacles, ...)
  boost::shared_ptr<base_local_planner::CostmapModel> costmap_model_;  
  TebConfig cfg_; //!< Config class that stores and manages all related parameters
  FailureDetector failure_detector_; //!< Detect if the robot got stucked
  
  std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
  
  
  // pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_; //!< Load costmap converter plugins at runtime
  // boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_; //!< Store the current costmap_converter  

  // boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg_; //!< Dynamic reconfigure server to allow config modifications at runtime
  // ros::Subscriber custom_obst_sub_; //!< Subscriber for custom obstacles received via a ObstacleMsg.
  boost::mutex custom_obst_mutex_; //!< Mutex that locks the obstacle array (multi-threaded)
  // costmap_converter::ObstacleArrayMsg custom_obstacle_msg_; //!< Copy of the most recent obstacle message

  // ros::Subscriber via_points_sub_; //!< Subscriber for custom via-points received via a Path msg.
  bool custom_via_points_active_; //!< Keep track whether valid via-points have been received from via_points_sub_
  boost::mutex via_point_mutex_; //!< Mutex that locks the via_points container (multi-threaded)

  PoseSE2 robot_pose_; //!< Store current robot pose
  PoseSE2 robot_goal_; //!< Store current robot goal
  geometry_msgs::Twist robot_vel_; //!< Store current robot translational and angular velocity (vx, vy, omega)
  bool goal_reached_; //!< store whether the goal is reached or not
  uint64_t time_last_infeasible_plan_; //!< Store at which time stamp the last infeasible plan was detected
  int no_infeasible_plans_; //!< Store how many times in a row the planner failed to find a feasible plan.
  uint32_t time_last_oscillation_; //!< Store at which time stamp the last oscillation was detected
  RotType last_preferred_rotdir_; //!< Store recent preferred turning direction
  geometry_msgs::Twist last_cmd_; //!< Store the last control command generated in computeVelocityCommands()
  
  std::vector<geometry_msgs::Point> footprint_spec_; //!< Store the footprint of the robot 
  double robot_inscribed_radius_; //!< The radius of the inscribed circle of the robot (collision possible)
  double robot_circumscribed_radius; //!< The radius of the circumscribed circle of the robot
  
  std::string global_frame_; //!< The frame in which the controller will run
  std::string robot_base_frame_; //!< Used as the base frame id of the robot
    
  // flags
  bool initialized_; //!< Keeps track about the correct initialization of this class
  ThreadPool* pool_ptr_;//存储线程池指针
  double current_max_vel_x_;
  double min_obs_dis_;

  float currentWheelTurnSet;

  bool rotate_to_global_plan_;
  double gobal_angle_backup_;
  SimPID * turnPID_;
  float final_angle_; //for testing rotation pid controller

  RotationInfo rotation_info_;

  DataLocalMap local_map_data_;
  TebLocalMap teb_local_map_;
  TebControllerState teb_controller_state_ = SUCCESS;
  // 接入Path_Follow_Planner相关
  int m_ev_deduction_period_for_us;
  size_t global_path_index_;//记录下发的全局路径起始点的index
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
// END_LOCAL_PLANNER(Teb)

/**
 * normalize the angle
 */
inline double normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}


/**
 * sign function.
 * @return the sign of x. +1 for x > 0, -1 for x < 0, 0 for x == 0
 */
template <typename T>
inline int sign(T x)
{
  if (x > 0)
    return 1;
  else if (x < 0)
    return -1;
  else
    return 0;
}





} 
// end namespace planning_controller
#endif // TEB_LOCAL_PLANNER_ROS_H_


