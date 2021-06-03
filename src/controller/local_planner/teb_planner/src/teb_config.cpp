
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/teb_log.h>
namespace planning_controller
{
DEFINE_CONFIG_READ_FUNC(TebPlanner)    
  READ_CONFIG_MEMBER(ev_deduction_period_for_us);
END_CONFIG_READ_FUNC(TebPlanner)

DEFINE_CONFIG_TYPE(CONFIG_TEB_PLANNER, TebPlanner);

DEFINE_CONFIG_READ_FUNC(TebTrajectory)
	READ_CONFIG_MEMBER(teb_autosize)
	READ_CONFIG_MEMBER(dt_ref)
	READ_CONFIG_MEMBER(dt_hysteresis)
	READ_CONFIG_MEMBER(min_samples)
	READ_CONFIG_MEMBER(max_samples)
	READ_CONFIG_MEMBER(global_plan_overwrite_orientation)
	READ_CONFIG_MEMBER(allow_init_with_backwards_motion)
	READ_CONFIG_MEMBER(global_plan_viapoint_sep)
	READ_CONFIG_MEMBER(no_viapoint_idx)
	READ_CONFIG_MEMBER(via_points_ordered)
	READ_CONFIG_MEMBER(max_global_plan_lookahead_dist)
	READ_CONFIG_MEMBER(global_plan_prune_distance)
	READ_CONFIG_MEMBER(exact_arc_length)
	READ_CONFIG_MEMBER(feasibility_check_no_poses)
	READ_CONFIG_MEMBER(publish_feedback)
	READ_CONFIG_MEMBER(control_look_ahead_poses)
END_CONFIG_READ_FUNC(TebTrajectory)

DEFINE_CONFIG_TYPE(CONFIG_TRAJECTORY, TebTrajectory);

DEFINE_CONFIG_READ_FUNC(TebRobot)
	READ_CONFIG_MEMBER(max_vel_x)
	READ_CONFIG_MEMBER(min_vel_x)
	READ_CONFIG_MEMBER(max_vel_x_backwards)
	READ_CONFIG_MEMBER(max_vel_y)
	READ_CONFIG_MEMBER(max_vel_theta)
	READ_CONFIG_MEMBER(min_vel_theta)
	READ_CONFIG_MEMBER(acc_lim_x)
	READ_CONFIG_MEMBER(acc_lim_y)
	READ_CONFIG_MEMBER(acc_lim_theta)
	READ_CONFIG_MEMBER(min_turning_radius)
	READ_CONFIG_MEMBER(wheelbase)
	READ_CONFIG_MEMBER(cmd_angle_instead_rotvel)
	READ_CONFIG_MEMBER(is_footprint_dynamic)
	READ_CONFIG_MEMBER(use_proportional_saturation)
END_CONFIG_READ_FUNC(TebRobot)

DEFINE_CONFIG_TYPE(CONFIG_TEBROBOT, TebRobot);

DEFINE_CONFIG_READ_FUNC(TebGoalTolerance)
	READ_CONFIG_MEMBER(yaw_goal_tolerance)
	READ_CONFIG_MEMBER(xy_goal_tolerance)
	READ_CONFIG_MEMBER(free_goal_vel)
	READ_CONFIG_MEMBER(complete_global_plan)
END_CONFIG_READ_FUNC(TebGoalTolerance)

DEFINE_CONFIG_TYPE(CONFIG_GOALTOLERANCE, TebGoalTolerance);

DEFINE_CONFIG_READ_FUNC(TebObstacle)
	READ_CONFIG_MEMBER(min_obstacle_dist)
	READ_CONFIG_MEMBER(inflation_dist)
	READ_CONFIG_MEMBER(dynamic_obstacle_inflation_dist)
	READ_CONFIG_MEMBER(include_dynamic_obstacles)
	READ_CONFIG_MEMBER(include_costmap_obstacles)
	READ_CONFIG_MEMBER(costmap_obstacles_behind_robot_dist)
	READ_CONFIG_MEMBER(obstacle_poses_affected)
	READ_CONFIG_MEMBER(legacy_obstacle_association)
	READ_CONFIG_MEMBER(obstacle_association_force_inclusion_factor)
	READ_CONFIG_MEMBER(obstacle_association_cutoff_factor)
	READ_CONFIG_MEMBER(obstacle_angle_increasment)
	READ_CONFIG_MEMBER(obstacle_proximity_ratio_max_vel)
	READ_CONFIG_MEMBER(obstacle_proximity_lower_bound)
	READ_CONFIG_MEMBER(obstacle_proximity_upper_bound)
	READ_CONFIG_MEMBER(use_sdf_map)
END_CONFIG_READ_FUNC(TebObstacle)

DEFINE_CONFIG_TYPE(CONFIG_OBSTACLE, TebObstacle);


DEFINE_CONFIG_READ_FUNC(TebOptimization)
	READ_CONFIG_MEMBER(no_inner_iterations)
	READ_CONFIG_MEMBER(no_outer_iterations)
	READ_CONFIG_MEMBER(optimization_activate)
	READ_CONFIG_MEMBER(optimization_verbose)
	READ_CONFIG_MEMBER(penalty_epsilon)
	READ_CONFIG_MEMBER(weight_max_vel_x)
	READ_CONFIG_MEMBER(weight_max_vel_y)
	READ_CONFIG_MEMBER(weight_max_vel_theta)
	READ_CONFIG_MEMBER(weight_acc_lim_x)
	READ_CONFIG_MEMBER(weight_acc_lim_y)
	READ_CONFIG_MEMBER(weight_acc_lim_theta)
	READ_CONFIG_MEMBER(weight_kinematics_nh)
	READ_CONFIG_MEMBER(weight_kinematics_forward_drive)
	READ_CONFIG_MEMBER(weight_kinematics_turning_radius)
	READ_CONFIG_MEMBER(weight_optimaltime)
	READ_CONFIG_MEMBER(weight_shortest_path)
	READ_CONFIG_MEMBER(weight_obstacle)
	READ_CONFIG_MEMBER(weight_inflation)
	READ_CONFIG_MEMBER(weight_dynamic_obstacle)
	READ_CONFIG_MEMBER(weight_dynamic_obstacle_inflation)
	READ_CONFIG_MEMBER(weight_viapoint)
	READ_CONFIG_MEMBER(weight_prefer_rotdir)
	READ_CONFIG_MEMBER(weight_adapt_factor)
	READ_CONFIG_MEMBER(obstacle_cost_exponent)
	READ_CONFIG_MEMBER(weight_velocity_obstacle_ratio)
END_CONFIG_READ_FUNC(TebOptimization)

DEFINE_CONFIG_TYPE(CONFIG_OPTIMIZATION, TebOptimization);

DEFINE_CONFIG_READ_FUNC(TebHomotopyClasses)
	READ_CONFIG_MEMBER(enable_homotopy_class_planning)
	READ_CONFIG_MEMBER(enable_multithreading)
	READ_CONFIG_MEMBER(simple_exploration)
	READ_CONFIG_MEMBER(max_number_classes)
	READ_CONFIG_MEMBER(selection_enable_same_orientation)
	READ_CONFIG_MEMBER(selection_same_orientation_tolerance)
	READ_CONFIG_MEMBER(selection_same_orientation_scale)
	READ_CONFIG_MEMBER(selection_cost_hysteresis)
	READ_CONFIG_MEMBER(selection_prefer_initial_plan)
	READ_CONFIG_MEMBER(selection_obst_cost_scale)
	READ_CONFIG_MEMBER(selection_viapoint_cost_scale)
	READ_CONFIG_MEMBER(selection_alternative_time_cost)
	READ_CONFIG_MEMBER(switching_blocking_period)
	READ_CONFIG_MEMBER(roadmap_graph_no_samples)
	READ_CONFIG_MEMBER(roadmap_graph_area_width)
	READ_CONFIG_MEMBER(roadmap_graph_area_length_scale)
	READ_CONFIG_MEMBER(h_signature_prescaler)
	READ_CONFIG_MEMBER(h_signature_threshold)
	READ_CONFIG_MEMBER(obstacle_keypoint_offset)
	READ_CONFIG_MEMBER(obstacle_heading_threshold)
	READ_CONFIG_MEMBER(viapoints_all_candidates)
	READ_CONFIG_MEMBER(visualize_hc_graph)
	READ_CONFIG_MEMBER(visualize_with_time_as_z_axis_scale)
	READ_CONFIG_MEMBER(delete_detours_backwards)
	READ_CONFIG_MEMBER(length_start_orientation_vector)
	READ_CONFIG_MEMBER(max_ratio_detours_duration_best_duration)
END_CONFIG_READ_FUNC(TebHomotopyClasses)

DEFINE_CONFIG_TYPE(CONFIG_HC, TebHomotopyClasses);


DEFINE_CONFIG_READ_FUNC(TebRecovery)
	READ_CONFIG_MEMBER(shrink_horizon_backup)
	READ_CONFIG_MEMBER(shrink_horizon_min_duration)
	READ_CONFIG_MEMBER(oscillation_recovery)
	READ_CONFIG_MEMBER(oscillation_v_eps)
	READ_CONFIG_MEMBER(oscillation_omega_eps)
	READ_CONFIG_MEMBER(oscillation_recovery_min_duration)
	READ_CONFIG_MEMBER(oscillation_filter_duration)
END_CONFIG_READ_FUNC(TebRecovery)

DEFINE_CONFIG_TYPE(CONFIG_RECOVERY, TebRecovery);

DEFINE_CONFIG_READ_FUNC(TebFootstr)
	READ_CONFIG_MEMBER(foot_str)
END_CONFIG_READ_FUNC(TebFootstr)

DEFINE_CONFIG_TYPE(CONFIG_FOOTSTR, TebFootstr);

DEFINE_CONFIG_READ_FUNC(Teblog)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(planning_frequency);
END_CONFIG_READ_FUNC(Teblog)

DEFINE_CONFIG_TYPE(CONFIG_TEBLOG, Teblog);

DEFINE_CONFIG_READ_FUNC(TebTurnPid)
    READ_CONFIG_MEMBER(turn_p);
    READ_CONFIG_MEMBER(turn_i);
    READ_CONFIG_MEMBER(turn_d);
    READ_CONFIG_MEMBER(turn_scale);
    READ_CONFIG_MEMBER(turn_min_error);
    READ_CONFIG_MEMBER(turn_min_output);
    READ_CONFIG_MEMBER(turn_max_output);
    READ_CONFIG_MEMBER(turn_error_inc);
    READ_CONFIG_MEMBER(turn_continuous_angle);
END_CONFIG_READ_FUNC(TebTurnPid)

DEFINE_CONFIG_TYPE(CONFIG_TEBTURNPID, TebTurnPid);

#define PRINT_INIT_PARAMETERS 1

void TebConfig::init(ConfigManager& cfg_mgr)
{ 
    ConfigTebTrajectory *trajectory_ptr
      = dynamic_cast<ConfigTebTrajectory*>(cfg_mgr.GetSubConfig(CONFIG_TRAJECTORY));
    ConfigTebRobot * robot_ptr 
      = dynamic_cast<ConfigTebRobot*>(cfg_mgr.GetSubConfig(CONFIG_TEBROBOT));
    ConfigTebGoalTolerance * goal_tolerance_ptr 
      = dynamic_cast<ConfigTebGoalTolerance*>(cfg_mgr.GetSubConfig(CONFIG_GOALTOLERANCE));
    ConfigTebObstacle * obstacles_ptr 
      = dynamic_cast<ConfigTebObstacle*>(cfg_mgr.GetSubConfig(CONFIG_OBSTACLE));
    ConfigTebOptimization * optim_ptr 
      = dynamic_cast<ConfigTebOptimization*>(cfg_mgr.GetSubConfig(CONFIG_OPTIMIZATION));
    ConfigTebHomotopyClasses * hcp_ptr 
      = dynamic_cast<ConfigTebHomotopyClasses*>(cfg_mgr.GetSubConfig(CONFIG_HC));
    ConfigTebRecovery * recovery_ptr 
      = dynamic_cast<ConfigTebRecovery*>(cfg_mgr.GetSubConfig(CONFIG_RECOVERY));
    ConfigTebFootstr * footprint_ptr 
      = dynamic_cast<ConfigTebFootstr*>(cfg_mgr.GetSubConfig(CONFIG_FOOTSTR));




    // Footprint
    if(footprint_ptr)
    {

      footprint.foot_str = footprint_ptr->foot_str;
      #ifdef PRINT_INIT_PARAMETERS
      printf("footprint_ptr ok:%s\r\n",footprint.foot_str.c_str());
      #endif
    }
    else
    {
      footprint.foot_str = "[[0.17, 0.17], [-0.17, 0.17], [-0.17,-0.17],[0.17,-0.17]]";
      #ifdef PRINT_INIT_PARAMETERS
      printf("footprint_ptr not ok\r\n");
      #endif
    }

    // Trajectory
    if(trajectory_ptr)
    {

      trajectory.teb_autosize = trajectory_ptr->teb_autosize;
      trajectory.dt_ref = trajectory_ptr->dt_ref;
      trajectory.dt_hysteresis = trajectory_ptr->dt_hysteresis;
      trajectory.min_samples = trajectory_ptr->min_samples;
      trajectory.max_samples = trajectory_ptr->max_samples;
      trajectory.global_plan_overwrite_orientation = trajectory_ptr->global_plan_overwrite_orientation;
      trajectory.no_viapoint_idx = trajectory_ptr->no_viapoint_idx;
      trajectory.allow_init_with_backwards_motion = trajectory_ptr->allow_init_with_backwards_motion;
      trajectory.global_plan_viapoint_sep = trajectory_ptr->global_plan_viapoint_sep;
      trajectory.via_points_ordered = trajectory_ptr->via_points_ordered;
      trajectory.max_global_plan_lookahead_dist = trajectory_ptr->max_global_plan_lookahead_dist;
      trajectory.global_plan_prune_distance = trajectory_ptr->global_plan_prune_distance;
      trajectory.exact_arc_length = trajectory_ptr->exact_arc_length;
      trajectory.feasibility_check_no_poses = trajectory_ptr->feasibility_check_no_poses;
      trajectory.publish_feedback = trajectory_ptr->publish_feedback;
      trajectory.control_look_ahead_poses = trajectory_ptr->control_look_ahead_poses;;

      // printf("trajectory_ptr ok:\r\n");
      // printf("teb_autosize:%f\r\n",trajectory_ptr->teb_autosize);
      // printf("dt_ref:%f\r\n",trajectory_ptr->dt_ref);
      // printf("dt_hysteresis:%f\r\n",trajectory_ptr->dt_hysteresis);
      // printf("min_samples:%d\r\n",trajectory_ptr->min_samples);
      // printf("max_samples:%d\r\n",trajectory_ptr->max_samples);
      // printf("global_plan_overwrite_orientation:%d\r\n",trajectory_ptr->global_plan_overwrite_orientation);
      // printf("allow_init_with_backwards_motion:%d\r\n",trajectory_ptr->allow_init_with_backwards_motion);
      // printf("global_plan_viapoint_sep:%f\r\n",trajectory_ptr->global_plan_viapoint_sep);
      // printf("via_points_ordered:%d\r\n",trajectory_ptr->via_points_ordered);
      // printf("max_global_plan_lookahead_dist:%f\r\n",trajectory_ptr->max_global_plan_lookahead_dist);
      // printf("global_plan_prune_distance:%f\r\n",trajectory_ptr->global_plan_prune_distance);
      // printf("exact_arc_length:%d\r\n",trajectory_ptr->exact_arc_length);
      // printf("feasibility_check_no_poses:%d\r\n",trajectory_ptr->feasibility_check_no_poses);
      // printf("publish_feedback:%d\r\n",trajectory_ptr->publish_feedback);



    }
    else
    {
      printf("trajectory_ptr not ok\r\n");
      trajectory.teb_autosize = 1;
      trajectory.dt_ref = 0.3;
      trajectory.dt_hysteresis = 0.1;
      trajectory.min_samples = 3;
      trajectory.max_samples = 500;
      trajectory.global_plan_overwrite_orientation = 1;
      trajectory.allow_init_with_backwards_motion = 0;
      trajectory.global_plan_viapoint_sep = -1;
      trajectory.no_viapoint_idx = 0;
      trajectory.via_points_ordered = 0;
      trajectory.max_global_plan_lookahead_dist = 1;
      trajectory.global_plan_prune_distance = 1;
      trajectory.exact_arc_length = 0;
      trajectory.force_reinit_new_goal_dist = 1;
      trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
      trajectory.feasibility_check_no_poses = 5;
      trajectory.publish_feedback = 0;
      trajectory.min_resolution_collision_check_angular = M_PI;
      trajectory.control_look_ahead_poses = 1;


    }
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.min_resolution_collision_check_angular = M_PI;
    

    // Robot
    if(robot_ptr)
    {

      robot.max_vel_x = robot_ptr->max_vel_x;
      robot.min_vel_x = robot_ptr->min_vel_x;
      robot.max_vel_x_backwards = robot_ptr->max_vel_x_backwards;
      robot.max_vel_y = robot_ptr->max_vel_y;
      robot.max_vel_theta = robot_ptr->max_vel_theta;
      robot.min_vel_theta = robot_ptr->min_vel_theta;
      robot.acc_lim_x = robot_ptr->acc_lim_x;
      robot.acc_lim_y = robot_ptr->acc_lim_y;
      robot.acc_lim_theta = robot_ptr->acc_lim_theta;
      robot.min_turning_radius = robot_ptr->min_turning_radius;
      robot.wheelbase = robot_ptr->wheelbase;
      robot.cmd_angle_instead_rotvel = robot_ptr->cmd_angle_instead_rotvel;
      robot.is_footprint_dynamic = robot_ptr->is_footprint_dynamic;
      robot.use_proportional_saturation = robot_ptr->use_proportional_saturation;
      // printf("robot_ptr ok\r\n");
      // printf("max_vel_x:%f\r\n",robot_ptr->max_vel_x);
      // printf("max_vel_x_backwards:%f\r\n",robot_ptr->max_vel_x_backwards);
      // printf("max_vel_y:%f\r\n",robot_ptr->max_vel_y);
      // printf("max_vel_theta:%f\r\n",robot_ptr->max_vel_theta);
      // printf("min_vel_theta:%f\r\n",robot_ptr->min_vel_theta);
      // printf("acc_lim_x:%f\r\n",robot_ptr->acc_lim_x);
      // printf("acc_lim_y:%f\r\n",robot_ptr->acc_lim_y);
      // printf("acc_lim_theta:%f\r\n",robot_ptr->acc_lim_theta);
      // printf("min_turning_radius:%f\r\n",robot_ptr->min_turning_radius);
      // printf("wheelbase:%f\r\n",robot_ptr->wheelbase);
      // printf("cmd_angle_instead_rotvel:%d\r\n",robot_ptr->cmd_angle_instead_rotvel);
      // printf("is_footprint_dynamic:%d\r\n",robot_ptr->is_footprint_dynamic);
      // printf("use_proportional_saturation:%d\r\n",robot_ptr->use_proportional_saturation);

    }
    else
    {
      printf("robot_ptr not ok\r\n");
      robot.max_vel_x = 0.3;
      robot.min_vel_x = 0.07;
      robot.max_vel_x_backwards = 0.2;
      robot.max_vel_y = 0.0;
      robot.max_vel_theta = 0.3;
      robot.min_vel_theta = 0.05;     
      robot.acc_lim_x = 0.5;
      robot.acc_lim_y = 0.5;
      robot.acc_lim_theta = 0.5;
      robot.min_turning_radius = 0;
      robot.wheelbase = 1.0;
      robot.cmd_angle_instead_rotvel = 0;
      robot.is_footprint_dynamic = 0;
      robot.use_proportional_saturation = false;
    }

    // GoalTolerance
    if(goal_tolerance_ptr)
    {
      goal_tolerance.xy_goal_tolerance = goal_tolerance_ptr->xy_goal_tolerance;
      goal_tolerance.yaw_goal_tolerance = goal_tolerance_ptr->yaw_goal_tolerance;
      goal_tolerance.free_goal_vel = goal_tolerance_ptr->free_goal_vel;
      goal_tolerance.complete_global_plan = goal_tolerance_ptr->complete_global_plan;
      
      // printf("goal_tolerance_ptr ok\r\n");

      // printf("xy_goal_tolerance:%f\r\n",goal_tolerance_ptr->xy_goal_tolerance);
      // printf("yaw_goal_tolerance:%f\r\n",goal_tolerance_ptr->yaw_goal_tolerance);
      // printf("free_goal_vel:%d\r\n",goal_tolerance_ptr->free_goal_vel);
      // printf("complete_global_plan:%d\r\n",goal_tolerance_ptr->complete_global_plan);

    }
    else
    {
      printf("goal_tolerance_ptr not ok\r\n");
      goal_tolerance.xy_goal_tolerance = 0.2;
      goal_tolerance.yaw_goal_tolerance = 0.2;
      goal_tolerance.free_goal_vel = 0;
      goal_tolerance.complete_global_plan = 1;
    }

    // Obstacles
    if(obstacles_ptr)
    {
      obstacles.min_obstacle_dist = obstacles_ptr->min_obstacle_dist;
      obstacles.inflation_dist = obstacles_ptr->inflation_dist;
      obstacles.dynamic_obstacle_inflation_dist = obstacles_ptr->dynamic_obstacle_inflation_dist;
      obstacles.include_dynamic_obstacles = obstacles_ptr->include_dynamic_obstacles;
      obstacles.include_costmap_obstacles = obstacles_ptr->include_costmap_obstacles;
      obstacles.costmap_obstacles_behind_robot_dist = obstacles_ptr->costmap_obstacles_behind_robot_dist;
      obstacles.obstacle_poses_affected = obstacles_ptr->obstacle_poses_affected;
      obstacles.legacy_obstacle_association = obstacles_ptr->legacy_obstacle_association;
      obstacles.obstacle_association_force_inclusion_factor = obstacles_ptr->obstacle_association_force_inclusion_factor;
      obstacles.obstacle_association_cutoff_factor = obstacles_ptr->obstacle_association_cutoff_factor;
      obstacles.obstacle_angle_increasment = obstacles_ptr->obstacle_angle_increasment;
      obstacles.obstacle_proximity_ratio_max_vel = obstacles_ptr->obstacle_proximity_ratio_max_vel;
      obstacles.obstacle_proximity_lower_bound = obstacles_ptr->obstacle_proximity_lower_bound;
      obstacles.obstacle_proximity_upper_bound = obstacles_ptr->obstacle_proximity_upper_bound;
      obstacles.use_sdf_map = obstacles_ptr->use_sdf_map;
      // printf("obstacles_ptr ok\r\n");
      // printf("min_obstacle_dist:%f\r\n",obstacles_ptr->min_obstacle_dist);
      // printf("inflation_dist:%f\r\n",obstacles_ptr->inflation_dist);
      // printf("dynamic_obstacle_inflation_dist:%f\r\n",obstacles_ptr->dynamic_obstacle_inflation_dist);
      // printf("include_dynamic_obstacles:%d\r\n",obstacles_ptr->include_dynamic_obstacles);
      // printf("include_costmap_obstacles:%d\r\n",obstacles_ptr->include_costmap_obstacles);
      // printf("costmap_obstacles_behind_robot_dist:%f\r\n",obstacles_ptr->costmap_obstacles_behind_robot_dist);
      // printf("obstacle_poses_affected:%d\r\n",obstacles_ptr->obstacle_poses_affected);
      // printf("legacy_obstacle_association:%d\r\n",obstacles_ptr->legacy_obstacle_association);
      // printf("obstacle_association_force_inclusion_factor:%f\r\n",obstacles_ptr->obstacle_association_force_inclusion_factor);
      // printf("obstacle_association_cutoff_factor:%f\r\n",obstacles_ptr->obstacle_association_cutoff_factor);
      // printf("obstacle_proximity_ratio_max_vel:%d\r\n",obstacles_ptr->obstacle_proximity_ratio_max_vel);
      // printf("obstacle_proximity_lower_bound:%f\r\n",obstacles_ptr->obstacle_proximity_lower_bound);
      // printf("obstacle_proximity_upper_bound:%f\r\n",obstacles_ptr->obstacle_proximity_upper_bound);
    
    
    }
    else
    {
      printf("obstacles_ptr not ok\r\n");
      obstacles.min_obstacle_dist = 0.5;
      obstacles.inflation_dist = 0.6;
      obstacles.dynamic_obstacle_inflation_dist = 0.6;
      obstacles.include_dynamic_obstacles = 1;
      obstacles.include_costmap_obstacles = 1;
      obstacles.costmap_obstacles_behind_robot_dist = 1.5;
      obstacles.obstacle_poses_affected = 25;
      obstacles.legacy_obstacle_association = 0;
      obstacles.obstacle_association_force_inclusion_factor = 1.5;
      obstacles.obstacle_association_cutoff_factor = 5;
      obstacles.obstacle_proximity_ratio_max_vel = 1;
      obstacles.obstacle_proximity_lower_bound = 0;
      obstacles.obstacle_proximity_upper_bound = 0.5;
      obstacles.use_sdf_map = 0;
    }

    // Optimization
    if(optim_ptr)
    {
      optim.no_inner_iterations = optim_ptr->no_inner_iterations;
      optim.no_outer_iterations = optim_ptr->no_outer_iterations;
      optim.optimization_activate = optim_ptr->optimization_activate;
      optim.optimization_verbose = optim_ptr->optimization_verbose;
      optim.penalty_epsilon = optim_ptr->penalty_epsilon;
      optim.weight_max_vel_x = optim_ptr->weight_max_vel_x;
      optim.weight_max_vel_y = optim_ptr->weight_max_vel_y;
      optim.weight_max_vel_theta = optim_ptr->weight_max_vel_theta;
      optim.weight_acc_lim_x = optim_ptr->weight_acc_lim_x;
      optim.weight_acc_lim_y = optim_ptr->weight_acc_lim_y;
      optim.weight_acc_lim_theta = optim_ptr->weight_acc_lim_theta;
      optim.weight_kinematics_nh = optim_ptr->weight_kinematics_nh;
      optim.weight_kinematics_forward_drive = optim_ptr->weight_kinematics_forward_drive;
      optim.weight_kinematics_turning_radius = optim_ptr->weight_kinematics_turning_radius;
      optim.weight_optimaltime = optim_ptr->weight_optimaltime;
      optim.weight_shortest_path = optim_ptr->weight_shortest_path;
      optim.weight_obstacle = optim_ptr->weight_obstacle;
      optim.weight_inflation = optim_ptr->weight_inflation;
      optim.weight_dynamic_obstacle = optim_ptr->weight_dynamic_obstacle;
      optim.weight_dynamic_obstacle_inflation = optim_ptr->weight_dynamic_obstacle_inflation;
      optim.weight_viapoint = optim_ptr->weight_viapoint;
      optim.weight_prefer_rotdir = optim_ptr->weight_prefer_rotdir;
      optim.weight_adapt_factor = optim_ptr->weight_adapt_factor;
      optim.obstacle_cost_exponent = optim_ptr->obstacle_cost_exponent;
      optim.weight_velocity_obstacle_ratio = optim_ptr->weight_velocity_obstacle_ratio;
      // printf("optim_ptr ok\r\n");

      // printf("no_inner_iterations:%d\r\n",optim_ptr->no_inner_iterations);
      // printf("no_outer_iterations:%d\r\n",optim_ptr->no_outer_iterations);
      // printf("optimization_activate:%d\r\n",optim_ptr->optimization_activate);
      // printf("optimization_verbose:%d\r\n",optim_ptr->optimization_verbose);
      // printf("penalty_epsilon:%f\r\n",optim_ptr->penalty_epsilon);
      // printf("weight_max_vel_x:%f\r\n",optim_ptr->weight_max_vel_x);
      // printf("weight_max_vel_y:%f\r\n",optim_ptr->weight_max_vel_y);
      // printf("weight_max_vel_theta:%f\r\n",optim_ptr->weight_max_vel_theta);
      // printf("weight_acc_lim_x:%f\r\n",optim_ptr->weight_acc_lim_x);
      // printf("weight_acc_lim_y:%f\r\n",optim_ptr->weight_acc_lim_y);
      // printf("weight_acc_lim_theta:%f\r\n",optim_ptr->weight_acc_lim_theta);
      // printf("weight_kinematics_nh:%f\r\n",optim_ptr->weight_kinematics_nh);
      // printf("weight_kinematics_forward_drive:%f\r\n",optim_ptr->weight_kinematics_forward_drive);
      // printf("weight_kinematics_turning_radius:%f\r\n",optim_ptr->weight_kinematics_turning_radius);
      // printf("weight_optimaltime:%f\r\n",optim_ptr->weight_optimaltime);
      // printf("weight_shortest_path:%f\r\n",optim_ptr->weight_shortest_path);
      // printf("weight_obstacle:%f\r\n",optim_ptr->weight_obstacle);
      // printf("weight_inflation:%f\r\n",optim_ptr->weight_inflation);
      // printf("weight_dynamic_obstacle:%f\r\n",optim_ptr->weight_dynamic_obstacle);
      // printf("weight_dynamic_obstacle_inflation:%f\r\n",optim_ptr->weight_dynamic_obstacle_inflation);
      // printf("weight_viapoint:%f\r\n",optim_ptr->weight_viapoint);
      // printf("weight_prefer_rotdir:%f\r\n",optim_ptr->weight_prefer_rotdir);
      // printf("weight_adapt_factor:%f\r\n",optim_ptr->weight_adapt_factor);
      // printf("obstacle_cost_exponent:%f\r\n",optim_ptr->obstacle_cost_exponent);
      // printf("weight_velocity_obstacle_ratio:%f\r\n",optim_ptr->weight_velocity_obstacle_ratio);

    }
    else
    {
      printf("optim_ptr not ok\r\n");
      optim.no_inner_iterations = 3;
      optim.no_outer_iterations = 2;
      optim.optimization_activate = 1;
      optim.optimization_verbose = 0;
      optim.penalty_epsilon = 0.05;
      optim.weight_max_vel_x = 2; //1
      optim.weight_max_vel_y = 0;
      optim.weight_max_vel_theta = 1;
      optim.weight_acc_lim_x = 1;
      optim.weight_acc_lim_y = 0;
      optim.weight_acc_lim_theta = 1;
      optim.weight_kinematics_nh = 1000;
      optim.weight_kinematics_forward_drive = 1;
      optim.weight_kinematics_turning_radius = 1;
      optim.weight_optimaltime = 1;
      optim.weight_shortest_path = 0;
      optim.weight_obstacle = 50;
      optim.weight_inflation = 0.1;
      optim.weight_dynamic_obstacle = 50;
      optim.weight_dynamic_obstacle_inflation = 0.1;
      optim.weight_viapoint = 1;
      optim.weight_prefer_rotdir = 50;

      optim.weight_adapt_factor = 2.0;
      optim.obstacle_cost_exponent = 1.0;
      optim.weight_velocity_obstacle_ratio = 0;
    }

    // Homotopy Class Planner
    if(hcp_ptr)
    {
      hcp.enable_homotopy_class_planning = hcp_ptr->enable_homotopy_class_planning;
      hcp.enable_multithreading = hcp_ptr->enable_multithreading;
      hcp.simple_exploration = hcp_ptr->simple_exploration;
      hcp.max_number_classes = hcp_ptr->max_number_classes;
      hcp.selection_enable_same_orientation = hcp_ptr->selection_enable_same_orientation;
      hcp.selection_same_orientation_tolerance = hcp_ptr->selection_same_orientation_tolerance;
      hcp.selection_same_orientation_scale = hcp_ptr->selection_same_orientation_scale;
      hcp.selection_cost_hysteresis = hcp_ptr->selection_cost_hysteresis;
      hcp.selection_prefer_initial_plan = hcp_ptr->selection_prefer_initial_plan;
      hcp.selection_obst_cost_scale = hcp_ptr->selection_obst_cost_scale;
      hcp.selection_viapoint_cost_scale = hcp_ptr->selection_viapoint_cost_scale;
      hcp.selection_alternative_time_cost = hcp_ptr->selection_alternative_time_cost;
      hcp.switching_blocking_period = hcp_ptr->switching_blocking_period;
      hcp.roadmap_graph_no_samples = hcp_ptr->roadmap_graph_no_samples;
      hcp.roadmap_graph_area_width = hcp_ptr->roadmap_graph_area_width;
      hcp.roadmap_graph_area_length_scale = hcp_ptr->roadmap_graph_area_length_scale;
      hcp.h_signature_prescaler = hcp_ptr->h_signature_prescaler;
      hcp.h_signature_threshold = hcp_ptr->h_signature_threshold;
      hcp.obstacle_keypoint_offset = hcp_ptr->obstacle_keypoint_offset;
      hcp.obstacle_heading_threshold = hcp_ptr->obstacle_heading_threshold;
      hcp.viapoints_all_candidates = hcp_ptr->viapoints_all_candidates;
      hcp.visualize_hc_graph = hcp_ptr->visualize_hc_graph;
      hcp.visualize_with_time_as_z_axis_scale = hcp_ptr->visualize_with_time_as_z_axis_scale;
      hcp.delete_detours_backwards = hcp_ptr->delete_detours_backwards;
      hcp.length_start_orientation_vector = hcp_ptr->length_start_orientation_vector;
      hcp.max_ratio_detours_duration_best_duration = hcp_ptr->max_ratio_detours_duration_best_duration;

      printf("hcp_ptr ok\r\n");
      printf("enable_homotopy_class_planning:%d\r\n",hcp_ptr->enable_homotopy_class_planning);
      printf("enable_multithreading:%d\r\n",hcp_ptr->enable_multithreading);
      printf("simple_exploration:%d\r\n",hcp_ptr->simple_exploration);
      printf("max_number_classes:%d\r\n",hcp_ptr->max_number_classes);
      printf("selection_cost_hysteresis:%f\r\n",hcp_ptr->selection_cost_hysteresis);
      printf("selection_prefer_initial_plan:%f\r\n",hcp_ptr->selection_prefer_initial_plan);
      printf("selection_obst_cost_scale:%f\r\n",hcp_ptr->selection_obst_cost_scale);
      printf("selection_viapoint_cost_scale:%f\r\n",hcp_ptr->selection_viapoint_cost_scale);
      printf("selection_alternative_time_cost:%d\r\n",hcp_ptr->selection_alternative_time_cost);
      printf("switching_blocking_period:%f\r\n",hcp_ptr->switching_blocking_period);
      printf("roadmap_graph_no_samples:%d\r\n",hcp_ptr->roadmap_graph_no_samples);
      printf("roadmap_graph_area_width:%f\r\n",hcp_ptr->roadmap_graph_area_width);
      printf("roadmap_graph_area_length_scale:%f\r\n",hcp_ptr->roadmap_graph_area_length_scale);
      printf("h_signature_prescaler:%f\r\n",hcp_ptr->h_signature_prescaler);
      printf("h_signature_threshold:%f\r\n",hcp_ptr->h_signature_threshold);
      printf("obstacle_keypoint_offset:%f\r\n",hcp_ptr->obstacle_keypoint_offset);
      printf("obstacle_heading_threshold:%f\r\n",hcp_ptr->obstacle_heading_threshold);
      printf("viapoints_all_candidates:%d\r\n",hcp_ptr->viapoints_all_candidates);
      printf("visualize_hc_graph:%d\r\n",hcp_ptr->visualize_hc_graph);
      printf("visualize_with_time_as_z_axis_scale:%f\r\n",hcp_ptr->visualize_with_time_as_z_axis_scale);
      printf("delete_detours_backwards:%d\r\n",hcp_ptr->delete_detours_backwards);
      printf("length_start_orientation_vector:%f\r\n",hcp_ptr->length_start_orientation_vector);
      printf("max_ratio_detours_duration_best_duration:%f\r\n",hcp_ptr->max_ratio_detours_duration_best_duration);

    }
    else
    {
      printf("hcp_ptr not ok\r\n");
      hcp.enable_homotopy_class_planning = 1;
      hcp.enable_multithreading = 1;
      hcp.simple_exploration = 0;
      hcp.max_number_classes = 4;
      hcp.selection_enable_same_orientation = 0;
      hcp.selection_same_orientation_tolerance = 0.785;
      hcp.selection_same_orientation_scale = 0.9;

      hcp.selection_cost_hysteresis = 1.0;
      hcp.selection_prefer_initial_plan = 0.9;
      hcp.selection_obst_cost_scale = 100.0;
      hcp.selection_viapoint_cost_scale = 1.0;
      hcp.selection_alternative_time_cost = 0;

      hcp.obstacle_keypoint_offset = 0.1;
      hcp.obstacle_heading_threshold = 0.45;
      hcp.roadmap_graph_no_samples = 15;
      hcp.roadmap_graph_area_width = 5; // [m]
      hcp.roadmap_graph_area_length_scale = 1.0;
      hcp.h_signature_prescaler = 0.5;
      hcp.h_signature_threshold = 0.1;
      hcp.switching_blocking_period = 0.0;

      hcp.viapoints_all_candidates = 1;

      hcp.visualize_hc_graph = 0;
      hcp.visualize_with_time_as_z_axis_scale = 0;
      hcp.delete_detours_backwards = 1;
      hcp.detours_orientation_tolerance = M_PI / 2.0;
      hcp.length_start_orientation_vector = 0.4;
      hcp.max_ratio_detours_duration_best_duration = 3.0;
    }
    hcp.detours_orientation_tolerance = M_PI / 2.0;

    // Recovery
    if(recovery_ptr)
    {
      recovery.shrink_horizon_backup = recovery_ptr->shrink_horizon_backup;
      recovery.shrink_horizon_min_duration = recovery_ptr->shrink_horizon_min_duration;
      recovery.oscillation_recovery = recovery_ptr->oscillation_recovery;
      recovery.oscillation_v_eps = recovery_ptr->oscillation_v_eps;
      recovery.oscillation_omega_eps = recovery_ptr->oscillation_omega_eps;
      recovery.oscillation_recovery_min_duration = recovery_ptr->oscillation_recovery_min_duration;
      recovery.oscillation_filter_duration = recovery_ptr->oscillation_filter_duration;

      // printf("recovery_ptr ok\r\n");
      // printf("shrink_horizon_backup:%d\r\n",recovery_ptr->shrink_horizon_backup);
      // printf("shrink_horizon_min_duration:%f\r\n",recovery_ptr->shrink_horizon_min_duration);
      // printf("oscillation_recovery:%d\r\n",recovery_ptr->oscillation_recovery);
      // printf("oscillation_v_eps:%f\r\n",recovery_ptr->oscillation_v_eps);
      // printf("oscillation_omega_eps:%f\r\n",recovery_ptr->oscillation_omega_eps);
      // printf("oscillation_recovery_min_duration:%f\r\n",recovery_ptr->oscillation_recovery_min_duration);
      // printf("oscillation_filter_duration:%f\r\n",recovery_ptr->oscillation_filter_duration);

    }
    else
    {
      printf("recovery_ptr not ok\r\n");
      recovery.shrink_horizon_backup = 1;
      recovery.shrink_horizon_min_duration = 10;
      recovery.oscillation_recovery = 1;
      recovery.oscillation_v_eps = 0.1;
      recovery.oscillation_omega_eps = 0.1;
      recovery.oscillation_recovery_min_duration = 10;
      recovery.oscillation_filter_duration = 10;
    }




    // trajectory.teb_autosize = true;
    // trajectory.dt_ref = 0.3;
    // trajectory.dt_hysteresis = 0.1;
    // trajectory.min_samples = 3;
    // trajectory.max_samples = 500;
    // trajectory.global_plan_overwrite_orientation = true;
    // trajectory.allow_init_with_backwards_motion = false;
    // trajectory.global_plan_viapoint_sep = -1;
    // trajectory.via_points_ordered = false;
    // trajectory.max_global_plan_lookahead_dist = 1;
    // trajectory.global_plan_prune_distance = 1;
    // trajectory.exact_arc_length = false;
    // trajectory.force_reinit_new_goal_dist = 1;
    // trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    // trajectory.feasibility_check_no_poses = 5;
    // trajectory.publish_feedback = false;
    // trajectory.min_resolution_collision_check_angular = M_PI;
    // trajectory.control_look_ahead_poses = 2;
    
    // // Robot

    // robot.max_vel_x = 0.4;
    // robot.max_vel_x_backwards = 0.2;
    // robot.max_vel_y = 0.0;
    // robot.max_vel_theta = 0.5;
    // robot.min_vel_theta = 0.05;    
    // robot.acc_lim_x = 0.5;
    // robot.acc_lim_y = 0.5;
    // robot.acc_lim_theta = 0.5;
    // robot.min_turning_radius = 0;
    // robot.wheelbase = 1.0;
    // robot.cmd_angle_instead_rotvel = false;
    // robot.is_footprint_dynamic = false;

    // // GoalTolerance

    // goal_tolerance.xy_goal_tolerance = 0.05;  //0.2
    // goal_tolerance.yaw_goal_tolerance = 0.06;  //0.1
    // goal_tolerance.free_goal_vel = false;
    // goal_tolerance.complete_global_plan = true;

    // // Obstacles

    // obstacles.min_obstacle_dist = 0.5;
    // obstacles.inflation_dist = 0.6;
    // obstacles.dynamic_obstacle_inflation_dist = 0.6;
    // obstacles.include_dynamic_obstacles = false;
    // obstacles.include_costmap_obstacles = true;
    // obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    // obstacles.obstacle_poses_affected = 25;
    // obstacles.legacy_obstacle_association = false;
    // obstacles.obstacle_association_force_inclusion_factor = 1.5;
    // obstacles.obstacle_association_cutoff_factor = 5;


    // // Optimization

    // optim.no_inner_iterations = 3;
    // optim.no_outer_iterations = 2;
    // optim.optimization_activate = true;
    // optim.optimization_verbose = false;//false
    // optim.penalty_epsilon = 0.1;
    // optim.weight_max_vel_x = 2; //1
    // optim.weight_max_vel_y = 2;
    // optim.weight_max_vel_theta = 1;
    // optim.weight_acc_lim_x = 1;
    // optim.weight_acc_lim_y = 1;
    // optim.weight_acc_lim_theta = 1;
    // optim.weight_kinematics_nh = 1000;
    // optim.weight_kinematics_forward_drive = 1;
    // optim.weight_kinematics_turning_radius = 1;
    // optim.weight_optimaltime = 1;
    // optim.weight_shortest_path = 0;
    // optim.weight_obstacle = 50;
    // optim.weight_inflation = 0.1;
    // optim.weight_dynamic_obstacle = 50;
    // optim.weight_dynamic_obstacle_inflation = 0.1;
    // optim.weight_viapoint = 1;
    // optim.weight_prefer_rotdir = 50;

    // optim.weight_adapt_factor = 2.0;
    // optim.obstacle_cost_exponent = 1.0;

    // // Homotopy Class Planner

    // hcp.enable_homotopy_class_planning = false;
    // hcp.enable_multithreading = true;
    // hcp.simple_exploration = false;
    // hcp.max_number_classes = 5;
    // hcp.selection_cost_hysteresis = 1.0;
    // hcp.selection_prefer_initial_plan = 0.95;
    // hcp.selection_obst_cost_scale = 100.0;
    // hcp.selection_viapoint_cost_scale = 1.0;
    // hcp.selection_alternative_time_cost = false;

    // hcp.obstacle_keypoint_offset = 0.1;
    // hcp.obstacle_heading_threshold = 0.45;
    // hcp.roadmap_graph_no_samples = 15;
    // hcp.roadmap_graph_area_width = 6; // [m]
    // hcp.roadmap_graph_area_length_scale = 1.0;
    // hcp.h_signature_prescaler = 1;
    // hcp.h_signature_threshold = 0.1;
    // hcp.switching_blocking_period = 0.0;

    // hcp.viapoints_all_candidates = true;

    // hcp.visualize_hc_graph = false;
    // hcp.visualize_with_time_as_z_axis_scale = 0.0;
    // hcp.delete_detours_backwards = true;
    // hcp.detours_orientation_tolerance = M_PI / 2.0;
    // hcp.length_start_orientation_vector = 0.4;
    // hcp.max_ratio_detours_duration_best_duration = 3.0;

    // // Recovery

    // recovery.shrink_horizon_backup = true;
    // recovery.shrink_horizon_min_duration = 10;
    // recovery.oscillation_recovery = true;
    // recovery.oscillation_v_eps = 0.1;
    // recovery.oscillation_omega_eps = 0.1;
    // recovery.oscillation_recovery_min_duration = 10;
    // recovery.oscillation_filter_duration = 10;






    checkParameters();

}

void TebConfig::reconfigure(TebConfig& cfg)
{ 
  boost::mutex::scoped_lock l(config_mutex_);
  
  // // Trajectory
  // trajectory.teb_autosize = cfg.teb_autosize;
  // trajectory.dt_ref = cfg.dt_ref;
  // trajectory.dt_hysteresis = cfg.dt_hysteresis;
  // trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  // trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
  // trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
  // trajectory.via_points_ordered = cfg.via_points_ordered;
  // trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
  // trajectory.exact_arc_length = cfg.exact_arc_length;
  // trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  // trajectory.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
  // trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
  // trajectory.publish_feedback = cfg.publish_feedback;
  
  // // Robot     
  // robot.max_vel_x = cfg.max_vel_x;
  // robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
  // robot.max_vel_y = cfg.max_vel_y;
  // robot.max_vel_theta = cfg.max_vel_theta;
  // robot.acc_lim_x = cfg.acc_lim_x;
  // robot.acc_lim_y = cfg.acc_lim_y;
  // robot.acc_lim_theta = cfg.acc_lim_theta;
  // robot.min_turning_radius = cfg.min_turning_radius;
  // robot.wheelbase = cfg.wheelbase;
  // robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;
  
  // // GoalTolerance
  // goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
  // goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  // goal_tolerance.free_goal_vel = cfg.free_goal_vel;
  
  // // Obstacles
  // obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
  // obstacles.inflation_dist = cfg.inflation_dist;
  // obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
  // obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
  // obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
  // obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
  // obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
  // obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
  // obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
  // obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;

  
  // // Optimization
  // optim.no_inner_iterations = cfg.no_inner_iterations;
  // optim.no_outer_iterations = cfg.no_outer_iterations;
  // optim.optimization_activate = cfg.optimization_activate;
  // optim.optimization_verbose = cfg.optimization_verbose;
  // optim.penalty_epsilon = cfg.penalty_epsilon;
  // optim.weight_max_vel_x = cfg.weight_max_vel_x;
  // optim.weight_max_vel_y = cfg.weight_max_vel_y;
  // optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
  // optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
  // optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
  // optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
  // optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
  // optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
  // optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
  // optim.weight_optimaltime = cfg.weight_optimaltime;
  // optim.weight_shortest_path = cfg.weight_shortest_path;
  // optim.weight_obstacle = cfg.weight_obstacle;
  // optim.weight_inflation = cfg.weight_inflation;
  // optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
  // optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
  // optim.weight_viapoint = cfg.weight_viapoint;
  // optim.weight_adapt_factor = cfg.weight_adapt_factor;
  // optim.obstacle_cost_exponent = cfg.obstacle_cost_exponent;
  
  // // Homotopy Class Planner
  // hcp.enable_multithreading = cfg.enable_multithreading;
  // hcp.max_number_classes = cfg.max_number_classes; 
  // hcp.selection_cost_hysteresis = cfg.selection_cost_hysteresis;
  // hcp.selection_prefer_initial_plan = cfg.selection_prefer_initial_plan;
  // hcp.selection_obst_cost_scale = cfg.selection_obst_cost_scale;
  // hcp.selection_viapoint_cost_scale = cfg.selection_viapoint_cost_scale;
  // hcp.selection_alternative_time_cost = cfg.selection_alternative_time_cost;
  // hcp.switching_blocking_period = cfg.switching_blocking_period;
  
  // hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
  // hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
  // hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
  // hcp.roadmap_graph_area_length_scale = cfg.roadmap_graph_area_length_scale;
  // hcp.h_signature_prescaler = cfg.h_signature_prescaler;
  // hcp.h_signature_threshold = cfg.h_signature_threshold;
  // hcp.viapoints_all_candidates = cfg.viapoints_all_candidates;
  // hcp.visualize_hc_graph = cfg.visualize_hc_graph;
  // hcp.visualize_with_time_as_z_axis_scale = cfg.visualize_with_time_as_z_axis_scale;
  
  // // Recovery
  
  // recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
  // recovery.oscillation_recovery = cfg.oscillation_recovery;
  





  checkParameters();
}
    
    
void TebConfig::checkParameters() const
{
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: max_vel_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: max_vel_x_backwards <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.max_vel_theta <= optim.penalty_epsilon)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: max_vel_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_x <= optim.penalty_epsilon)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: acc_lim_x <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
  
  if (robot.acc_lim_theta <= optim.penalty_epsilon)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: acc_lim_theta <= penalty_epsilon. The resulting bound is negative. Undefined behavior... Change at least one of them!");
      
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: dt_ref <= dt_hysteresis. The hysteresis is not allowed to be greater or equal!. Undefined behavior... Change at least one of them!");
    
  // min number of samples
  if (trajectory.min_samples <3)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter min_samples is smaller than 3! Sorry, I haven't enough degrees of freedom to plan a trajectory for you. Please increase ...");
  
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter 'costmap_obstacles_behind_robot_dist' should be positive or zero.");
    
  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter obstacle_heading_threshold must be in the interval ]0,1[. 0=0deg opening angle, 1=90deg opening angle.");
  
  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but wheelbase is set to zero: undesired behavior.");
  
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
    TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter cmd_angle_instead_rotvel is non-zero but min_turning_radius is set to zero: undesired behavior. You are mixing a carlike and a diffdrive robot");
  
  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
      TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter weight_adapt_factor shoud be >= 1.0");
  
  if (recovery.oscillation_filter_duration < 0)
      TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter oscillation_filter_duration must be >= 0");

  // weights
  if (optim.weight_optimaltime <= 0)
      TEB_WARN_LOG("TebLocalPlannerROS() Param Warning: parameter weight_optimaltime shoud be > 0 (even if weight_shortest_path is in use)");
  
}    


    
} // namespace teb_local_planner
