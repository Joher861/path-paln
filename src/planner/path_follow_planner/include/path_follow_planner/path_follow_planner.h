#ifndef PLANNER_PATH_FOLLOW_PLANNER_H
#define PLANNER_PATH_FOLLOW_PLANNER_H

// #include "local_trajectory_generator/local_trajectory_generator.h"
// #include "rollout_generator/rollout_generator.h"
// #include "cubic_spline.h"

#include "planner/global_planner.h"

#include "data/slam_data.h"
#include "data/local_map_data.h"
#include "data/control_speed_data.h"


namespace planning_planner
{
    DEFINE_GLOBAL_PLANNER(PathFollow)

private:
    DataSlam m_slam_data;
    DataControlSpeed m_control_speed;

    float loadConfig(ConfigManager &cfg_mgr);
    void initRunSM();
    void reset();
    void getData();
    void finishPlanning();
    bool handleInput(const GlobalPlannerInputPtr input);
    ContextPtr saveContext();
    Transition restoreContext(ContextPtr ctx);

    std::vector<std::string> Split(const std::string &s, const std::string &delim);
    void insert_ellipse_between_clean_lines(const PosePath &input_path, PosePath &output_path);
    void insert_straight_line_between_clean_lines(const PosePath &input_path, PosePath &output_path);
    void insert_bezier_between_clean_lines(const PosePath &input_path, PosePath &output_path);
    bool check_path_point_collision(const int &p_, const int &q_, const int &i_, bool use_rect_shape_ = false);
    float getCurvatureFrom3points(const RobotPose &pose1, const RobotPose &pose2, const RobotPose &pose3);
    float getPathAverageCurvature(const PosePath &input_path);
    void stopRobot();

    void saveCleanStatusLog();
    void saveEndPtLog();

    void preprocess_global_path_points(void);
    void update_path_index_and_reached_skipped();
    void prepare_for_next_task();
    void path_follow_main_loop();
    void op_local_planner_sparse_and_dense();
    void print_reached_and_skipped();
    void check_escape_mode_with_us_and_local_map(const bool &us_lasting_front_, const bool &us_lasting_front_left_, const bool &us_lasting_front_right_, const bool &us_lasting_left_, const bool &us_lasting_right_,
                                                 const bool &lm_lasting_front_, const bool &lm_lasting_front_left_, const bool &lm_lasting_front_right_, const bool &lm_lasting_right_, const bool &lm_lasting_left_,
                                                 const bool &emergency_us_front_left_, const bool &emergency_us_front_right_, const bool &emergency_us_left_pure_, const bool &emergency_us_right_pure_,
                                                 const bool &emergency_lm_front_left_, const bool &emergency_lm_front_right_, const bool &emergency_lm_right_pure_, const bool &emergency_lm_left_pure_,
                                                 bool &escape_fl_rotate_right_, bool &escape_l_rotate_right_, bool &escape_fr_rotate_left_, bool &escape_r_rotate_left_, bool &escape_f_rotate_right_, bool &escape_f_rotate_left_,
                                                 bool &rotate_request_left_dir_, int &rotate_request_flag_, int &ignore_rotate_request_flag_, const float &tmp_yaw_next_path_point_, int &last_rotate_dir_);

    int m_ev_deduction_period_for_us;
    int m_set_oa_off;
    int m_set_collis_lasting_off;
    float m_rotate_vel_default;
    float m_delta_rotate_angle;
    int m_path_follow_finished;
    int m_path_follow_error;
    int m_start_deviation;
    float m_max_velocity;
    float m_min_velocity;
    float m_max_curvature;
    float m_min_curvature;
    float m_planning_velocity;
    int m_const_velocity;
    float m_test_velocity;
    bool m_need_path_insert;
    bool m_feedback_path_points_states;
    int64_t m_input_idx;
#ifdef OD_SWITCH
    bool m_small_obstacle_flag;
    bool m_far_circle_obstacle_flag;
    bool m_near_circle_obstacle_flag;
#else
    bool m_collision_flag;
    bool m_collision_lasting_flag;
    float m_collision_lasting_threshold;
#endif

    PosePath path_to_mpc;
    int m_points_num_to_mpc = 20;

    PosePath path_to_pure_pursuit;
    int m_points_num_to_pp = 8;

    PosePath path_to_teb;
    std::vector<int> path_to_teb_states;
    PosePath path_to_teb_prepare;
    PosePath path_buffer; 
    float m_necessary_T_num_for_buffer = 6; // 3
    size_t m_first_to_teb_num = 50; 

    PosePath path_from_teb;
    size_t start_idx_from_teb;
    int end_idx_from_teb;
    int m_rx_teb_path;


    size_t mpc_return_index;
    std::vector<int> mpc_path_points_states;

    size_t mpc_return_next_dense_index;

    size_t goal_index_mpc;

    size_t pure_pursuit_return_index;
    std::vector<int> pure_pursuit_path_points_states;

    size_t pure_pursuit_return_next_sparse_index;
    size_t goal_index_pure_pursuit; // pure_pursuit用

    size_t teb_return_index;
    std::vector<int> teb_path_points_states;
    size_t teb_return_next_dense_index;
    size_t goal_index_teb;

    // 记录上次控制周期发送给local_planner的目标点的index
    size_t goal_index_mpc_last;          // MPC用, 接收MPC发送event中的global_path_index
    size_t goal_index_pure_pursuit_last; // pure_pursuit用, 接收PP发送event中的global_path_index
    size_t goal_index_teb_last;          // teb用, 接收TEB发送event中的global_path_index


    /////////////////////
    // 轨迹点
    // 在handleInput中赋值
    PosePath global_path_points;

    // 定义两个路径
    float m_interval_pure_pursuit;
    float m_interval_mpc;
    float m_interval_teb;
    float jmp_r_mpc2pp;
    float max_jmp_r_pp2mpc;
    float jmp_tol_angle;
    float oa_switch_radius;

    // path_points_dense在没有障碍物的时候给MPC使用
    PosePath path_points_mpc;
    std::vector<int> path_points_mpc_situation;

    PosePath path_points_pure_pursuit;
    std::vector<int> path_points_pure_pursuit_situation;

    std::vector<size_t> pp_index_correspond_mpc_index;
    std::vector<std::pair<size_t, size_t>> pair_pp_index_to_mpc_index; // sparse, dense

    DataLocalMap m_simple_local_map;
    float m_local_map_resolution;
    int m_local_map_size_x;
    int m_local_map_size_y;
    int m_robot_pose_in_local_map_x_default;
    int m_robot_pose_in_local_map_y_default;
    float m_vehicle_longitude_size;
    float m_vehicle_lateral_size;
    float m_inflate_longitude_param = 0.0;
    float m_inflate_lateral_param = 0.0;

    float m_running_freq;

    friend struct PathFollowPlannerStates;

    END_GLOBAL_PLANNER(PathFollow)

} // namespace planning_planner

extern planning_planner::PathFollowPlanner &g_path_follow_planner;

#endif // PLANNER_PATH_FOLLOW_PLANNER_H