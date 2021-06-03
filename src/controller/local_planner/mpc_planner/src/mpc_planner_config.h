#ifndef CONTROLLER_MPC_PLANNER_CONFIG_H
#define CONTROLLER_MPC_PLANNER_CONFIG_H

#include <string>

#include "config/config.h"

#define CONFIG_MPC_PLANNER    "cfg_MPC_planner"

DEFINE_CONFIG(MPCPlanner)
    std::string log_name;
    std::string log_path;
    std::string log_extension;
    int log_ts_mask;
    bool log_print_to_console;
    int log_max_file_size_mb;
    int log_max_file_size_kb;
    int log_max_file_cnt;
    int log_level;
    // vehicle parameters
    float conf_lr;
    float conf_lf;
    // optimization parameters
    int conf_NX;                            // should be size_t
    int conf_NU;                            // should be size_t
    int conf_N;                             // should be size_t
    // bounds
    float conf_delta_lower_bound;           // degree
    float conf_delta_upper_bound;
    // weights        
    float conf_Qx;
    float conf_Qy;
    float conf_Qphi;
    float conf_Qdelta;

    int ev_deduction_period_for_us;

    float planning_frequency;
END_CONFIG(MPCPlanner)

DEFINE_CONFIG_READ_FUNC(MPCPlanner)
    READ_CONFIG_MEMBER(log_name);
    READ_CONFIG_MEMBER(log_path);
    READ_CONFIG_MEMBER(log_extension);
    READ_CONFIG_MEMBER(log_ts_mask);
    READ_CONFIG_MEMBER(log_print_to_console);
    READ_CONFIG_MEMBER(log_max_file_size_mb);
    READ_CONFIG_MEMBER(log_max_file_size_kb);
    READ_CONFIG_MEMBER(log_max_file_cnt);
    READ_CONFIG_MEMBER(log_level);
    READ_CONFIG_MEMBER(conf_lr);
    READ_CONFIG_MEMBER(conf_lf);
    READ_CONFIG_MEMBER(conf_NX);
    READ_CONFIG_MEMBER(conf_NU);
    READ_CONFIG_MEMBER(conf_N);
    READ_CONFIG_MEMBER(conf_delta_lower_bound);
    READ_CONFIG_MEMBER(conf_delta_upper_bound);
    READ_CONFIG_MEMBER(conf_Qx);
    READ_CONFIG_MEMBER(conf_Qy);
    READ_CONFIG_MEMBER(conf_Qphi);
    READ_CONFIG_MEMBER(conf_Qdelta);
    READ_CONFIG_MEMBER(ev_deduction_period_for_us);
    READ_CONFIG_MEMBER(planning_frequency);
END_CONFIG_READ_FUNC(MPCPlanner)

#endif // CONTROLLER_MPC_PLANNER_CONFIG_H