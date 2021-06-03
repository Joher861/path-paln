#ifndef CONTROLLER_MPC_PLANNER_H
#define CONTROLLER_MPC_PLANNER_H

#include "local_planner/local_planner.h"
#include "mpc_planner_task.h"
#include "mpc_planner_events.h"
#include "data/slam_data.h"
#include "robot_nlp.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

namespace planning_controller
{
    DEFINE_LOCAL_PLANNER(MPC)

      public:
    
        static MPCPlannerPtr& getInstance(ThreadPool *pool);

      private:

        MPCPlanner(ThreadPool *pool);
        virtual float loadConfig(ConfigManager &cfg_mgr);
        virtual void initRunSM();
        virtual void reset();
        virtual void getData();
        virtual bool handleTask(const Task& task);
        virtual void finishTask();

        virtual void prepare_data_for_mpc(const PosePath &m_global_mpc_path_, size_t &index_selected_point, 
                                  const std::vector<float> &robot_current_pose_, std::vector<MPCRefPoints>& local_mpc_path_, size_t N_, std::vector<int> &path_points_states);
        virtual bool trans2local_coord(const PosePath &m_global_mpc_path_, const size_t &index_selected_point,
                                   const std::vector<float> &robot_current_pose_, std::vector<MPCRefPoints> &temp_local_mpc_path_, size_t N_);
        virtual void do_mpc(const std::vector<float> &robot_current_pose_, std::vector<MPCRefPoints> &local_mpc_path_, std::vector<float> &X_OPT_, std::vector<float> &opt_global_);
        virtual size_t get_current_index();

        ApplicationReturnStatus status;
        Ipopt::SmartPtr<IpoptApplication> app;

        size_t m_selected_index;
        DataSlam m_slam_data;             // input: current pose
        PosePath m_global_mpc_path;     // unit: m    input: global path to follow
        size_t m_start_segment_index;     // input: start index
        int m_global_path_index;       // input: record the current path index in global path
        bool m_final_path_point_arrived;
        int m_final_path_point_infeasible;
        float m_search_radius = 0.2; // 0.2       > v*T_MPC
        float m_final_radius = 0.2;  
        float m_max_ipopt_cpu_time;
        // vehicle parameters
        float m_lr;
        float m_lf;
        float m_l;
        float m_rho;
        // optimization parameters
        size_t m_NX;
        size_t m_NU;
        size_t m_NXU;
        size_t m_N;
        float m_T_MPC;
        // bounds
        float m_v_lower_bound;               // m/s
        float m_v_upper_bound;               // m/s
        float m_v_upper_bound_default;       // m/s
        float m_delta_lower_bound;           // rad
        float m_delta_upper_bound; 
        // weights
        float m_Qx;
        float m_Qy;
        float m_Qphi;
        float m_Qdelta;  
            
        int m_ev_deduction_period_for_us;
        TaskMPC::MPCType m_type;

        friend struct MPCPlannerStates;

    END_LOCAL_PLANNER(MPC)
}

#endif // CONTROLLER_MPC_PLANNER_H