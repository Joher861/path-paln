#include "ccpp_coverage_planner.h"
#include "ccpp_coverage_planner_config.h"
#include "ccpp_coverage_planner_log.h"

#include "path_follow_planner/path_follow_planner.h"
#include "path_follow_planner/path_follow_planner_input.h"

#include "planner/global_planner_events.h"
#include "geometry/geometry_func.h"
#include "timer/timer.h"
#include "data_center/data_center.h"
#include "event_center/event_center.h"
#include "task/task_manager.h"

#include "local_planner/local_planner_events.h"

#include "rotate_planner/rotate_task.h"
#include "basic_planner/basic_task.h"

#include "misc/planning_common_config.h"
#include "misc/robot_config.h"
#include "event/global_common_event_def.h"
#include "sys/stat.h"

using namespace planning_planner;
using namespace planning_utils;
using namespace planning_data;
using namespace planning_controller;

#define CCPPTEST 0
#define SAVEFILECCPP 1

#if CCPPTEST
string saveFileAdress3 = "ccpp_img/";
string name_path = "ccpp_img/map_sub/cell_map";
#else
string saveFileAdress3 = "/tmp/log/ccppLog/";
string name_path = "/tmp/log/ccppLog/";
#endif

namespace planning_planner
{
    struct ccppPlannerStates
    { // StateWithOwner
        // Class that clients can use instead of deriving directly from State that provides convenient
        // typed access to the Owner. This class can also be chained via the StateBaseType parameter,
        // which is useful when inheriting state machines.
        struct BaseState : StateWithOwner<ccppCoveragePlanner>
        {
        };

        struct Disable : BaseState
        {
            virtual void OnEnter() // OnEnter is invoked when a State is created
            {
            }

            virtual void OnExit()
            {
                CCPP_INFO_LOG("Exit Disable...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_run_loop_enabled)
                {
                    return SiblingTransition<Enable>();
                }
                else
                {
                    return NoTransition();
                }
            }

            virtual void Update()
            {
                CCPP_INFO_LOG("Disable...");
            }
        };

        struct Enable : BaseState
        {
            virtual void OnEnter()
            {
                CCPP_INFO_LOG("Enter Enable...");
            }

            virtual void OnExit()
            {
                CCPP_INFO_LOG("Exit Enable...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().ccpp_planner_finished == 1)
                {
                    CCPP_INFO_LOG("enable to finished...");
                    return SiblingTransition<Finished>();
                }
                if (!Owner().m_run_loop_enabled)
                {
                    CCPP_INFO_LOG("enable to disable...");
                    return SiblingTransition<Disable>();
                }
                return InnerEntryTransition<ccppPlannerGo>();
            }

            virtual void Update()
            {
                CCPP_INFO_LOG("Enable...");
            }
        };
        struct ccppPlannerGo : BaseState
        {
            virtual void OnEnter()
            {
                Owner().ccpp_planner_finished = 0;
                CCPP_INFO_LOG("Enter ccppPlannerGo...");
                Owner().dispatchThread();
                ipa_building_msgs::RoomExplorationActionGoal goal_ = Generate_goal(Owner().param_ccpp); //预处理

                Owner().ccpp_coverage_path_output = explore_room(goal_, name_path, Owner().param_ccpp); //分区规划
                ofstream saveFileCCPP(saveFileAdress3 +  "CCPPDATA.txt");
                for (int i = 0 ;i<(int)Owner().ccpp_coverage_path_output.ccppData.size();i++)
                {
                    for (int j = 0 ;j<Owner().ccpp_coverage_path_output.ccppData[i].current_path.size();j++)
                    {
                        saveFileCCPP<<i<<" "<<Owner().ccpp_coverage_path_output.ccppData[i].current_path[j].pt.x<<" "<<Owner().ccpp_coverage_path_output.ccppData[i].current_path[j].pt.y<<" "<<Owner().ccpp_coverage_path_output.ccppData[i].current_path[j].theta<<endl;
                    }
                }

                if ((int)Owner().ccpp_coverage_path_output.ccppData.size() > 0)
                {
                    // update到data_center
                    g_dc.updateData<DataCCPP>(&Owner().ccpp_coverage_path_output);
                    CCPP_INFO_LOG("update to DataCenter");
                }
                else
                {
                    CCPP_INFO_LOG("fail to find ccpp path");
                    g_dc.updateData<DataCCPP>(&Owner().ccpp_coverage_path_output);
                }
            }

            virtual void OnExit()
            {
                CCPP_INFO_LOG("Exit ccppPlannerGo...");
                Owner().stopThread();
            }

            virtual Transition GetTransition()
            {
                return SiblingTransition<Finished>();
            }

            virtual void Update()
            {
                CCPP_INFO_LOG("ccppPlannerGoUpdate");
            }
            int64_t m_ccpp_coverage_planner_input_id;
        };

        struct Finished : BaseState
        {
            virtual void OnEnter()
            {
                Owner().finishPlanning();

                CCPP_INFO_LOG("set stop flag, change to Idle");
                Owner().m_stop_flag = true;
                Owner().stopThread();
            }

            virtual void OnExit()
            {
                CCPP_INFO_LOG("Exit Finished");
            }

            virtual Transition GetTransition()
            {
                return NoTransition();
            }
        };
    };
} // namespace planning_planner

DEFINE_CONFIG_TYPE(CONFIG_CCPP_COVERAGE_PLANNER, CCPPCoveragePlanner);
ccppCoveragePlanner &g_ccpp_coverage_planner = ccppCoveragePlanner::getInstance();

float ccppCoveragePlanner::loadConfig(ConfigManager &cfg_mgr)
{
    ConfigCCPPCoveragePlanner *cfg_ccpp = dynamic_cast<ConfigCCPPCoveragePlanner *>(
        cfg_mgr.GetSubConfig(CONFIG_CCPP_COVERAGE_PLANNER));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning *>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));

    cfg_ccpp->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_CCPP_PLANNER_FLAG, LOG_CCPP_PLANNER,
               cfg_ccpp->log_name, cfg_ccpp->log_path,
               cfg_ccpp->log_extension, cfg_ccpp->log_ts_mask,
               cfg_ccpp->log_print_to_console,
               (cfg_ccpp->log_max_file_size_mb)MB + (cfg_ccpp->log_max_file_size_kb)KB,
               cfg_ccpp->log_max_file_cnt, cfg_ccpp->log_level);

    param_ccpp.complementary_path_distance = cfg_ccpp->complementary_path_distance;
    param_ccpp.coverage_radius = cfg_ccpp->coverage_radius;
    param_ccpp.distance_to_boundary = cfg_ccpp->distance_to_boundary;
    param_ccpp.grid_spacing_in_meter = cfg_ccpp->grid_spacing_in_meter;
    param_ccpp.min_cell_area_ = cfg_ccpp->min_cell_area_;
    param_ccpp.min_line_cleaning_distance = cfg_ccpp->min_line_cleaning_distance;
    param_ccpp.need_hybrid_distance = cfg_ccpp->need_hybrid_distance;
    param_ccpp.resolution = cfg_ccpp->resolution;
    param_ccpp.robot_radius = cfg_ccpp->robot_radius;

    return cfg_ccpp->planning_frequency;
}

bool ccppCoveragePlanner::handleInput(const GlobalPlannerInputPtr input)
{
    CCPPPlannerInputPtr ccppPlanner_input = std::dynamic_pointer_cast<CCPPPlannerInput>(input);
    param_ccpp.start_pos = ccppPlanner_input->START_POS;
    param_ccpp.planMode = ccppPlanner_input->planMode;
    param_ccpp.replanIndex = ccppPlanner_input->replanIndex;
    param_ccpp.ccppDataInput = ccppPlanner_input->ccppDataInput;
    param_ccpp.missOutDataInput = ccppPlanner_input->missOutDataInput;
    return true;
}

void ccppCoveragePlanner::initRunSM()
{
    m_run_sm.Initialize<ccppPlannerStates::Disable>(this);
    m_run_sm.SetDebugInfo("ccppPlanner", TraceLevel::None);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

void ccppCoveragePlanner::reset()
{
}

void ccppCoveragePlanner::stopRobot()
{
}

ContextPtr ccppCoveragePlanner::saveContext()
{
}

Transition ccppCoveragePlanner::restoreContext(ContextPtr ctx)
{
}

void ccppCoveragePlanner::getData()
{
}

void ccppCoveragePlanner::finishPlanning()
{
    stopRobot();
    CCPP_INFO_LOG("ccppCoveragePlanner ccppCoveragePlanner::finishPlanning()   put_event");
    CREATE_EVENT(EvPlannerFinished, ev_ccpp_finished);
    ev_ccpp_finished->id = m_input_id;
    g_ec.pushEvent(ev_ccpp_finished);
}