#ifndef PLANNER_GLOBAL_PLANNER_H
#define PLANNER_GLOBAL_PLANNER_H

#include "global_planner_input.h"
#include "global_planner_context.h"
#include "hsm/hsm.h"
#include "task/task.h"
#include "config/config_manager.h"

using planning_utils::HSM;
using planning_utils::ConfigManager;

#define DEFINE_GLOBAL_PLANNER(PlannerType)                                     \
    class PlannerType##Planner : public planning_planner::GlobalPlanner     \
    {                                                                          \
      protected:                                                               \
        PlannerType##Planner() : planning_planner::GlobalPlanner() {}       \
      public:                                                                  \
        static PlannerType##Planner &getInstance()                             \
        {                                                                      \
            static PlannerType##Planner instance;                              \
            return instance;                                                   \
        }                                                                      \
        PlannerType##Planner(const PlannerType##Planner &) = delete;           \
        void operator=(const PlannerType##Planner &) = delete;
#define END_GLOBAL_PLANNER(PlannerType)                                        \
    };
namespace planning_planner
{

    class GlobalPlanner : public HSM
    {
      public:


        GlobalPlanner(const GlobalPlanner &) = delete;
        void operator=(const GlobalPlanner &) = delete;

        virtual ~GlobalPlanner() {};

        virtual void init(ConfigManager &cfg_mgr, ThreadPool *pool)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (m_initialized)
                return;

            float frequency = loadConfig(cfg_mgr);

            HSM::init(frequency, pool);
        }


        virtual int64_t startPlanner(GlobalPlannerInputPtr input)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isRunning())
            {
                HSM_INFO_LOG(m_log_name, "start planner, input id = %ld, "
                    "type = %s...", input->id, input->getTypeName().c_str());
                m_input_id = input->id;
                reset();
                if (!handleInput(input))
                {
                    HSM_INFO_LOG(m_log_name, "failed to handle input...");
                    m_input_id = -1;
                    return -1;
                }
                start();
                return m_input_id;
            }
            return -1;
        }


        virtual int64_t startPlanner(ContextGlobalPlannerPtr ctx)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isRunning())
            {
                HSM_INFO_LOG(m_log_name, "start planner from pause, "
                    "input id = %ld...", ctx->input_id);
                m_input_id = ctx->input_id;
                start(ctx);
                return m_input_id;
            }
            
            return -1;
        }

        virtual void stopPlanner()
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isIdle())
            {
                HSM_INFO_LOG(m_log_name, "stop planner, input id = %ld",
                    m_input_id);
                if (m_input_id < 0)
                    return;
                m_input_id = -1;
                stop();
            }
        }

        virtual ContextPtr pausePlanner()
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (isRunning())
            {
                HSM_INFO_LOG(m_log_name, "pause planner, input id = %ld",
                    m_input_id);
                if (m_input_id < 0)
                    return nullptr;
                m_input_id = -1;
                return pause();
            }
            return nullptr;
        }

      protected:
        GlobalPlanner()
            : HSM(),
              m_input_id(-1)
        {}

        virtual float loadConfig(ConfigManager &cfg_mgr) = 0;

        using HSM::init;
        using HSM::start;
        using HSM::stop;
        using HSM::pause;

        virtual void finishPlanning() = 0;

        virtual bool handleInput(const GlobalPlannerInputPtr input) = 0;

        int64_t m_input_id;
        // GlobalPlannerInputPtr m_input;
    };
}

#endif // PLANNER_GLOBAL_PLANNER_H
