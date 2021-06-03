#ifndef CONTROLLER_LOCAL_PLANNER_H
#define CONTROLLER_LOCAL_PLANNER_H

#include "hsm/hsm.h"
#include "config/config_manager.h"
#include "task/task.h"

using planning_utils::HSM;
using planning_utils::ConfigManager;

#define DEFINE_LOCAL_PLANNER(PlannerType)                                      \
    class PlannerType##Planner;                                                \
    using PlannerType##PlannerPtr = std::shared_ptr<PlannerType##Planner>;     \
    class PlannerType##Planner : public planning_controller::LocalPlanner   \
    {                                                                          \
      public:                                                                  \
        PlannerType##Planner(const PlannerType##Planner &) = delete;           \
        void operator=(const PlannerType##Planner &) = delete;
#define END_LOCAL_PLANNER(PlannerType)                                         \
    };

namespace planning_controller
{

    class LocalPlanner;

    using LocalPlannerPtr = std::shared_ptr<LocalPlanner>;

    class LocalPlanner : public HSM
    {
      public:

        LocalPlanner(const LocalPlanner &) = delete;
        void operator=(const LocalPlanner &) = delete;

        virtual ~LocalPlanner() {};

        virtual void init(ConfigManager &cfg_mgr)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            float frequency = loadConfig(cfg_mgr);

            HSM::init(frequency, nullptr);

            // if (m_initialized)
            //     return;

            // if (frequency <= 0.0f)
            //     m_frequency = 50;
            // else if (frequency > 200)
            //     m_frequency = 200;
            // else
            //     m_frequency = frequency;
            // m_loop_us = (uint64_t)(1000000 / m_frequency);
            // m_loop_ms = (uint64_t)(1000 / m_frequency);
            // initSM();
            // initRunSM();
            // m_initialized = true;
        }

        virtual bool startTask(const Task& task)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isRunning())
            {
                HSM_INFO_LOG(m_log_name, "start task id = %ld...", task.id);
                m_task_id = task.id;
                reset();
                if (!handleTask(task))
                {
                    HSM_INFO_LOG(m_log_name, "failed to handle task...");
                    return false;
                }
                start(nullptr);
                return true;
            }
            return false;
        }

        virtual void stopTask()
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (!isIdle())
            {
                HSM_INFO_LOG(m_log_name, "stop task id = %ld...", m_task_id);
                if (m_task_id < 0)
                    return;
                m_task_id = -1;
                stop();
            }
        }

      protected:

        LocalPlanner(ThreadPool* pool)
            : HSM(pool),
              m_task_id(-1)
        {}


        virtual float loadConfig(ConfigManager &cfg_mgr) = 0;

        using HSM::init;
        using HSM::start;
        using HSM::stop;
        using HSM::pause;

        virtual void finishTask() = 0;

        virtual bool handleTask(const Task& task) = 0;

        int64_t m_task_id;
    };
}

#endif // CONTROLLER_LOCAL_PLANNER_H