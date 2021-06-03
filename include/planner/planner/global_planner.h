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

        // 抽象基类不能定义对象，所有子类将通过DEFINE_GLOBAL_PLANNER定义getInstance函数
        // /**
        //   * @brief  获取GlobalPlanner的单例
        //   * @param  None
        //   * @retval None
        //   */
        // static GlobalPlanner &getInstance()
        // {
        //     static GlobalPlanner instance;
        //     return instance;
        // }

        // 禁止拷贝构造和赋值函数
        GlobalPlanner(const GlobalPlanner &) = delete;
        void operator=(const GlobalPlanner &) = delete;

        virtual ~GlobalPlanner() {};

        /**
          * @brief  通过配置管理器读取配置参数，初始化规划器
          * @param  cfg_mgr 配置管理器
          *         pool    线程池
          * @retval None
          */
        virtual void init(ConfigManager &cfg_mgr, ThreadPool *pool)
        {
            std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
            if (m_initialized)
                return;

            float frequency = loadConfig(cfg_mgr);

            HSM::init(frequency, pool);
        }

        /**
          * @brief  开始运行全局规划器
          * @param  input   规划的输入
          * @retval int64_t 全局规划器是否成功开始执行，大于等于0成功，小于0失败
          */
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

        /**
          * @brief  开始运行全局规划器
          * @param  ctx     上一次中断时执行的上下文
          * @retval int64_t 全局规划器是否成功开始执行，大于等于0成功，小于0失败
          */
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

        /**
          * @brief  停止当前运行全局规划器
          * @param  None
          * @retval None
          */
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

        /**
          * @brief  暂停当前运行全局规划器
          * @param  None
          * @retval ContextPtr  暂停时当前规划器运行的上下文
          */
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
        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        GlobalPlanner()
            : HSM(),
              m_input_id(-1)
        {}

        /**
          * @brief  加载具体配置参数
          * @param  cfg_mgr 配置管理器
          * @retval 规划频率（frequency）
          */
        virtual float loadConfig(ConfigManager &cfg_mgr) = 0;

        using HSM::init;
        using HSM::start;
        using HSM::stop;
        using HSM::pause;

        /**
          * @brief  完成当前全局规划后调用的函数
          * @param  None
          * @retval None
          */
        virtual void finishPlanning() = 0;

        /**
          * @brief  处理接收到的Input
          * @param  input  收到的input
          * @retval None
          */
        virtual bool handleInput(const GlobalPlannerInputPtr input) = 0;

        int64_t m_input_id;
        // GlobalPlannerInputPtr m_input;
    };
}

#endif // PLANNER_GLOBAL_PLANNER_H
