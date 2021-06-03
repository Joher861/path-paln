#ifndef HSM_HSM_H
#define HSM_HSM_H

#include <list>
// #include <mutex>

#include "hsm/include/state_machine.h"
#include "thread_pool/ThreadPool.h"
#include "mutex/shared_recursive_mutex.h"

#include "event_center/event.h"
#include "log/log_manager.h"

#include "context.h"

using namespace hsm;

using planning_data::Event;
using planning_data::EventPtr;
using planning_utils::LEVEL_DEBUG;
using planning_utils::LEVEL_INFO;
using planning_utils::LEVEL_WARN;
using planning_utils::LEVEL_ERROR;

#define HSM_DEBUG_LOG(name, fmt, ...)                                          \
    LOG(LOG_NAME(name), __LINE__, __FILE__, planning_utils::LEVEL_DEBUG,    \
        fmt, ##__VA_ARGS__)
#define HSM_INFO_LOG(name, fmt, ...)                                           \
    LOG(LOG_NAME(name), __LINE__, __FILE__, planning_utils::LEVEL_INFO,     \
        fmt, ##__VA_ARGS__)
#define HSM_WARN_LOG(name, fmt, ...)                                           \
    LOG(LOG_NAME(name), __LINE__, __FILE__, planning_utils::LEVEL_WARN,     \
        fmt, ##__VA_ARGS__)
#define HSM_ERROR_LOG(name, fmt, ...)                                          \
    LOG(LOG_NAME(name), __LINE__, __FILE__, planning_utils::LEVEL_ERROR,    \
        fmt, ##__VA_ARGS__)

namespace planning_utils
{
    /**
      * @desc   可以self running的状态机封装，提供start, stop, pause，reset等接口
      * @eg     None
      */
    class HSM
    {
      public:
        
        /**
          * @brief  构造函数
          * @param  pool 线程池对象，可以在此时给定，也可在之后init时给定
          * @retval None
          */
        HSM(ThreadPool *pool = nullptr);
        virtual ~HSM();

        /**
          * @brief  初始化状态机，设置频率，线程池对象等一些列工作
          * @param  frequency   状态工作频率
          *         pool        线程池对象
          * @retval None
          */
        virtual void init(float frequency, ThreadPool *pool = nullptr);

        /**
          * @brief  状态机开始运行
          * @param  context     如果context对象为空，则默认从停止状态启动，如果非空，则
          *                     认为从，暂停状态启动，恢复context保存的运行状态
          * @retval None
          */
        virtual void start(ContextPtr context = nullptr);

        /**
          * @brief  状态机停止
          * @param  None
          * @retval None
          */
        virtual void stop();

        /**
          * @brief  暂停状态机工作
          * @param  None
          * @retval ContextPtr  暂停时保存的状态机运行状态
          */
        virtual ContextPtr pause();

        /**
          * @brief  重置状态机状态
          * @param  None
          * @retval None
          */
        virtual void reset();

      protected:

        void initSM();

        /**
          * @brief  初始化内部自定义的StateMachine对象
          * @param  None
          * @retval None
          */
        virtual void initRunSM();
        bool isIdle();
        bool isRunning();
        bool isPause();
        void dispatchThread();
        void stopThread();
        void run();
        void act();
        void runSMLoop();

        /**
          * @brief  每个运行周期内获取当前所需数据
          * @param  None
          * @retval None
          */
        virtual void getData();

        /**
          * @brief  保存状态机运行的状态
          * @param  None
          * @retval ContextPtr  运行状态
          */
        virtual ContextPtr saveContext();

        /**
          * @brief  恢复Context
          * @param  ctx     需要恢复的上下文装填
          * @retval Transition 恢复context后的需要进行的context跳转
          */
        virtual Transition restoreContext(ContextPtr ctx);

        friend struct HSMStates;

        float m_frequency;
        uint64_t m_loop_us;
        uint64_t m_loop_ms;
        ThreadPool *m_pool;
        StateMachine m_run_sm;
        std::atomic<bool> m_start_flag;
        std::atomic<bool> m_stop_flag;
        std::atomic<bool> m_pause_flag;
        std::atomic<bool> m_run_loop_enabled;
        ContextPtr m_context;
        shared_recursive_mutex m_loop_mtx;
        std::list<EventPtr> m_evt_list;
        std::string m_log_name;

        bool m_initialized;

      private:

        StateMachine m_sm;
        std::atomic<bool> m_run_flag;
        std::atomic<bool> m_thread_stopped;
        uint64_t m_cur_ts;
        uint64_t m_last_ts;
    };

    struct HSMStates;
}

#endif // HSM_HSM_H


