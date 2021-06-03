#include "hsm.h"

#include "misc/planning_defs.h"
#include "log/log_manager.h"
#include "timer/sleep_timer.h"
#include "event_center/event_center.h"

using namespace planning_utils;
using namespace planning_data;

namespace planning_utils
{
    struct HSMStates
    {
        struct BaseState : StateWithOwner<HSM>
        {};

        struct Idle : BaseState
        {
            virtual void OnEnter()
            {
                HSM_INFO_LOG(Owner().m_log_name, "Enter Idle...");
            }

            virtual void OnExit()
            {
                HSM_INFO_LOG(Owner().m_log_name, "Exit Idle...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_start_flag)
                {
                    Owner().m_start_flag = false;
                    return SiblingTransition<Running>();
                }
                return NoTransition();
            }

            virtual void Update()
            {
                // HSM_DEBUG_LOG(Owner().m_log_name, "Idle...");
            }
        };

        struct Running : BaseState
        {
            virtual void OnEnter()
            {
                HSM_INFO_LOG(Owner().m_log_name, "Enter Running...");
                Owner().m_run_loop_enabled = true;
            }

            virtual void OnExit()
            {
                HSM_INFO_LOG(Owner().m_log_name, "Exit Running...");
                Owner().m_run_loop_enabled = false;
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_stop_flag)
                {
                    Owner().m_stop_flag = false;
                    return SiblingTransition<Idle>();
                }
                else if (Owner().m_pause_flag)
                {
                    Owner().m_pause_flag = false;
                    return SiblingTransition<Pause>();
                }
                return NoTransition();
            }

            virtual void Update()
            {
                // HSM_DEBUG_LOG(Owner().m_log_name, "Running...");
            }
        };

        struct Pause : BaseState
        {
            virtual void OnEnter()
            {
                HSM_INFO_LOG(Owner().m_log_name, "Enter Pause...");
            }

            virtual void OnExit()
            {
                HSM_INFO_LOG(Owner().m_log_name, "Exit Pause...");
            }

            virtual Transition GetTransition()
            {
                if (Owner().m_stop_flag)
                {
                    Owner().m_stop_flag = false;
                    return SiblingTransition<Idle>();
                }
                else if (Owner().m_start_flag)
                {
                    Owner().m_start_flag = false;
                    return SiblingTransition<Running>();
                }
                return NoTransition();
            }

            virtual void Update()
            {
                // HSM_DEBUG_LOG(Owner().m_log_name, "Pause...");
            }
        };
    };
}

HSM::HSM(ThreadPool *pool)
    : m_frequency(0.0f),
      m_pool(pool),
      m_start_flag(false),
      m_stop_flag(false),
      m_pause_flag(false),
      m_run_loop_enabled(false),
      m_context(nullptr),
      m_log_name(""),
      m_initialized(false),
      m_run_flag(false),
      m_thread_stopped(true)
{}

HSM::~HSM()
{
    m_run_flag = false;
}

void HSM::init(float frequency, ThreadPool *pool)
{
    std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);

    if (m_initialized)
        return;
    
    if (pool != nullptr)
    {
        m_pool = pool;
    }

    if (m_pool == nullptr)
    {
        throw std::runtime_error("thread pool is not allocated, hsm cannot run");
    }

    if (frequency <= 0.0f)
        m_frequency = 50;
    else if (frequency > 200)
        m_frequency = 200;
    else
        m_frequency = frequency;
    m_loop_us = (uint64_t)(1000000 / m_frequency);
    m_loop_ms = (uint64_t)(1000 / m_frequency);
    initSM();
    initRunSM();
    m_initialized = true;
}

void HSM::initSM()
{
    HSM_DEBUG_LOG(m_log_name, "init sm...");
	m_sm.Initialize<HSMStates::Idle>(this);
	m_sm.SetDebugInfo("HSM", TraceLevel::None);
    m_sm.ProcessStateTransitions();
    m_sm.UpdateStates();
}

void HSM::initRunSM()
{}

void HSM::start(ContextPtr context)
{
    std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
    if (!isRunning())
    {
        HSM_INFO_LOG(m_log_name, "start...");
        m_start_flag = true;
        getData();
        m_context = context;
        runSMLoop();
    }
}

void HSM::stop()
{
    std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
    if (!isIdle())
    {
        HSM_INFO_LOG(m_log_name, "stop...");
        m_stop_flag = true;
        getData();
        runSMLoop();
        m_context = nullptr;
    }
}

ContextPtr HSM::pause()
{
    std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
    if (isRunning())
    {
        HSM_INFO_LOG(m_log_name, "pause...");
        m_pause_flag = true;
        getData();
        runSMLoop();
        ContextPtr ctx = m_context;
        m_context = nullptr;
        return ctx;
    }

    return nullptr;
}

bool HSM::isIdle()
{
    return m_sm.IsInState<HSMStates::Idle>();
}

bool HSM::isRunning()
{
    return m_sm.IsInState<HSMStates::Running>();
}

bool HSM::isPause()
{
    return m_sm.IsInState<HSMStates::Pause>();
}

void HSM::reset()
{}

void HSM::dispatchThread()
{
    HSM_INFO_LOG(m_log_name, "dispatch running thread...");
    m_last_ts = Timer::getSystemTimestampUS();
    m_run_flag = true;
    if (m_thread_stopped)
    {
        HSM_INFO_LOG(m_log_name, "add running thread to thread pool");
        m_pool->enqueue([&]() { run(); });
        m_thread_stopped = false;
    }
    else
    {
        HSM_INFO_LOG(m_log_name, "thread still not running in thread pool");
    }
    HSM_INFO_LOG(m_log_name, "dispatch running thread success...");
}

void HSM::stopThread()
{
    HSM_INFO_LOG(m_log_name, "stop running thread...");
    m_run_flag = false;
}

void HSM::run()
{
    SleepTimer timer(m_frequency);

    // 状态机以固定频率运行
    while (true)
    {
        std::unique_lock<shared_recursive_mutex> lock(m_loop_mtx);
        if (!m_run_flag)
        {
            m_thread_stopped = true;
            HSM_INFO_LOG(m_log_name, "run finished...");
            break;
        }

        // HSM_DEBUG_LOG(m_log_name, "");
        // HSM_DEBUG_LOG(m_log_name, "in run loop...");
        act();
        lock.unlock();
        timer.sleep();
    }
}

void HSM::act()
{
    m_cur_ts = Timer::getSystemTimestampUS();

    // HSM_DEBUG_LOG(m_log_name, "loop ts = %lu", m_cur_ts);

    // 每个运行周期去获取上个周期内发生的事件
    // HSM_DEBUG_LOG(m_log_name, "start event deduction, start_time = %lu, "
    //     "end_time = %lu", m_last_ts, m_cur_ts);
    m_evt_list.clear();
    g_ec.eventDeduction(m_last_ts, m_cur_ts, m_evt_list);
    // HSM_DEBUG_LOG(m_log_name, "finish event deduction");

    getData();
    // HSM_DEBUG_LOG(m_log_name, "finish get data");

    // 运行状态机，处理事件
    runSMLoop();
    if (m_stop_flag || m_pause_flag)
    {
        HSM_INFO_LOG(m_log_name, "hsm self stopped or paused, run loop again "
            "to change the right state");
        runSMLoop();
    }

    m_last_ts = m_cur_ts;
}

void HSM::getData()
{
    // HSM_DEBUG_LOG(m_log_name, "default get data...");
}

void HSM::runSMLoop()
{
    // HSM_DEBUG_LOG(m_log_name, "start transition...");
    m_sm.ProcessStateTransitions();
    // HSM_DEBUG_LOG(m_log_name, "finish transition...");
    // HSM_DEBUG_LOG(m_log_name, "start update state...");
    m_sm.UpdateStates();
    // HSM_DEBUG_LOG(m_log_name, "finish update state...");
    // HSM_DEBUG_LOG(m_log_name, "start run sm transition...");
    m_run_sm.ProcessStateTransitions();
    // HSM_DEBUG_LOG(m_log_name, "finish run sm transition...");
    // HSM_DEBUG_LOG(m_log_name, "start run sm update state...");
    m_run_sm.UpdateStates();
    // HSM_DEBUG_LOG(m_log_name, "finish run sm update state...");
}

ContextPtr HSM::saveContext()
{
    HSM_INFO_LOG(m_log_name, "save context...");
    return nullptr;
}

Transition HSM::restoreContext(ContextPtr ctx)
{
    HSM_INFO_LOG(m_log_name, "restore HSM context");
    return NoTransition();
}