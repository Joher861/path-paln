#include "test_hsm.h"
#include "log/log_manager.h"

using namespace planning_utils;
using namespace std;

struct TestHSMStates
{
	struct BaseState : StateWithOwner<TestHSM>
	{
	};

    struct Disable : BaseState
	{
		virtual Transition GetTransition()
		{
            if (Owner().m_run_loop_enabled)
            {
                return SiblingTransition<Enable>();
            }

            return NoTransition();            
        }

        virtual void Update()
        {
            TEST_HSM_DEBUG_LOG("Disable...");
        }
	};

    struct Enable : BaseState
    {
        virtual void OnEnter()
        {
            Owner().dispatchThread();
        }

        virtual void OnExit()
        {
            Owner().stopThread();
        }

        virtual Transition GetTransition()
		{
            ContextPtr &ctx = Owner().m_context;
            if (!Owner().m_run_loop_enabled)
            {
                ctx = Owner().saveContext();
                return SiblingTransition<Disable>();
            }

            if (ctx == nullptr)
            {
                // 正常开始
                TEST_HSM_DEBUG_LOG("InnerEntry first 1");
                return InnerEntryTransition<First>();
            }
            else
            {
                // 恢复context
                Transition trans = Owner().restoreContext(ctx);
                ctx = nullptr;
                return trans;
            }
		}

        virtual void Update()
        {
            TEST_HSM_DEBUG_LOG("Enable...");
        }
    };

	struct First : BaseState
	{
		virtual Transition GetTransition()
		{
            // 处理事件
            TEST_HSM_DEBUG_LOG("First process event start...");
            list<EventPtr>& evt_list = Owner().m_evt_list;
            
            for (auto evt : evt_list)
            {
                std::type_index evt_type = evt->getType();
                
                if (TYPE_EQUALS(evt_type, EvToSecond))
                {
                    TEST_HSM_INFO_LOG("First process event end1...");
                    return SiblingTransition<Second>();
                }
            }
            TEST_HSM_DEBUG_LOG("First process event end2...");
			return NoTransition();
		}

		virtual void Update()
		{
            TEST_HSM_DEBUG_LOG("First...");
		}
	};

	struct Second : BaseState
	{
		virtual Transition GetTransition()
		{
            // 处理事件
            TEST_HSM_DEBUG_LOG("Second process event start...");
            list<EventPtr>& evt_list = Owner().m_evt_list;
            
            for (auto evt : evt_list)
            {
                std::type_index evt_type = evt->getType();
                
                if (TYPE_EQUALS(evt_type, EvToThird))
                {
                    TEST_HSM_INFO_LOG("Second process event end1...");
                    return SiblingTransition<Third>();
                }
            }
            TEST_HSM_DEBUG_LOG("Second process event end2...");
			return NoTransition();
		}

		virtual void Update()
		{
            TEST_HSM_DEBUG_LOG("Second...");
		}
	};

	struct Third : BaseState
	{
		virtual Transition GetTransition()
		{
            TEST_HSM_DEBUG_LOG("Third process event...");
			return NoTransition();
		}

		virtual void Update()
		{
            TEST_HSM_DEBUG_LOG("Third...");
		}
	};
};

TestHSM::TestHSM(ThreadPool* pool)
     : HSM(pool)
{}

TestHSM::~TestHSM()
{}

// void TestHSM::init(float frequency)
// {
//     unique_lock<mutex> lock(m_loop_mtx);

//     if (m_initialized)
//         return;

//     if (frequency <= 0.0f)
//         m_frequency = 50;
//     else if (frequency > 200)
//         m_frequency = 200;
//     else
//         m_frequency = frequency;
//     m_loop_us = (uint64_t)(1000000 / m_frequency);
//     m_loop_ms = (uint64_t)(1000 / m_frequency);
//     initSM();
//     initRunSM();
//     m_log_name = LOG_TEST_HSM;
//     m_initialized = true;
//     // initRunSM();
// }

void TestHSM::initRunSM()
{
    m_log_name = LOG_TEST_HSM;

    TEST_HSM_DEBUG_LOG("set init run sm");
	m_run_sm.Initialize<TestHSMStates::Disable>(this);
	m_run_sm.SetDebugInfo("TestHSM", TraceLevel::Diagnostic);
    m_run_sm.ProcessStateTransitions();
    m_run_sm.UpdateStates();
}

ContextPtr TestHSM::saveContext()
{
    TEST_HSM_INFO_LOG("save context");
    auto enable_state = m_run_sm.GetState<TestHSMStates::Enable>();
    
    CREATE_CONTEXT(ContextTest, test_ctx);
    auto pause_state = enable_state->GetImmediateInnerState();
    if (pause_state == nullptr)
    {
        TEST_HSM_ERROR_LOG("failed to get pause state");
        test_ctx->pause_state = StateTypeId{};
    }
    else
    {
        TEST_HSM_INFO_LOG("find top level enabled state");
        test_ctx->pause_state = pause_state->GetStateType();
    }

    return test_ctx;
}

Transition TestHSM::restoreContext(ContextPtr ctx)
{
    // TEST_HSM_INFO_LOG("restore TestHSM context");
    ContextTestPtr test_ctx = std::dynamic_pointer_cast<ContextTest>(ctx);
    if (test_ctx->pause_state == GetStateType<TestHSMStates::First>())
        return InnerEntryTransition<TestHSMStates::First>();
    else if (test_ctx->pause_state == GetStateType<TestHSMStates::Second>())
        return InnerEntryTransition<TestHSMStates::Second>();
    else if (test_ctx->pause_state == GetStateType<TestHSMStates::Third>())
        return InnerEntryTransition<TestHSMStates::Third>();
    return NoTransition();
}