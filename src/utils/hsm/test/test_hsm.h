#ifndef TEST_HSM
#define TEST_HSM

#include "hsm.h"
#include "event_center/event.h"

using planning_utils::HSM;
using planning_utils::ContextPtr;

#define LOG_TEST_HSM_FLAG   test_hsm
#define LOG_TEST_HSM        "test_hsm"
    
#define TEST_HSM_DEBUG_LOG(fmt, ...)                                           \
    LOG(LOG_NAME(LOG_TEST_HSM), __LINE__, __FILE__, LEVEL_DEBUG,           \
        fmt, ##__VA_ARGS__)
#define TEST_HSM_INFO_LOG(fmt, ...)                                            \
    LOG(LOG_NAME(LOG_TEST_HSM), __LINE__, __FILE__, LEVEL_INFO,            \
        fmt, ##__VA_ARGS__)
#define TEST_HSM_WARN_LOG(fmt, ...)                                            \
    LOG(LOG_NAME(LOG_TEST_HSM), __LINE__, __FILE__, LEVEL_WARN,            \
        fmt, ##__VA_ARGS__)
#define TEST_HSM_ERROR_LOG(fmt, ...)                                           \
    LOG(LOG_NAME(LOG_TEST_HSM), __LINE__, __FILE__, LEVEL_ERROR,           \
        fmt, ##__VA_ARGS__)

DEFINE_EVENT(TestStart)
END_EVENT(TestStart)

DEFINE_EVENT(TestStop)
END_EVENT(TestStop)

DEFINE_EVENT(ToSecond)
END_EVENT(ToSecond)

DEFINE_EVENT(ToThird)
END_EVENT(ToThird)

DEFINE_CONTEXT(Test)
    StateTypeId pause_state;
END_CONTEXT(Test)

class TestHSM : HSM
{
  public:

    TestHSM(ThreadPool* pool);
    ~TestHSM();

    // void init(float frequency);
    using HSM::init;
    using HSM::start;
    using HSM::stop;
    using HSM::pause;
    using HSM::reset;

  protected:

    virtual void initRunSM();
    virtual ContextPtr saveContext();
    virtual Transition restoreContext(ContextPtr ctx);

    friend struct TestHSMStates;
};

struct TestHSMStates;

#endif // TEST_HSM
