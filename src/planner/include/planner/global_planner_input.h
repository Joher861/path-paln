#ifndef PLANNER_GLOBAL_PLANNER_INPUT_H
#define PLANNER_GLOBAL_PLANNER_INPUT_H

#include <memory>
#include <typeindex>

#include "misc/planning_defs.h"


#define CREATE_GLOBAL_PLANNER_INPUT(T, input)                                  \
    std::shared_ptr<T> input = T::createGlobalPlannerInput()


#define DEFINE_GLOBAL_PLANNER_INPUT(InputType)                                 \
    struct InputType##PlannerInput;                                            \
    using InputType##PlannerInput##Ptr                                         \
        = std::shared_ptr<InputType##PlannerInput>;                            \
    struct InputType##PlannerInput : GlobalPlannerInput                        \
    {                                                                          \
      protected:                                                               \
        InputType##PlannerInput() : GlobalPlannerInput() {}                    \
      public:                                                                  \
        static InputType##PlannerInput##Ptr createGlobalPlannerInput()         \
        {                                                                      \
            InputType##PlannerInput *input = new InputType##PlannerInput();    \
            return InputType##PlannerInput##Ptr(input);                        \
        }                                                                      \
        InputType##PlannerInput(const InputType##PlannerInput &) = delete;     \
        void operator=(const InputType##PlannerInput &) = delete;              \
        virtual std::string getTypeName() const                                \
        {                                                                      \
            return type_name;                                                  \
        }                                                                      \
        inline static const std::string type_name                              \
            = STR_CONN(InputType, PlannerInput);
#define END_GLOBAL_PLANNER_INPUT(InputType)                                    \
    };

namespace planning_planner
{

    struct GlobalPlannerInput
    {
      public:

        static std::shared_ptr<GlobalPlannerInput> createGlobalPlannerInput()
        {
            GlobalPlannerInput *input = new GlobalPlannerInput();
            return std::shared_ptr<GlobalPlannerInput>(input);
        }

        GlobalPlannerInput(const GlobalPlannerInput &) = delete;
        void operator=(const GlobalPlannerInput &) = delete;

        virtual ~GlobalPlannerInput() {};


        virtual std::string getTypeName() const
        {
            return "GlobalPlannerInput";
        }

        const int64_t id   = -1;   
        uint64_t ts         = 0;   

      protected:

        GlobalPlannerInput()
            : id(m_id_cnt++)
        {
            m_id_cnt = m_id_cnt < 0 ? 0 : m_id_cnt;
        }

      private:

        inline static int64_t m_id_cnt = 0;
    };
    using GlobalPlannerInputPtr = std::shared_ptr<GlobalPlannerInput>; 
}

#endif // PLANNER_GLOBAL_PLANNER_INPUT_H
