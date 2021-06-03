#ifndef PLANNER_GLOBAL_PLANNER_INPUT_H
#define PLANNER_GLOBAL_PLANNER_INPUT_H

#include <memory>
#include <typeindex>

#include "misc/planning_defs.h"

/**
  * @brief  创建Input对象
  * @param  T       Input类型
  *         input   创建的该input对象的指针
  * @retval None
  */
#define CREATE_GLOBAL_PLANNER_INPUT(T, input)                                  \
    std::shared_ptr<T> input = T::createGlobalPlannerInput()

/**
  * @brief  定义全局规划器输入类型，新任输入类型继承自GlobalPlannerInput
  * @param  None
  * @retval None
  * @eg     定义NavigationPlannerInput输入 
  *         DEFINE_GLOBAL_PLANNER_INPUT(Navigation)
  *             RobotPose pose;       // 具体输入数据
  *             ...
  *         END_GLOBAL_PLANNER_INPUT(Navigation)
  */
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


    /**
     * @desc    全局规划器输入基类
     * @eg      
     */
    struct GlobalPlannerInput
    {
      public:

        static std::shared_ptr<GlobalPlannerInput> createGlobalPlannerInput()
        {
            GlobalPlannerInput *input = new GlobalPlannerInput();
            return std::shared_ptr<GlobalPlannerInput>(input);
        }

        // 禁止拷贝构造和赋值函数
        GlobalPlannerInput(const GlobalPlannerInput &) = delete;
        void operator=(const GlobalPlannerInput &) = delete;

        virtual ~GlobalPlannerInput() {};

        /**
          * @brief  获取GlobalPlannerInput类型名
          * @param  None
          * @retval 数据名
          */
        virtual std::string getTypeName() const
        {
            return "GlobalPlannerInput";
        }

        /* 全局规划器输入的共有成员 */
        const int64_t id   = -1;   // 该输入数据id 
        uint64_t ts         = 0;    // 该输入数据产生的时间戳，（系统上电时间），单位us

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
