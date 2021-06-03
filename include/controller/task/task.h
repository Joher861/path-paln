#ifndef CONTROLLER_TASK_H
#define CONTROLLER_TASK_H

#include <memory>
#include <typeindex>

#include "misc/planning_defs.h"

namespace planning_controller
{
    enum PREEMPT_TYPE
    {
        NONE_PREEMPT,            // 不抢占，顺序执⾏
        PREEMPT,                 // 抢占优先执⾏
        PREEMPT_DELETE           // 抢占并删除之前的所有任务
    };

    struct Task;

    using TaskPtr = std::shared_ptr<Task>; 

    /**
      * @brief  创建任务对象
      * @param  TaskType   某一任务类型
      *         task 创建的该任务对象的指针
      * @retval None
      */
    #define CREATE_TASK(TaskType, task)                                        \
        TaskType##Ptr task = (TaskType::createTask())

    /**
      * @brief  定义任务类型，新任务类型继承自Task
      * @param  None
      * @retval None
      * @eg     定义TaskLine任务 
      *         DEFINE_TASK(Line)
      *             int line_dis;       // 任务对应的数据
      *             int line_speed; 
      *             ...
      *         END_TASK(Line)
      */
    #define DEFINE_TASK(TaskType, _preempt_type, _preemptible)                 \
        struct Task##TaskType;                                                 \
        using Task##TaskType##Ptr = std::shared_ptr<Task##TaskType>;           \
        struct Task##TaskType : Task                                           \
        {                                                                      \
          protected:                                                           \
            Task##TaskType() : Task()                                          \
            {                                                                  \
                preempt_type = _preempt_type;                                  \
                preemptible = _preemptible;                                    \
            }                                                                  \
          public:                                                              \
            static Task##TaskType##Ptr createTask()                            \
            {                                                                  \
                Task##TaskType *task = new Task##TaskType;                     \
                return Task##TaskType##Ptr(task);                              \
            }                                                                  \
            virtual std::string getTypeName() const                            \
            {                                                                  \
                return type_name;                                              \
            }                                                                  \
            inline static const std::string type_name                          \
                = STR_CONN(Task, TaskType);
    #define END_TASK(TaskType)                                                        \
        };

    /**
      * @desc    任务基类
      * @eg      
      */
    struct Task
    {
      public:
        /**
          * @brief  创建任务（任务都创建在堆上）
          * @param  None
          * @retval None
          */
        static TaskPtr createTask()
        {
            Task* task = new Task;
            return TaskPtr(task);
        }

        virtual ~Task() 
        {
        }

        virtual std::string getTypeName() const
        {
            return "Task";
        }

        /**
          * @brief  获取本任务类型的type_index
          * @param  None
          * @retval 本任务类型的type_index
          */
        inline std::type_index getType() const
        {
            return typeid(*this);
        }

        /* 任务的共有成员 */
        int64_t id                  = -1;               // 任务id 
        uint64_t ts                 = 0;                // 任务发生时间戳
                                                        //（系统上电时间），单位us，
        PREEMPT_TYPE preempt_type   = NONE_PREEMPT;     // 任务的抢占类型
        bool preemptible            = true;             // 任务本⾝是否可以被其他任务抢占

      protected:

        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        Task()
        {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            this->ts = (uint64_t)(ts.tv_sec * 1e6 + ts.tv_nsec * 0.001);                
        }
    };
}

#endif // CONTROLLER_TASK_H