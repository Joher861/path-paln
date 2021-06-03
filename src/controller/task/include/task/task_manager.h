
#ifndef CONTROLLER_TASK_MANAGER_H
#define CONTROLLER_TASK_MANAGER_H

#include <map>
#include <list>
#include <shared_mutex>
#include <thread>

#include "task.h"
#include "include/task_log.h"
#include "local_planner/local_planner.h"

#include "log/log_manager.h"
#include "config/config_manager.h"

using planning_utils::ConfigManager;

namespace planning_controller
{
    class TaskManager
    {
      public:
        /**
          * @brief  获取事件中心的单例
          * @param  None
          * @retval None
          */
        static TaskManager &getInstance();

        // 禁止拷贝和赋值构造函数
        TaskManager(const TaskManager &) = delete;
        void operator=(const TaskManager &) = delete;

        ~TaskManager();

        void init(ConfigManager &cfg_mgr);

        /**
          * @brief  添加任务
          * @param  task    任务的共享指针对象
          * @retval 被添加任务分配到的任务id
          */
        int64_t addTask(TaskPtr task);

        /**
          * @brief  删除任务
          * @param  task_id     需要删除任务的任务id
          * @retval None
          */
        void deleteTask(int64_t task_id);

        /**
          * @brief  注册某种任务对应的局部规划器对象
          * @param  TaskType    某类任务的type_index对象
          *         planner     local_planner的共享指针对象
          * @retval None
          */
        template<typename TaskType>
        void registerPlanner(LocalPlannerPtr planner)
        {
            std::unique_lock<std::shared_timed_mutex> lock(m_local_planners_mtx);
            TASK_INFO_LOG("register task type %s", TaskType::type_name.c_str());
            m_local_planners[typeid(TaskType)] = planner;
        }
    
      private:
        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        TaskManager();

        /**
          * @brief  像task对应的局部规划器分发任务 就是开始任务
          * @param  task    要分配的任务的指针对象
          * @retval 分发是否成功，true成功，false失败
          */
        bool dispatchTask(const TaskPtr task);

        /**
          * @brief  停止当前执行任务
          * @param  cur_task    当前执行任务的指针对象
          * @retval 停止是否成功，true成功，false失败
          */
        bool stopCurrentTask(const TaskPtr cur_task);        


        /**
          * @brief  任务管理线程函数
          * @param  None
          * @retval None
          */
        void taskPolling();

        static int64_t m_task_id_cnt;                   // task_id分配计数器
        std::list<TaskPtr> m_task_list;                 // 存放所有任务的列表
        std::mutex m_task_list_mtx;                     // 任务列表的互斥量
        TaskPtr m_cur_task;                             // 当前正在执行的任务
        std::map<std::type_index, LocalPlannerPtr> m_local_planners;
                                                // 各类任务和其局部规划器的对应表
        std::shared_timed_mutex m_local_planners_mtx;   // 对应表的互斥量
        float m_task_polling_frequency;
        std::thread *m_task_polling_thread;              // 任务管理线程
        bool m_run_polling = false;
        bool m_polling_thread_stopped = false;
    };
}

extern planning_controller::TaskManager& g_tm;       // 任务管理器的全局对象

#endif // CONTROLLER_TASK_MANAGER_H