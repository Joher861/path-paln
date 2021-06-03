#include "task_manager.h"

#include <iostream>
#include <typeindex>

#include "task_log.h"
#include "task_config.h"

#include "data/event_center/event_center.h"
#include "local_planner/local_planner_events.h"
#include "timer/timer.h"
#include "timer/sleep_timer.h"
#include "misc/planning_common_config.h"

using namespace planning_controller;
using namespace planning_utils;
using namespace std;

int64_t TaskManager::m_task_id_cnt = 0;

DEFINE_CONFIG_TYPE(CONFIG_TASK, Task);
TaskManager& g_tm = TaskManager::getInstance();

TaskManager& TaskManager::getInstance()
{
    static TaskManager instance;
    return instance;
}

TaskManager::TaskManager()
    : m_cur_task(nullptr), m_task_polling_thread(nullptr)
{}

TaskManager::~TaskManager()
{
    m_run_polling = false;

    SleepTimer t(10000.0f);
    while (m_polling_thread_stopped)
    {
        t.sleep();
    }

    if (m_task_polling_thread)
    {
        delete m_task_polling_thread;
        m_task_polling_thread = nullptr;
    }
}

void TaskManager::init(ConfigManager &cfg_mgr)
{
    ConfigTask *cfg_task = dynamic_cast<ConfigTask*>(
        cfg_mgr.GetSubConfig(CONFIG_TASK));
    ConfigPlaning *cfg_planning = dynamic_cast<ConfigPlaning*>(
        cfg_mgr.GetSubConfig(CONFIG_PLANNING));
    
    cfg_task->log_path = cfg_planning->log_path;

    CREATE_LOG(PlainText, LOG_TASK_FLAG, LOG_TASK,
        cfg_task->log_name, cfg_task->log_path,
        cfg_task->log_extension, cfg_task->log_ts_mask,
        cfg_task->log_print_to_console,
        (cfg_task->log_max_file_size_mb) MB
        + (cfg_task->log_max_file_size_kb) KB,
        cfg_task->log_max_file_cnt, cfg_task->log_level);

    m_run_polling = true;
    m_polling_thread_stopped = false;
    m_task_polling_frequency = cfg_task->task_polling_frequency;
    m_task_polling_thread = new std::thread(&TaskManager::taskPolling, this);
    m_task_polling_thread->detach();
}


int64_t TaskManager::addTask(TaskPtr task)
{
    unique_lock<mutex> lock(m_task_list_mtx);
    if (++m_task_id_cnt < 0)
        m_task_id_cnt = 0;

    task->id = m_task_id_cnt;
    task->ts = Timer::getSystemTimestampUS();

    TASK_INFO_LOG("add task");
    TASK_INFO_LOG("task type            = %s", task->getTypeName().c_str());
    TASK_INFO_LOG("     id              = %ld", task->id);
    TASK_INFO_LOG("     ts              = %lu", task->ts);
    TASK_INFO_LOG("     premmpt type    = %d", task->preempt_type);
    TASK_INFO_LOG("     preemptible     = %d",
        static_cast<int>(task->preemptible));
    
    if(m_task_list.empty() && m_cur_task == nullptr) // 
    {
        TASK_DEBUG_LOG("no task running currently, start this task");
        m_cur_task = task;
        if(dispatchTask(m_cur_task) == false)
        {
            //分发任务失败，异常处理
        }   
    }
    else
    {
        if(task->preempt_type == NONE_PREEMPT) //新任务顺序执行的
        {
            TASK_DEBUG_LOG("push task to the end of task list");
            m_task_list.push_back(task);
        }
        else if(task->preempt_type == PREEMPT_DELETE) //新任务抢占并删除之前所有任务
        {
            if(m_cur_task->preemptible == false) //当前任务本身不能被抢占
            {
                TASK_DEBUG_LOG("clear all task in list, add this task to the " \
                    "end");
                m_task_list.clear();
                m_task_list.push_front(task);
            }
            else 
            {
                TASK_DEBUG_LOG("stop current task, clear all task in list, "   \
                    "and start this task");
                stopCurrentTask(m_cur_task);
                m_task_list.clear();
                m_cur_task = task;
                if(dispatchTask(m_cur_task) == false)
                {
                    //分发任务失败，异常处理
                }
            }
        }
        else if(task->preempt_type == PREEMPT) //新任务抢占优先执行
        {
            if(m_cur_task->preemptible == false) //当前任务本身不能被抢占
            {
                TASK_DEBUG_LOG("add this task to the end");
                m_task_list.push_front(task);         
            }
            else 
            {
                TASK_DEBUG_LOG("stop current task, put current task back to, " \
                    "the front of list, and start this task");
                stopCurrentTask(m_cur_task);
                m_task_list.push_front(m_cur_task);
                m_cur_task = task;
                if(dispatchTask(m_cur_task) == false)
                {
                    //分发任务失败，异常处理
                }                
            }
        }
    }    

    return task->id;
}

void TaskManager::deleteTask(int64_t task_id)
{
    TASK_INFO_LOG("delete task, id = %ld", task_id);

    std::list<TaskPtr>::iterator it;
    unique_lock<mutex> lock(m_task_list_mtx);
    if(m_cur_task->id == task_id)  //需要删除的任务就是当前执行任务
    {
        TASK_DEBUG_LOG("delete cur task");
        stopCurrentTask(m_cur_task);
        if(m_task_list.empty())
        {
            m_cur_task = nullptr;
        }
        else 
        {
            TASK_DEBUG_LOG("start a new task");
            m_cur_task = m_task_list.front();
            m_task_list.pop_front();
            if(dispatchTask(m_cur_task) == false)
            {
                //分发任务失败，异常处理
            }           
        }
    }
    else 
    {
        TASK_DEBUG_LOG("delete task in the list");
        // TASK_DEBUG_LOG("current m_task_list size is: %d", m_task_list.size());
        // for(auto task_t : m_task_list)
        // {
        //     TASK_DEBUG_LOG("task id: %d \n", task_t->id);
        //     TASK_DEBUG_LOG("task name: %s \n", task_t->getType().name());
        //     TASK_DEBUG_LOG("task ts: %lu \n", task_t->ts);
        //     TASK_DEBUG_LOG("task preemptible: %d \n", task_t->preemptible);
        //     TASK_DEBUG_LOG("task preempt_type: %d \n", task_t->preempt_type);
        //     TASK_DEBUG_LOG("\n");
        // }        

        if(m_task_list.empty())
        {
            TASK_WARN_LOG("task list is emptym no task deleted");
        }
        else 
        {
            //遍历任务列表
            for(it = m_task_list.begin(); it != m_task_list.end(); it++)
            {
                if((*it)->id == task_id) //需要删除的任务存在任务列表里
                {
                    TASK_DEBUG_LOG("find task %ld in the list, delete it",
                        task_id);
                    m_task_list.remove(*it);
                    break;
                }
            }            
        }
    }
}

bool TaskManager::dispatchTask(const TaskPtr task)
{
    TASK_INFO_LOG("dispatch task, id = %ld, type = %s", task->id,
        task->getTypeName().c_str());
    shared_lock<shared_timed_mutex> lock(m_local_planners_mtx);
    if (!task)
    {
        TASK_ERROR_LOG("dispatch task failed, task is nullptr");
        return false;
    }
    type_index task_type_id = typeid(*task);
    if (m_local_planners.find(task_type_id) == m_local_planners.end())
    {
        TASK_ERROR_LOG("dispatch task failed, task type %s is not registered",
            task->getTypeName().c_str());
        return false;
    }

    LocalPlannerPtr planner = m_local_planners[task_type_id];
    TASK_INFO_LOG("planner:%s", typeid(*planner).name());
    if (planner == nullptr)
    {
        TASK_ERROR_LOG("dispatch task failed, corresponding local planner is " \
            "nullptr");
        return false;
    }
    
    planner->startTask(*task);

    return true;
}

bool TaskManager::stopCurrentTask(const TaskPtr cur_task)
{
    TASK_DEBUG_LOG("stopCurrentTask cur_task_id:%d, cur_task_name:%s, cur_task_ts:%lu, cur_task_preemptible:%d, cur_task_preempt_type:%d \n", cur_task->id, cur_task->getType().name(), cur_task->ts, cur_task->preemptible, cur_task->preempt_type);
    shared_lock<shared_timed_mutex> lock(m_local_planners_mtx);

    type_index cur_task_type_id = typeid(*cur_task);
    if (m_local_planners.find(cur_task_type_id) == m_local_planners.end())
    {
        TASK_ERROR_LOG("stop current task failed, failed to find the "         \
            "corresponding planner");
        return false;
    }

    LocalPlannerPtr planner = m_local_planners[cur_task_type_id];
    if (planner == nullptr)
    {
        TASK_ERROR_LOG("stop current task failed, corresponding planner is "   \
            "nullptr");
        return false;
    }
    
    planner->stopTask();
    return true;
}


void TaskManager::taskPolling()
{
    static uint64_t last_deduction_ts = Timer::getSystemTimestampUS();
    static std::list<EventPtr> evt_list;  

    SleepTimer t(m_task_polling_frequency);

    while (m_run_polling)
    {
        // TASK_DEBUG_LOG("current task list size is = %u", m_task_list.size());
        for(auto task_t : m_task_list)
        {
            TASK_DEBUG_LOG("task id:            %d ", task_t->id);
            TASK_DEBUG_LOG("     name:          %s ",
                task_t->getTypeName().c_str());
            TASK_DEBUG_LOG("     ts:            %lu ", task_t->ts);
            TASK_DEBUG_LOG("     preempt_type:  %d ", task_t->preempt_type);
            TASK_DEBUG_LOG("     preemptible:   %d ",
                static_cast<int>(task_t->preemptible));
            TASK_DEBUG_LOG("");
        }

        unique_lock<mutex> lock(m_task_list_mtx);

        // 获取事件
        evt_list.clear();
        uint64_t cur_ts = Timer::getSystemTimestampUS();
        g_ec.eventDeduction(last_deduction_ts, cur_ts, evt_list);
        last_deduction_ts = cur_ts;

        for (auto evt : evt_list)
        {
            std::type_index type = evt->getType();
            if (TYPE_EQUALS(type, EvTaskFinished))
            {
                EvTaskFinishedPtr finished_evt
                    = std::dynamic_pointer_cast<EvTaskFinished>(evt);
                TASK_INFO_LOG("get task finished evt, id = %ld",
                    finished_evt->task_id);

                if(finished_evt->task_id == m_cur_task->id)
                {
                    TASK_INFO_LOG("current task finished, id = %ld, type = %s",
                        m_cur_task->id, m_cur_task->getTypeName().c_str());
                    if(m_task_list.empty())
                    {
                        m_cur_task = nullptr;
                        break;
                    }
                    else 
                    {
                        m_cur_task = m_task_list.front();
                        m_task_list.pop_front();
                        if(dispatchTask(m_cur_task) == false)
                        {
                            //分发任务失败，异常处理
                        }
                        break;                            
                    }
                }
                else
                {
                    bool in_list = false;
                    for (auto it = m_task_list.begin();
                        it != m_task_list.end(); ++it)
                    {
                        if ((*it)->id == finished_evt->task_id)
                        {
                            TASK_ERROR_LOG("finished task is not the "         \
                                "current running task, but in task list");
                            m_task_list.erase(it);
                            in_list = true;
                            break;
                        }
                    }

                    if (!in_list)
                    {
                        TASK_WARN_LOG("finished task is not the "
                            "current running task, and not in task list");
                    }
                }
            }
        }

        if(m_cur_task == nullptr && m_task_list.empty())
        {
            //当前无任务执行
        }
        else if(m_cur_task == nullptr && (!m_task_list.empty()))
        {
            m_cur_task = m_task_list.front();
            m_task_list.pop_front();
            TASK_DEBUG_LOG("start a new task");
            TASK_DEBUG_LOG("task id:            %d ", m_cur_task->id);
            TASK_DEBUG_LOG("     name:          %s ",
                m_cur_task->getTypeName().c_str());
            TASK_DEBUG_LOG("     ts:            %lu ", m_cur_task->ts);
            TASK_DEBUG_LOG("     preempt_type:  %d ", m_cur_task->preempt_type);
            TASK_DEBUG_LOG("     preemptible:   %d ",
                static_cast<int>(m_cur_task->preemptible));
            TASK_DEBUG_LOG("");
            if(dispatchTask(m_cur_task) == false)
            {
                //分发任务失败，异常处理
            }
        }

        lock.unlock();

        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        t.sleep();
    }

    m_polling_thread_stopped = true;
}





