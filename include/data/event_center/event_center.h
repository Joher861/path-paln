
#ifndef EVENT_CENTER_EVENT_CENTER_H
#define EVENT_CENTER_EVENT_CENTER_H

#include <cinttypes>
#include <typeinfo>
#include <typeindex>
#include <mutex>
#include <map>
#include <list>

#include "event.h"
#include "config/config_manager.h"

using planning_utils::ConfigManager;

namespace planning_data
{
    using EvtList = std::list<EventPtr>;
    
    class EventCenter
    {
      public:

        /**
          * @brief  获取事件中心的单例
          * @param  None
          * @retval None
          */
        static EventCenter &getInstance();

        // 禁止拷贝和赋值构造函数
        EventCenter(const EventCenter &) = delete;
        void operator=(const EventCenter &) = delete;

        void init(ConfigManager &cfg_mgr);

        /**
          * @brief  添加事件
          * @param  event 事件的指针
          * @retval None
          */
        void pushEvent(EventPtr event);

        /**
          * @brief  获取(last_ts, cur_ts]时间段内发生的所有事件，删除超时的事件
          *         一般在线程内每个周期调用该函数去查看上个周期内发生了哪些时间
          * @param  last_ts   开始时间点
          *         cur_ts    结束时间点
          *         evt_list  获取到的所有事件存放的list，按事件发生顺序先后排列
          * @retval None
          */
        void eventDeduction(uint64_t last_ts, uint64_t cur_ts,
                            std::list<EventPtr>& evt_list);
      private:

        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        EventCenter();

        std::list<EventPtr> m_evt_list;      // 保存所有事件的列表
        std::mutex m_evt_list_mtx;          // 列表的互斥量
        uint64_t m_evt_timeout;             // 事件在事件中心存在的时长
    };
}

extern planning_data::EventCenter& g_ec;   // 事件中心的全局变量
#endif