#ifndef EVENT_CENTER_EVENT_H
#define EVENT_CENTER_EVENT_H

#include <cinttypes>
#include <typeindex>
#include <string>
#include <memory>

#include "misc/planning_defs.h"
#include "timer/timer.h"

using planning_utils::Timer;

namespace planning_data
{
    /**
      * @brief  创建事件对象
      * @param  EventType   某一事件类型
      *         evt         创建的该事件对象的指针
      * @retval None
      */
    #define CREATE_EVENT(EventType, evt)                                       \
        std::shared_ptr<EventType> evt                                         \
            = (EventType::createEvent())
    
    /**
      * @brief  定义事件类型，新事件类型继承自Event，对每一个子类会添加一个type_name的静态
      *         成员变量，值是和类型名一致的字符串
      * @param  T   事件类型，最终的事件名将被拓展成EvT这样的类型
      * @retval None
      * @eg     定义EvBump事件
      *         DEFINE_EVENT(Bump)
      *             int bump;       // 事件对应的数据
      *             ...
      *         END_EVENT(Bumper)
      */
    #define DEFINE_EVENT(T)                                                    \
        struct Ev##T : Event                                                   \
        {                                                                      \
          protected: Ev##T() : Event() {}                                      \
          public:                                                              \
            static std::shared_ptr<Ev##T> createEvent()                        \
            {                                                                  \
                Ev##T * evt = new Ev##T;                                       \
                return std::shared_ptr<Ev##T>(evt);                            \
            }                                                                  \
            virtual std::string getTypeName() { return type_name; }            \
            inline static const std::string type_name                          \
                = STR_CONN(Ev, T);
    #define END_EVENT(T)                                                       \
        };                                                                     \
        using Ev##T##Ptr = std::shared_ptr<Ev##T>;


    struct Event;

    using EventPtr = std::shared_ptr<Event>;    

    /**
      * @desc   事件基类
      * @eg     所有事件和事件的基类，所有时间的定义和创建用上面的宏定义
      */
    struct Event
    {
      public:

        /**
          * @brief  创建事件（事件都创建在堆上，由EventCenter释放）
          * @param  None
          * @retval None
          */
        static EventPtr createEvent()
        {
            Event* evt = new Event;
            return EventPtr(evt);
        }

        virtual ~Event() {};

        /**
          * @brief  获取本事件类型的type_index
          * @param  None
          * @retval 本事件类型的type_index
          */
        inline std::type_index getType()
        {
            return typeid(*this);
        }

        /**
          * @brief  判断当前事件是否为某一类型
          * @param  T   目标类型
          * @retval 当前时间为T类型返回true，否则返回false
          */
        template <typename T>
        inline bool isType()
        {
            return this->getType() == typeid(T);
        }

        /**
          * @brief  获取事件类型名
          * @param  None
          * @retval 事件名
          */
        virtual std::string getTypeName()
        {
            return "Event";
        }

        /* 所有事件的共有成员 */
        uint64_t ts;                        // 事件发生时间戳（系统上电时间），单位us，

      protected:

        /**
          * @brief  构造函数
          * @param  None
          * @retval None
          */
        Event()
        {
            this->ts = Timer::getSystemTimestampUS();
        }
    };
}

#endif