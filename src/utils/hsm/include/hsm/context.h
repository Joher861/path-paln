#ifndef UTILS_CONTEXT_H
#define UTILS_CONTEXT_H

#include <string>
#include <memory>

#include "misc/planning_defs.h"

/**
  * @brief  创建Context对象
  * @param  T       Context类型
  *         ctx     创建的该context对象的指针
  * @retval None
  */
#define CREATE_CONTEXT(T, ctx)                                                 \
    std::shared_ptr<T> ctx = T::createContext()

/**
  * @brief  定义Context的子结构体
  * @param  T context类型
  * @retval None
  * @eg     定义ContextNavigation数据
  *         DEFINE_CONTEXT(Navigation)
  *             ...     // NavigationPlanner需要保存的具体内部状态
  *         END_CONTEXT(Navigation)
  */
#define DEFINE_CONTEXT(T)                                                      \
    struct Context##T : planning_utils::Context                             \
    {                                                                          \
      protected:                                                               \
        Context##T() : planning_utils::Context() {}                         \
      public:                                                                  \
        static std::shared_ptr<Context##T> createContext()                     \
        {                                                                      \
            Context##T *ctx = new Context##T();                                \
            return std::shared_ptr<Context##T>(ctx);                           \
        }                                                                      \
        Context##T(const Context##T &) = delete;                               \
        void operator=(const Context##T &) = delete;                           \
        virtual std::string getTypeName() { return type_name; }                \
        inline static const std::string type_name = STR_CONN(Context, T);
#define END_CONTEXT(T)                                                         \
    };                                                                         \
    using Context##T##Ptr = std::shared_ptr<Context##T>;

namespace planning_utils
{
    /**
      * @desc   状态机运行上下文环境基类
      * @eg     None
      */
    struct Context
    {
      public:
      
        /**
          * @brief  创建上下文
          * @param  None
          * @retval None
          */
        static std::shared_ptr<Context> createContext()
        {
            Context *ctx = new Context();
            return std::shared_ptr<Context>(ctx);
        }

        virtual ~Context() {}

        // 禁止拷贝构造和赋值函数
        Context(const Context &) = delete;
        void operator=(const Context &) = delete;

        /**
          * @brief  获取Context类型名
          * @param  None
          * @retval 数据名
          */
        virtual std::string getTypeName()
        {
            return "Context";
        }

      protected:

        Context() {}
    };
    using ContextPtr = std::shared_ptr<Context>;
}

#endif // UTILS_CONTEXT_H