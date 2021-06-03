#ifndef DETECTOR_DETECTOR_CONTEXT_H
#define DETECTOR_DETECTOR_CONTEXT_H

#include "hsm/context.h"

using planning_utils::Context;
using planning_utils::ContextPtr;

/**
  * @brief  创建Context对象
  * @param  T       Context类型
  *         ctx     创建的该Context对象的指针
  * @retval None
  */
#define CREATE_DETECTOR_CONTEXT(T, ctx)                                        \
    std::shared_ptr<T> ctx = T::createDetectorContext()

/**
  * @brief  定义ContextDetector的子结构体
  * @param  T context类型
  * @retval None
  * @eg     定义ContextCloseEdge数据
  *         DEFINE_DETECTOR_CONTEXT(CloseEdge)
  *             ...     // CloseEdge需要保存的具体内部状态
  *         DEFINE_DETECTOR_CONTEXT(CloseEdge)
  */
#define DEFINE_DETECTOR_CONTEXT(T)                                             \
    struct Context##T##Detector : planning_detector::ContextDetector        \
    {                                                                          \
      protected:                                                               \
        Context##T##Detector() : planning_detector::ContextDetector() {}    \
      public:                                                                  \
        static std::shared_ptr<Context##T##Detector> createDetectorContext()   \
        {                                                                      \
            Context##T##Detector *ctx = new Context##T##Detector();            \
            return std::shared_ptr<Context##T##Detector>(ctx);                 \
        }                                                                      \
        Context##T##Detector(const Context##T##Detector &) = delete;           \
        void operator=(const Context##T##Detector &) = delete;                 \
        virtual std::string getTypeName() { return type_name; }                \
        inline static const std::string type_name                              \
            = STR_CONN3(Context, T, Detector);
#define END_DETECTOR_CONTEXT(T)                                                \
    };                                                                         \
    using Context##T##Detector##Ptr = std::shared_ptr<Context##T##Detector>;

namespace planning_detector
{
    DEFINE_CONTEXT(Detector)
    END_CONTEXT(Detector);
}

#endif // DETECTOR_DETECTOR_CONTEXT_H