#ifndef DETECTOR_DETECTOR_PARAM_H
#define DETECTOR_DETECTOR_PARAM_H

#include <memory>
#include <typeindex>

#include "misc/planning_defs.h"

/**
  * @brief  创建Param对象
  * @param  T       Param类型
  *         param   创建的该param对象的指针
  * @retval None
  */
#define CREATE_DETECTOR_PARAM(T, param)                                        \
    std::shared_ptr<T> param = T::createDetectorParam()

/**
  * @brief  定义检测器参数类型，新任参数类型继承自DetectorParam
  * @param  None
  * @retval None
  * @eg     定义DetectorParam输入 
  *         DEFINE_DETECTOR_PARAM(CloseEdge)
  *             RobotPose start_pose;       // 具体参数数据
  *             ...
  *         END_DETECTOR_PARAM(CloseEdge)
  */
#define DEFINE_DETECTOR_PARAM(T)                                               \
    struct T##DetectorParam;                                                   \
    using T##DetectorParam##Ptr                                                \
        = std::shared_ptr<T##DetectorParam>;                                   \
    struct T##DetectorParam : planning_detector::DetectorParam              \
    {                                                                          \
      protected:                                                               \
        T##DetectorParam() : planning_detector::DetectorParam() {}          \
      public:                                                                  \
        static T##DetectorParam##Ptr createDetectorParam()                     \
        {                                                                      \
            T##DetectorParam *input = new T##DetectorParam();                  \
            return T##DetectorParam##Ptr(input);                               \
        }                                                                      \
        T##DetectorParam(const T##DetectorParam &) = delete;                   \
        void operator=(const T##DetectorParam &) = delete;                     \
        virtual std::string getTypeName() const                                \
        {                                                                      \
            return type_name;                                                  \
        }                                                                      \
        inline static const std::string type_name                              \
            = STR_CONN(T, DetectorParam);
#define END_DETECTOR_PARAM(T)                                                  \
    };

namespace planning_detector
{
    /**
     * @desc    检测器参数基类
     * @eg      
     */
    struct DetectorParam
    {
      public:

        static std::shared_ptr<DetectorParam> createDetectorParam()
        {
            DetectorParam *input = new DetectorParam();
            return std::shared_ptr<DetectorParam>(input);
        }

        // 禁止拷贝构造和赋值函数
        DetectorParam(const DetectorParam &) = delete;
        void operator=(const DetectorParam &) = delete;

        virtual ~DetectorParam() {};

        /**
          * @brief  获取DetectorParam类型名
          * @param  None
          * @retval 数据名
          */
        virtual std::string getTypeName() const
        {
            return "DetectorParam";
        }

      protected:

        DetectorParam()
        {}

    };
    using DetectorParamPtr = std::shared_ptr<DetectorParam>; 
}

#endif // DETECTOR_DETECTOR_PARAM_H
