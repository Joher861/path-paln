#ifndef DATA_DATA_H
#define DATA_DATA_H

#include <string>

#include "misc/planning_defs.h"

namespace planning_data
{
    /**
      * @brief  定义数据类型，新数据类型继承自Data，对每一个子类会添加一个type_name的静态
      *         成员变量，值是和类型名一致的字符串
      * @param  None
      * @retval None
      * @eg     定义DataSlam数据
      *         DEFINE_EVENT(Slam)
      *             Pose pose;       // 具体数据
      *             ...
      *         END_EVENT(Slam)
      */
    #define DEFINE_DATA(DataType)                                              \
        struct Data##DataType : Data                                           \
        {                                                                      \
          public:                                                              \
            Data##DataType() : Data() {}                                       \
            virtual std::string getTypeName() { return type_name; }            \
            inline static const std::string type_name = STR_CONN(Data, DataType);
    #define END_DATA(DataType)                                                 \
        };
        
    /**
      * @desc   pms数据的基类，所有各类的pms数据都要从该类继承
      * @eg     None
      */
    struct Data
    {
      public:
        virtual ~Data() {};

        /**
          * @brief  获取数据类型名
          * @param  None
          * @retval 数据名
          */
        virtual std::string getTypeName()
        {
            return "Data";
        }
    };
}

#endif // DATA_DATA_H