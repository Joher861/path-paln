
#ifndef CONTROLLER_BASIC_TASK_H
#define CONTROLLER_BASIC_TASK_H

#include "task/task.h"

namespace planning_controller
{
    DEFINE_TASK(Basic, NONE_PREEMPT, true) //TaskBasic
        enum BasicType 
        {
            SPEED_L_R = 0, 
            FORWARD,   
            BACKWARD,  
            ROTATE,    
            LEFT_ROTATE, 
            RIGHT_ROTATE,
            LEFT_CURVE_FORWARD,
            LEFT_CURVE_BACKWARD,
            RIGHT_CURVE_FORWARD,
            RIGHT_CURVE_BACKWARD,
            CURVE_V, 
            CURVE_W, 
            SPEED_V_W,
            STOP,  //停止  无需设置速度参数，但可设置是否smooth及smooth_time
            MAX_BASIC_TYPE
        };

        float v                 = 0.0f;
        float w                 = 0.0f;
        float vl                = 0.0f;
        float vr                = 0.0f;
        BasicType type          = MAX_BASIC_TYPE;
        bool smooth             = false;
        uint64_t smooth_time    = 200;
        float radius            = 0.0f;
    END_TASK(Basic)
}

#endif // CONTROLLER_BASIC_TASK_H