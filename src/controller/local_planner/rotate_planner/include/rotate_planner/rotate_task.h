#ifndef CONTROLLER_ROTATE_TASK_H
#define CONTROLLER_ROTATE_TASK_H

#include "task/task.h"

namespace planning_controller
{
    DEFINE_TASK(Rotate, NONE_PREEMPT, true)
        enum RotateType 
        {
            TARGET_ANGLE = 0,       // 旋转到目标角度
            // DELTA_ANGLE,            // 旋转给定角度
            // DELTA_TIME,             // 旋转给定时间
            MAX_ROTATE_TYPE
        };

        enum RotateDirection
        {
            ROTATE_LEFT = 0,        // 指定向左旋转
            ROTATE_RIGHT,           // 指定向右旋转
            MAX_ROTATE_DIRECTION
        };

        enum RotateReason
        {
            ESCAPE_FRONT_LEFT = 0,        // 由于左前方持续触发
            ESCAPE_LEFT,           // 
            ESCAPE_FRONT_RIGHT,
            ESCAPE_RIGHT,
            ESCAPE_FRONT,
            MAX_ROTATE_REASON
        };

        RotateType type         = MAX_ROTATE_TYPE;          // 旋转的类型
        RotateDirection dir     = MAX_ROTATE_DIRECTION;     // 旋转的方向
        RotateReason reason     = MAX_ROTATE_REASON;
        float rotate_vel        = 0.0f;     // 旋转时车速　m/s
        float target_angle      = 0.0f;     // 目标角度（TARGET_ANGLE）rad
        // float delta_angle       = 0.0f;     // 给定角度（DELTA_ANGLE）
        // uint64_t delta_time     = 0;        // 给定时间（DELTA_TIME）
    END_TASK(Rotate)
}

#endif // CONTROLLER_ROTATE_TASK_H