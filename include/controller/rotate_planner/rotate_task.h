#ifndef CONTROLLER_ROTATE_TASK_H
#define CONTROLLER_ROTATE_TASK_H

#include "task/task.h"

namespace planning_controller
{
    DEFINE_TASK(Rotate, NONE_PREEMPT, true)
        enum RotateType 
        {
            TARGET_ANGLE = 0,       
            MAX_ROTATE_TYPE
        };

        enum RotateDirection
        {
            ROTATE_LEFT = 0,        
            ROTATE_RIGHT,           
            MAX_ROTATE_DIRECTION
        };

        enum RotateReason
        {
            ESCAPE_FRONT_LEFT = 0,  
            ESCAPE_LEFT,           
            ESCAPE_FRONT_RIGHT,
            ESCAPE_RIGHT,
            ESCAPE_FRONT,
            MAX_ROTATE_REASON
        };

        RotateType type         = MAX_ROTATE_TYPE;          
        RotateDirection dir     = MAX_ROTATE_DIRECTION;     
        RotateReason reason     = MAX_ROTATE_REASON;
        float rotate_vel        = 0.0f;     
        float target_angle      = 0.0f;     
    END_TASK(Rotate)
}

#endif // CONTROLLER_ROTATE_TASK_H