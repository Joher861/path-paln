
#ifndef ENEVT_GLOBAL_COMMON_EVENT_DEF_H
#define ENEVT_GLOBAL_COMMON_EVENT_DEF_H


#include "event_center/event_center.h"
#include "misc/planning_typedefs.h"

using namespace planning_data;


DEFINE_EVENT(ChassisEndSelfTest)
END_EVENT(ChassisEndSelfTest)

DEFINE_EVENT(KeyPowerPress)
END_EVENT(KeyPowerPress)

DEFINE_EVENT(KeyPowerLongPress)
END_EVENT(KeyPowerLongPress)

DEFINE_EVENT(ChassisError)
	int error_id;	
END_EVENT(ChassisError)

DEFINE_EVENT(ChassisWarnning)
	int warnning_id;	
END_EVENT(ChassisWarnning)

DEFINE_EVENT(VoiceBroadcast)
    uint8_t voice_id;
    uint16_t idx;
END_EVENT(VoiceBroadcast)

typedef enum
{
	ENTITY_BUMPER,   //实体
	SIDE_TOF_BUMPER, //侧边
	WALL_DET_BUMPER, //左右红外墙检
	TOF_BUMPER, 	 //3D TOF bumper
	SONAR_BUMPER,    //超声
}BUMPER_TYPE;

DEFINE_EVENT(Bumper)
	BUMPER_TYPE bumper_type;
	int bumper;
	uint64_t ts;   
	int16_t angle; //某种碰撞下的转角参考角度  单位：度
END_EVENT(Bumper)

typedef enum
{
	GROUND_CLIFF,
	MAGNET_CLIFF,
	WHEEL_FALL,
}CLIFF_TYPE;

DEFINE_EVENT(Cliff)
	CLIFF_TYPE cliff_type;
	uint16_t cliff;
	uint64_t ts; 
	int16_t angle;	//某种cliff下的转角参考角度  单位：度
END_EVENT(Cliff)

DEFINE_EVENT(LeftWheelFall)
END_EVENT(LeftWheelFall)

DEFINE_EVENT(RightWheelFall)
END_EVENT(RightWheelFall)

DEFINE_EVENT(LRWheelFall)
END_EVENT(LRWheelFall)

DEFINE_EVENT(EdgeMoveLineEnd)
END_EVENT(EdgeMoveLineEnd);

DEFINE_EVENT(EdgeMoveRotateEnd)
END_EVENT(EdgeMoveRotateEnd)

DEFINE_EVENT(EdgeStartLineRun)
END_EVENT(EdgeStartLineRun)

DEFINE_EVENT(CloseEdgeReachWall)
    int8_t reach_wall_type;   //0:实体墙类型  1：虚拟墙类型  2：其他
END_EVENT(CloseEdgeReachWall)

//local_map数据更新事件
DEFINE_EVENT(LocalMapUpdated)
    bool is_update = false;
END_EVENT(LocalMapUpdated)


#endif 