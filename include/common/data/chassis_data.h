
#ifndef DATA_CENTER_CHASSIS_DATA_H
#define DATA_CENTER_CHASSIS_DATA_H

#include "misc/planning_typedefs.h"
#include "data_center/data.h"

using planning_data::Data;

// 地检
struct GroundDet
{
    uint8_t lb_det;
    uint8_t l_det;
    uint8_t lf_det;
    uint8_t rf_det;
    uint8_t r_det;
    uint8_t rb_det;
    uint16_t ground_det;
};

// 按键
struct KeySignal
{
    uint8_t key_press;
    uint8_t key_long_press;
};

// 传感器
DEFINE_DATA(Sensor)
    uint8_t bumper;     // 0, 1, 2, 3
    uint8_t magnet;
    uint8_t wheel_fall;
    GroundDet gnd_det;
    KeySignal key_signal;
    // uint8_t charging;
    uint8_t carpet;
END_DATA(Sensor)

// 充电信号
DEFINE_DATA(ChargeSignal)
    bool charge_signal = false;
END_DATA(ChargeSignal)

// 速度反馈
DEFINE_DATA(FeedbackSpeed)
    int32_t l_encoder;  //mm
    int32_t r_encoder;  //mm
    float l_fb_speed;   //m/s
    float r_fb_speed;   //m/s
    float v;            //m/s
    float w;            //rad/s
    uint64_t ts;        
END_DATA(FeedbackSpeed)

// 底盘反馈控制模式
DEFINE_DATA(ChassisCtrlMode)
    uint8_t ctrl_mode;
END_DATA(ChassisCtrlMode)

// 红外墙检
DEFINE_DATA(WallDet)
    uint8_t l_det;      //左墙检触发标志
    uint8_t lf_det;     //左前墙检触发标志
    uint8_t rf_det;     //右前墙检触发标志
    uint8_t r_det;      //右墙检触发标志
    uint16_t l_val;     //左墙检adc值
    uint16_t lf_val;    //左前墙检adc值
    uint16_t rf_val;    //右前墙检adc值
    uint16_t r_val;     //右墙检adc值

    uint16_t lb_gd_val; //左后地检adc值
    uint16_t l_gd_val;  //左地检adc值
    uint16_t lf_gd_val; //左前地检adc值
    uint16_t rf_gd_val; //右前地检adc值
    uint16_t r_gd_val;  //右地检adc值
    uint16_t rb_gd_val; //右后地检adc值
END_DATA(WallDet)

// 执行机构（电机类）电流（adc）值
DEFINE_DATA(ActuatorAdc)
    uint16_t left_side_brush_adc;
    uint16_t right_side_brush_adc;
    uint16_t main_brush_adc;
    uint16_t fan_adc;
    uint16_t left_wheel_adc;
    uint16_t right_wheel_adc;
    uint16_t water_pump_adc;
END_DATA(ActuatorAdc)

// 超声
DEFINE_DATA(Ultrasonic)
    uint16_t ultrasonic_01;
    uint16_t ultrasonic_02;
    uint16_t ultrasonic_03;
    uint16_t ultrasonic_04;
    uint16_t ultrasonic_05;
    uint16_t ultrasonic_06;
    uint16_t ultrasonic_07;
    uint16_t ultrasonic_08;
    uint16_t ultrasonic_09;
    // uint16_t ultrasonic_10;
    // uint16_t ultrasonic_11;
    // uint16_t ultrasonic_12;
END_DATA(Ultrasonic)

// 伺服数据
DEFINE_DATA(Servo)
    uint8_t status;
    uint8_t calib_status;
    int16_t angle;
END_DATA(Servo)

// 侧边沿墙tof
DEFINE_DATA(SideTof)
    uint8_t rf_val; // 0 - 255
    uint8_t r_val;
END_DATA(SideTof)

// 电池信息
DEFINE_DATA(Battery)
    uint8_t percent;
END_DATA(Battery)

// // 错误信息
// DEFINE_DATA(ChassisError)
//     uint8_t id;
//     uint8_t error;
// END_DATA(ChassisError)

// 底盘组件状态 (错误信息)
DEFINE_DATA(ChassisModulesStaus)
    uint8_t small_wheel;
    uint8_t degree;
    uint8_t bump_single_count;
    uint8_t gnd_detect;
    uint8_t front_c_long_trig;
    uint8_t bump_long_trig;
    uint8_t wheel_l_motor_cur;
    uint8_t wheel_r_motor_cur;
    uint8_t side_l_motor_cur;
    uint8_t side_r_motor_cur;
    uint8_t roll_motor_cur;

    uint8_t water_box_not;
	uint8_t dust_box_not;
	uint8_t battery_err;
	uint8_t rtof_err;
	uint8_t ltof_err;
	uint8_t rag_not;
	uint8_t dust_box_full;
	uint8_t fan_motor_oc;
	uint8_t pump_oc;
	uint8_t l_impact_err;
	uint8_t r_impact_err;
END_DATA(ChassisModulesStaus);

DEFINE_DATA(ActuatorInfo)
    uint8_t fan;			/* fan motor level */
    uint8_t main_brush;	/* main brush level */	
    uint8_t side_brush;
    uint8_t pump;
    int8_t servo; /* servo angle ctrl*/
    uint8_t servo_ctrl_mode; /* 0:角度控制   1：精准校零控制   2：校零完成*/    
END_DATA(ActuatorInfo);

DEFINE_DATA(ChassisVersion)
    uint8_t type;
	uint8_t sw[14];
	uint8_t hw[6];
END_DATA(ChassisVersion)


DEFINE_DATA(AllChassisInfo)
    DataSensor sensor;
    DataChargeSignal charge_signal;
    DataFeedbackSpeed feedback_speed;
    DataChassisCtrlMode ctrl_mode;
    DataWallDet wall_det;
    DataActuatorAdc actuator_adc;
    DataUltrasonic ultrasonic;
    DataServo servo;
    DataSideTof side_tof;
    DataBattery battery;
    DataChassisModulesStaus modules_status;
    DataActuatorInfo actuator_info;
    DataChassisVersion software_version;
END_DATA(AllChassisInfo)

#endif // DATA_CENTER_CHASSIS_DATA_H