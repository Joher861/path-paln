
#ifndef CHASSIS_LOG_H
#define CHASSIS_LOG_H

#include "log/csv_log_data.h"
#include "log/log_manager.h"

using namespace planning_utils;

// 模式状态信息
struct CSVChassisMsgGeneralData : CSVLogData
{
    uint8_t msg_id;
    uint8_t data;

    CSVChassisMsgGeneralData(uint8_t _msg_id, uint8_t _data)
        : msg_id(_msg_id), data(_data)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(msg_id) + delimiter
                            + std::to_string(data);

        return log_str;
    }
};
#define LOG_CHASSIS_MSG_GENERAL_FLAG chassis_msg_general
#define LOG_CHASSIS_MSG_GENERAL      "chassis_msg_general"

#define CHASSIS_MSG_GENERAL_LOG(msg_id, data)   \
    LOG(LOG_NAMES(LOG_CHASSIS_MSG_GENERAL), std::make_shared<CSVChassisMsgGeneralData>(msg_id, data).get())


// 传感器
struct CSVChassisSensorData : CSVLogData
{
    uint8_t bumper;
    uint8_t magnet;
    uint8_t wheel_fall;
    uint16_t ground_det;
    uint16_t l_det_val;
    uint16_t lf_det_val;
    uint16_t rf_det_val;
    uint16_t r_det_val;
    uint8_t key_press;
    uint8_t key_long_press;
    uint8_t carpet;

    CSVChassisSensorData(uint8_t _bumper, uint8_t _magnet, uint8_t _wheel_fall, uint16_t _ground_det, uint16_t _l_det_val,
                         uint16_t _lf_det_val, uint16_t _rf_det_val, uint16_t _r_det_val, uint8_t _key_press, uint8_t _key_long_press, uint8_t _carpet)
        : bumper(_bumper), magnet(_magnet), wheel_fall(_wheel_fall), ground_det(_ground_det), l_det_val(_l_det_val), lf_det_val(_lf_det_val),
          rf_det_val(_rf_det_val), r_det_val(_r_det_val), key_press(_key_press), key_long_press(_key_long_press), carpet(_carpet)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(bumper) + delimiter
                            + std::to_string(magnet) + delimiter
                            + std::to_string(wheel_fall) + delimiter
                            + std::to_string(ground_det) + delimiter
                            + std::to_string(l_det_val) + delimiter
                            + std::to_string(lf_det_val) + delimiter
                            + std::to_string(rf_det_val) + delimiter
                            + std::to_string(r_det_val) + delimiter
                            + std::to_string(key_press) + delimiter
                            + std::to_string(key_long_press) + delimiter
                            + std::to_string(carpet);

        return log_str;
    }
};
#define LOG_CHASSIS_SENSOR_FLAG chassis_sensor
#define LOG_CHASSIS_SENSOR      "chassis_sensor"

#define CHASSIS_SENSOR_LOG(bumper, magnet, wheel_fall, ground_det, l_det_val, lf_det_val, rf_det_val, r_det_val, key_press, key_long_press, carpet)   \
    LOG(LOG_NAMES(LOG_CHASSIS_SENSOR), std::make_shared<CSVChassisSensorData>(bumper, magnet, wheel_fall, ground_det, l_det_val, lf_det_val, rf_det_val, r_det_val, key_press, key_long_press, carpet).get())


// 速度反馈
struct CSVChassisOdoData : CSVLogData
{
    int32_t l_encoder;  //mm
    int32_t r_encoder;  //mm
    float l_fb_speed;   //m/s
    float r_fb_speed;   //m/s
    float v;            //m/s
    float w;            //rad/s
    uint64_t ts; 

    CSVChassisOdoData(int32_t _l_encoder, int32_t _r_encoder,float _l_fb_speed,float _r_fb_speed,float _v,float _w,uint64_t _ts)
        : l_encoder(_l_encoder), r_encoder(_r_encoder),l_fb_speed(_l_fb_speed),r_fb_speed(_r_fb_speed),v(_v),w(_w),ts(_ts)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(l_encoder) + delimiter
							+ std::to_string(r_encoder) + delimiter
							+ std::to_string(l_fb_speed) + delimiter
							+ std::to_string(r_fb_speed) + delimiter
							+ std::to_string(v) + delimiter
							+ std::to_string(w) + delimiter
                            + std::to_string(ts);
        return log_str;
    }
};
#define LOG_CHASSIS_ODO_FLAG 	chassis_odo
#define LOG_CHASSIS_ODO    		"chassis_odo"

#define CHASSIS_ODO_LOG(l_encoder, r_encoder, l_fb_speed,r_fb_speed,v,w,ts)   \
    LOG(LOG_NAMES(LOG_CHASSIS_ODO), std::make_shared<CSVChassisOdoData>(l_encoder, r_encoder, l_fb_speed,r_fb_speed,v,w,ts).get())


// 红外墙检、地检
struct CSVInfraredDetData : CSVLogData
{
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

    CSVInfraredDetData(uint8_t _l_det, uint8_t _lf_det,uint8_t _rf_det,uint8_t _r_det,uint16_t _l_val,uint16_t _lf_val,uint16_t _rf_val,uint16_t _r_val,uint16_t _lb_gd_val,uint16_t _l_gd_val,uint16_t _lf_gd_val,uint16_t _rf_gd_val,uint16_t _r_gd_val,uint16_t _rb_gd_val)
        : l_det(_l_det), lf_det(_lf_det),rf_det(_rf_det),r_det(_r_det),l_val(_l_val),lf_val(_lf_val),rf_val(_rf_val),r_val(_r_val),lb_gd_val(_lb_gd_val),l_gd_val(_l_gd_val),lf_gd_val(_lf_gd_val),rf_gd_val(_rf_gd_val),r_gd_val(_r_gd_val),rb_gd_val(_rb_gd_val)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(l_det) + delimiter
							+ std::to_string(lf_det) + delimiter
							+ std::to_string(rf_det) + delimiter
							+ std::to_string(r_det) + delimiter
							+ std::to_string(l_val) + delimiter
							+ std::to_string(lf_val) + delimiter
							+ std::to_string(rf_val) + delimiter
                            + std::to_string(r_val) + delimiter
							+ std::to_string(lb_gd_val) + delimiter
							+ std::to_string(l_gd_val) + delimiter
							+ std::to_string(lf_gd_val) + delimiter
                            + std::to_string(rf_gd_val) + delimiter
							+ std::to_string(r_gd_val) + delimiter
							+ std::to_string(rb_gd_val);

        return log_str;
    }
};
#define LOG_INFRARED_DET_FLAG 	infrared_det
#define LOG_INFRARED_DET   		"infrared_det"

#define INFRARED_DET_LOG(l_det, lf_det, rf_det,r_det,l_val,lf_val,rf_val,r_val,lb_gd_val,l_gd_val,lf_gd_val,rf_gd_val,r_gd_val,rb_gd_val)   \
    LOG(LOG_NAMES(LOG_INFRARED_DET), std::make_shared<CSVInfraredDetData>(l_det, lf_det, rf_det,r_det,l_val,lf_val,rf_val,r_val,lb_gd_val,l_gd_val,lf_gd_val,rf_gd_val,r_gd_val,rb_gd_val).get())


// 超声
struct CSVUltrasonicData : CSVLogData
{
    uint16_t l_val;
    uint16_t c_val;
    uint16_t r_val;
    uint16_t d_val;

    uint16_t ultrasonic_01;
    uint16_t ultrasonic_02;
    uint16_t ultrasonic_03;
    uint16_t ultrasonic_04;
    uint16_t ultrasonic_05;

    CSVUltrasonicData(uint16_t _ultrasonic_01, uint16_t _ultrasonic_02,uint16_t _ultrasonic_03,uint16_t _ultrasonic_04, uint16_t _ultrasonic_05)
        : ultrasonic_01(_ultrasonic_01), ultrasonic_02(_ultrasonic_02),ultrasonic_03(_ultrasonic_03),ultrasonic_04(_ultrasonic_04),ultrasonic_05(_ultrasonic_05)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(ultrasonic_01) + delimiter
                            + std::to_string(ultrasonic_02) + delimiter
							+ std::to_string(ultrasonic_03) + delimiter
                            + std::to_string(ultrasonic_04) + delimiter
							+ std::to_string(ultrasonic_05);
        return log_str;
    }
};
#define LOG_ULTRASONIC_FLAG 		ultrasonic
#define LOG_ULTRASONIC    		"ultrasonic"

#define ULTRASONIC_LOG(ultrasonic_01, ultrasonic_02, ultrasonic_03, ultrasonic_04, ultrasonic_05)   \
    LOG(LOG_NAMES(LOG_ULTRASONIC), std::make_shared<CSVUltrasonicData>(ultrasonic_01, ultrasonic_02, ultrasonic_03, ultrasonic_04, ultrasonic_05).get())


// 伺服数据（步进电机）
struct CSVServoData : CSVLogData
{
    uint8_t status;
    uint8_t calib_status;
    int16_t angle;

    CSVServoData(uint8_t _status,uint8_t _calib_status,int16_t _angle)
        : status(_status), calib_status(_calib_status),angle(_angle)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(status) + delimiter
                            + std::to_string(calib_status) + delimiter
							+ std::to_string(angle);
        return log_str;
    }
};
#define LOG_SERVO_FLAG 		servo
#define LOG_SERVO    		"servo"

#define SERVO_LOG(status, calib_status, angle)   \
    LOG(LOG_NAMES(LOG_SERVO), std::make_shared<CSVServoData>(status, calib_status, angle).get())


//侧边沿墙tof
struct CSVSideTofData : CSVLogData
{
    uint8_t rf_val; // 0 - 255
    uint8_t r_val;

    CSVSideTofData(uint8_t _rf_val,uint8_t _r_val)
        : rf_val(_rf_val), r_val(_r_val)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(rf_val) + delimiter
                            + std::to_string(r_val);
        return log_str;
    }
};
#define LOG_SIDE_TOF_FLAG 		side_tof
#define LOG_SIDE_TOF    		"side_tof"

#define SIDE_TOF_LOG(rf_val, r_val)   \
    LOG(LOG_NAMES(LOG_SIDE_TOF), std::make_shared<CSVSideTofData>(rf_val, r_val).get())


// 电池信息
struct CSVBatteryData : CSVLogData
{
	uint8_t percent;

    CSVBatteryData(uint8_t _percent)
        : percent(_percent)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(percent);
        return log_str;
    }
};
#define LOG_BATTERY_FLAG 	battery
#define LOG_BATTERY    		"battery"

#define BATTERY_LOG(percent)   \
    LOG(LOG_NAMES(LOG_BATTERY), std::make_shared<CSVBatteryData>(percent).get())


//ActuatorInfo
struct CSVActuatorData : CSVLogData
{
    uint8_t fan;			/* fan motor level */
    uint8_t main_brush;	/* main brush level */	
    uint8_t side_brush;
    uint8_t pump;
    int8_t  servo;            /* servo angle ctrl*/
    uint8_t servo_ctrl_mode; /* 0:角度控制   1：精准校零控制   2：校零完成*/    


    CSVActuatorData(uint8_t _fan, uint8_t _main_brush,uint8_t _side_brush,uint8_t _pump,int8_t _servo,uint8_t _servo_ctrl_mode)
        : fan(_fan), main_brush(_main_brush),side_brush(_side_brush),pump(_pump),servo(_servo),servo_ctrl_mode(_servo_ctrl_mode)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(fan) + delimiter
							+ std::to_string(main_brush) + delimiter
							+ std::to_string(side_brush) + delimiter
							+ std::to_string(pump) + delimiter
							+ std::to_string(servo) + delimiter
							+ std::to_string(servo_ctrl_mode);
        return log_str;
    }
};
#define LOG_ACTUATOR_FLAG 	actuator
#define LOG_ACTUATOR    	"actuator"

#define ACTUATOR_LOG(fan, main_brush, side_brush,pump,servo,servo_ctrl_mode)   \
    LOG(LOG_NAMES(LOG_ACTUATOR), std::make_shared<CSVActuatorData>(fan, main_brush, side_brush,pump,servo,servo_ctrl_mode).get())


//ActuatorAdcInfo
struct CSVActuatorAdcData : CSVLogData
{
    uint16_t left_side_brush_adc;
    uint16_t right_side_brush_adc;
    uint16_t main_brush_adc;
    uint16_t fan_adc;
    uint16_t left_wheel_adc;
    uint16_t right_wheel_adc;
    uint16_t water_pump_adc; 

    CSVActuatorAdcData(uint16_t _adc0, uint16_t _adc1,uint16_t _adc2,uint16_t _adc3,uint16_t _adc4,uint16_t _adc5,uint16_t _adc6)
        : left_side_brush_adc(_adc0), right_side_brush_adc(_adc1),main_brush_adc(_adc2),fan_adc(_adc3),left_wheel_adc(_adc4),right_wheel_adc(_adc5),water_pump_adc(_adc6)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(left_side_brush_adc) + delimiter
							+ std::to_string(right_side_brush_adc) + delimiter
							+ std::to_string(main_brush_adc) + delimiter
							+ std::to_string(fan_adc) + delimiter
							+ std::to_string(left_wheel_adc) + delimiter
                            + std::to_string(right_wheel_adc) + delimiter
							+ std::to_string(water_pump_adc);
        return log_str;
    }
};
#define LOG_ACTUATOR_ADC_FLAG 	actuator_adc
#define LOG_ACTUATOR_ADC 		"actuator_adc"

#define ACTUATOR_ADC_LOG(left_side_brush_adc, right_side_brush_adc, main_brush_adc,fan_adc,left_wheel_adc,right_wheel_adc,water_pump_adc)   \
    LOG(LOG_NAMES(LOG_ACTUATOR_ADC), std::make_shared<CSVActuatorAdcData>(left_side_brush_adc, right_side_brush_adc, main_brush_adc,fan_adc,left_wheel_adc,right_wheel_adc,water_pump_adc).get())



// 底盘组件状态 (错误信息)
struct CSVChassisStausData : CSVLogData
{
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

    CSVChassisStausData(uint8_t _small_wheel,uint8_t _degree, uint8_t _bump_single_count,uint8_t _gnd_detect,
						uint8_t _front_c_long_trig, uint8_t _bump_long_trig, uint8_t _wheel_l_motor_cur, 
						uint8_t _wheel_r_motor_cur,uint8_t _side_l_motor_cur,uint8_t _side_r_motor_cur,
						uint8_t _roll_motor_cur,uint8_t _water_box_not,uint8_t _dust_box_not,uint8_t _battery_err,
						uint8_t _rtof_err,uint8_t _ltof_err,uint8_t _rag_not,uint8_t _dust_box_full,
						uint8_t _fan_motor_oc,uint8_t _pump_oc,uint8_t _l_impact_err,uint8_t _r_impact_err)
        : small_wheel(_small_wheel), degree(_degree),bump_single_count(_bump_single_count),gnd_detect(_gnd_detect),
		front_c_long_trig(_front_c_long_trig),bump_long_trig(_bump_long_trig),wheel_l_motor_cur(_wheel_l_motor_cur),
		wheel_r_motor_cur(_wheel_r_motor_cur),side_l_motor_cur(_side_l_motor_cur),side_r_motor_cur(_side_r_motor_cur),
		roll_motor_cur(_roll_motor_cur),water_box_not(_water_box_not),dust_box_not(_dust_box_not),battery_err(_battery_err),
		rtof_err(_rtof_err),ltof_err(_ltof_err),rag_not(_rag_not),dust_box_full(_dust_box_full),fan_motor_oc(_fan_motor_oc),
		pump_oc(_pump_oc),l_impact_err(_l_impact_err),r_impact_err(_r_impact_err)
    {}

    virtual std::string getLogStr(std::string delimiter) const
    {
        std::string log_str = std::to_string(small_wheel) + delimiter + std::to_string(degree) + delimiter
							+ std::to_string(bump_single_count) + delimiter + std::to_string(gnd_detect) + delimiter
							+ std::to_string(front_c_long_trig) + delimiter + std::to_string(bump_long_trig) + delimiter
							+ std::to_string(wheel_l_motor_cur) + delimiter + std::to_string(wheel_r_motor_cur) + delimiter
							+ std::to_string(side_l_motor_cur) + delimiter + std::to_string(side_r_motor_cur) + delimiter
							+ std::to_string(roll_motor_cur) + delimiter + std::to_string(water_box_not) + delimiter
							+ std::to_string(dust_box_not) + delimiter + std::to_string(battery_err) + delimiter
							+ std::to_string(rtof_err) + delimiter + std::to_string(ltof_err) + delimiter
							+ std::to_string(rag_not) + delimiter + std::to_string(dust_box_full) + delimiter
							+ std::to_string(fan_motor_oc) + delimiter + std::to_string(pump_oc) + delimiter
                            + std::to_string(l_impact_err)+ delimiter + std::to_string(r_impact_err);
        return log_str;
    }
};
#define LOG_CHASSIS_STATUS_FLAG 	chassis_status
#define LOG_CHASSIS_STATUS   		"chassis_status"

#define CHASSIS_STATUS_LOG(small_wheel, degree, bump_single_count, gnd_detect, front_c_long_trig, bump_long_trig, \
						wheel_l_motor_cur,wheel_r_motor_cur,side_l_motor_cur,side_r_motor_cur,roll_motor_cur, \
						water_box_not,dust_box_not,battery_err,rtof_err,ltof_err,rag_not,dust_box_full, \
						fan_motor_oc,pump_oc,l_impact_err,r_impact_err)   \
    LOG(LOG_NAMES(LOG_CHASSIS_STATUS), std::make_shared<CSVChassisStausData>(small_wheel, degree, bump_single_count, gnd_detect, front_c_long_trig, bump_long_trig, \
				wheel_l_motor_cur,wheel_r_motor_cur,side_l_motor_cur,side_r_motor_cur,roll_motor_cur, \
				water_box_not,dust_box_not,battery_err,rtof_err,ltof_err,rag_not,dust_box_full, \
				fan_motor_oc,pump_oc,l_impact_err,r_impact_err).get())








#endif //CHASSIS_LOG_H