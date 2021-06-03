#ifndef _SPEED_CONTROLLER_
#define _SPEED_CONTROLLER_

#include <thread>
#include <mutex>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <functional>
#include "data/slam_data.h"
#include "data/control_speed_data.h"

#include "config/config_manager.h"

#define VEL_ARR_LEN      20
#define SMOOTH_TIME      400

using planning_utils::ConfigManager;

namespace planning_controller
{
      typedef enum
      {
            MOVE_SET_STOP = 0,
            MOVE_SET_MOVING,
            MOVE_SET_STEERING_ONLY,
            // MOVE_SET_FORWARD,
            // MOVE_SET_BACKWARD,
            // MOVE_SET_L_ROTATE,
            // MOVE_SET_R_ROTATE,
      } MOVE_SET_STATUS;


     class SpeedControl
     {
     private:
     /**
       * @brief  底盘速度下发线程函数
       * @param  None
       * @retval None
       */          
     void setChassisSpeedPolling();
     std::thread *m_set_chassis_speed_polling_thread;              // 底盘速度下发管理线程
     int m_set_chassis_speed_period; //底盘速度下发线程控制周期 unit: ms
     std::function<void(float, float)> m_send_speed_func;
     bool m_initialized;

     DataSlam m_slam_data;
     DataControlSpeed m_control_speed;
     float m_steer_bias;   // deg

     public:
     float linear_vel[VEL_ARR_LEN] = {0};     /* 前轮线速度下发数组, unit: mm/ */
     float angular_steer[VEL_ARR_LEN] = {0};  /* 前轮转角下发数组, unit: 0.001 rad */
//      float linear_vel = 0;     /* 前轮线速度下发, unit: mm/s */
//      float angular_steer = 0;  /* 前轮转角下发, unit: 0.001 rad*/
     int8_t m_vel_arr_pointer = 0;    /* 当前从速度数组取数下发的序号  该变量改变时需要加锁保护*/ 
     std::mutex m_vel_mtx;          // 执行速度列表的互斥量

     MOVE_SET_STATUS move_set_status;
     
     float m_max_linear_vel, m_max_angular_steer; //允许的最大执行 要根据plan.yaml配置转换 unit:m
     float m_min_linear_vel, m_min_angular_steer; //允许的最小执行 要根据plan.yaml配置转换 unit:rad
     float m_rotate_vel;
     float remain_angle = M_PI;

     public:
     /**
           * @brief  构造函数
           * @param  None
           * @retval None
           */  
     SpeedControl();
     ~SpeedControl();

     /**
           * @brief  速度控制层相关参数配置函数 配置底盘轮距、运动速度最大最小值、底盘速度控制周期  系统初始化时需要调用
           * @param  cfg_mgr            配置管理器
           *         send_speed_func    速度发送的调用函数
           * @retval None
           */       
     void init(ConfigManager &cfg_mgr,
        const std::function<void(float, float)>& send_speed_func);
     
     /**
           * @brief  速度平滑处理函数
           * @param  v_curr： 当前前轮线速度m/s  delta_curr：当前前轮转角rad  v：前轮线速度m/s  delta：前轮转角rad  smooth_time：平滑时间 unit:ms
           * @retval None
           */         
     void SpeedSmoothHandle(float v_curr, float delta_curr, float v, float delta, int smooth_time = SMOOTH_TIME);

     /**
           * @brief  无速度平滑处理函数
           * @param  v：前轮线速度m/s  delta：前轮转角rad
           * @retval None
           */      
     void NoSpeedSmoothHandle(float v, float delta);

     /**
           * @brief  设置底盘目标控制量的控制函数，有限幅
           * @param  v：前轮线速度 unit:m/s  delta：前轮转角 unit:rad   smooth：是否需要速度平滑(ture需要)  smooth_time：平滑时间 unit:ms
           * @retval None
           */      
     void SetChassisControl(float v, float delta, bool smooth = false, int smooth_time = SMOOTH_TIME);     

    //  /**
    //        * @brief  前进控制接口
    //        * @param  v：前进速度m/s  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */         
    //  void SetForward(float v = 0.15, bool smooth = false, int smooth_time = SMOOTH_TIME); 

    //  /**
    //        * @brief  后退控制接口
    //        * @param  v：前进速度m/s  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */     
    //  void SetBackward(float v = 0.105, bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  停止控制接口
    //        * @param  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */   
    //   void SetStop(bool smooth = false, int smooth_time = SMOOTH_TIME); 

     /**
           * @brief  停止控制接口
           * @param  smooth：是否需要速度平滑(ture需要)  smooth_time：平滑时间 unit:ms
           * @retval None
           */   
      void SetStop(bool smooth = false, int smooth_time = SMOOTH_TIME); 

     /**
           * @brief  停止控制接口(保留转角)
           * @param  smooth：是否需要速度平滑(ture需要)  smooth_time：平滑时间 unit:ms
           * @retval None
           */ 
      void SetStopKeepSteering(bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  旋转控制接口
    //        * @param  w：旋转角速度 unit:rad/s  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */      
    //  void SetRotate(float w, bool smooth = false, int smooth_time = SMOOTH_TIME);  

         /**
           * @brief  旋转控制接口
           * @param  left_dir：向左旋转为true，向右旋转为false
           * @retval None
           */      
     float SetRotate(const bool &left_dir, const float &target_yaw_angle);  

    //  /**
    //        * @brief  左旋转控制接口
    //        * @param  v：旋转轮速度 unit:rad/s  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */        
    //  void SetRotate1(float w, bool smooth = false, int smooth_time = SMOOTH_TIME); 

    //  /**
    //        * @brief  右旋转控制接口
    //        * @param  v：旋转轮速度 unit:rad/s  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */     
    //  void SetRotate2(float w, bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  左弧形前进控制接口
    //        * @param  radius：弧形半径 unit:m  v：弧形速度m/s  smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */ 
    //  void SetCurveForward1(float radius, float v, bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  左弧形后退控制接口
    //        * @param  radius：弧形半径 unit:m  v：弧形速度m/s   smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */     
    //  void SetCurveBackwar1(float radius, float v, bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  右弧形前进控制接口
    //        * @param  radius：弧形半径 unit:m  v：弧形速度m/s   smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */ 
    //  void SetCurveForward2(float radius, float v, bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  右弧形后退控制接口
    //        * @param  radius：弧形半径 unit:m  v：弧形速度m/s   smooth：是否需要速度平滑(true需要) smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */ 
    //  void SetCurveBackwar2(float radius, float v, bool smooth = false, int smooth_time = SMOOTH_TIME);
     
    //  /**
    //        * @brief  底盘速度控制接口
    //        * @param  v：线速度 unit:m/s  w：角速度 unit：rad/s  smooth：是否需要速度平滑(true需要)  smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */     
    //  void SetSpeed1(float v, float w, bool smooth = false, int smooth_time = SMOOTH_TIME);

    //  /**
    //        * @brief  底盘速度控制接口
    //        * @param  vl：左轮速度 unit:m/s  vr：右轮速度 unit：m/s  smooth：是否需要速度平滑(true需要)  smooth_time：平滑时间 unit:ms
    //        * @retval None
    //        */   
    //  void SetSpeed2(float vl, float vr, bool smooth = false, int smooth_time = SMOOTH_TIME);

     /**
           * @brief  底盘速度下发函数
           * @param  v_mili：前轮线速度mm/s  delta_mili：前轮转角0.001rad 
           * @retval None
           */    
     int SendChassisControl(float v_mili, float delta_mili);
     };
}


extern planning_controller::SpeedControl g_speed_controller;  //定义速度管理器全局变量 

#endif