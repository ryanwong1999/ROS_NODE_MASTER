#ifndef MOTO_DRV_H_
#define MOTO_DRV_H_

#include <stddef.h>
#include <stdio.h>
#include <serial/v8stdint.h>
#include <stdint.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include "robotserial.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "yzmr9_msgs/PMS_get_status.h"
#include "yzmr9_msgs/Autocharge_result.h"
#include "yzmr9_msgs/Emergency_Switch.h"
#include "yzmr9_msgs/Auto_Charging.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include "move_base_msgs/MoveBaseGoal.h"
#include <sensor_msgs/Range.h>


#define TIME_OUT        2
#define PI              3.1415926535
#define PULSE_CYCLE     5600      //1400*4
#define WHEEL_DIAMETER  0.169
#define WHEEL_DISTANCE  0.381

class MotoSerial
{
   public:
      MotoSerial(serial::Serial *pSerial, int margs);
      ~MotoSerial();

      int8_t MotoSerialRead(uint8_t len); 
      int8_t SetMotoEnable(uint8_t l_sta, uint8_t r_sta, uint8_t &en_sta);
      int8_t SetMotoSpeed(short int lear, short int angle, int16_t &real_lear, int16_t &real_angle);
      int8_t GetMotoOdom(int16_t &a_dir, int16_t &b_dir);
   private:
      serial::Serial *mSerial = NULL;
      uint8_t RecvBUF[MAX_RX_LEN];
};

class MDRV
{
   public:
      MDRV(ros::NodeHandle & n, MotoSerial *pMotoSerial);
      ~MDRV();
      // 发布里程计信息
      int pub_odom(short int vx, short int vy, short int t1, short int t2, float angle_v, float angle);
      void moto_Timer_deal_odom(void);
      void moto_Timer_deal_autoCharge(void);
      void cmd_Vel_Callback(const geometry_msgs::Twist& cmd_vel); 
      void Autocharge_Callback(const yzmr9_msgs::Autocharge_result& auto_result);
      void StopSwitch_Callback(const yzmr9_msgs::Emergency_Switch& stop_flag);
      void GetPms_Callback(const yzmr9_msgs::PMS_get_status& pms);
      void pub_cmd_Vel(int16_t real_lear, int16_t real_angle, int16_t a_pos, int16_t b_pos); 
   public:
      ros::Time last_time;
   private:
      tf::TransformBroadcaster odom_broadcaster;
      ros::Publisher odom_pub;
      uint8_t moto_en;
      int stop_switch;
      int charge_flag;
      int autoChargeTaskFlag;
      // float cur_radian;
      float last_radian;
      float totle_radian;
      //机器人的当前坐标  估计
      float g_x;
      float g_y;

      float set_vx;
      float set_vy;
      float set_vth;

      float auto_vx;
      float auto_vy;
      float auto_vth;

      float real_vx;
      float real_vy;
      float real_vth;
      // MotoSerial *mMotoSerial = NULL;
      /* data */
};


#endif