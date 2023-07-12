#ifndef PLC_H_
#define PLC_H_

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

#include "sensor_msgs/Imu.h"
#include "yzmr9_msgs/PMS_get_status.h"
//#include "yzmr9_msgs/Force_cmd.h"
#include "yzmr9_msgs/Check_out_line.h"
#include "yzmr9_msgs/Robot_pose.h"
#include "yzmr9_msgs/Leg_Cmd.h"
#include "yzmr9_msgs/PSC_key_neck_control.h"
#include "yzmr9_msgs/PSC_get_neck_status.h"
#include "yzmr9_msgs/Ultrasound.h"
#include "sensor_msgs/LaserScan.h"
#include "yzmr9_msgs/CarLight_Cmd.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <yzmr9_msgs/Fmq_set.h>
#define PRE_RATE      0.0174532925
#define Sign(y) (y>=0?1:-1)
// 定义一个机器人位子结构体
struct Robot_pose
{
  float robot_x;   // X坐标
  float robot_y;   // Y坐标
  float robot_yaw; // 角度
};

class PLC
{
  public:
    PLC(ros::NodeHandle & n, RobotSerial *pRobotserial);
    ~PLC();
    // 发布里程计信息
    int pub_odom(short int vx, short int  vy, short int t1,short int  t2, float angle_v, float angle, float n_vx, float n_vth);
    void pub_robot_pose_fanction(int left_t, int right_t, double angle_t, double angle_l);
    //建图速度异常开关控制函数
    void plc_Timer_deal_odom(bool SpeedAbnormalSwitch);
    void get_auto_speed();
    // 监听joystick回调callback 
    void Ultrasound_Callback(const yzmr9_msgs::Ultrasound& ultra); 
    void Carlight_Callback(const yzmr9_msgs::CarLight_Cmd& light);
    void cmd_Vel_Callback(const geometry_msgs::Twist& cmd_vel);  
    void cmd_pwm_Vel_Callback(const geometry_msgs::Twist& cmd_vel);
    void amcl_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped amcl_pose);
  private:
    RobotSerial *mRobotSerial;
  public:
    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub;
    ros::Publisher pub_robot_pose;      // 发布机器人位子
    ros::Publisher pub_robot_position;  // 发布机器人位子(rfid)
    ros::Subscriber sub_pose;           // 订阅机器人的amcl_pose位子
    ros::Subscriber sub_auto_charging;
    ros::Publisher pub_setlocation;
    ros::Subscriber sub_fmq;  
    ros::Subscriber sub_imu;
    ros::Subscriber sub_amclpose;
    Robot_pose my_robot_pose;           // 机器人位子信息
    int fmq;
    ros::Time last_time;
    ros::Time robot_pose_lasttime;
    //机器人的当前坐标  估计
    float g_x;
    float g_y;
    //这个是 cmd_vel 中的速度 
    float g_vx;
    float g_vy;
    float g_vth;
    double mD;
    double mRobotPose_x;
    double mRobotPose_y;
    double mRobotPose_yaw;
    //这个是 左轮 右轮目标速度
    float obj_v1;
    float obj_v2;
    //当前的弧度累计  角度 累计  
    float th_total_h; 
    float th_total; 
    long per_time1;             //上一个脉冲数的时间 单位秒
    float pulse_sec = 0;        //每个脉冲对应距离
    float wheel_diameter = 0;   //轮子直径
    float wheel_distance = 0;   //轮距
    int mindex_cmd;
};



#endif  // V8STDINT_H_
