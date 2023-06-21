#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <time.h>
#include <sys/timeb.h>
#include "yzmr9_msgs/PMS_get_status.h"
#include "yzmr9_msgs/Check_out_line.h"  //校正走值
#include "yzmr9_msgs/Robot_pose.h"   
#include "yzmr9_msgs/Position_robot.h"
#include "serial/serial.h"
#include "robotserial.h"
#include "plc.h"
#include "yzmr9_msgs/Charging_Control.h"

using namespace std;


const double PI = 3.1415926535;
unsigned char timeout = 0;
int fmq_status_flag = 0;

void get_fmq_call_back(const yzmr9_msgs::Fmq_set &msg)
{
  fmq_status_flag = msg.switch_status;
}

// use imu to rectify odom angle (hang 20220917)
geometry_msgs::Quaternion imu_quat;
float imu_x, imu_y, imu_z, imu_w , imu_yaw, dt_yaw;
void get_imu_call_back(const sensor_msgs::Imu &imu_msg)
{ 
  tf::Quaternion quat;
  tf::quaternionMsgToTF(imu_msg.orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  dt_yaw = (yaw - imu_yaw)*1.25;
  
  if(abs(dt_yaw)>0.3) dt_yaw=0;
  imu_yaw = yaw;
  ROS_WARN("dt_yaw = %f, imu_yaw = %f", dt_yaw, imu_yaw);

  if(abs(imu_msg.angular_velocity.z)<0.1)
  {
    // imu_x = imu_msg.orientation.x;
    // imu_y = imu_msg.orientation.y;
    // imu_z = imu_msg.orientation.z;
    // imu_w = imu_msg.orientation.w;
    imu_yaw = yaw;
  }
  else
  {
    imu_yaw += imu_msg.angular_velocity.z/5;
  }
  imu_quat = tf::createQuaternionMsgFromYaw(imu_yaw);
}

/**
    订阅机器人坐标信息
**/
void PLC::amcl_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped amcl_pose)
{
  //ROS_WARN("mmmmmRobot pose:(%.2f, %.2f)", amcl_pose.pose.pose.position.x, amcl_pose.pose.pose.position.y);
  mRobotPose_x = amcl_pose.pose.pose.position.x;
  mRobotPose_y = amcl_pose.pose.pose.position.y;
  mD = mRobotPose_y - 14.3;
  my_robot_pose.robot_x = amcl_pose.pose.pose.position.x;
  my_robot_pose.robot_y = amcl_pose.pose.pose.position.y;
  tf::Quaternion  quat;
  tf::quaternionMsgToTF(amcl_pose.pose.pose.orientation, quat);
  double roll,pitch,yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  mRobotPose_yaw = yaw; 
  my_robot_pose.robot_yaw = yaw;
}

/**
  构造函数初始化
**/
PLC::PLC(ros::NodeHandle & n, RobotSerial *pRobotserial)
{
  ros::NodeHandle private_nh("~");
  private_nh.param<float>("pulse_sec", pulse_sec, -1);
  private_nh.param<float>("wheel_diameter", wheel_diameter, -1);
  private_nh.param<float>("wheel_distance", wheel_distance, -1);
  mRobotPose_x = 0;
  mRobotPose_y = 0;
  mRobotPose_yaw = 0;
  mindex_cmd = 30;
  mD = -1;
  g_x = 0;
  g_y = 0;
  g_vx = 0;
  g_vy = 0;
  g_vth = 0;
  obj_v1 = 0;
  obj_v2 = 0;
  th_total_h = 0; 
  th_total = 0;
  last_time = ros::Time::now();
  robot_pose_lasttime = ros::Time::now();
  my_robot_pose.robot_x = 0;
  my_robot_pose.robot_y = 0;
  my_robot_pose.robot_yaw = 0;
  mRobotSerial = pRobotserial ;
  odom_pub = n.advertise<nav_msgs::Odometry>("odom_stm32", 50);
  sub_fmq = n.subscribe("Fmq_set", 5, get_fmq_call_back);
  
  sub_imu = n.subscribe("imu", 20, get_imu_call_back);
  pub_robot_pose = n.advertise<yzmr9_msgs::Robot_pose>("Robot_pose", 1);
}

PLC::~PLC()
{

}

void PLC::pub_robot_pose_fanction(int left_t, int right_t, double angle_t, double angle_l)
{
  //ROS_INFO("angle_l:%.2f", angle_l);
  static float per_angle = 0;
  static float trueth_angle = 0;
  yzmr9_msgs::Robot_pose  robot_pose;
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - robot_pose_lasttime).toSec();   
  // 角度  弧度 
  double dt_th = (angle_t - per_angle) *PRE_RATE;

  double dt_x = ((left_t + right_t) * pulse_sec * cos(my_robot_pose.robot_yaw + dt_th/2.0))/2.0;
  double dt_y = ((left_t + right_t) * pulse_sec * sin(my_robot_pose.robot_yaw + dt_th/2.0))/2.0;
  double sin_y = sin(my_robot_pose.robot_yaw + dt_th/2.0);
  //ROS_WARN("dt_x left_t right_t my_robot_pose.robot_yaw  dt_th siny :(%f, %d, %d, %f, %f, %f)", dt_y, left_t, right_t, my_robot_pose.robot_yaw, dt_th, sin_y);

  my_robot_pose.robot_yaw = angle_t;

  //通过角度 排除x y 正负问题
   double my_angle =  my_robot_pose.robot_yaw*(180/3.1415926);
  // ROS_INFO("my_angle----------------------->%f", my_angle);
  if(-90<my_angle && my_angle<90)
  {
    if((left_t>0 && right_t>0) && dt_x<0)
    {
      dt_x = -1*dt_x;
    }
    if((left_t<0 && right_t<0) && dt_x>0)
    {
      dt_x = -1*dt_x;
    }
  }
  else
  {
    if((left_t<0 && right_t<0) && dt_x<0)
    {
      dt_x = -1*dt_x;
    }
    if((left_t>0 && right_t>0) && dt_x>0)
    {
      dt_x = -1*dt_x;
    }
  }

  if(my_angle>0)
  {
    if((left_t>0 && right_t>0) && dt_y<0)
    {
      dt_y = -1*dt_y;
    }
    if((left_t<0 && right_t<0) && dt_y>0)
    {
      dt_y = -1*dt_y;
    }
    // if(dt_y < 0)
    // {
    //   dt_y = -1*dt_y;
    // }
  }
  else
  {
    if((left_t<0 && right_t<0) && dt_y<0)
    {
      dt_y = -1*dt_y;
    }
    if((left_t>0 && right_t>0) && dt_y>0)
    {
      dt_y = -1*dt_y;
    }
    // if(dt_y>0)
    // {
    //   dt_ y = -1*dt_y;
    // }
  }

  my_robot_pose.robot_x += dt_x;
  my_robot_pose.robot_y += dt_y;

  //ROS_WARN("160 Robot pose:(%.2f,%.2f)", my_robot_pose.robot_y, dt_y);

  //mRobotPose_yaw = mRobotPose_yaw + (Sign(mRobotPose_yaw)*dt_th);
  //动态amcl坐标+静态odom角度 剔除+-3.14切换数据 发布robot_pose
  //ROS_INFO("dt_th:%.2f %.2f", dt_th, mRobotPose_yaw);
  if(dt_th > 1) dt_th = 0;
  mRobotPose_yaw = mRobotPose_yaw + dt_th;
  if(fabs(mRobotPose_yaw) > 3.14) mRobotPose_yaw = 3.14 * Sign(mRobotPose_yaw) * -1 + (Sign(mRobotPose_yaw) * fabs(fabs(mRobotPose_yaw) - 3.14));
  robot_pose.yaw = mRobotPose_yaw;
  if(my_robot_pose.robot_yaw <= -PI)
  {
    my_robot_pose.robot_yaw = 2 * PI + my_robot_pose.robot_yaw;
  }
  else if(my_robot_pose.robot_yaw > PI)
  {
    my_robot_pose.robot_yaw = -2 * PI + my_robot_pose.robot_yaw;
  }
  //这个是 发布 robot 的位置
  robot_pose.robot_x = my_robot_pose.robot_x;
  robot_pose.robot_y = my_robot_pose.robot_y;
  robot_pose.robot_yaw = my_robot_pose.robot_yaw;
  //ROS_INFO("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^    %f", robot_pose.robot_yaw);
  pub_robot_pose.publish(robot_pose);
  per_angle = angle_t;
}

/**
  发布odom话题
**/ 
int PLC::pub_odom(short int vx, short int vy, short int t1, short int t2, float angle_v, float angle, float n_vx, float n_vth)
{
  static int tmp_t1 = 0;
  static int tmp_t2 = 0;
  static float per_angle = 0;
  tmp_t1 += t1;
  tmp_t2 += t2;
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  //这个是 角速度  （弧度制）
  //ROS_INFO("angle : %.2f   per_angle:%.2f", angle, per_angle);
  double vth = (angle - per_angle) * PRE_RATE/dt;   //(angle_v*PRE_RATE)/dt;
  double th_dt_h = (angle - per_angle)*PRE_RATE;

  th_total_h = angle*PRE_RATE;
  //ROS_INFO("th_total_h = %f", th_total_h);
  //systh_total_h += th_dt_h;//(angle - per_angle)*PRE_RATE;
  if(th_total_h > PI)
  {
    th_total_h -= 2* PI; 
  }
  else if(th_total_h <= -PI)
  {
    th_total_h += 2*PI;
  }
  double dt_x = ((t1 + t2) * pulse_sec * cos(imu_yaw - dt_yaw + dt_yaw/2.0))/2.0;
  double dt_y = ((t1 + t2) * pulse_sec * sin(imu_yaw - dt_yaw + dt_yaw/2.0))/2.0;
  //通过角度 排除x y 正负问题
  double my_angle = imu_yaw*(180/3.1415926);

  int left_t= t1;
  int right_t= t2;
  if(-90<my_angle&&my_angle<90)
  {
    if((left_t>0&&right_t>0)&&dt_x<0)
    {
      dt_x = -1*dt_x;
    }
    if((left_t<0&&right_t<0)&&dt_x>0)
    {
      dt_x = -1*dt_x;
    }
  }else{
    if((left_t<0&&right_t<0)&&dt_x<0)
    {
      dt_x = -1*dt_x;
    }
    if((left_t>0&&right_t>0)&&dt_x>0)
    {
      dt_x = -1*dt_x;
    }
  }

  if(my_angle>0)
  {
    if((left_t>0&&right_t>0)&&dt_y<0)
    {
      dt_y = -1*dt_y;
    }
    if((left_t<0&&right_t<0)&&dt_y>0)
    {
      dt_y = -1*dt_y;
    }
  }
  else
  {
    if((left_t<0&&right_t<0)&&dt_y<0)
    {
      dt_y = -1*dt_y;
    }
    if((left_t>0&&right_t>0)&&dt_y>0)
    {
      dt_y = -1*dt_y;
    }
  }
  g_x += dt_x;
  g_y += dt_y;
  per_angle = angle;
  //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_yaw);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";
  
  odom_trans.transform.translation.x = g_x;
  odom_trans.transform.translation.y = g_y;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = imu_quat;
  //  odom_trans.transform.rotation.x = imu_x;
  //  odom_trans.transform.rotation.y = imu_y;
  //  odom_trans.transform.rotation.z = imu_z;
  //  odom_trans.transform.rotation.w = imu_w;
  //send transform
  odom_broadcaster.sendTransform(odom_trans);
  ROS_WARN("!!!!!!!!!!!!!!!1odom_stm32 x:%f y:%f yaw:%f dt_x:%f dt_y:%f", g_x, g_y , imu_yaw, dt_x, dt_y); 
  //publish the odometry message over Ros
  //  nav_msgs::Odometry odom;
  //  odom.header.stamp = current_time;
  //  odom.header.frame_id = "odom";
  //  odom.pose.pose.position.x = g_x;
  //  odom.pose.pose.position.y = g_y;
  //  odom.pose.pose.position.z = 0;
  //  odom.pose.pose.orientation = odom_quat;
  //  ROS_WARN("!!!!!!!!!!!!!!!1odom_stm32 x:%f y:%f", g_x, g_y);  //wsy
  //  //set velocity
  //  odom.child_frame_id = "base_link";
  //  odom.twist.twist.linear.x = (t1 + t2) * pulse_sec/dt/2.0  ;//vx;
  //  odom.twist.twist.linear.y = 0;
  //  odom.twist.twist.angular.z = vth;

  //  if((t1 + t2) * pulse_sec/dt/2.0 != 0 || vth != 0)
  //  {
  //     odom.pose.covariance = {0.001, 0, 0, 0, 0, 0, 
  //                             0, 0.001, 0, 0, 0, 0,
  //                             0, 0, 1000000, 0, 0, 0,
  //                             0, 0, 0, 1000000, 0, 0,
  //                             0, 0, 0, 0, 1000000, 0,
  //                             0, 0, 0, 0, 0, 1000};
  //     odom.twist.covariance = {0.001, 0, 0, 0, 0, 0, 
  //                             0, 0.001, 0, 0, 0, 0,
  //                             0, 0, 1000000, 0, 0, 0,
  //                             0, 0, 0, 1000000, 0, 0,
  //                             0, 0, 0, 0, 1000000, 0,
  //                             0, 0, 0, 0, 0, 1000};
  //  }else{
  //     odom.pose.covariance = {0.000000001, 0, 0, 0, 0, 0, 
  //                             0, 0.001, 0.000000001, 0, 0, 0,
  //                             0, 0, 1000000, 0, 0, 0,
  //                             0, 0, 0, 1000000, 0, 0,
  //                             0, 0, 0, 0, 1000000, 0,
  //                             0, 0, 0, 0, 0, 0.000000001};
  //     odom.twist.covariance = {0.000000001, 0, 0, 0, 0, 0, 
  //                             0, 0.001, 0.000000001, 0, 0, 0,
  //                             0, 0, 1000000, 0, 0, 0,
  //                             0, 0, 0, 1000000, 0, 0,
  //                             0, 0, 0, 0, 1000000, 0,
  //                             0, 0, 0, 0, 0, 0.000000001};
  //  }

  //  if(vth > -10 && vth < 10)
  //  {
  //     odom_pub.publish(odom);
  //  }
  // last_time = current_time;
  // ROS_INFO("pub odom vx:%.2f vth:%.2f",(t1 + t2) * pulse_sec/dt/2.0 ,vth);
}

/**
  订阅速度话题转化成左右轮子PWM
**/
void PLC::cmd_pwm_Vel_Callback(const geometry_msgs::Twist& cmd_vel)
{
  double Vy = cmd_vel.linear.y;
  double Vx = cmd_vel.linear.x;
  double Vth = cmd_vel.angular.z;
  double V_right;
  double V_left;
  if(Vx == 0)
  {
    V_right = Vth * wheel_distance / 2.0;
    V_left = (-1) * V_right;
  }
  else if(Vth == 0)
  {
    V_left = Vx;
    V_right = Vx;
  }
  else{
    V_left = Vx - Vth * wheel_distance / 2.0;
    V_right = Vx + Vth * wheel_distance / 2.0;
  }
  obj_v1 = V_left;
  obj_v2 = V_right; 
  ROS_INFO("setspeed get obj_v1 %.2f obj_v2 %.2f", obj_v1, obj_v2);
}

/**
  订阅超声话题
**/
void PLC::Ultrasound_Callback(const yzmr9_msgs::Ultrasound& ultra)
{

}

/**
  订阅速度话题
**/
void PLC::cmd_Vel_Callback(const geometry_msgs::Twist& cmd_vel)
{
  g_vx = cmd_vel.linear.x;
  g_vy = cmd_vel.linear.y;
  g_vth = cmd_vel.angular.z;
  mindex_cmd = 30;
}

/**
  定时器函数，不断获取脉冲信息，发布odom话题，与下发速度
**/
void PLC::plc_Timer_deal_odom(bool SpeedAbnormalSwitch)
{
  mindex_cmd --;
  unsigned char frameIndex = 0;
  int16_t t1 = 0, t2 = 0;
  float dbTh = 0, dbVth = 0, dbTh_l = 0;
  int16_t vx = 0, vth = 0;
  int back_ = mRobotSerial->GetOdom(frameIndex, t1, t2, dbTh, dbTh_l, dbVth , fmq_status_flag, vx, vth); 
  pub_odom(g_vx, g_vy, t1, t2,(float)(dbVth/100.0), (float)(dbTh/100.0), (float)(vx/1000.0), (float)(vth/1000.0));  
  pub_robot_pose_fanction(t1, t2, (float)(dbTh/100.0), (float)(dbTh_l/100.0));
  //if(g_vx == 0 && fabs(g_vth) > 0) g_vth = g_vth * 1/fabs(g_vth) * 0.2;
  //下发速度 判断如果3秒没有速度变化 视为速度源异常中断 下发停止
  
  /**if(mindex_cmd < 0) mRobotSerial->SetSpeed(0, 0, 0x15, mRobotPose_yaw, mD);
  else mRobotSerial->SetSpeed(g_vx*1000, g_vth*1000, 0x15, mRobotPose_yaw, mD);
  **/
  if(SpeedAbnormalSwitch)
  {
    if(mindex_cmd < 0) mRobotSerial->SetSpeed(0, 0, 0x15, mRobotPose_yaw, mD);
    else mRobotSerial->SetSpeed(g_vx*1000, g_vth*1000, 0x15, mRobotPose_yaw, mD);
  }
  else
  {
    mRobotSerial->SetSpeed(g_vx*1000, g_vth*1000, 0x15, mRobotPose_yaw, mD);
  }
  usleep(1000 * 10);
}


void PLC::get_auto_speed()
{
  int autospeed = mRobotSerial->GetAutoSpeed();
  // usleep(1000 * 10);
}