#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include "yzmr9_msgs/PSC_key_control.h"
#include "yzmr9_msgs/Robot_button.h"
#include "yzmr9_msgs/PSC_get_status.h"
#include "robotserial.h"
#include "psc.h"
#include "yzmr9_msgs/test.h"

const int FAILED = -255;	//这个是表示错误返回
const int SUCCESS = 0;
//#define  OPEN_PRINTF_DEBUG

using namespace std;

PSC::PSC(ros::NodeHandle & n, RobotSerial *pRobotserial)
{
  psc_angle_control_flag = 0;
  psc_angle_neck_control_flag =0;
  psc_key_neck_control_flag = 0;
  psc_goal_level = 90;
  psc_goal_pitch = 90;
  psc_goal_height = 0;
  psc_key_control_neck_direction = 0;
  mRobotSerial = pRobotserial;
  psc_status_set.level = 0; 
  psc_status_set.pitch = 0;
  psc_envir_set.temp = 0.0;
  psc_envir_set.hum = 0.0;
  psc_envir_set.Co2 = 0.0;
  psc_envir_set.Voc = 0.0;
  psc_envir_set.Pm10 = 0.0;
  psc_envir_set.Pm1_0 = 0.0;
  psc_envir_set.Pm25 = 0.0;
  psc_envir_set.stata = 0.0;
  mPsc_neck_status.height = 0;
  mPsc_neck_status.limit = 0;
  mPsc_neck_status.done = 0;

  Emergency_Switch_Flag = 0;
  PSC_status_pub = n.advertise<yzmr9_msgs::PSC_get_status>("PSC_get_status", 5);
  Wheel_pub = n.advertise<yzmr9_msgs::Wheel_Switch>("Wheel_Switch", 5);
  PSC_neck_status_pubs= n.advertise<yzmr9_msgs::PSC_get_neck_status>("PSC_get_neck_status", 5);
  //Envirment_pub = n.advertise<yzmr9_msgs::Envirment_data>("Envirment_data", 5);
  obs_pub = n.advertise<yzmr9_msgs::Ultrasound_result>("Ultrasound_result", 5);
  robot_button_pub = n.advertise<yzmr9_msgs::Robot_button>("Robot_button", 5);
}

PSC::~PSC()
{
}

int PSC::Query_Head_Pose(void)
{
  int tmp;
  tmp = mRobotSerial->GetHeadPose(psc_status_set.level, psc_status_set.pitch, Emergency_Switch_Flag);
  ROS_INFO("psc_status_set.level ------>%d", psc_status_set.level);
  ROS_INFO("psc_status_set.pitch ------>%d", psc_status_set.pitch);
  if(tmp != 0) return 0;
  else return 1;
}

//查询温度湿度Ｃ０２　ｖｏｃ
// int PSC::Query_TempHum_State(void)
// {
//   int tmp;
//   tmp = mRobotSerial->GetTempHum(psc_envir_set.Co2, psc_envir_set.Voc, psc_envir_set.temp, psc_envir_set.hum);
//   if(tmp != 0) return 0;
//   else return 1;
// }

//查询ＰＭ
// int PSC::Query_PM_State(void)
// {
//   int tmp;
//   tmp = mRobotSerial->GetPM(psc_envir_set.Pm25, psc_envir_set.Pm10, psc_envir_set.Pm1_0, psc_envir_set.stata);
//   if(tmp != 0) return 0;
//   else return 1;
// }

//查询障碍信息
int PSC::Query_OBS_State(void)
{
  int tmp;
  tmp = mRobotSerial->get_ultrasound_result(obs_status.cs, obs_status.fz);
  //ROS_INFO("obs_status.cs ---- >%d  obs_status.fz ---->%d", obs_status.cs, obs_status.fz);
  yzmr9_msgs::Ultrasound_result psc_obs_msg;
  psc_obs_msg.cs_obs = obs_status.cs;
  psc_obs_msg.fz_obs = obs_status.fz;
  obs_pub.publish(psc_obs_msg);
  if(tmp != 0) return 0;
  else return 1;
}

//查询按钮状态 和 噪声分贝值
int PSC::Query_Robot_Button(void)
{
  int tmp;
  tmp = mRobotSerial->get_robot_button(robot_button.audio_button, robot_button.power_button, robot_button.zs);

  yzmr9_msgs::Robot_button robot_button_msg;
  robot_button_msg.button_audio = robot_button.audio_button;
  robot_button_msg.button_power = robot_button.power_button;
  //接收到关机信号
  if(robot_button_msg.button_power==1) system("shutdown now");
  //robot_button_msg.zs = robot_button.zs;
  robot_button_pub.publish(robot_button_msg);
}

//查询所有
int PSC::Query_Psc_All(void)
{
  int tmp1 = 1;  // 姿态
  int tmp2 = 1;
  static int per_emergency_switch_flag = 0;
  static int read_flag = 0;
  read_flag ++;
  read_flag %= 3;   
  //ROS_INFO("get  pose head"); 
  usleep(2000);  
  //ROS_INFO("get  read_flag @@@@@@@@@@@@@@@@@@@@@@@@@ %d", read_flag);
  if(read_flag >= 0)
  {
    //ROS_INFO("PSC_neck_status_pubs !!!!! %d", mPsc_neck_status.height);
    //发布升降干状态话题
    yzmr9_msgs::PSC_get_neck_status psc_neck_msg;
    psc_neck_msg.get_height = mPsc_neck_status.height;
    psc_neck_msg.get_limit = mPsc_neck_status.limit;
    psc_neck_msg.get_done = mPsc_neck_status.done;
    PSC_neck_status_pubs.publish(psc_neck_msg);
  }

  if(tmp1 && tmp2)
  { 
    //发送 紧急开关信号 的话题
    //ROS_INFO("SWITCH @@@@@@@@@@@@@ %d", Emergency_Switch_Flag);
    yzmr9_msgs::Wheel_Switch wheel_msg;
    wheel_msg.Switch = Emergency_Switch_Flag;
    Wheel_pub.publish(wheel_msg); 
    //发布头部状态
    yzmr9_msgs::PSC_get_status psc_msg;
    psc_msg.get_level = psc_status_set.level;
    psc_msg.get_pitch = psc_status_set.pitch;
    PSC_status_pub.publish(psc_msg);
    //发布环境数据
    // yzmr9_msgs::Envirment_data envir_msg;
    // envir_msg.Co2 = psc_envir_set.Co2;
    // envir_msg.Voc = psc_envir_set.Voc;
    // envir_msg.Temp = psc_envir_set.temp;
    // envir_msg.Hum = psc_envir_set.hum;
    // envir_msg.Pm25 = psc_envir_set.Pm25;
    // envir_msg.Pm10 = psc_envir_set.Pm10;
    // envir_msg.Pm1_0 = psc_envir_set.Pm1_0;
    // envir_msg.Stata = psc_envir_set.stata;
    // Envirment_pub.publish(envir_msg);
    //ROS_INFO("%.2f TEMP", envir_msg.Temp);
    /*
     *这个地方做个急停开关的事件
     * */
    return 0;
  }
  else
  {
    return FAILED;
  }
}

//控制 精确
void PSC::Control_By_Angle(const yzmr9_msgs::PSC_angle_control msg)
{
  psc_angle_control_flag = 1;
  psc_goal_level = msg.set_level;
  psc_goal_pitch = msg.set_pitch;
  ROS_INFO("PSC_angle ********** %d %d*8", psc_goal_level, psc_goal_pitch);
}

//键盘升降干
void PSC::Neck_Control_By_Key(const yzmr9_msgs::PSC_key_neck_control key_msg)
{
  psc_key_neck_control_flag = 1; 
  psc_key_control_neck_direction = key_msg.psc_neck_direction;
  ROS_INFO("Neck_Control_By_Key %d", psc_key_control_neck_direction);
}

//升降干控制 精确
void PSC::Neck_Control_By_Height(const yzmr9_msgs::PSC_angle_neck_control msg)
{
  ROS_INFO("Neck_Control_By_Height");
  psc_angle_neck_control_flag = 1;  
  psc_goal_height = msg.set_height;
}

//键盘控制
void PSC::Control_By_Key(const yzmr9_msgs::PSC_key_control key_msg)
{
  psc_key_control_direction = key_msg.psc_direction;
}

//这个是给定时器调用的方法
int PSC::GetNeckStatus(void)
{
  //ROS_INFO("GetNeckStatus");
  int tmp = mRobotSerial->GetNeckPose(mPsc_neck_status.height, mPsc_neck_status.limit, mPsc_neck_status.done, flag_light, flag_bebebe);
  if(tmp != 0) return 0;
  else return 1;
}

