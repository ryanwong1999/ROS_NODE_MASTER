#ifndef PSC_H_
#define PSC_H_

#include <stddef.h>
#include <stdio.h>
#include <serial/v8stdint.h>
#include <stdint.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include "robotserial.h"

#include "yzmr9_msgs/PSC_get_status.h"
#include "yzmr9_msgs/PSC_key_control.h"
#include "yzmr9_msgs/Envirment_data.h"
//#include "yzmr9_msgs/PSC_accuracy_control.h"
#include "yzmr9_msgs/Wheel_Switch.h"
#include "yzmr9_msgs/PSC_angle_control.h"
#include "yzmr9_msgs/PSC_key_neck_control.h"
#include "yzmr9_msgs/PSC_angle_neck_control.h"
#include "yzmr9_msgs/PSC_get_neck_status.h"
#include "yzmr9_msgs/test.h"
#include "yzmr9_msgs/Ultrasound_result.h"

//psc 的状态
struct psc_obs
{
  int cs;
  int fz;

};

//按钮状态 和 噪声分贝值
struct psc_robot_button
{
  int audio_button;
  int power_button;
  int zs;
};

//psc 的状态
struct psc_status
{
  int level;
  int pitch;

};

//环境数据
struct psc_envir
{
  float temp; //温度
  float hum;
  float Co2;
  float Voc;
  float Pm25;
  float Pm10;
  float Pm1_0;
  float stata;
};

struct psc_neck_status
{
  int height;
  int limit;
  int done;
};

struct test_m
{
  int h;
  int v;
};

class PSC
{
  private:
    RobotSerial *mRobotSerial;
  public:
    PSC(ros::NodeHandle & n, RobotSerial *pRobotserial);
    ~PSC();
  public:
    int inssss;
    int psc_angle_control_flag;           //直接调节头部角度的标记
    int psc_angle_neck_control_flag;      //直接调节头部角度的标记
    int psc_key_neck_control_flag;
    int psc_goal_level;
    int psc_goal_pitch;
    int psc_goal_height;                  //升降杆高度
    int Emergency_Switch_Flag;            //这个是急停开光的标记
    int psc_key_control_direction;        //这个是键盘控制的方向  
    int flag_light;                       //大灯
    int flag_bebebe;                      //峰鸣
    psc_status psc_status_set;            //这个是控制的结构体  
    psc_envir psc_envir_set;              //环境数据结构提
    psc_obs obs_status;                   //障碍信息数据结构提
    psc_robot_button robot_button;        //按钮状态和噪声分贝值结构体
    psc_neck_status mPsc_neck_status;     //这个是升降干状态结构提
    test_m test_mm;                       //这个是实验头部的控制手柄代码
    //int psc_accuracy_control_flag;      //这个是精确控制标记
    ros::Publisher PSC_status_pub;
    ros::Publisher Wheel_pub;             //紧急开关的发布 
    ros::Publisher PSC_neck_status_pubs;  //发布升降干状态话题
    ros::Publisher Envirment_pub;         //发布环境数据
    ros::Publisher obs_pub;               //发布超声防撞
    ros::Publisher robot_button_pub;      //发布按钮状态和噪声分贝
    int psc_key_control_neck_direction;   //这个是键盘控升降干的方向     
    //查询俯仰电机状态
    int Query_Pitch_State(void);
    //查询水平电机的状态
    int Query_Level_State(void);
    //查询限位开关的状态
    int Query_Limit_Switch(void);
    int Query_TempHum_State(void);
    int Query_PM_State(void);
    int Query_Head_Pose(void);
    int Query_OBS_State(void);
    int Query_Robot_Button(void);
    //查询 所有
    int Query_Psc_All(void);
    int GetNeckStatus(void);
  
    //接受 具体角度的控制
    void Neck_Control_By_Height(const yzmr9_msgs::PSC_angle_neck_control msg);
    //void Control_Mechine(const yzmr9_msgs::PSC_accuracy_control control_msg);
    //手动控制头
    void Control_By_Key(const yzmr9_msgs::PSC_key_control key_msg);  
    void HeardCtrl_Key(void);
    void joystick_Callback(const geometry_msgs::Twist& cmd_vel);
    void Control_By_Angle(const yzmr9_msgs::PSC_angle_control msg);
    void Neck_Control_By_Key(const yzmr9_msgs::PSC_key_neck_control key_msg);
};

#endif  // V8STDINT_H_
