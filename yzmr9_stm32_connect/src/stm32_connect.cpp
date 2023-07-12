#include "stm32_connect.h"
#include <boost/bind.hpp> 
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <boost/typeof/typeof.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Joy.h"
#include "yzmr9_msgs/Check_out_line.h"  //校正走值
#include "yzmr9_msgs/Robot_pose.h"   
#include "robotserial.h"
#include "serial/serial.h"
#include "plc.h"
#include "psc.h"
#include "pms.h"

bool SpeedAbnormalSwitch = true;

serial::Serial *mSerial = NULL;
// 定义一个机器人串口
RobotSerial *mRobotSerial = NULL;
PLC *mPLC = NULL;
PSC *mPSC = NULL;
PMS *mPMS = NULL;
using namespace std;

 
void Timer_Deal(void)
{
  //ROS_INFO("Timer_Deal!!!!!!!!!!!!!!!!!");
  static int flag = 0;
  static int test_count = 0;
  static int get_psc_time = 0;
  flag ++;
  flag = flag % 30;
  // mPLC->plc_Timer_deal_odom(SpeedAbnormalSwitch);  //驱动器放在上面就不要这个，否则会通讯异常
  mPLC->get_auto_speed();
  if(flag % 3 == 0)
  {
    //升降部指令控制指令
    mPSC->GetNeckStatus();
    mPSC->Query_Psc_All();//发布关于　头部　环境温度
  }
  if(flag == 1)
  {
    mPSC->Query_Head_Pose();//查询头姿态
  }
  if(flag % 5 == 0)
  {
    mPSC->Query_Robot_Button();//查询按钮状态
  }
  // if(flag == 17)
  // {
  //   mPSC->Query_TempHum_State();//查询环境数据
  // }
  // if(flag == 19)
  // {
  //   mPSC->Query_PM_State();//查询环境数据
  // }
  if(flag == 7)
  {
    
  }
  if(flag == 29)
  {
    mPMS->Query_Pms_All();
  }
  else if(flag == 11){
    //下发查询头部指令控制指令
    // mPSC->GetNeckStatus();
  }
  else if(flag == 13 || flag == 23)
  {
    //下发查询避障信息
    mPSC->Query_OBS_State();
  }
  //这个是 发送下电机方向
  if(mPSC->psc_key_control_direction)
  {
    static int dir_per = 0;
    //这个是为了不要重复的发送一个方向的命令
    if(mPSC->psc_key_control_direction != dir_per)
    {
      mRobotSerial->SendHeardCtrl(mPSC->psc_key_control_direction);
	    ROS_INFO("!!!!!!!!!!!!!!!SendHeardCtrl!!!!!!!!!!!!!!!!!!!!!!!!! %d", mPSC->psc_key_control_direction);
    }
    dir_per = mPSC->psc_key_control_direction;  
    mPSC->psc_key_control_direction = 0;
  }
  if(mPSC->psc_angle_control_flag)
  {
    mRobotSerial->SendHead_angle(mPSC->psc_goal_level, mPSC->psc_goal_pitch);
    //ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    mPSC->psc_angle_control_flag = 0;
    usleep(1000);
  }
  if(mPSC->psc_angle_neck_control_flag)
  {
    mRobotSerial->SendNeck_Height(mPSC->psc_goal_height);
    //ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    mPSC->psc_angle_neck_control_flag = 0;
    usleep(1000);
  }
  //这个是 发送下电机方向
  //ROS_INFO("send psc_key_neck_control_flag ---->%d", mPSC->psc_key_neck_control_flag);
  if(mPSC->psc_key_neck_control_flag)
  {
    mRobotSerial->SendNeckCtrl(mPSC->psc_key_control_neck_direction);
    mPSC->psc_key_neck_control_flag = 0;
  }
 }

void *start_qr(void * arge)
{
  // system("python /home/robot/ws/src/yzbot_nav/nodes/EWM.py");
}
 
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "stm32_connect");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  nh.param("SpeedAbnormalSwitch", SpeedAbnormalSwitch, SpeedAbnormalSwitch);
  int margs;
  if(argc>1)
  {
    margs = atoi(argv[1]);
  }else
  {
    margs = 11;
  }
  
  ROS_INFO("margs--1115-->%d",margs);

  // pthread_t pid;
  // pthread_attr_t attr;
  // pthread_attr_init(&attr);
  // pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
  // int ret = pthread_create(&pid,&attr,start_qr,NULL);
  // 创建一个全局串口对象
  mSerial = new serial::Serial("/dev/mcuusb", 9600, serial::Timeout::simpleTimeout(1000));
  mSerial->setTimeout(serial::Timeout::max(), 50, 0, 50, 0);
  mRobotSerial = new RobotSerial(mSerial, margs);
  mPLC = new PLC(n, mRobotSerial);
  mPSC = new PSC(n, mRobotSerial);
  mPMS = new PMS(n, mRobotSerial);
  //订阅控制命令
  ros::Subscriber cmd_pwm = n.subscribe("smooth_cmd_vel", 1, &PLC::cmd_Vel_Callback, mPLC); 
  ros::Subscriber sub_amclpose = n.subscribe("amcl_pose", 5, &PLC::amcl_pose_call_back, mPLC);
  //订阅精确控制 psc 命令
  ros::Subscriber psc_angle_control = n.subscribe("PSC_angle_control", 1, &PSC::Control_By_Angle, mPSC);
  //订阅手动控制 psc 的命令
  ros::Subscriber psc_key_control = n.subscribe("PSC_key_control", 1, &PSC::Control_By_Key, mPSC);
  //ros::Subscriber psc_light_control = n.subscribe("PSC_light_control", 1, &PSC::LightControlCallback, mPSC);
  //定时器分时处理
  ros::Timer timer = n.createTimer(ros::Duration(0.10), boost::bind(&Timer_Deal));
  //订阅手动控制 升降干命令
  ros::Subscriber psc_key_neck_control = n.subscribe("PSC_key_neck_control", 1, &PSC::Neck_Control_By_Key, mPSC);
  ros::Subscriber psc_height_neck_control = n.subscribe("PSC_height_neck_control", 1, &PSC::Neck_Control_By_Height, mPSC);  
  //订阅超声
  ros::Subscriber sub_ultra = n.subscribe("Ultrasound", 1, &PLC::Ultrasound_Callback, mPLC); 

  ros::Subscriber sub_carlight_cmd = n.subscribe("CarLight_Cmd", 1, &PLC::Carlight_Callback, mPLC); 
  //mRobotSerial->SetPid();
  ros::spin();
  return 0;
}



