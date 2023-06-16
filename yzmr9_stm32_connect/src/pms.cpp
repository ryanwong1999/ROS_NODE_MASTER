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
#include "robotserial.h"
#include "pms.h"

int auto_charging_flag = 0;
RobotSerial *mRobot_Serial;

void get_call_back(const yzmr9_msgs::Charging_Control &msg)
{
  auto_charging_flag = msg.auto_charging_flag;
  ROS_INFO("auto_charging_flag: %d", auto_charging_flag);
  if(auto_charging_flag == 0)
  {
    mRobot_Serial->Auto_Charging(0);
  }
  else if(auto_charging_flag == 1)
  {
    mRobot_Serial->Auto_Charging(1);
  }
}

//开辟线程 弄个 巡逻 debug
PMS::PMS(ros::NodeHandle &n,RobotSerial *pRobot_Serial)
{
  mRobot_Serial = pRobot_Serial;
  PMS_Charging_nearby_pub = n.advertise<yzmr9_msgs::Charging_not_nearby>("Charging_not_nearby", 1);
  PMS_battery_status_pub = n.advertise<yzmr9_msgs::PMS_get_status>("PMS_get_status", 5);
  sub_auto_charging = n.subscribe("Auto_Charging", 5, get_call_back);
}

PMS::~PMS()
{

}

//查询所有
int PMS::Query_Pms_All(void)
{
  static int tmp = 0;
  int battary_level = 0;
  int charging_flag = 0;
  int back_ = 0;

  back_ = mRobot_Serial->Get_Pms(charging_flag, battary_level);
  if(back_ != 0) return -1;

  yzmr9_msgs::PMS_get_status pms_msg;
  pms_msg.pms_charging_flag  = charging_flag;
  pms_msg.pms_battary_level = battary_level;
  //发布消息
  PMS_battery_status_pub.publish(pms_msg);
  
  return 0;
}
