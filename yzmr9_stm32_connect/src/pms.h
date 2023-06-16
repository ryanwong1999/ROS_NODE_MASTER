#ifndef PMS_H_
#define PMS_H_

#include <stddef.h>
#include <stdio.h>
#include <serial/v8stdint.h>
#include <stdint.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include "robotserial.h"
#include "yzmr9_msgs/PMS_get_status.h"
#include "yzmr9_msgs/Charging_not_nearby.h"
#include "yzmr9_msgs/Charging_Control.h"
#include "yzmr9_msgs/testt.h"

//电池的电流和电压
struct pms_status
{
  int cur;
  int vol;
};

class PMS
{
  public:
    ros::Publisher PMS_battery_status_pub;
    ros::Publisher PMS_Charging_nearby_pub;
    ros::Subscriber sub_auto_charging;
    PMS(ros::NodeHandle & n, RobotSerial *pRobotserial);
    ~PMS();
    //查询所有
    int isWork;    //是否开始工作标记，如果为Ｆ　不工作　一直下发充电
    int Query_Pms_All(void);
};

#endif  // V8STDINT_H_
