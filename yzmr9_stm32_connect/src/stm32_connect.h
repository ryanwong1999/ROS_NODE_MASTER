#ifndef __STM32_CONNECT_H__
#define __STM32_CONNECT_H__

#include <geometry_msgs/PoseWithCovarianceStamped.h>
//----------------plc -----------------------------
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
//-----------------psc ----------------------------
#include "yzmr9_msgs/PSC_get_status.h"
#include "yzmr9_msgs/PMS_get_status.h"
#include "yzmr9_msgs/PSC_key_control.h"
#include "yzmr9_msgs/Check_out_line.h"
#include "yzmr9_msgs/Set_Rfid.h"
#include  "robotserial.h"
#include "yzmr9_msgs/PSC_get_neck_status.h"
#include "yzmr9_msgs/Ultrasound.h"

#endif
