#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <tf/transform_broadcaster.h>
#include <boost/typeof/typeof.hpp>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "robotserial.h"
#include "serial/serial.h"
#include "plc.h"
#include "pms.h"
#include <sensor_msgs/Joy.h>
#include "class_loader/meta_object.hpp"
#include <pluginlib/class_loader.h>
#include "moto_drv.h"
#include <tf2_msgs/TFMessage.h>

using namespace std;

uint16_t Time_Cnt = 0;
serial::Serial *mUart = NULL;
// serial::Serial *mSerial = NULL;
// 定义一个机器人串口
MotoSerial *mMotoSerial = NULL;
MDRV *mMDRV = NULL;

ros::Publisher pub_auto_charge;
yzmr9_msgs::Auto_Charging charge_msg;

unsigned short ModBus_CrC16(const uint8_t *s, int n);

void *Moto_Timer_Call_Back(void * arge)
{
  while(1)
  {
    Time_Cnt++;
    // ROS_INFO("######Time_Cnt: %d", Time_Cnt);
    sleep(1);
  }
}

void Moto_Timer_Deal(void)
{
  // ROS_INFO("Moto_Timer_Deal");
  static uint16_t cnt = 0;
  cnt++;
  mMDRV->moto_Timer_deal_odom();
  mMDRV->moto_Timer_deal_autoCharge();
  if(cnt%5 == 0)
  {
    // ROS_INFO("cnt: %d", cnt);
    // mMDRV->moto_Timer_deal_odom();
  }
}

void aaa(const tf2_msgs::TFMessage &msg)
{
  for(int i=0; i<msg.transforms.size(); i++)
  {
    // ROS_INFO("tf: frame_id: %s child_frame_id: %s now: %.4f stamp: %.4f",
    //         msg.transforms[i].header.frame_id.c_str(), msg.transforms[i].child_frame_id.c_str(),
    //         ros::Time::now().toSec(), msg.transforms[i].header.stamp.toSec());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "yzbot_motodrv");
  ros::NodeHandle n;

  int margs;
  if(argc > 1) margs = atoi(argv[1]);
  else margs = 11;

  ROS_INFO("motodrv version   yzmr9-st2-wsy-230614");
  // 创建一个全局串口对象
  mUart = new serial::Serial("/dev/mdrvusb", 115200, serial::Timeout::simpleTimeout(1000));
  mUart->setTimeout(serial::Timeout::max(), 50, 0, 50, 0);
  mMotoSerial = new MotoSerial(mUart, margs);
  mMDRV = new MDRV(n, mMotoSerial);
  // 订阅控制命令
  ros::Subscriber cmd = n.subscribe("smooth_cmd_vel", 1, &MDRV::cmd_Vel_Callback, mMDRV);  
  // ros::Subscriber cmd = n.subscribe("cmd_vel", 1, &MDRV::cmd_Vel_Callback, mMDRV); 
  // 订阅自动充电速度
  ros::Subscriber auto_speed = n.subscribe("Autocharge_result", 1, &MDRV::Autocharge_Callback, mMDRV); 
  // 订阅急停按键
  ros::Subscriber stop_switch = n.subscribe("Emergency_Switch", 1, &MDRV::StopSwitch_Callback, mMDRV); 
  // 订阅机器人电量
  ros::Subscriber sub_pms_status = n.subscribe("PMS_get_status", 1,  &MDRV::GetPms_Callback, mMDRV);
  // 订阅tf
  ros::Subscriber tfa = n.subscribe("tf", 1, aaa);
  // 发布自动充电话题
  pub_auto_charge = n.advertise<yzmr9_msgs::Auto_Charging>("Auto_Charging", 1);
  // 定时器分时处理
  ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&Moto_Timer_Deal));
  ros::spin();
  return 0;
}

MDRV::MDRV(ros::NodeHandle & n, MotoSerial *pMotoSerial)
{
  moto_en = 0;
  last_radian = 0;
  totle_radian = 0;
  g_x = 0;
  g_y = 0;

  set_vx = 0;
  set_vy = 0;
  set_vth = 0;

  real_vx = 0;
  real_vy = 0;
  real_vth = 0;

  last_time = ros::Time::now();

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  // mscan_distance.pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
 
  pthread_t pid;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  unsigned int goal_cell_x, goal_cell_y;
  goal_cell_x = 8.49;
  goal_cell_y = 7.03;
}

MDRV::~MDRV()
{
  mMotoSerial->SetMotoEnable(0, 0, moto_en);
}

void MDRV::moto_Timer_deal_autoCharge(void)
{
  // ROS_INFO_STREAM("moto_Timer_deal_autoCharge");
  //急停按键按下
  if(stop_switch == 1)
  {
    set_vx = 0.0;
    set_vth = 0.0;
  }
  //急停按键松开
  else
  {
    //进入自动对接充电桩任务
    if(autoChargeTaskFlag == 1)
    {
      set_vx = auto_vx;
      set_vth = auto_vth;
    }
  }
}

void MDRV::GetPms_Callback(const yzmr9_msgs::PMS_get_status& pms)
{
  charge_flag = pms.pms_charging_flag;
//   ROS_INFO("charge_flag: %d", charge_flag);
}

void MDRV::StopSwitch_Callback(const yzmr9_msgs::Emergency_Switch& stop_flag)
{
  stop_switch = stop_flag.Emergency_flag;
  // ROS_INFO_STREAM("stop_switch:" << stop_switch);
}

void MDRV::Autocharge_Callback(const yzmr9_msgs::Autocharge_result& auto_result)
{
  auto_vx = auto_result.linear;
  auto_vth = auto_result.angular;
  autoChargeTaskFlag = auto_result.task_flag;
  // ROS_INFO("auto_vx: %f, auto_vth: %f", auto_vx, auto_vth);
}

int cnt = 0;
int reset_cnt = 0;
void MDRV::cmd_Vel_Callback(const geometry_msgs::Twist& cmd_vel)
{
  //急停按键按下
  if(stop_switch == 1)
  {
    set_vx = 0.0;
    set_vth = 0.0;
  }
  //急停按键松开
  else
  {
    //进入自动对接充电桩任务
    if(autoChargeTaskFlag == 1)
    {
      cnt++;
      if(cnt >= 6)
      {
        charge_msg.auto_charging_flag = 0;
        pub_auto_charge.publish(charge_msg);
        cnt = 0;
        reset_cnt = 0;
      }
      else
      {
        reset_cnt++;
        if(reset_cnt >= 10) cnt = 0;
      }
    }
    else
    {
      set_vx = cmd_vel.linear.x;
      set_vy = cmd_vel.linear.y;
      set_vth = cmd_vel.angular.z;
      // ROS_INFO("set_vx: %.2f, set_vy: %.2f, set_vth: %.2f", set_vx, set_vy, set_vth);
    }
  }
}

void MDRV::pub_cmd_Vel(int16_t real_lear, int16_t real_angle, int16_t a_pos, int16_t b_pos)
{
  static int16_t last_right = a_pos, last_left = b_pos;
  int16_t right_pos, left_pos;
  int16_t right_pulse, left_pulse;
  int16_t pulse_tmp;

  right_pos = a_pos;
  left_pos = b_pos;
  
  if(right_pos < PULSE_CYCLE*0.25 && last_right > PULSE_CYCLE*0.75)
  {
    pulse_tmp = PULSE_CYCLE;
  }
  else if(right_pos > PULSE_CYCLE*0.75 && last_right < PULSE_CYCLE*0.25)
  {
    pulse_tmp = 0 - PULSE_CYCLE;
  }
  else
  {
    pulse_tmp = 0;
  }

  right_pulse = pulse_tmp + right_pos - last_right;

  if(left_pos > PULSE_CYCLE*0.75 && last_left < PULSE_CYCLE*0.25)
  {
    pulse_tmp = PULSE_CYCLE;
  }
  else if(left_pos < PULSE_CYCLE*0.25 && last_left > PULSE_CYCLE*0.75)
  {
    pulse_tmp = 0 - PULSE_CYCLE;
  }
  else
  {
    pulse_tmp = 0;
  }

  left_pulse = pulse_tmp + last_left - left_pos;
  last_right = right_pos;
  last_left = left_pos;
  // ROS_INFO("pulse: %d, %d", left_pulse, right_pulse);
  float cur_radian;
  float dbVth = 0;
  double PULSE_Sec = PI*WHEEL_DIAMETER/PULSE_CYCLE;
  // double PRE_RATE = PI/180;
  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  cur_radian = (right_pulse - left_pulse) * PULSE_Sec/WHEEL_DISTANCE ;   // rad
  double vth = cur_radian/dt;     // rad/s 
  double th_dt_h = cur_radian;

  // ROS_INFO("---dt: %f, PULSE_Sec: %f, cur_radian: %f, vth:  %f", dt, PULSE_Sec, cur_radian, vth);
  // ROS_INFO("--- %f, %f, %f, %f", dt, PULSE_Sec, cur_radian, vth);

  if(fabs(th_dt_h) > 0.0035 && fabs(th_dt_h) < 0.2)
  {
    totle_radian += cur_radian;
  }
  
  if(totle_radian > PI) totle_radian -= 2*PI; 
  else if(totle_radian <= -PI) totle_radian += 2*PI;

  double dt_x = ((left_pulse + right_pulse) * PULSE_Sec * cos(totle_radian - th_dt_h + th_dt_h/2.0))/2.0;
  double dt_y = ((left_pulse + right_pulse) * PULSE_Sec * sin(totle_radian - th_dt_h + th_dt_h/2.0))/2.0;
  // 通过角度 排除x y 正负问题
  double my_angle = totle_radian*(180/3.1415926);
  
  // ROS_INFO("dxy: %.2f, %.2f", dt_x, dt_y);
  int left_t = left_pulse;
  int right_t = right_pulse;
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
  }

  // 累计下  x  y  的坐标 
  g_x += dt_x;
  g_y += dt_y;
  // ROS_INFO("dxy: %.2f, %.2f", dt_x, dt_y);
  // per_angle = angle;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(totle_radian);

  // printf("*:gx=%f, gy=%f, gyaw=%f\n", g_x, g_y, th_total_h / PRE_RATE);
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = g_x;
  odom_trans.transform.translation.y = g_y;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = odom_quat;
  // send transform
  odom_broadcaster.sendTransform(odom_trans);
  // publish the odometry message over Ros
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.pose.pose.position.x = g_x;
  odom.pose.pose.position.y = g_y;
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = odom_quat;
  // set velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = (double)real_lear/1000;
  // odom.twist.twist.linear.x = (left_pulse + right_pulse) * PULSE_Sec/dt/2.0 ;//vx;
  odom.twist.twist.linear.y = 0;
  // odom.twist.twist.angular.z = vth;
  odom.twist.twist.angular.z = (double)real_angle/1000;
  // ROS_INFO("-----real_learv: %d, real_angle: %d, linear.x: %f, angular.z: %f", real_lear, real_angle, odom.twist.twist.linear.x, odom.twist.twist.angular.z);
  // ROS_INFO("----- %d, %d, %f, %f", real_lear, real_angle, odom.twist.twist.linear.x, odom.twist.twist.angular.z);

  // if(vth > -10 && vth < 10)
  // {
  odom_pub.publish(odom);
  // }

  last_time = current_time;
}

int move_cnt = 0;
void MDRV::moto_Timer_deal_odom(void)
{
  int16_t tx_lear, tx_angle;
  int16_t rx_lear, rx_angle;
  int16_t left_pos, right_pos;
  // set_vx = 0.4;
  // set_vth = 0.2;
  if(set_vx > 0.8) set_vx = 0.8;
  if(set_vx < -0.7) set_vx = -0.7;  
  if(set_vth > 0.5) set_vx = 0.5;
  if(set_vth < -0.5) set_vx = -0.5;

  tx_lear = set_vx*1000;
  tx_angle = set_vth*1000;
  // ROS_INFO("moto_en: %d", moto_en);
  if((tx_lear!= 0 || tx_angle!=0) || stop_switch == 1 || charge_flag == 1)
  {
    if(moto_en != 1)
    {
      mMotoSerial->SetMotoEnable(1, 1, moto_en);
    }
  }
  else
  {
    if(moto_en != 0)
    {
      move_cnt++;
      if(move_cnt > 20)
      {
        // mMotoSerial->SetMotoEnable(1, 1, moto_en);    //不可推动
        mMotoSerial->SetMotoEnable(0, 0, moto_en);    //可推动
        move_cnt = 0;
      }
    }
  }
  mMotoSerial->SetMotoSpeed(tx_lear, tx_angle, rx_lear, rx_angle);
  mMotoSerial->GetMotoOdom(right_pos, left_pos);
  pub_cmd_Vel(rx_lear, rx_angle, right_pos, left_pos);
  // ROS_INFO("rx speed: %d, %d", rx_lear, rx_angle);
  real_vx = rx_lear/1000;
  real_vth = rx_angle/1000;
}

MotoSerial::MotoSerial(serial::Serial *pSerial, int margs)
{
  // margs_p = margs;
  mUart = pSerial ;

  pthread_t pid;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  int ret = pthread_create(&pid, &attr, Moto_Timer_Call_Back, NULL);

  ros::NodeHandle n1;
  ros::NodeHandle n;
  // pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

MotoSerial::~MotoSerial()
{
  
}

int8_t MotoSerial::MotoSerialRead(uint8_t len)
{
  static unsigned char dat=0;
  unsigned short cal_crc;
  unsigned short rx_crc;

	mUart->read(RecvBUF, len+1);

  if(RecvBUF[0] == 0x01)
  {
    rx_crc = (unsigned short)(RecvBUF[len-2])<< 8 | RecvBUF[len-1];
    cal_crc = ModBus_CrC16(RecvBUF, len-2);
    if(rx_crc == cal_crc)
    {
      return 0;
    }
    else
    {
      ROS_INFO("crc err: %02x, %02x", rx_crc, cal_crc);
      return 1;
    }
  }
  else return -1;
}

int8_t MotoSerial::SetMotoEnable(uint8_t l_sta, uint8_t r_sta, uint8_t &en_sta)
{
  uint8_t SendBUF[MAX_RX_LEN];
  uint8_t buf_cnt;
  unsigned short crc;
   
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  buf_cnt = 0;
  SendBUF[buf_cnt++] = 0x01;
  SendBUF[buf_cnt++] = 0x44;
  SendBUF[buf_cnt++] = 0x21;
  SendBUF[buf_cnt++] = 0x00;
  SendBUF[buf_cnt++] = 0x31;
  SendBUF[buf_cnt++] = 0x00;
  SendBUF[buf_cnt++] = 0x00;
  SendBUF[buf_cnt++] = l_sta;
  SendBUF[buf_cnt++] = 0x00;
  SendBUF[buf_cnt++] = r_sta;
  crc = ModBus_CrC16(SendBUF, buf_cnt);
  SendBUF[buf_cnt++] = crc >> 8;
  SendBUF[buf_cnt++] = crc;

  mUart->write(SendBUF, buf_cnt);

  // ROS_INFO("send enable: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
  //         SendBUF[0], SendBUF[1], SendBUF[2], SendBUF[3],
  //         SendBUF[4], SendBUF[5], SendBUF[6], SendBUF[7],
  //         SendBUF[8], SendBUF[9], SendBUF[10], SendBUF[11]);

  Time_Cnt = 0;
  while(1)
  {
    if(Time_Cnt == TIME_OUT)
    {
      ROS_INFO("time out");
      return 1;
    }
   
    int8_t tmp = MotoSerialRead(12);   // 读数据
    // ROS_INFO("read enable:  %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
    //         RecvBUF[0], RecvBUF[1], RecvBUF[2], RecvBUF[3],
    //         RecvBUF[4], RecvBUF[5], RecvBUF[6], RecvBUF[7],
    //         RecvBUF[8],RecvBUF[9], RecvBUF[10], RecvBUF[11]);

    if(tmp == 0)
    {
      if(RecvBUF[7] == 1 || RecvBUF[9] == 1)
      {
        en_sta = 1;        
        return 1;
      }
      else
      {
        en_sta = 0;
        return 1;
      }
    }
    else
    {
      // return 0;
    }
  }
}

int8_t MotoSerial::SetMotoSpeed(int16_t set_lear, int16_t set_angle, int16_t &real_lear, int16_t &real_angle)
{
  // int8_t RecvBUF[MAX_RX_LEN];
  uint8_t SendBUF[MAX_RX_LEN];
  uint8_t buf_cnt;
  unsigned short crc;
  memset(SendBUF, 0, MAX_RX_LEN);     // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  buf_cnt = 0;
  SendBUF[buf_cnt++] = 0x01;
  SendBUF[buf_cnt++] = 0xea;
  SendBUF[buf_cnt++] = set_lear;
  SendBUF[buf_cnt++] = set_lear>>8;   // 长度
  SendBUF[buf_cnt++] = set_angle;     // 源地址
  SendBUF[buf_cnt++] = set_angle>>8;  // 目的地址
  crc = ModBus_CrC16(SendBUF, buf_cnt);
  SendBUF[buf_cnt++] = crc>>8;        // 命令
  SendBUF[buf_cnt++] = crc;

  mUart->write(SendBUF, buf_cnt);
  // ROS_INFO("set speed: %d, %d", set_lear, set_angle);
  // ROS_INFO("send speed: %02x %02x %02x %02x %02x %02x %02x %02x",
  //         SendBUF[0], SendBUF[1], SendBUF[2], SendBUF[3],
  //         SendBUF[4], SendBUF[5], SendBUF[6], SendBUF[7]);

  Time_Cnt = 0;
  while(1)
  {
    if(Time_Cnt == TIME_OUT)
    {
      // ROS_INFO("111");
      return 1;
    }
   
    int8_t tmp = MotoSerialRead(buf_cnt);   // 读数据
    
    // ROS_INFO("read speed: %02x %02x %02x %02x %02x %02x %02x %02x",
    //         RecvBUF[0], RecvBUF[1], RecvBUF[2], RecvBUF[3],
    //         RecvBUF[4], RecvBUF[5], RecvBUF[6], RecvBUF[7]);

    if(!(tmp<0))   // tmp = -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        // ROS_INFO("222");
        return 1;
      }
      else
      {
       real_lear = RecvBUF[3]<<8 | RecvBUF[2];
       real_angle = RecvBUF[5]<<8 | RecvBUF[4];
       return 0;
      }
    }
  }
}

int8_t MotoSerial::GetMotoOdom(int16_t &a_dir, int16_t &b_dir)
{
  uint8_t SendBUF[MAX_RX_LEN];
  uint8_t buf_cnt;
  unsigned short crc;
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  buf_cnt = 0;
  SendBUF[buf_cnt++] = 0x01;
  SendBUF[buf_cnt++] = 0x43;
  SendBUF[buf_cnt++] = 0x50;
  SendBUF[buf_cnt++] = 0x04;    // 长度
  SendBUF[buf_cnt++] = 0x51;    // 源地址
  SendBUF[buf_cnt++] = 0x04;    // 目的地址
  crc = ModBus_CrC16(SendBUF, buf_cnt);
  SendBUF[buf_cnt++] = crc>>8;  // 命令
  SendBUF[buf_cnt++] = crc;

  mUart->write(SendBUF, buf_cnt);

  Time_Cnt = 0;
  while(1)
  {
    if(Time_Cnt == TIME_OUT)
    {
      // ROS_INFO("111");
      return 1;
    }

    int8_t tmp = MotoSerialRead(12);   // 读数据

    if(tmp == 0)
    {
      a_dir = RecvBUF[6]<<8 | RecvBUF[7];
      b_dir = RecvBUF[8]<<8 | RecvBUF[9];
      // ROS_INFO("moto dir: %d, %d", b_dir, a_dir);
      return 0;
    }
  }
}

unsigned short ModBus_CrC16(const uint8_t *s, int n)
{
  unsigned short c = 0xffff;
  unsigned short b;
  char i;
  int k;
  for(k = 0; k < n; k++)
  {
    b = ((int8_t*)s)[k];
    for(i = 0; i < 8; i++)
    {
      c = ((b^c)&1) ? (c>>1)^0xA001 : (c>>1);
      b >>= 1;
    }
  }
  return (c<<8) | (c>>8);
}



