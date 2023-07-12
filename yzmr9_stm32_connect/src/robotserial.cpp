#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <netinet/in.h>
#include "serial/serial.h"
#include "signal.h"
#include "robotserial.h"
#include "yzmr9_msgs/Charging_not_nearby.h"
#include "yzmr9_msgs/Emergency_Switch.h"
#include "yzmr9_msgs/Autocharge_result.h"
#include <time.h>
#include <sys/timeb.h>

using namespace std;

#define TIME_OUT 10
#define OPEN_PRINTF_DEBUG

int Time_Count = 0;
ros::Publisher Emergency_pub;
ros::Publisher pub_cmd;  
ros::Publisher autocharge_pub; 

geometry_msgs::Twist cmd_vel;
yzmr9_msgs::Emergency_Switch emerg_msg;
yzmr9_msgs::Autocharge_result autocharge_msg;

unsigned char CRC8_Table(unsigned char *p, char counter);

void *Timer_Call_Back(void * arge)
{
  while(1)
  {
    Time_Count ++;
    //ROS_INFO("###### %d", Time_Count);
    //printf("&$$$$$$$$$$$$$$$$$$$&&& %d\r\n", Time_Count);
    sleep(1);
  }
}

int8_t RobotSerial::SetPid()
{
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN); // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x20;  // 命令
  SendBUF[7] = 0x08;
  memset(SendBUF+8, 0, 8);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;

  short int P = 580;
  short int I = 500;
  short int D = 0;

  P = (int16_t) htons(P);
  I = (int16_t) htons(I);
  D = (int16_t) htons(D);

	memcpy(&SendBUF[8], &P, sizeof(short int));
	memcpy(&SendBUF[10], &I, sizeof(short int));
  memcpy(&SendBUF[12], &D, sizeof(short int));

  mSerial->write(SendBUF, 19);
  Time_Count = 0;
  
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    int8_t tmp = RobotSerialRead();
    if(!(tmp<0))
    {
      if(tmp)
      {
        return 1; // error
      }
      else
      {
        // 解析数据
        int16_t rP = 0;
        int16_t rI = 0;
        int16_t rD = 0;

        rP = (int)RecvBUF[8]<<8 | RecvBUF[9];
        rI = (int)RecvBUF[10]<<8 | RecvBUF[11];
        rD = (int)RecvBUF[12]<<8 | RecvBUF[13];
        
        ROS_INFO("P:%d %d", RecvBUF[8], RecvBUF[9]);
        ROS_INFO("P:%d", rP);
        ROS_INFO("I:%d", rI);
        ROS_INFO("D:%d", rD);

        return 0;
      }
    }
  }
  return 0;
}

void RobotSerial::set_hand_callback(const yzmr9_msgs::testt msg)
{
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x75;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[8] = 0x00;
  SendBUF[9] = 0x00;

  ROS_INFO("set_hand_callback!!!!!!!!");
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D ;
  SendBUF[18] = 0x0A ;
  mSerial->write(SendBUF,19);  
}

RobotSerial::RobotSerial(serial::Serial *pSerial,int margs)
{
  ros::NodeHandle private_nh("~");
  private_nh.param<float>("pulse_sec", pulse_sec, -1);
  private_nh.param<float>("wheel_diameter", wheel_diameter, -1);
  private_nh.param<float>("wheel_distance", wheel_distance, -1);

  margs_p = margs;
  index = 0;
  mSerial = pSerial;
  memset(RecvBUF, 0, MAX_RX_LEN);  // 初始化接收缓冲 

  pthread_t pid;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
  int ret = pthread_create(&pid, &attr, Timer_Call_Back, NULL);

  // ros::NodeHandle n1;
  ros::NodeHandle n;
  Emergency_pub = n.advertise<yzmr9_msgs::Emergency_Switch>("Emergency_Switch", 5);
  pub_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  autocharge_pub = n.advertise<yzmr9_msgs::Autocharge_result>("Autocharge_result", 1);

  //SetTimer(1,100,Timer_Call_Back);
}

RobotSerial::~RobotSerial()
{
  
}

int8_t RobotSerial::RobotSerialRead(void)
{
	static unsigned char dat=0;
  
  RecvBUF[0] = dat;
	mSerial->read(RecvBUF+1, 1);
	dat = RecvBUF[1];
  //printf("#%x",RecvBUF[1]);
	if(RecvBUF[1]==0xAA)
	{
		if(RecvBUF[0]==0x55)
		{
			size_t read_byte = mSerial->read(RecvBUF+2, 17);
			if(((RecvBUF[RecvBUF[3]-2])== 0x0D) && (RecvBUF[RecvBUF[3]-1]))
			{
     			// 校验CRC
     	 		uint8_t crc = CRC8_Table(RecvBUF, 16);
      		//ROS_INFO("%02X", crc);
      		if(RecvBUF[16] == crc )
      		{
        		return 0;  // 校验正确返回0
      		}
          else
      		{
        		#ifdef OPEN_PRINTF_DEBUG
        		ROS_INFO("校验错误! crc:%02X\r\n", crc);
					  #endif
        		return 1;  // 校验错误返回1
      		}
			}
      else
			{
				return 1;    // 结束符检验错误
			}
		}
	}
  else
	{
    //memset();
		return -1;
	}
}

int RobotSerial::Auto_Charging(int chargeFlag)
{
  ROS_INFO("Auto_Charging----%d", chargeFlag);
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x08;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[8] = chargeFlag;
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);  
  Time_Count = 0;
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
        int gongneng = RecvBUF[6];
        if(gongneng != 0x08) return -1;    
        ROS_INFO("--------Autocharging--------");
        return 0;
      }
    }
  } 
}

int RobotSerial::SetSpeed(short int st1, short int st2, short int tag, double yaw, double d)   
{
  //checkOnline_bycenter(st1, st2);
  //ROS_INFO("SetSpeed");
  //ROS_INFO("out st1 st2 %d %d %.2f %.2f", st1, st2, yaw, d);
  //if(neck_height > 20)return -1;
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);

  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = tag;   // 命令
  SendBUF[7] = 0x08;

  st1 = (int16_t) htons(st1);
	st2 = (int16_t) htons(st2);
  int32_t mYaw = yaw * 100; 
  int32_t mDD  = d * 100;
  mYaw = (int32_t)htons(mYaw);
  mDD = (int32_t)htons(mDD);
	memcpy(&SendBUF[8], &st1, sizeof(short int));
	memcpy(&SendBUF[10], &st2, sizeof(short int));
  memcpy(&SendBUF[12], &mYaw, sizeof(short int));
	memcpy(&SendBUF[14], &mDD, sizeof(short int));
  // ROS_INFO("---------12-%d", SendBUF[12]);
  // ROS_INFO("---------13-%d", SendBUF[13]);
  // ROS_INFO("---------14-%d", SendBUF[14]);
  // ROS_INFO("---------15-%d", SendBUF[15]);
  int temp1 = 0;
  int temp2 = 0;
  memcpy(&temp1, &SendBUF[8], sizeof(short int));
  memcpy(&temp2, &SendBUF[10], sizeof(short int));
  //SendBUF[12] = 0x01;
  //ROS_INFO("@@@@@@@@@@@speed=%d,%d %d\n", temp1, temp2);
  //ROS_INFO("%X %X %X %X", SendBUF[8], SendBUF[9], SendBUF[10], SendBUF[11]);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF,19); 
  Time_Count = 0;
  while(1)
  {
    if(Time_Count == TIME_OUT)
    {
      //ROS_INFO("setspeed time out %d", tag);
      return 1;
    }
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
        int  gongneng = RecvBUF[6];
        if(gongneng != tag)
        {
          //ROS_INFO("setspeed faild %d",tag);
          return -1;
        }      
        return 0;
      }
    }
  } 
}

int RobotSerial::Get_Pms(int &Charging_Flag, int &Battary_Level)
{
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);
  memset(SendBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA; 
  SendBUF[2] = index;
  SendBUF[3] = 0x13;
  SendBUF[4] = 0xFF;
  SendBUF[5] = 0x01;
  SendBUF[6] = 0x09;
  SendBUF[7] = 0x08;
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);

  Time_Count = 0;
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
       /* 
        for(int k = 0 ; k < 19 ; k ++)
        {
          ROS_INFO("pms %x",RecvBUF[k]);
        }*/
        unsigned char recvFrameIndex=0;
        unsigned char length;
        unsigned char s_devid;
        unsigned char p_devid;
        unsigned char cmd;
        int charging_flag = 0;
        int battary_level = 0;
        int gongneng = RecvBUF[6];
        if(gongneng != 0x09)
        {
          return -1;
        }
        recvFrameIndex =  RecvBUF[2];
        //for(int j = 0; j < 19 ; j ++){
        //  ROS_INFO("##  %d",RecvBUF[j]);
        //}
        Charging_Flag = RecvBUF[11];
        Battary_Level = RecvBUF[15];
        //for(int t =0 ; t  < 19 ; t++){
          //ROS_INFO("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@pms  %d",Charging_Flag);  
        //}
        //ROS_INFO("PMS_______    %d ,%d",Charging_Flag,Battary_Level);
        return 0;
      }
    }
  }
}

/*
问底下获取当前rfid在哪个位置了
*/
int8_t RobotSerial::GetPosition(char &row, char &column, char &destination, char &turndown, char &deviate, char &offline, char &t1, char &t2,
                           float &dbVth, float &dbTh)
{
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x60;  // 命令
  SendBUF[7] = 0x08;
  memset(SendBUF+8, 0, 8);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count  = 0;
  
  while(1){
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();
    /*printf("$$$$$$$$$$$$$  %d",tmp);
    for(int i = 0; i < 19 ; i ++){
      printf("* %x ",RecvBUF[i]);
    }*/
    if(!(tmp<0))
    {
      if(tmp)
      {
        return 1; // error
      }
      else
      {
        // 解析数据
        short int mrow = 0;
        short int mcolwmn = 0;
        short int mdestination = 0;
        short int mturndown = 0;
        //short_int mSwitch_flag = 0;
        short int mSwitch_flag = 0;
        short int mDeviate = 0;
        int gongneng = RecvBUF[6];
       
        if(gongneng != 0x10)
        {
          //return -1;
        }
        row = RecvBUF[8];
        column = RecvBUF[9];
        destination = RecvBUF[10];
        turndown = RecvBUF[11];
        //deviate = RecvBUF[12];
        offline = RecvBUF[12];
        t1 = RecvBUF[13];
        t2 = RecvBUF[14];
        //ROS_INFO("turndown- -$$$$$$$$$$$$$$$$- %d",deviate);
        return 0;
      }
    }
  }
  return 0;
}

int RobotSerial::SendNeckCtrl(int direction)
{ 
  ROS_INFO("SendNeckCtrl");
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x81;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[8] = direction;
  memset(RecvBUF+9, 0, 7);
  //ROS_INFO("@@@@@@@@@@@@SendNeckCtrl    %d",direction);
  //printf("@@@@@@@@@@@@@@@speed=%d,%d\n",temp1,temp2);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  ROS_INFO("SendNeckCtrl %d", direction);
  mSerial->write(SendBUF, 19);    
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
        int gongneng = RecvBUF[6];
        if(gongneng != 0x81)
        {
          return -1;
        }      
        ROS_INFO("-----Send neck ctrl------");
        return 0;
      }
    }
  }
}

int RobotSerial::SendNeck_Height(int height){
  ROS_INFO("SendNeck_Height");
  //height = height - 71;
  if(height <= 0) height = 0;
  if(height >= 120) height = 120;
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x71;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[8] = height >> 8;
  SendBUF[9] = height & 0xFF;
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  ROS_INFO("neck height-----> %d", height);
  mSerial->write(SendBUF, 19);   
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;

    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
        int gongneng = RecvBUF[6];
        if(gongneng != 0x71)
        {
          return -1;
        }      
        ROS_INFO("-----Send neck height-----");
        return 0;
      }
    }
  } 
}

int ttt1;
int ttt2;
int8_t RobotSerial::GetOdom(unsigned char &frameIndex, int16_t &t1, int16_t &t2,
                           float &dbVth, float &dbVth_l, float &dbTh, int fmq, int16_t &vx, int16_t &vth)                       
{
  static float angle_per = 0;
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x01;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[9] = fmq;
  memset(SendBUF+11, 0, 6);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  //ROS_INFO("get fmq -----------%d",SendBUF[9]);
  Time_Count = 0;
  memset(RecvBUF, 0, MAX_RX_LEN);
  while(1)
  {
    // dingyi  3 s   Time_out
    if(Time_Count == TIME_OUT)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.linear.z = 0;
      pub_cmd.publish(cmd_vel);
      //ROS_INFO("timeout");
      // emerg_msg.Emergency_flag = 2;
      // Emergency_pub.publish(emerg_msg); 
      return 1;
    }
    //这个地方加 一个计时
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        #ifdef OPEN_PRINTF_DEBUG
        // cout << "Read error!" << endl;
        #endif 
        return 1;  // 失败
      }
      else
      {   
        // 返回0,表示接收到有效数据
        #ifdef OPEN_PRINTF_DEBUG
        // cout << "Odom Data Read sucessful!" << endl;
        #endif
        // 解析数据
        static float per_Angle = 0;
        static float per_Angle_l = 0;
        static float per_get_Angle = 0; 
        unsigned char recvFrameIndex = 0;
        unsigned char length;
        unsigned char s_devid;
        unsigned char p_devid;
        unsigned char cmd;
        int16_t mTick1 = 0;
        int16_t mTick2 = 0;
        int16_t mYaw = 0;  // 航向角
        int16_t mAngular_Rate = 0;
        recvFrameIndex =  RecvBUF[2];
        int gongneng = RecvBUF[6];
        // for(int t = 0; t < 19 ; t ++)
        // {
        //    ROS_INFO("* %x",RecvBUF[t]);
        // }
        // ROS_INFO("get gongneng -----------%d", RecvBUF[6]);
        if(gongneng != 1)
        {                                                   
          return 1;
        }
        mTick1 = (int)RecvBUF[8]<<8 | RecvBUF[9];   // 转速脉冲1
        mTick2 = (int)RecvBUF[10]<<8 | RecvBUF[11]; // 转速脉冲2
        vx = (int16_t)RecvBUF[12]<<8 | RecvBUF[13];
        vth = (int16_t)RecvBUF[14]<<8 | RecvBUF[15];
        // if(ttt1 % 10 == 0 && mTick1 > 0) mTick1 = mTick1+1;
        ttt1+=mTick1;
        ttt2+=mTick2;
        // ROS_INFO("mTick1 = %d  --  mTick2 = %d  %d", mTick1, mTick2, ttt1);
        // ROS_INFO("pwm = %d pwm = %d ",(int)RecvBUF[12]<<8 | RecvBUF[13], (int)RecvBUF[14]<<8 | RecvBUF[15]);        
        memcpy(&mYaw,&RecvBUF[12], sizeof(int16_t));
        memcpy(&mAngular_Rate, &RecvBUF[14], sizeof(int16_t));
        mYaw = ntohs(mYaw);
        mAngular_Rate = ntohs(mAngular_Rate);
        #ifdef OPEN_PRINTF_DEBUG
        // printf("mTick1=%d, mTick2=%d, mYaw=%.2lf, mAngular_Rate=%.2lf\r\n", mTick1, mTick2, mYaw/100.0, mAngular_Rate/100.0);
        #endif  
        frameIndex = recvFrameIndex;
        t1 = mTick1;
        t2 = mTick2;

        struct tm *ptm;
        struct timeb stTimeb;
        static char szTime[19];

        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        sprintf(szTime, "%02d:%02d:%02d.%03d",
                 ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm);
        szTime[18] = 0;

        //ROS_WARN("@@@@@mAngular_Rate:%d vth:%d RecvBUF[14]:%d RecvBUF[15]:%d", mAngular_Rate, vth, RecvBUF[14], RecvBUF[15]);
        //ROS_INFO("%s %x %d @ %x %d--odom ma %x %d", szTime, t1, t1, t2, t2, RecvBUF[6], RecvBUF[6]);
         
        int16_t maichong = 0;  // 航向角
        memcpy(&maichong, &RecvBUF[8], 8);
        int16_t maichong2 = 0;  // 航向角
        memcpy(&maichong2, &RecvBUF[9], 8);
        //ROS_INFO("@@@@@@@@@@@@@  %d @@@@@ %d", maichong, maichong2);
        //如果是建图启动 使用陀螺数据
        // if(margs_p == 22){
        //   dbVth = mYaw;
        // }
        dbTh = mAngular_Rate;
        //ROS_INFO("############## %f  %f",mYaw/100.0, mAngular_Rate/100.0);
        if(mYaw < 0.0001 && mYaw > -0.0001)
        {
          for(int t = 0; t < 19 ; t ++)
          {
            // ROS_INFO("* %x",RecvBUF[t]);
          }
        }
        //if((per_get_Angle == mYaw || fabs(per_get_Angle - mYaw)>20 * 100) && (t1 != 0 || t2 != 0)){
        //if((per_get_Angle == mYaw &&(t1 != 0 || t2 != 0))||(fabs(per_get_Angle - mYaw) > 20* 100)){  
        //这个是陀螺仪给过来的数据有问提了
          //脉冲 算 机器人当前角度 如果是正常启动 11
          //if(margs_p == 11){
          //  dbVth =per_Angle + ((t2 - t1) * PULSE_Sec/(2 *R * PRE_RATE)) * 100;
          dbVth =per_Angle + ((t2 - t1) * pulse_sec/( wheel_distance * PRE_RATE)) * 100;
          //}
          dbVth_l = per_Angle_l + ((t2 - t1) * pulse_sec/( wheel_distance * PRE_RATE)) * 100;
          if(dbVth > (180 * 100)) dbVth -= 360 * 100;
          else if(dbVth < (-180 * 100)) dbVth += 360 * 100;
          //ROS_INFO("@@@@@@@@@@@@@ t1:%d t2%d %f", t1, t2, dbVth/100.0);
        //}else{
        //  dbVth = mYaw;
        //}
        //per_get_Angle = mYaw;
        per_Angle = dbVth;
        per_Angle_l = dbVth_l;
        #ifdef OPEN_PRINTF_DEBUG
        // printf("t1=%d, t2=%d, dbVth=%.2lf, dbTh=%.2lf\r\n", t1, t2, dbVth, dbTh);
        #endif  
        return 0;
      }
    }
  }
}

int8_t RobotSerial::GetAutoSpeed()
{
  //获取自动充电时stm32给的速度
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x04;  // 命令
  SendBUF[7] = 0x08;
  memset(SendBUF+8, 0, 8);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count = 0;

  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();
    /*printf("$$$$$$$$$$$$$  %d", tmp);
    for(int i = 0; i < 19; i ++)
    {
      printf("* %x ", RecvBUF[i]);
    }*/
    if(!(tmp<0))
    {
      if(tmp)
      {
        ROS_INFO("error");
        return 1;   // error
      }
      else
      {
        // 解析数据
        double linear = (short)((RecvBUF[8]<<8) + RecvBUF[9]);     // 线速度
        double angular = (short)((RecvBUF[10]<<8) + RecvBUF[11]);  // 角速度
        int autoChargeTaskFlag = RecvBUF[12];
        int Emergency_flag = RecvBUF[13];

        emerg_msg.Emergency_flag = Emergency_flag;
        Emergency_pub.publish(emerg_msg); 

        autocharge_msg.linear = (linear/1000)*0.7;
        autocharge_msg.angular = (angular/1000)*0.7;

        autocharge_msg.task_flag = autoChargeTaskFlag;
        autocharge_pub.publish(autocharge_msg);

        // ROS_INFO("@@@ linear:%.2f  angular:%.2f  autoChargeTaskFlag:%d", autocharge_msg.linear, autocharge_msg.angular, autoChargeTaskFlag);
        return 0;
      }
    }
  }
  return 0;
}

int8_t RobotSerial::SetSensorEn(int carLight, int turnLight)
{
  ROS_INFO("SetSensorEn: %d", carLight);
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x77;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[11] = carLight;
  SendBUF[12] = turnLight;
  memset(SendBUF+13, 0, 8);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count = 0;

  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();
    /*printf("$$$$$$$$$$$$$  %d", tmp);
    for(int i = 0; i < 19; i ++)
    {
      printf("* %x ", RecvBUF[i]);
    }*/
    if(!(tmp<0))
    {
      if(tmp)
      {
        ROS_INFO("error");
        return 1;   // error
      }
      else
      {
        // 解析数据
        int carLight_result = RecvBUF[9];
        int turnLight_result = RecvBUF[10];
        // ROS_INFO("@@@ linear:%.2f  angular:%.2f  autoChargeTaskFlag:%d", autocharge_msg.linear, autocharge_msg.angular, autoChargeTaskFlag);
        return 0;
      }
    }
  }
  return 0;
}

int8_t RobotSerial::get_robot_button(int &audio_button, int &power_button, int &zs)
{
  //获取按钮开关状态和噪声分贝值
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x09;  // 命令
  SendBUF[7] = 0x08;
  memset(SendBUF+8, 0, 8);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count = 0;

  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();
    /*printf("$$$$$$$$$$$$$  %d", tmp);
    for(int i = 0; i < 19; i ++)
    {
      printf("* %x ", RecvBUF[i]);
    }*/
    if(!(tmp<0))
    {
      if(tmp)
      {
        ROS_INFO("error");
        return 1;   // error
      }
      else
      {
        // 解析数据
        audio_button = RecvBUF[10];
        power_button = RecvBUF[9];
        zs = (int)RecvBUF[12]<<8 | RecvBUF[13];
        zs = zs/10;
        //ROS_INFO("zs ------- >%d", zs);
        return 0;
      }
    }
  }
  return 0;
}

int8_t RobotSerial::GetNeckPose(int &height, int &limit, int &done, int light, int bebebe)
{
  //获取升降干状态
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x61;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[8] = light;
  SendBUF[9] = bebebe;
  memset(SendBUF+10, 0, 6);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count = 0;
  //ROS_INFO("GetNeckPose");
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();
    /*printf("$$$$$$$$$$$$$  %d",tmp);
    for(int i = 0; i < 19 ; i ++)
    {
      printf("* %x ",RecvBUF[i]);
    }*/
    if(!(tmp<0))
    {
      if(tmp)
      {
        ROS_INFO("error");
        return 1; // error
      }
      else
      {
        // 解析数据
        short int mheight = 0;
        short int mlimit = 0;
        short int mdone = 0;

        height = (int)RecvBUF[8]<<8 | RecvBUF[9];

        limit = RecvBUF[10];
        done = RecvBUF[11];
        neck_height = height;
        // height = height + 71;
        
        // for (size_t i = 0; i < 13; i++)
        // {
        //   /* code for loop body */
          // ROS_INFO("height- -$$$$$$$$$$$$$$$$- %d ",neck_height);
        // }
        return 0;
      }
    }
  }
  return 0;
}

int8_t RobotSerial::GetHeadPose(int &Level, int &Vertical, int &switch_flag)
{
  // ROS_INFO("GetHeadPose+++++++");
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;
  SendBUF[4] = 0xFF;
  SendBUF[5] = 0x01;
  SendBUF[6] = 0x10;   //功能吗
  SendBUF[7] = 0x08;   //长度
  memset(SendBUF+8,0,8);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count = 0;
  
  while(1){
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();
    /*printf("$$$$$$$$$$$$$  %d", tmp);
    for(int i = 0; i < 19; i ++)
    {
      printf("* %x ", RecvBUF[i]);
    }*/
    if(!(tmp<0))
    {
      if(tmp)
      {
        return 1; // error
      }
      else
      {
        // 解析数据
        short int mlevel = 0;
        short int mpitch = 0;
        //short_int mSwitch_flag = 0;
        short int mSwitch_flag = 0;
        int gongneng = RecvBUF[6];
        
        if(gongneng != 0x10)
        {
          return -1;
        }
        memcpy(&mlevel, &RecvBUF[8], sizeof(int16_t));
        memcpy(&mpitch, &RecvBUF[10], sizeof(int16_t));
        // memcpy(&mSwitch_flag, &RecvBUF[13], sizeof(int8_t));
        // 急停按键
        switch_flag = RecvBUF[13];
        Level = ntohs(mlevel);
        Vertical = ntohs(mpitch);
        ROS_INFO("switch_flag:%d+++++++",switch_flag);
        return 0;
      }
    }
  }
  return 0;
}

/***查询温度　湿度　ＣＯ２　ＶＯＣ****/
// int8_t RobotSerial::GetTempHum(float &Co2, float &Voc, float &Temp, float &Hum)                       
// {
//   uint8_t SendBUF[MAX_RX_LEN];
//   memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
//   memset(RecvBUF, 0, MAX_RX_LEN);
//   SendBUF[0] = 0x55;
//   SendBUF[1] = 0xAA;
//   SendBUF[2] = index;
//   SendBUF[3] = 0x13;  // 长度
//   SendBUF[4] = 0xFF;  // 源地址
//   SendBUF[5] = 0x01;  // 目的地址
//   SendBUF[6] = 0x62;  // 命令
//   SendBUF[7] = 0x08;
//   memset(SendBUF+8,0,8);
//   SendBUF[16] = CRC8_Table(SendBUF, 16);
//   SendBUF[17] = 0x0D;
//   SendBUF[18] = 0x0A;
//   mSerial->write(SendBUF, 19);
//   Time_Count = 0;
  
//   while(1)
//   {
//     if(Time_Count == TIME_OUT) return 1;
    
//     int8_t tmp = RobotSerialRead();   // 读数据
//     if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
//     {
//       if(tmp)  
//       {
//         // 调试打印,需要时打开此宏定义
//         #ifdef OPEN_PRINTF_DEBUG
//         // cout << "Read error!" << endl;
//         #endif 
//         return 1;  // 失败
//         break;
//       }
//       else
//       {   // 返回0,表示接收到有效数据
//         // 
//         #ifdef OPEN_PRINTF_DEBUG
//         // cout << "Temp&Hum Data Read sucessful!" << endl;
//         #endif
//         // 解析数据
//         unsigned char recvFrameIndex = 0;
//         unsigned char length;
//         unsigned char s_devid;
//         unsigned char p_devid;
//         unsigned char cmd;
//         short int mTemp = 0;
//         short int mHum = 0;
//         short int mCo2 = 0;
//         short int mVoc = 0;
//         int gongneng = RecvBUF[6];
//         if(gongneng != 0x62)
//         {
//           return -1;
//         }
//         memcpy(&mCo2,&RecvBUF[8], sizeof(int16_t));
//         memcpy(&mVoc,&RecvBUF[10], sizeof(int16_t));
//         mCo2 = ntohs(mCo2);
//         mVoc = ntohs(mVoc);
//         memcpy(&mTemp,&RecvBUF[14], sizeof(int16_t));
//         memcpy(&mHum,&RecvBUF[12], sizeof(int16_t));
//         mTemp = ntohs(mTemp);
//         mHum = ntohs(mHum);
//         // #ifdef OPEN_PRINTF_DEBUG
//         //  printf("mTemp=0x%02X,mHum=0x%02X\r\n",mTemp,mHum);
//         //  printf("温度=%.2lf,湿度=%.2lf\r\n",mTemp/100.0,mHum/100.0);
//         // #endif 
//         Co2 = (float)mCo2;
//         Voc = (float)mVoc/100; 
//         Temp = (float)mTemp/10;
//         Hum = (float)mHum/10;
//         // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!C)@ %.2f %.2f %.2f %.2f",Co2, Voc, Temp, Hum);
//         // ROS_INFO("################### %x %x %x %x", RecvBUF[8], RecvBUF[9], RecvBUF[10], RecvBUF[11]);
//         /*if(Temp > -0.001 && Temp < 0.001)
//         {
//           for(int t = 0; t < 19; t ++)
//           {
//             printf("tem  %x",RecvBUF[t]);
//           }
//         }*/

//         #ifdef OPEN_PRINTF_DEBUG
//         // printf("Temp=%.2lf, Hum=%.2lf\r\n", Temp, Hum);
//         #endif  
//         return 0;
//       }
//     }
//   }
// }

// int8_t RobotSerial::GetPM(float &Pm25, float &Pm10, float &Pm1_0, float &Stata)
// {
//   uint8_t SendBUF[MAX_RX_LEN];
//   memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
//   memset(RecvBUF, 0, MAX_RX_LEN);
//   SendBUF[0] = 0x55;
//   SendBUF[1] = 0xAA;
//   SendBUF[2] = index;
//   SendBUF[3] = 0x13;  // 长度
//   SendBUF[4] = 0xFF;  // 源地址
//   SendBUF[5] = 0x01;  // 目的地址
//   SendBUF[6] = 0x63;  // 命令
//   SendBUF[7] = 0x08;
//   memset(SendBUF+8, 0, 8);
//   SendBUF[16] = CRC8_Table(SendBUF, 16);
//   SendBUF[17] = 0x0D;
//   SendBUF[18] = 0x0A;
//   mSerial->write(SendBUF, 19);
//   Time_Count = 0;

//   while(1)
//   {
//     if(Time_Count == TIME_OUT) return 1;
    
//     int8_t tmp = RobotSerialRead();   // 读数据
//     if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
//     {
//       if(tmp)  
//       {
//         // 调试打印,需要时打开此宏定义
//         #ifdef OPEN_PRINTF_DEBUG
//         // cout << "Read error!" << endl;
//         #endif 
//         return 1;  // 失败
//         break;
//       }
//       else
//       {   
//         // 返回0,表示接收到有效数据
//         #ifdef OPEN_PRINTF_DEBUG
//         // cout << "Temp&Hum Data Read sucessful!" << endl;
//         #endif
//         // 解析数据
//         short int mPm25 = 0;
//         short int mPm10 = 0;
//         short int mPm1_0 = 0;
//         short int mStata = 0;
//         int gongneng = RecvBUF[6];
//         if(gongneng != 0x63)
//         {
//           return -1;
//         }
//         memcpy(&mPm25,&RecvBUF[8], sizeof(int16_t));
//         memcpy(&mPm10,&RecvBUF[10], sizeof(int16_t));
//         mPm25 = ntohs(mPm25);
//         mPm10 = ntohs(mPm10);
//         memcpy(&mPm1_0,&RecvBUF[12], sizeof(int16_t));
//         memcpy(&mStata,&RecvBUF[14], sizeof(int16_t));
//         mPm1_0 = ntohs(mPm1_0);
//         mStata = ntohs(mStata);
//         // #ifdef OPEN_PRINTF_DEBUG
//         //   printf("mTemp=0x%02X, mHum=0x%02X\r\n", mTemp, mHum);
//         //   printf("温度=%.2lf,湿度=%.2lf\r\n", mTemp/100.0, mHum/100.0);
//         // #endif 
//         Pm25 = (float)mPm25;
//         Pm10 = (float)mPm10; 
//         Pm1_0 = (float)mPm1_0;
//         Stata = (float)mStata;
//         /*if(Temp > -0.001 && Temp < 0.001)
//         {
//           for(int t = 0; t < 19; t ++)
//           {
//             printf("tem  %x", RecvBUF[t]);
//           }
//         }*/
//         // ROS_INFO("pm10-------->%.2f %.2f",Pm10,Pm25);

//         #ifdef OPEN_PRINTF_DEBUG
//           //printf("Temp=%.2lf,Hum=%.2lf\r\n",Temp,Hum);
//         #endif  
//         return 0;
//       }
//     }
//   }
// }

//-----ultrasound --------------------------------------
int RobotSerial::get_ultrasound_result(int &cs_obs, int &fz_obs)
{
  ROS_INFO("get_ultrasound_result------");
  static int tmp = 0;
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x07;  // 命令
  SendBUF[7] = 0x08;   //数据长度
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);
  Time_Count = 0;

  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        return 0;  // 失败
        break;
      }
      else
      {   
        // 返回0,表示接收到有效数据
        // 解析数据
        unsigned char recvFrameIndex=0;
        unsigned char length;
        unsigned char s_devid;
        unsigned char p_devid;
        unsigned char cmd;
        short int font_t = 0;
        short int left_t = 0;
        short int right_t = 0;
        short int back_t = 0;
        memcpy(&right_t, &RecvBUF[8], sizeof(int16_t));
        memcpy(&font_t, &RecvBUF[10], sizeof(int16_t));
        memcpy(&left_t, &RecvBUF[12], sizeof(int16_t));
        memcpy(&back_t, &RecvBUF[14], sizeof(int16_t));
        // ROS_INFO("ultra +++  %d, %d, %d", right_t, font_t, left_t);
        font_t = ntohs(font_t);
        left_t = ntohs(left_t);
        right_t = ntohs(right_t);
        back_t = ntohs(back_t);
        // ROS_INFO("ultra====  %d, %d, %d", right_t, font_t, left_t);
        cs_obs = 0;
        fz_obs = 0;
        cs_obs = RecvBUF[8];
        // ROS_INFO("cs %d", cs_obs);
        fz_obs = RecvBUF[9];
        // ROS_INFO("fz %d", fz_obs);
        double moto_cur = (short)((RecvBUF[11]<<8) + RecvBUF[12]);
        double bettery_vol = (short)((RecvBUF[14]<<8) + RecvBUF[15]);
        ROS_INFO("moto_cur %f, bettery_vol %f", moto_cur, bettery_vol);
        return 1;
        break;
      }
    }
  }
}

int RobotSerial::SendHead_angle(int level, int pitch)
{
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x11;  // 命令
  SendBUF[7] = 0x08;
  SendBUF[8] = level >> 8;
  SendBUF[9] = level & 0xFF;
  SendBUF[10] = pitch >> 8;
  SendBUF[11] = pitch & 0xFF;
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);   
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
        int  gongneng = RecvBUF[6];
        if(gongneng != 0x11){
          return -1;
        }      
        ROS_INFO("-----Send Head Angle ----- ");
        return 0;
      }
    }
  } 
}

int RobotSerial::SendHeardCtrl(int direction)
{ 
  uint8_t SendBUF[MAX_RX_LEN];
  memset(SendBUF, 0, MAX_RX_LEN);    // 初始化
  memset(RecvBUF, 0, MAX_RX_LEN);
  SendBUF[0] = 0x55;
  SendBUF[1] = 0xAA;
  SendBUF[2] = index;
  SendBUF[3] = 0x13;  // 长度
  SendBUF[4] = 0xFF;  // 源地址
  SendBUF[5] = 0x01;  // 目的地址
  SendBUF[6] = 0x06;  // 命令
  SendBUF[7] = 0x08;
  switch(direction)
  {
    case 5:
      SendBUF[8]=0x05; 
      break;
    case 6:
      SendBUF[8]=0x06; 
      break;
    case 7:
      SendBUF[8]=0x07; 
      break;
    case 8:
      SendBUF[8]=0x08; 
    case 9:
      SendBUF[8]=0x09; 
      break;
    default:
      SendBUF[8]=0x10; 
      break;
  }
  SendBUF[8] = direction;
  memset(RecvBUF+9, 0, 7);
  // printf("@@@@@@@@@@@@@@@psc=%d\n", SendBUF[8]);
  SendBUF[16] = CRC8_Table(SendBUF, 16);
  SendBUF[17] = 0x0D;
  SendBUF[18] = 0x0A;
  mSerial->write(SendBUF, 19);   
  while(1)
  {
    if(Time_Count == TIME_OUT) return 1;
    
    int8_t tmp = RobotSerialRead();   // 读数据
    if(!(tmp<0))   // tmp= -1 // 未收到任何无线数据
    {
      if(tmp)  
      {
        // 调试打印,需要时打开此宏定义
        return 1;
      }
      else
      {
        int gongneng = RecvBUF[6];
        if(gongneng != 0x06)
        {
          return -1;
        }      
        ROS_INFO("-----Send Head Ctrl-----");
        return 0;
      }
    }
  } 
}


/*****************************************************
* CRC 校验表
*****************************************************/
const char CRC8Table[]=
{
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
  190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
  219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
  87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};


/**
* 函数名：CRC8_Table
* 描述：
* 参数：
* 返回：
*/
unsigned char CRC8_Table(unsigned char *p, char counter)
{
  unsigned char crc8 = 0;
  for( ; counter > 0; counter--)
  {
    crc8 = CRC8Table[crc8^*p];
    p++;
  }
  return(crc8);
}