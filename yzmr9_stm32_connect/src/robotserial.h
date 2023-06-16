#ifndef ROBOTSERIAL_H_
#define ROBOTSERIAL_H_

#include <stddef.h>
#include <stdio.h>
#include <serial/v8stdint.h>
#include <stdint.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include "yzmr9_msgs/walk_state.h"
#include "yzmr9_msgs/Set_Rfid.h"
#include "yzmr9_msgs/testt.h"
#define MAX_RX_LEN    64
#define PRE_RATE      0.0174532925


class RobotSerial
{
  private:
    serial::Serial *mSerial = NULL;
    unsigned char index;
    unsigned char RecvBUF[MAX_RX_LEN];
    float angle_coe;
    float angle_coe_right;
    float angle_sz[4];
    float angle_szz[4];
    float distance_left;
    float distance_right;
    int is_nearby;
    bool no_narrow;

    float online_value = 0;
    float online_yaw = 0;
    int neck_height = 0;
    float pulse_sec = 0;
    float wheel_diameter = 0;
    float wheel_distance = 0;
  public:
    RobotSerial(serial::Serial *pSerial, int margs);
    ~RobotSerial();
  private:
    int8_t RobotSerialRead(void);
    int margs_p ;//是否建图;
  public:
    int Auto_Charging(int chargeFlag);
    //查询  pms
    int Get_Pms(int &Charging_Flag, int &Battary_Level);
    //设定两轮 速度
    int SetSpeed(short int st1, short int st2, short int tag, double yaw, double d);

    int8_t SetPid();
    //获取的脉冲
    int8_t GetOdom(unsigned char &frameIndex, int16_t &t1, int16_t &t2, float &dbVth, float &dbVth_l, float &dbTh, int fmq, int16_t &vx, int16_t &vth);
    //获取rfid 位置
    int8_t GetPosition(char &row, char &column, char &destination, char &turndown, char &deviate, char &offline, char &t1, char &t2,
                           float &dbVth, float &dbTh);
    // 获取 头部的状态
    int8_t GetHeadPose(int &Level, int &Vertical, int &switch_flag);
    // 获取温度
    int8_t GetTempHum(float &Co2, float &Voc, float &Temp, float &Hum);
    // 获取PM
    int8_t GetPM(float &Pm25, float &Pm10, float &Pm1_0, float &Stata);

    int8_t GetNeckPose(int &height, int &limit, int &done, int light, int bebebe);
    
    int8_t get_robot_button(int &audio_button, int &power_button, int &zs);

    int8_t GetAutoSpeed(void);
    
    // 告诉下面 当前ｙ 的差。。i角度还用传不？
    void tell_stm_y(double x, double cha_y);
    // //-----------------------psc----------------------------------
    // //查询电机水平角度
    // int get_psc_level(void);
    // //查询电机的垂直角度
    // int get_psc_pitch(void);
    // //精确控制头的转动角度
    // int accuracy_control_psc(int level,int pitch);
    // //键盘控制头部
    // int key_control_psc(int direction);
    // //-----------------------pms----------------------------------
    // int get_pms_vol(void);
    // int get_pms_cur(void);
    // //-- ultrasound --------------------------------------
    int get_ultrasound_result(int &cs_obs, int &fz_obs);
    int SendHeardCtrl(int direction);
    int SendHead_angle(int level, int pitch);
    int SendNeckCtrl(int direction);
    int SendNeck_Height(int height);
    void GetonlineStateCallback(const yzmr9_msgs::walk_state msg);
    void set_hand_callback(const yzmr9_msgs::testt msg);
};



#endif  // V8STDINT_H_
