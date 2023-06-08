#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <iostream>
#include <serial/serial.h>
#include <signal.h>
// #include <robotserial.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "yzmr9_msgs/Ultrasound_result.h"

yzmr9_msgs::Ultrasound_result msgs;
geometry_msgs::Twist cmd_vel;

int ultraFlag1 = 0;
int ultraFlag2 = 0;
int ultraFlag3 = 0;
uint8_t read_data_1[1024];
uint8_t read_data_2[1024];
uint8_t read_data_3[1024];

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    //初始化节点
    ros::init(argc, argv, "ultrasonic_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布主题
    ros::Publisher ultra_pub = nh.advertise<yzmr9_msgs::Ultrasound_result>("Ultrasound_result", 10);
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //创建一个serial类
    serial::Serial ser1;
    serial::Serial ser2;
    serial::Serial ser3;
    //设置串口属性，并打开串口
    ser1.setPort("/dev/ultra1");
    ser2.setPort("/dev/ultra2");
    ser3.setPort("/dev/ultra3");
    //设置串口通讯波特率
    ser1.setBaudrate(115200);
    ser2.setBaudrate(115200);
    ser3.setBaudrate(115200);
    //串口设置timeout
    ser1.setTimeout(to);
    ser2.setTimeout(to);
    ser3.setTimeout(to);
    //打卡串口
    try 
    {
        ser1.open();
        ser2.open();
        ser3.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("超声串口打开异常！");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if(ser1.isOpen()) ROS_INFO_STREAM("\033[32m超声串口1已经打开！\033[0m");
    else return -1;
    if(ser2.isOpen()) ROS_INFO_STREAM("\033[32m超声串口1已经打开！\033[0m");
    else return -1;
    if(ser3.isOpen()) ROS_INFO_STREAM("\033[32m超声串口1已经打开！\033[0m");
    else return -1;
    //指定循环的频率
    ros::Rate loop_rate(50);
    while(ros::ok())
    {   
        try
        {
            size_t n1 = ser1.available();
            size_t n2 = ser2.available();
            size_t n3 = ser3.available();
            if (n1)
            {
                n1 = ser1.read(read_data_1, n1);
                if (((read_data_1[0] + read_data_1[1] + read_data_1[2]) & 0x00FF) == read_data_1[3])
                {
                    int high1 = read_data_1[1];
                    int low1 = read_data_1[2];
                    int distance1 = (high1 << 8) + low1;
                    if (distance1 < 400) ROS_WARN("\033[31m超声1距离: %d\033[0m", distance1);
                    else ROS_INFO_STREAM("超声1距离: " << distance1);
                }
            }
            if (n2)
            {
                n2 = ser2.read(read_data_2, n2);
                if (((read_data_2[0] + read_data_2[1] + read_data_2[2]) & 0x00FF) == read_data_2[3])
                {
                    int high2 = read_data_2[1];
                    int low2 = read_data_2[2];
                    int distance2 = (high2 << 8) + low2;
                    if (distance2 < 400) ROS_WARN("\033[31m超声2距离: %d\033[0m", distance2);
                    else ROS_INFO_STREAM("超声2距离: " << distance2);
                }
            }
            if (n3)
            {
                n3 = ser3.read(read_data_3, n3);
                if (((read_data_3[0] + read_data_3[1] + read_data_3[2]) & 0x00FF) == read_data_3[3])
                {
                    int high3 = read_data_3[1];
                    int low3 = read_data_3[2];
                    int distance3 = (high3 << 8) + low3;
                    if (distance3 < 400) ROS_WARN("\033[31m超声3距离: %d\033[0m", distance3);
                    else ROS_INFO_STREAM("超声3距离: " << distance3);
                }
            }
            //判断目前是前进还是后退，如果前进时ultra1或者ultra2距离小于40或后退时ultra3小于,下发速度0
        }
        catch(const std::exception& e)
        {
            ROS_WARN_STREAM("超声串口错误:" << e.what());
        }
        loop_rate.sleep();
    }
    ser1.close();
    ser2.close();
    ser3.close();
    return 0;
}

