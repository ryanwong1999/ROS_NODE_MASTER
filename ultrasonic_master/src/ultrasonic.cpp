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
#include "yzmr9_msgs/Ultrasound_result.h"

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
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //创建一个serial类
    serial::Serial ser1;
    serial::Serial ser2;
    serial::Serial ser3;
    //设置串口属性，并打开串口
    ser1.setPort("/dev/ttyUSB1");
    ser2.setPort("/dev/ttyUSB2");
    ser3.setPort("/dev/ttyUSB3");
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
    yzmr9_msgs::Ultrasound_result msgs;
    while(ros::ok())
    {   
        size_t n1 = ser1.available();
        size_t n2 = ser2.available();
        size_t n3 = ser3.available();
        if(n1)
        {
            n1 = ser1.read(read_data_1, n1);
            if(((read_data_1[0] + read_data_1[1] + read_data_1[2]) & 0x00FF) == read_data_1[3])
            {
                int high1 = read_data_1[1];
                int low1 = read_data_1[2];
                int distance1 = (high1 << 8) + low1;
                if(distance1 < 400) ROS_WARN("\033[31m超声1距离: %d\033[0m", distance1);
                else ROS_INFO_STREAM("超声1距离: " << distance1);
            }
        }
        if(n2)
        {
            n2 = ser2.read(read_data_2, n2);
            if(((read_data_2[0] + read_data_2[1] + read_data_2[2]) & 0x00FF) == read_data_2[3])
            {
                int high = read_data_2[1];
                int low = read_data_2[2];
                int distance = (high << 8) + low;
                if(distance < 400) ROS_WARN("\033[31m超声2距离: %d\033[0m", distance);
                else ROS_INFO_STREAM("超声2距离: " << distance);
            }
        }
        if(n3)
        {
            n3 = ser3.read(read_data_3, n3);
            if(((read_data_3[0] + read_data_3[1] + read_data_3[2]) & 0x00FF) == read_data_3[3])
            {
                int high = read_data_3[1];
                int low = read_data_3[2];
                int distance = (high << 8) + low;
                if(distance < 400) ROS_WARN("\033[31m超声3距离: %d\033[0m", distance);
                else ROS_INFO_STREAM("超声3距离: " << distance);
            }
        }
        //清理串口存储空间
        ser1.flush();
        ser2.flush();
        ser3.flush();
        loop_rate.sleep();
    }
    ser1.close();
    ser2.close();
    ser3.close();
    return 0;
}

