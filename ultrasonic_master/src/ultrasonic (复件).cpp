#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sstream>
#include <string>
#include <iostream>
#include <serial/serial.h>
#include <signal.h>
// #include <robotserial.h>
#include "std_msgs/String.h"
#include "yzmr9_msgs/Ultra_msgs.h"

int ultraFlag1 = 0;
int ultraFlag2 = 0;
int ultraFlag3 = 0;

int main(int argc, char** argv)
{
    //初始化节点
    ros::init(argc, argv, "ultrasonic_node");
    //声明节点句柄
    ros::NodeHandle nh;
    //发布主题
    ros::Publisher ultra_pub = nh.advertise<yzmr9_msgs::Ultra_msgs>("Ultra_msgs", 10);
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    //创建一个serial类
    serial::Serial ser1;
    serial::Serial ser2;
    serial::Serial ser3;
    //设置串口属性，并打开串口
    ser1.setPort("/dev/Ultra_1");
    ser2.setPort("/dev/Ultra_2");
    ser3.setPort("/dev/Ultra_3");
    //设置串口通讯波特率
    ser1.setBaudrate(115200);
    ser2.setBaudrate(115200);
    ser3.setBaudrate(115200);
    //串口设置timeout
    ser1.setTimeout(to);
    ser2.setTimeout(to);
    ser3.setTimeout(to);
    //打卡串口
    try {ser1.open();}
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Ultraport_1");
        return -1;
    }
    try {ser2.open();}
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Ultraport_2");
        return -1;
    }
    try {ser3.open();}
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open Ultraport_3");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if(ser1.isOpen()) ROS_INFO_STREAM("UltraSerial_1 Port initialized");
    else return -1;
    if(ser2.isOpen()) ROS_INFO_STREAM("UltraSerial_2 Port initialized");
    else return -1;
    if(ser3.isOpen()) ROS_INFO_STREAM("UltraSerial_3 Port initialized");
    else return -1;
    //指定循环的频率
    ros::Rate loop_rate(50);
    yzmr9_msgs::Ultra_msgs msgs;
    while(ros::ok())
    {
        if(ser1.available())
        {
            ser1.read(read_data_1, 3)
            if(read_data_1[0] == 0x0F)
            {
                int high1 = read_data_1[1];
                int low1 = read_data_1[2];
                int distance1 = (high1 << 8) + low1;
                ROS_INFO_STREAM("Ultra_data_1: " << distance1);
            }
        }
        if(ser2.available())
        {
            ser2.read(read_data_2, 3)
            if(read_data_2[0] == 0x0F)
            {
                int high2 = read_data_2[1];
                int low2 = read_data_2[2];
                int distance2 = (high2 << 8) + low2;
                ROS_INFO_STREAM("Ultra_data_2: " << distance2);
            }
        }
        if(ser3.available())
        {
            ser3.read(read_data_3, 3)
            if(read_data_3[0] == 0x0F)
            {
                int high3 = read_data_3[1];
                int low3 = read_data_3[2];
                int distance3 = (high3 << 8) + low3;
                ROS_INFO_STREAM("Ultra_data_3: " << distance3);
            }
        }
        //清理串口存储空间
        ser1.flush();
        ser2.flush();
        ser3.flush();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ser1.close();
    ser2.close();
    ser3.close();
    return 0;
}

