#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "yzmr9_msgs/PMS_get_status.h"
#include "yzmr9_msgs/Envirment_data.h"
#include "yzmr9_msgs/Robot_button.h"
#include "yzmr9_msgs/Robot_move.h"
#include <pthread.h>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>

using namespace std;
ros::Publisher pub;
ros::Publisher voice_pub;
ros::Publisher pub_robotmove;
geometry_msgs::Twist vel_cmd;
pthread_t pth_[5];

// void* vel_ctr(void* arg)
// {
//     while(true)
//     {
//         pub.publish(vel_cmd);
//         ros::spinOnce();
//         sleep(1);
//     }
//     return 0;
// }

int iElectricAmount;
int iCharging_flag;
// 环境数据变量
int iHumidity;
int iTemperature;
int iCo2;
int iVoc;
int iPm25;
int iPm10;
int iPm1_0;
// 噪声
int zs;

// 这个是电量的回调信息
void GetPmsCallback(const yzmr9_msgs::PMS_get_status pms_info)
{
    iElectricAmount = pms_info.pms_battary_level;
    iCharging_flag = pms_info.pms_charging_flag;
}

// 这个是环境温度的回调信息
void GetEnvirmentCallback(const yzmr9_msgs::Envirment_data psc_info)
{
    iHumidity = psc_info.Hum;
    iTemperature = psc_info.Temp - 4;
    iCo2 = psc_info.Co2;
    iVoc = psc_info.Voc;
    iPm25 = psc_info.Pm25;
    iPm10 = psc_info.Pm10;
    iPm1_0 = psc_info.Pm1_0;
    // return;
}

// 订阅噪声
void GetRobotButtonCallback(const yzmr9_msgs::Robot_button msg)
{
    zs = msg.zs;
}

void callback(const std_msgs::String::ConstPtr &msg)
{
    cout << "接收到指令：" << msg->data.c_str() << endl;
    string str1 = msg->data.c_str();
    string str2 = "前进。";
    // string str2_1 = "up.";
    string str3 = "后退。";
    // string str3_1 = "back.";
    string str4 = "左转。";
    // string str4_1 = "left.";
    string str5 = "右转。";
    // string str5_1 = "right.";
    string str6 = "停止。";
    // string str6_1 = "stop.";
    string str7 = "设备状态。";
    string str8 = "环境状态。";
    string str8_1 = "空气情况。";
    string str9 = "开始巡检。";
    string str9_1 = "开始巡逻。";
    string str10 = "停止巡检。";
    string str10_1 = "结束巡检。";
    string str11 = "回去充电。";
    // if(str1 == str2 || str1 == str2_1)
    if (str1 == str2)
    {
        vel_cmd.linear.x = 0.3;
        vel_cmd.angular.z = 0;
        pub.publish(vel_cmd);
        // pthread_create(&pth_[0],NULL,vel_ctr,NULL);
    }
    else if (str1 == str3)
    {
        vel_cmd.linear.x = -0.3;
        vel_cmd.angular.z = 0;
        pub.publish(vel_cmd);
        // pthread_create(&pth_[1],NULL,vel_ctr,NULL);
    }
    else if (str1 == str4)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = 0.3;
        // pthread_create(&pth_[2],NULL,vel_ctr,NULL);
    }
    else if (str1 == str5)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = -0.3;
        pub.publish(vel_cmd);
        // pthread_create(&pth_[3],NULL,vel_ctr,NULL);
    }
    else if (str1 == str6)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.angular.z = 0;
        pub.publish(vel_cmd);
        // pthread_create(&pth_[4],NULL,vel_ctr,NULL);
    }
    else if (str1 == str7)
    {
        std::ostringstream oss1;
        oss1 << "电量为:" << iElectricAmount;
        std::string str_1 = oss1.str();
        std_msgs::String msg;
        msg.data = str_1;
        voice_pub.publish(msg);
    }
    else if (str1 == str8 || str1 == str8_1)
    {
        ROS_INFO("pub envirment:iTemperature %d iHumidity %d", iTemperature, iHumidity);
        std::ostringstream oss;
        oss << "环境状态为:"
            << "温度" << iTemperature << "度"
            << "湿度"
            << "百分之" << iHumidity
            << "噪声" << zs;
        std::string str_2 = oss.str();
        std_msgs::String msg;
        msg.data = str_2;
        voice_pub.publish(msg);
    }
    else if (str1 == str9 || str1 == str9_1)
    {
        ROS_INFO("收到开始巡检语音指令");
        std::ostringstream oss;
        oss << "收到开始巡检语音指令,执行任务号为0001的任务";
        std::string str_2 = oss.str();
        std_msgs::String msg;
        msg.data = str_2;
        voice_pub.publish(msg);

        yzmr9_msgs::Robot_move move_msg;
        move_msg.type = 233;
        move_msg.value1 = 1;
        move_msg.value3 = "0001";
        pub_robotmove.publish(move_msg);
    }
    else if (str1 == str10 || str1 == str10_1)
    {
        ROS_INFO("收到停止巡检语音指令");
        std::ostringstream oss;
        oss << "收到停止巡检语音指令";
        std::string str_2 = oss.str();
        std_msgs::String msg;
        msg.data = str_2;
        voice_pub.publish(msg);

        yzmr9_msgs::Robot_move move_msg;
        move_msg.type = 233;
        move_msg.value1 = 4;
        pub_robotmove.publish(move_msg);
    }
    else if (str1 == str11)
    {
        ROS_INFO("收到回去充电语音指令");
        std::ostringstream oss;
        oss << "收到回去充电指令";
        std::string str_2 = oss.str();
        std_msgs::String msg;
        msg.data = str_2;
        voice_pub.publish(msg);

        yzmr9_msgs::Robot_move move_msg;
        move_msg.type = 407;
        pub_robotmove.publish(move_msg);
    }
    else
    {
        std::ostringstream oss;
        oss << "我不明白，请对我说指定的关键词。";
        std::string str_3 = oss.str();
        std_msgs::String msg;
        msg.data = str_3;
        voice_pub.publish(msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_word");
    ros::NodeHandle n;

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    voice_pub = n.advertise<std_msgs::String>("xfwords", 10);
    pub_robotmove = n.advertise<yzmr9_msgs::Robot_move>("Robot_move", 1);
    ros::Subscriber sub = n.subscribe("voiceWords", 10, callback);
    // 订阅电量话题
    ros::Subscriber sub_pms_status = n.subscribe("PMS_get_status", 1, GetPmsCallback);
    // 订阅环境数据话题
    ros::Subscriber sub_envirment = n.subscribe("Envirment_data", 10, GetEnvirmentCallback);
    // 订阅噪声数据话题
    ros::Subscriber sub_zs = n.subscribe("Robot_button", 1, GetRobotButtonCallback);

    cout << "语音控制程序已开启" << endl;
    cout << "中文关键字控制字：前进 后退 左转 右转 停止 环境状态 设备状态 开始巡检 停止巡检 回去充电" << endl;
    // cout<<"英文关键字控制字：UP BACK LEFT RIGHT STOP"<<endl;
    cout << "程序将识别标准的普通话" << endl;

    ros::spin();
}
