#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from yzmr9_msgs.msg import Robot_button
from yzmr9_msgs.msg import Envirment_data
import serial
import time

noiseQuery = "02 03 00 00 00 01 84 39"              # 噪声传感器问询帧

def main():
    ser1 = serial.Serial('/dev/ttyUSB5', 9600)  # 选择串口，设置波特率
    ser2 = serial.Serial('/dev/ttyUSB4', 9600)  # 选择串口，设置波特率

    rospy.init_node('environment_node')
    environment_pub = rospy.Publisher('environment_pub', Envirment_data, queue_size = 10)
    noise_pub = rospy.Publisher('noise_pub', Robot_button, queue_size = 10)
    msgEnvironment = Envirment_data()               # 创建 msg 对象
    msgNoise = Robot_button()                       # 创建 msg 对象


    if (ser1.is_open and ser2.is_open):
        print("environment port open success")
        print("noise port open success")

        # 设置循环频率
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            # 噪声传感器
            sendNoiseQuery = noiseQuery.replace(' ','').decode('hex')       # 发送数据转换为b'\xff\xff\xff\xff\xff'
            ser1.write(sendNoiseQuery)                      # 发送数据
            time.sleep(0.3)                                 # 延时，否则len_return_data将返回0
            lenReturnNoise = ser1.inWaiting()               # 将获取缓冲数据（接收数据）长度
            print('len_return_noise', lenReturnNoise)
            if lenReturnNoise:
                returnNoise = ser1.read(lenReturnNoise)     # 读取缓冲数据
                strReturnNoise = str(returnNoise.encode('hex'))
                print(strReturnNoise)

                noise = int(strReturnNoise[6:10], 16)
                print("noise", noise)
                # msgNoise.zs = noise/10
                # noise_pub.publish(msgNoise)


            # 环境传感器
            time.sleep(0.3)
            lenReturnEnvironment = ser2.inWaiting()         # 将获取缓冲数据（接收数据）长度
            # print('len_return_environment', lenReturnEnvironment)
            if lenReturnEnvironment:
                returnEnvironment = ser2.read(lenReturnEnvironment)     # 读取缓冲数据
                strReturnEnvironment = str(returnEnvironment.encode('hex'))
                print(strReturnEnvironment)
                # CO2
                CO2_H = int(strReturnEnvironment[4:6], 16)
                CO2_L = int(strReturnEnvironment[6:8], 16)
                CO2 = CO2_H*256 + CO2_L
                # CH2O
                CH20_H = int(strReturnEnvironment[8:10], 16)
                CH20_L = int(strReturnEnvironment[10:12], 16)
                CH2O = CH20_H*256 + CH20_L
                # TVOC
                TVOC_H = int(strReturnEnvironment[12:14], 16)
                TVOC_L = int(strReturnEnvironment[14:16], 16)
                TVOC = TVOC_H*256 + TVOC_L
                # PM2.5
                PM25_H = int(strReturnEnvironment[16:18], 16)
                PM25_L = int(strReturnEnvironment[18:20], 16)
                PM25 = PM25_H*256 + PM25_L
                # PM10
                PM10_H = int(strReturnEnvironment[20:22], 16)
                PM10_L = int(strReturnEnvironment[22:24], 16)
                PM10 = PM10_H*256 + PM10_L
                # TEMP
                TEMP_H = int(strReturnEnvironment[24:26], 16)
                TEMP_L = int(strReturnEnvironment[26:28], 16)
                TEMP = TEMP_H + TEMP_L*0.01
                # HUM
                HUM_H = int(strReturnEnvironment[28:30], 16)
                HUM_L = int(strReturnEnvironment[30:32], 16)
                HUM = HUM_H + HUM_L*0.01

                print("CO2: ", CO2)
                print("CH2O: ", CH2O)
                print("TVOC: ", TVOC)
                print("PM25: ", PM25)
                print("PM10: ", PM10)
                print("TEMP: ", TEMP)
                print("HUM: ", HUM)

            rate.sleep()

    else:
        print("environment or noise port open failed")



if __name__ == "__main__":
     try:
         main()
     except rospy.ROSInterruptException:
         pass
