#!/usr/bin/python
# -*- coding: utf-8 -*-
import serial   # 导入串口模块
import time     # 导入时间模块

def main():
    try:
        # 打开串口，并且获得串口对象
        Ultra1 = serial.Serial('/dev/ttyUSB2', 115200)
        print("Ultra1",Ultra1)
        # 判断是否打开成功
        if Ultra1.is_open:
            print("串口已经打开！")
    except Exception as exc:
        print("串口打开异常:", exc)

    while True:
        time.sleep(0.1)
        buffer_size1 = Ultra1.inWaiting()
        if buffer_size1:
            try:
                data1 = Ultra1.read(buffer_size1)    # 读取缓冲数据
                str_data1 = str(data1.encode('hex'))
                if (int(str_data1[0:2], 16) + int(str_data1[2:4], 16) + int(str_data1[4:6], 16) & 0x00FF) == int(str_data1[6:8], 16):
                    high1 = int(str_data1[2:4], 16)
                    low1 = int(str_data1[4:6], 16)
                    distance1 = ((high1 * 256) + low1) * 0.1
                    print("ultra1:", distance1)
                    if distance1 < 40:
                        ultra1_flag = 1
                    else:
                        ultra1_flag = 0
                    print("ultra1_flag", ultra1_flag)
            except:
                pass


if __name__ == "__main__":
     main()
