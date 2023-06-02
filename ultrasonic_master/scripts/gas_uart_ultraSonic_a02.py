# !/usr/bin/python
# -*- coding:UTF-8 -*-
import serial   # 导入串口模块
import time     # 导入时间模块

if __name__ == "__main__":
    try:
        # 打开串口，并且获得串口对象
        Ultra1 = serial.Serial("COM5", 115200, timeout=50)
        print(Ultra1)
        # 判断是否打开成功
        if Ultra1.isOpen():
            print("串口已经打开！")
    except Exception as exc:
        print("串口打开异常:", exc)

    while True:
        time.sleep(0.1)
        buffer_size1 = Ultra1.in_waiting
        if buffer_size1:
            try:
                data1 = Ultra1.read_all()
                # data1 = str(data.encode('hex'))    #python2.7前这样写
                data1 = data1.hex()  # python3.5后这样写
                high1 = int(data1[2:4], 16)
                low1 = int(data1[4:6], 16)
                # print(high1, low1)
                distance1 = ((high1 * 256) + low1) * 0.1
                print("ultra1:", distance1)
                if distance1 < 40:
                    ultra1_flag = 1
                else:
                    ultra1_flag = 0
                print("ultra1_flag", ultra1_flag)

            except:
                pass

    Ultra1.close()



