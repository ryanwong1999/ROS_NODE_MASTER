#!/usr/bin/python
# -*- coding:UTF-8 -*-
import snowboydecoder
import sys
import os
import signal
import rospy
from std_msgs.msg import String

interrupted = False


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


def interrupt_callback():
    global interrupted
    return interrupted

def callback():
    print("yes!")
    msg.data = '1'
    pub_wakeup.publish(msg)
    os.system('aplay /home/robot/ws/src/yzbot_sensors/wakeup/resources/ding.wav')

# if len(sys.argv) == 1:
#     print("Error: need to specify model name")
#     print("Usage: python demo.py your.model")
#     sys.exit(-1)

# model = sys.argv[1]
model = '/home/robot/ws/src/yzbot_sensors/wakeup/resources/models/snowboy.umdl'
# model = '/home/robot/ws/src/yzbot_sensors/wakeup/resources/models/computer.umdl'

# capture SIGINT signal, e.g., Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

detector = snowboydecoder.HotwordDetector(model, sensitivity=0.9)
print("\033[32mListening... Press Ctrl+C to exit\033[0m")

if __name__ == "__main__":
    rospy.init_node("wakeup")
    pub_wakeup = rospy.Publisher("/xfwakeup", String ,queue_size=10)
    msg = String()
    # main loop
    detector.start(detected_callback=callback,
                interrupt_check=interrupt_callback,
                sleep_time=0.03)

    detector.terminate()
