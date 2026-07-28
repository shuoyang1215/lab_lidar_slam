#!/usr/bin/env python
#coding=utf-8
import rospy
#导入自定义的数据类型
from std_msgs.msg import UInt8, UInt8MultiArray
import numpy as np

def talker():
    pub = rospy.Publisher('/Usb_Ai', UInt8MultiArray , queue_size=1)
    rospy.init_node('pub_vehicle_status', anonymous=False)
    #更新频率是1hz
    rate = rospy.Rate(10) 

    # 第一位预留，用于复位
    # 第二位表示上下车状态，0-非上下车，1-上车，2-下车
    # 第三位表示前视后视，0-前视，1-后视
    # 第四位表示是否在工作区（即是否在草地里)，0表示不在草地里，1表示在草地里
    state = np.array([0, 0, 0, 1], dtype=np.uint8)
    uint8_ = state.tostring()
    U8MA = UInt8MultiArray()
    U8MA.data = uint8_
    while not rospy.is_shutdown():
        pub.publish(U8MA)
        rate.sleep()

if __name__ == '__main__':
    talker()
