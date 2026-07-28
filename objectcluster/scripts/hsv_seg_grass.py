#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
 
resule_image = None
img_header = None

def image_callback(data):
    global resule_image, img_header
    img_header = data.header
    bridge = CvBridge()
    # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print e

    # img_hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)

    resule_image = cv_image



def nothing(x):
    pass

if __name__ == '__main__':
    # 初始化ros节点
    rospy.init_node("seg_grass")
    bridge = CvBridge()
    # rospy.loginfo("Starting cv_bridge_test node")
    image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)  # 前视摄像头
    # image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)  # 深度相机
    image_pub = rospy.Publisher("/seg_ros/results", Image, queue_size=1)
    Image_pub_ = Image()

    winName = 'hsv_grass'
    #新建窗口
    cv2.namedWindow(winName)
    #新建6个滑动条，表示颜色范围的上下边界，这里滑动条的初始化位置即为黄色的颜色范围
    cv2.createTrackbar('LowerbH',winName,0,255,nothing)
    cv2.createTrackbar('LowerbS',winName,48,255,nothing)
    cv2.createTrackbar('LowerbV',winName,41,255,nothing)
    cv2.createTrackbar('UpperbH',winName,64,255,nothing)
    cv2.createTrackbar('UpperbS',winName,97,255,nothing)
    cv2.createTrackbar('UpperbV',winName,105,255,nothing)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if resule_image is not None:
            resule_image = cv2.medianBlur(resule_image, 5)
            resule_image = cv2.GaussianBlur(resule_image,(5,5),0)
            #函数cv2.getTrackbarPos()范围当前滑块对应的值
            lowerbH=cv2.getTrackbarPos('LowerbH',winName)
            lowerbS=cv2.getTrackbarPos('LowerbS',winName)
            lowerbV=cv2.getTrackbarPos('LowerbV',winName)
            upperbH=cv2.getTrackbarPos('UpperbH',winName)
            upperbS=cv2.getTrackbarPos('UpperbS',winName)
            upperbV=cv2.getTrackbarPos('UpperbV',winName)
            #得到目标颜色的二值图像，用作cv2.bitwise_and()的掩模
            img_target=cv2.inRange(resule_image,(lowerbH,lowerbS,lowerbV),(upperbH,upperbS,upperbV))
            # img_target = cv2.medianBlur(img_target, 5)
            kernel_dilate = np.ones((7, 7), dtype=np.uint8)
            kernel_erode = np.ones((5, 5), dtype=np.uint8)
            dilation = cv2.dilate(img_target,kernel_dilate,iterations = 1)
            erosion = cv2.erode(dilation, kernel_erode, iterations=1)
            # closing = cv2.morphologyEx(img_target, cv2.MORPH_CLOSE, kernel)  ## 有缺陷，填补缺陷
            # ss = np.hstack((img_target, erosion))
            #输入图像与输入图像在掩模条件下按位与，得到掩模范围内的原图像
            # img_specifiedColor=cv2.bitwise_and(resule_image,resule_image,mask=img_target)
            cv2.imshow(winName, erosion)
            try:
                Image_pub_ = bridge.cv2_to_imgmsg(erosion, "mono8")
                Image_pub_.header = img_header
                image_pub.publish(Image_pub_)
            except CvBridgeError as e:
                print e
            rate.sleep()
        if cv2.waitKey(1) == ord('q'):
            break
    cv2.destroyAllWindows()#删除建立的全部窗口