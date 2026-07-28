#!/usr/bin/env python
# -*- coding: utf-8 -*-

from customized_msgs.msg import BboxList, Bbox
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
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


if __name__ == '__main__':
    rospy.init_node('pub_BBox')
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)  # 前视摄像头
    # image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)  # 深度相机
    image_pub = rospy.Publisher("/det_ros/detect_img", Image, queue_size=1)
    BBox_pub = rospy.Publisher("/det_ros/results", BboxList, queue_size=1)

    

    BBox_1 = Bbox()
    BboxList_ = BboxList()
        #   x1   y1   x2  y2
    # box = [300, 200, 350, 240]
    box = [400, 300, 450, 340]
    BBox_1.bbox[0] = box[0]
    BBox_1.bbox[1] = box[1]
    BBox_1.bbox[2] = box[2]
    BBox_1.bbox[3] = box[3]
    BBox_1.conf = 0.9
    BBox_1.label = 1
    BboxList_.Bboxlist.append(BBox_1)

    BBox_2 = Bbox()
        #   x1   y1   x2  y2
    # box = [300, 200, 350, 240]
    box = [200, 300, 250, 340]
    BBox_2.bbox[0] = box[0]
    BBox_2.bbox[1] = box[1]
    BBox_2.bbox[2] = box[2]
    BBox_2.bbox[3] = box[3]
    BBox_2.conf = 0.9
    BBox_2.label = 1
    BboxList_.Bboxlist.append(BBox_2)


    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if resule_image is not None:
            for BBox_ in BboxList_.Bboxlist:
                resule_image = cv2.rectangle(resule_image, (BBox_.bbox[0],BBox_.bbox[1]), (BBox_.bbox[2],BBox_.bbox[3]), (0,255,0), 2)
            cv2.imshow("bbox_img", resule_image)
            BboxList_.header = img_header
            try:
                Image_pub_ = bridge.cv2_to_imgmsg(resule_image, "bgr8")
                Image_pub_.header = img_header
                image_pub.publish(Image_pub_)
                BBox_pub.publish(BboxList_)
            except CvBridgeError as e:
                print e
            rate.sleep()
        if cv2.waitKey(1) == ord('q'):
            break
    cv2.destroyAllWindows()#删除建立的全部窗口