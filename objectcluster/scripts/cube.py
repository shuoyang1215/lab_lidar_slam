from cv2 import add
from numpy import rate
from yaml import Mark
from visualization_msgs.msg import Marker
import rospy

front = 10
right = 0
z = -0.53
scale_x = 0.4
scale_y = 0.33
scale_z = 0.7


rospy.init_node("ObjGT")

rate = rospy.Rate(10)
while not rospy.is_shutdown():

    marker = Marker()
    marker.header.frame_id = "zvision_lidar"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ""
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 0.5
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.id = 1


    marker.pose.position.x = right - 0.5 * scale_x
    marker.pose.position.y = front + 0.13 + 0.5 * scale_y
    marker.pose.position.z = z + 0.5 * scale_z
    marker.scale.x = scale_x
    marker.scale.y = scale_y
    marker.scale.z = scale_z

    marker.lifetime = rospy.Duration();
    pub = rospy.Publisher("GT",Marker, queue_size=1)
    pub.publish(marker)
    rate.sleep()