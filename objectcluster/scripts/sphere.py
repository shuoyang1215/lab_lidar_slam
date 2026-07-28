from visualization_msgs.msg import Marker
import rospy


front = 1
right = 0
z = -0.53
r = 0.5



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
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = 1


    marker.pose.position.x = right - 0.5 * r
    marker.pose.position.y = front + 0.13 + 0.5 * r
    marker.pose.position.z = z + 0.5 * r
    marker.scale.x = r
    marker.scale.y = r
    marker.scale.z = r

    marker.lifetime = rospy.Duration()
    pub = rospy.Publisher("GT",Marker, queue_size=1)
    pub.publish(marker)
    rate.sleep()