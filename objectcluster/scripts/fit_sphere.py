# -*- coding: UTF-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from sensor_msgs.msg import PointCloud2, geometry_msgs
from sensor_msgs import point_cloud2
import rospy
from visualization_msgs.msg import Marker, MarkerArray

min_counter_point = None

def marker_callback(data):
    global min_counter_point
    assert isinstance(data, MarkerArray)
    markerarray = data.markers
    minpoint = None
    mindis = 100
    for marker in markerarray:
        for point in marker.points:
            distance = np.sqrt(np.square(point.x) + np.square(point.y))
            if distance < mindis:
                mindis = distance
                minpoint = np.array([point.x, point.y])
    min_counter_point = minpoint



def point_callback(data):
    global min_counter_point
    assert isinstance(data, PointCloud2)
    points = np.array(point_cloud2.read_points_list(data, field_names=("x", "y", "z", "intensity")))

    if len(points) == 0:
        return
# ================================
    index=[]
    for i in points[:,3]:
        if i not in index:
            index.append(i)
# ================================
    min_dis = 100
    min_points = None
    for i in index:
        points_select = points[points[:,3]==i][:,:3]
        points_center = np.mean(points_select, axis=0)
        distance = np.sqrt(np.square(points_center[0]) + np.square(points_center[1]) + np.square(points_center[2]))
        if distance < min_dis:
            min_dis = distance
            min_points = points_select
    

# ==============================================
    Ax = -2 * min_points[:,0].reshape(-1,1)
    Ay = -2 * min_points[:,1].reshape(-1,1)
    Az = -2 * min_points[:,2].reshape(-1,1)
    Ad = np.ones((min_points[:,0].shape[0],1),dtype=float)
    A = np.concatenate([Ax,Ay,Az,Ad],axis=1)
    r = np.ones((min_points[:,0].shape[0],1), dtype=float).reshape(-1,1) * 0.235
    b = (-min_points[:,0]*min_points[:,0] -min_points[:,1]*min_points[:,1] -min_points[:,2]*min_points[:,2]).reshape(-1,1) + r*r
    
    # X = inv(A'*A)*A'*b
    ATA = np.dot(A.T, A)
    X = np.dot(np.dot(np.linalg.inv(ATA), A.T), b)
    
    print("func: "+str(X[0])+"^2+"+str(X[1])+"^2+"+str(X[2])+"=0.235^2")

    print(X[0])
    print(X[1])
    print(X[2])
    
    error = []
    for i in range(min_points[:,0].shape[0]):
        error2 = math.sqrt(pow((min_points[i,0]-X[0]), 2) + pow((min_points[i,1]-X[1]), 2) + pow((min_points[i,2]-X[2]), 2)) - 0.235
        error.append(error2)
    error = np.array(error)
    mean_error = error.mean()
    max_error = error[np.argmax(np.abs(error))]
    min_error = error[np.argmin(np.abs(error))]
    print("error: "+str(mean_error))

# ======================================

    sphere_pub = rospy.Publisher("FitSphere",Marker, queue_size=1)

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


    marker.pose.position.x = X[0]
    marker.pose.position.y = X[1]
    marker.pose.position.z = X[2]
    marker.scale.x = 0.47
    marker.scale.y = 0.47
    marker.scale.z = 0.47

    marker.lifetime = rospy.Duration()
    sphere_pub.publish(marker)

# ======================================
    text_pub = rospy.Publisher("FitSphereError", Marker, queue_size=1)

    text_marker = Marker()
    text_marker.header.frame_id = "zvision_lidar"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = ""
    text_marker.color.r = 0
    text_marker.color.g = 1
    text_marker.color.b = 0
    text_marker.color.a = 1
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.id = 0


    text_marker.pose.position.x = X[0]
    text_marker.pose.position.y = X[1]
    text_marker.pose.position.z = X[2]+0.5
    text_marker.scale.x = 0.2
    text_marker.scale.y = 0.2
    text_marker.scale.z = 0.2

    sphere_distance = np.sqrt(np.square(X[0]) + np.square(X[1]) + np.square(X[2]))
    min_counter_point_error = math.sqrt(pow((min_counter_point[0]-X[0]), 2) + pow((min_counter_point[1]-X[1]), 2)) - 0.235
    # text_marker.text = str(round(mean_error,4)) + " " + str(round(min_counter_point_error, 4)) + " " + str(round(sphere_distance, 4))
    text_marker.text =  "points_mean_error " + str(round(mean_error,4)) + \
                        "\npoints_max_error " + str(round(max_error,4)) + \
                        "\npoints_min_error " + str(round(min_error,4)) + \
                        "\nBounding_mean_error " + str(round(min_counter_point_error, 4))

    text_marker.lifetime = rospy.Duration()
    text_pub.publish(text_marker)


if __name__ == '__main__':
    rospy.init_node('fit_sphere')
    rospy.Subscriber('/objseg_gridclass/ObjPointCloud', PointCloud2, point_callback)
    rospy.Subscriber('/objseg_gridclass/BoundingMarkArray', MarkerArray, marker_callback)
    rospy.spin()