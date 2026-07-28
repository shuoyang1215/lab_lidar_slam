# -*- coding: UTF-8 -*-
#

import math
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np

inrange_pub = None
filter_pub = None
nearest_points_pub = None



def lidar_callback(cloud):
    global inrange_pub, filter_pub, nearest_points_pub

    # 总点数
    NUM_DATA_POINTS = 512000
    # 判断草地平地的点数阈值
    TOTAL_POINT_THR = 400

    # 点云范围
    X_MIN = -0.5
    X_MAX = 0.5
    Y_MIN = 0.5
    Y_MAX = 2


    GRID_SIZE = 0.1         # 栅格大小0.1m
    X_GRID_NUM = 11         # X方向栅格数，即(X_MAX-X_MIN)/GRID_SIZE+1，需结合范围调整 20220522 # 更改！
    Y_GRID_NUM = 16         # Y方向栅格数，即(Y_MAX-Y_MIN)/GRID_SIZE+1，需结合范围调整 20220522 # 更改！
    X_GRID_OFFSET = 5       # 将栅格地图索引转换为正值的偏移量，-X_MIN/GRID_SIZE，需结合范围调整 20220522
    Y_GRID_OFFSET = -5      # 将栅格地图索引转换为正值的偏移量，-Y_MIN/GRID_SIZE，需结合范围调整 20220522

    # 栅格内最大点数
    GRID_POINT = 100        # 更改！


    # 栅格内达到此点数即为障碍物栅格
    grid_point_thr = 10

    # 平地时点云的 z 轴坐标范围
    stick_z1 = -0.50
    stick_z2 = -0.1
    # 草地时点云的 z 轴坐标范围
    person_z1 = -0.16
    person_z2 = -0.1

    count = 0
    nearest_grid_x = 100
    nearest_grid_y = 100

    nearest_x = 1000.0
    nearest_y = 1000.0
    nearest_z = 0.0

    lidar_height = 0.58

    min_point = [99, 99]    # 更改！
    max_point = [0 ,0]      # 更改！

    # 每个栅格里的点数
    grid_map_point_num = []
    # 每个栅格里的点
    grid_map = []

    # 读取点云
    gen = point_cloud2.read_points(
        cloud, field_names=("x", "y", "z"), skip_nans=True)
    points = []
    for p in gen:
        points.append(p)
    points = np.array(points)

    # select_points = []
    # # 均匀降采样
    # for i in range(points.shape[0]):
    #     if i % 4 == 0:
    #         select_points.append(points[i, :])
    # points = np.array(select_points)

    # 随机降采样
    mask = np.random.binomial(n=1, p=0.25, size=points.shape[0])>0
    # sum = np.sum(mask)
    points = points[mask]

    # point = [0, 0, 0]
    # 创建栅格
    for i in range(X_GRID_NUM):
        grid_map_point_num.append([])
        grid_map.append([])
        for j in range(Y_GRID_NUM):
            grid_map_point_num[i].append(0)
            grid_map[i].append([])

    inrange_points = []
    for i in range(points.shape[0]):
        point = points[i]
        if ((point[0] > X_MIN) and (point[0] < X_MAX) and (point[1] > Y_MIN) and (point[1] < Y_MAX)):
                inrange_points.append(point)
    print("point inrange:" + str(len(inrange_points)))

    filtered_points = []

    # 第一次过滤地面点,用于找平地上的 stick
    for i in range(points.shape[0]):
        point = points[i]
        if ((point[0] > X_MIN) and (point[0] < X_MAX) and (point[1] > Y_MIN) and (point[1] < Y_MAX)):
            if ((point[2] > stick_z1) and (point[2] < stick_z2)):
                filtered_points.append(point)
                count += 1

                if (point[0]>max_point[0]): max_point[0] = point[0] # 更改！
                if (point[0]<min_point[0]): min_point[0] = point[0] # 更改！
                if (point[1]>max_point[1]): max_point[1] = point[1] # 更改！
                if (point[1]<min_point[1]): min_point[1] = point[1] # 更改！

    print("point after filter:" + str(count))
    

    # 如果是草地,第二次过滤地面点 person, 并分配到格子中
    if(count > TOTAL_POINT_THR) or (max_point[0]!=0 and math.sqrt((max_point[0]-min_point[0])*(max_point[0]-min_point[0]) + (max_point[1]-min_point[1])*(max_point[1]-min_point[1]))>0.8): # 更改！
        print("grass scene")
        for i in range(count):
            point = filtered_points[i]
            if ((point[2] > person_z1) and (point[2] < person_z2)):
                px = (int)(point[0] / GRID_SIZE + 0.5)
                py = (int)(point[1] / GRID_SIZE + 0.5)

                x_offset = px + X_GRID_OFFSET
                y_offset = py + Y_GRID_OFFSET

                # 更改！
                # if ((x_offset == X_GRID_NUM) or (y_offset == Y_GRID_NUM)):
                #     continue

                idx = grid_map_point_num[x_offset][y_offset]

                if(idx >= GRID_POINT):
                    continue
                
                grid_map_point_num[x_offset][y_offset] += 1

                if(idx > grid_point_thr):  # 更改！
                    if(x_offset*x_offset + y_offset*y_offset < nearest_grid_x*nearest_grid_x + nearest_grid_y*nearest_grid_y):
                        nearest_grid_x = x_offset
                        nearest_grid_y = y_offset

                grid_map[x_offset][y_offset].append(point)

    # 如果不是草地直接分配到格子中
    else:
        print("plane scene")
        for i in range(count):
            point = filtered_points[i]
            px = (int)(point[0] / GRID_SIZE + 0.5)
            py = (int)(point[1] / GRID_SIZE + 0.5)

            x_offset = px + X_GRID_OFFSET
            y_offset = py + Y_GRID_OFFSET

            # 更改！
            # if ((x_offset == X_GRID_NUM) or (y_offset == Y_GRID_NUM)):
            #     continue

            idx = grid_map_point_num[x_offset][y_offset]

            if(idx >= GRID_POINT):
                continue

            grid_map_point_num[x_offset][y_offset] += 1

            if(idx > grid_point_thr):  # 更改！
                if(x_offset*x_offset + y_offset*y_offset < nearest_grid_x*nearest_grid_x + nearest_grid_y*nearest_grid_y):
                    nearest_grid_x = x_offset
                    nearest_grid_y = y_offset

            grid_map[x_offset][y_offset].append(point)


    if ((nearest_grid_x == 100) or (nearest_grid_y == 100)):
        nearest_x = 1000
        nearest_y = 1000
        nearest_z = 0
        print("no object")
    else:
        nearest_grid_point_num = grid_map_point_num[nearest_grid_x][nearest_grid_y]

        for i in range(nearest_grid_point_num):
            point = grid_map[nearest_grid_x][nearest_grid_y][i]

            if(point[0]*point[0] + point[1]*point[1] < nearest_x*nearest_x + nearest_y*nearest_y):
                nearest_x = point[0]
                nearest_y = point[1]
                nearest_z = point[2]


        print("obj_position: x:"+str(nearest_x)+" y:"+str(nearest_y)+" z:"+str(nearest_z))


    # 可视化

    # 发送范围内的点云
    points_np = np.array(inrange_points)
    points_msg = PointCloud2()
    points_msg.header.stamp = rospy.Time().now()
    points_msg.header.frame_id = "zvision_lidar"
    points_msg.height = 1
    points_msg.width = points_np.shape[0]
    points_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    points_msg.is_bigendian = False
    points_msg.point_step = 12
    points_msg.row_step = points_msg.point_step * points_np.shape[0]
    points_msg.is_dense = False
    points_msg.data = np.asarray(points_np, np.float32).tostring()
    inrange_pub.publish(points_msg)

    # 发送过滤地面后的点云
    points_np = np.array(filtered_points)
    points_msg = PointCloud2()
    points_msg.header.stamp = rospy.Time().now()
    points_msg.header.frame_id = "zvision_lidar"
    points_msg.height = 1
    points_msg.width = points_np.shape[0]
    points_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    points_msg.is_bigendian = False
    points_msg.point_step = 12
    points_msg.row_step = points_msg.point_step * points_np.shape[0]
    points_msg.is_dense = False
    points_msg.data = np.asarray(points_np, np.float32).tostring()
    filter_pub.publish(points_msg)


    # 发送障碍物点云
    nearest_grid_points = np.array([nearest_x, nearest_y, -0.55]).reshape((1,-1))
    # nearest_grid_points = np.array([0, 0, 0])
    points_msg = PointCloud2()
    points_msg.header.stamp = rospy.Time().now()
    points_msg.header.frame_id = "zvision_lidar"
    points_msg.height = 1
    points_msg.width = nearest_grid_points.shape[0]
    points_msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)
    ]
    points_msg.is_bigendian = False
    points_msg.point_step = 12
    points_msg.row_step = points_msg.point_step * nearest_grid_points.shape[0]
    points_msg.is_dense = False
    points_msg.data = np.asarray(nearest_grid_points, np.float32).tostring()
    nearest_points_pub.publish(points_msg)


    return







    # # 裁减点云 一个for循环
    # mask = (points[:, 0] > area[0]) & (points[:, 0] < area[1]) & (
    #     points[:, 1] > area[2]) & (points[:, 1] < area[3])
    # points = points[mask]

    # # # 随机降采样
    # # mask = np.random.binomial(n=1, p=0.2, size=points.shape[0])>0
    # # sum = np.sum(mask)
    # # points = points[mask]

    # select_points = []
    # # 均匀降采样
    # for i in range(points.shape[0]):
    #     if i % 5 == 0:
    #         select_points.append(points[i, :])
    # points = np.array(select_points)

    # # 发送采样后的点云
    # points_msg = PointCloud2()
    # points_msg.header.stamp = rospy.Time().now()
    # points_msg.header.frame_id = "zvision_lidar"
    # points_msg.height = 1
    # points_msg.width = points.shape[0]
    # points_msg.fields = [
    #     PointField('x', 0, PointField.FLOAT32, 1),
    #     PointField('y', 4, PointField.FLOAT32, 1),
    #     PointField('z', 8, PointField.FLOAT32, 1)
    # ]
    # points_msg.is_bigendian = False
    # points_msg.point_step = 12
    # points_msg.row_step = points_msg.point_step * points.shape[0]
    # points_msg.is_dense = False
    # points_msg.data = np.asarray(points, np.float32).tostring()
    # lidar_pub.publish(points_msg)

    # # =====================================
    # #           算法开始
    # # 去除地面，一个for循环
    # ground_mask = (points[:, 2] > area[4]) & (points[:, 2] < area[5])
    # points = points[ground_mask]
    # print str(points.shape[0])
    # # 分散到网络中 0.1m*0.1m的格子，每个点增加两维，表示其栅格坐标，另外用一个栅格数组记录点数
    # # 第二个for循环
    # grid_map_point_num = np.zeros((x_grid_num, y_grid_num))

    # # 制造栅格图
    # grid_map = []
    # for i in range(x_grid_num):
    #     grid_map.append([])
    #     for j in range(y_grid_num):
    #         grid_map[i].append([])

    # nearest_grid = [100, 100]

    # # 点云分配到格子中
    # for p in points:
    #     x = int(round(p[0] // grid_size))
    #     y = int(round(p[1] // grid_size))
    #     x_offset = x + x_grid_offset
    #     y_offset = y + y_grid_offset
    #     grid_map[x_offset][y_offset].append(p)

    #     grid_map_point_num[x_offset, y_offset] += 1

    #     if (grid_map_point_num[x_offset, y_offset] > grid_point_thr):

    #         if (x*x + y*y) < (nearest_grid[0]*nearest_grid[0] + nearest_grid[1]*nearest_grid[1]):
    #             nearest_grid = [x, y]

    # # 发布最近栅格
    # nearest_grid_points = np.array([])
    # # 取出最近的格子的点云，发送出来
    # if nearest_grid[0] != 100:
    #     nearest_grid_points = np.array(
    #         grid_map[nearest_grid[0] + x_grid_offset][nearest_grid[1] + y_grid_offset])
    #     print "obj:"+str(nearest_grid[0]*grid_size)+","+str(nearest_grid[1]*grid_size)
    # #       算法结束
    # # ==============================

    # # 发送障碍物点云
    # points_msg = PointCloud2()
    # points_msg.header.stamp = rospy.Time().now()
    # points_msg.header.frame_id = "zvision_lidar"
    # points_msg.height = 1
    # points_msg.width = nearest_grid_points.shape[0]
    # points_msg.fields = [
    #     PointField('x', 0, PointField.FLOAT32, 1),
    #     PointField('y', 4, PointField.FLOAT32, 1),
    #     PointField('z', 8, PointField.FLOAT32, 1)
    # ]
    # points_msg.is_bigendian = False
    # points_msg.point_step = 12
    # points_msg.row_step = points_msg.point_step * nearest_grid_points.shape[0]
    # points_msg.is_dense = False
    # points_msg.data = np.asarray(nearest_grid_points, np.float32).tostring()
    # nearest_points_pub.publish(points_msg)


if __name__ == '__main__':
    try:
        rospy.init_node("SafeDet")

        lidar_sub = rospy.Subscriber(
            "/zvision_lidar_points", PointCloud2, lidar_callback, queue_size=1, buff_size=52428800)
        inrange_pub = rospy.Publisher(
            '/pointcloud_inrange', PointCloud2, queue_size=1)
        filter_pub = rospy.Publisher(
            '/pointcloud_filter', PointCloud2, queue_size=1)
        nearest_points_pub = rospy.Publisher(
            '/nearest_points_topic', PointCloud2, queue_size=1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            rate.sleep()

    except KeyboardInterrupt:
        print "shutting down"
