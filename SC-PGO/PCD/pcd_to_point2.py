'''
Author: zhenya-lei 04181254@cumt.edu.cn
Date: 2025-01-02 16:32:43
LastEditors: zhenya-lei 04181254@cumt.edu.cn
LastEditTime: 2025-01-22 20:30:47
FilePath: /paper_1/pcd_to_point2.py
Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
'''
#将点云pcd文件中点云转换为ros话题消息并发布
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

def main():
    # 初始化ROS节点
    rospy.init_node('pcd_publisher', anonymous=True)
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    
    # 设置点云文件路径
    pcd_file_path = 'scans_lc.pcd'

    # 读取点云文件
    pcd = o3d.io.read_point_cloud(pcd_file_path)
    
    # 获取点云数据和颜色
    points = np.asarray(pcd.points)
    
    # # 添加x和y方向的偏移量（1.5米）
    points[:, 0] += 2.0  # x方向偏移1.5米
    points[:, 1] += 2.0  # y方向偏移1.5米
    
    # 如果点云没有颜色，我们可以根据坐标生成一些颜色
    if not pcd.has_colors():
        # 使用点的坐标来生成颜色（这只是一个示例，你可以使用其他方式生成颜色）
        colors = np.zeros_like(points)
        colors[:, 0] = (points[:, 0] - points[:, 0].min()) / (points[:, 0].max() - points[:, 0].min())  # R
        colors[:, 1] = (points[:, 1] - points[:, 1].min()) / (points[:, 1].max() - points[:, 1].min())  # G
        colors[:, 2] = (points[:, 2] - points[:, 2].min()) / (points[:, 2].max() - points[:, 2].min())  # B
    else:
        colors = np.asarray(pcd.colors)
    
    # 将颜色值转换为RGB格式（0-255范围）
    colors_rgb = (colors * 255).astype(np.uint8)
    
    # 创建带有颜色的点云字段
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
        PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
        PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
    ]
    
    # 组合点和颜色数据
    cloud_data = []
    for i in range(len(points)):
        cloud_data.append([
            points[i][0], points[i][1], points[i][2],
            colors_rgb[i][0], colors_rgb[i][1], colors_rgb[i][2]
        ])

    # 创建PointCloud2消息
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    
    # 转换为ROS消息格式（包含RGB信息）
    pc2_msg = pc2.create_cloud(header, fields, cloud_data)
    
    rate = rospy.Rate(1)  # 1Hz
    
    while not rospy.is_shutdown():
        pub.publish(pc2_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
