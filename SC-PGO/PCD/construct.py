import os
import numpy as np
import pandas as pd


def readpcd(pcd_folder):
    with open(pcd_folder, 'r') as f:
        lines = f.readlines()
    # 提取点云数据行
    data_lines = [line.split() for line in lines if not line.startswith('#')]
    # 将数据转换为NumPy数组
    # print(data_lines[0:10])
    point_cloud = np.array(data_lines[10:],dtype=np.float32)
    points = point_cloud[:, :3]
    return points

def savepcd(points, pcd_folder):
    with open(pcd_folder, 'w') as f:
        f.write('VERSION .7\n')
        f.write('FIELDS x y z\n')
        f.write('SIZE 4 4 4\n')
        f.write('TYPE F F F\n')
        f.write('COUNT 1 1 1\n')
        f.write(f'WIDTH {points.shape[0]}\n')
        f.write('HEIGHT 1\n')
        f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {points.shape[0]}\n')
        f.write('DATA ascii\n')

        for point in points:
            f.write(' '.join([str(value) for value in point]))
            f.write('\n')

    return


pcd_file_path_1 = os.path.join( "./bongeunsa_result.pcd")
pcd_file_path_2= os.path.join( "./points_top.pcd")

# point_cloud_1 = o3.io.read_point_cloud(pcd_file_path_1)
# point_cloud_2 = o3.io.read_point_cloud(pcd_file_path_2)
points1 = readpcd(pcd_file_path_1)
points2 = readpcd(pcd_file_path_2)

N1 = points1.shape[0]
N2 = points2.shape[0]
points = np.zeros((N1+N2,3))
points[:N1, :] = points1
points[N1:N2+N1, :] = points2

savepcd(points, 'result.pcd')