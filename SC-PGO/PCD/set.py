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


pcd_folder = './pcds_ori'
pcd_files =os.listdir( pcd_folder)

if not os.path.exists('./pcds'):
    os.mkdir('./pcds')

for  pcd_file in pcd_files:
    pcd_file_path = os.path.join(pcd_folder, pcd_file)
    # 打开.pcd文件并读取数据

    points = readpcd(pcd_file_path)
    # print("点云数据：")
    # print(points)
    # print(points.shape)
    # break    
    indexes = points[:,2] > 2.
    points_top = points[indexes]

    indexes = points[:,2] <= 2.
    points_ground = points[indexes]


    points_top_pcd = os.path.join( './pcds_top', pcd_file)
    points_ground_pcd = os.path.join( './pcds', pcd_file)


    savepcd(points_ground, points_ground_pcd)



pcd_file_path = os.path.join( "./scans_lc.pcd")
# points = np.array(o3.io.read_point_cloud(pcd_file_path).points) 
points =  readpcd(pcd_file_path)
print(points.shape)
indexes = points[:,2] > 2.
points_top = points[indexes]
print(points_top.shape)
indexes = points[:,2] <= 2.
points_ground = points[indexes]
print(points_ground.shape)

points_top_pcd = os.path.join( './points_top.pcd')
points_ground_pcd = os.path.join( './points_ground.pcd')


savepcd(points_ground, points_ground_pcd)
savepcd(points_top, points_top_pcd)
# pcl_top = o3.PointCloud()
# pcl_top.points = o3.Vector3dVector(points_top)
# o3.write_point_cloud(points_top_pcd, pcl_top)

# pcl_ground = o3.PointCloud()
# pcl_ground.points = o3.Vector3dVector(points_ground)
# o3.write_point_cloud(points_ground_pcd, pcl_ground)



def rpyToQua(roll_angle, pitch_angle, yaw_angle):
    roll_half = roll_angle / 2
    pitch_half = pitch_angle / 2
    yaw_half = yaw_angle / 2

    w = np.cos(roll_half) * np.cos(pitch_half) * np.cos(yaw_half) + np.sin(roll_half) * np.sin(pitch_half) * np.sin(yaw_half)
    x = np.sin(roll_half) * np.cos(pitch_half) * np.cos(yaw_half) - np.cos(roll_half) * np.sin(pitch_half) * np.sin(yaw_half)
    y = np.cos(roll_half) * np.sin(pitch_half) * np.cos(yaw_half) + np.sin(roll_half) * np.cos(pitch_half) * np.sin(yaw_half)
    z = np.cos(roll_half) * np.cos(pitch_half) * np.sin(yaw_half) - np.sin(roll_half) * np.sin(pitch_half) * np.cos(yaw_half)
    
    return x, y, z, w

posefile = os.path.join('./lio_livox_slam.txt')
poses = np.loadtxt(posefile, dtype=float)
csvfile = os.path.join('./poses_lidar2body.csv')
i = 0
new_posex = []
new_posey = []
new_posez = []
new_poseqx = []
new_poseqy = []
new_poseqz = []
new_poseqw = []
new_timestamp = []
index = []
for pose in poses:
    
    qx, qy, qz, qw = rpyToQua(pose[4], pose[5], pose[6])
    new_posex.append(pose[1])
    new_posey.append(pose[2])
    new_posez.append(pose[3])
    new_poseqx.append(qx)
    new_poseqy.append(qy)
    new_poseqz.append(qz)
    new_poseqw.append(qw)
    # new_timestamp.append("{:0>6d}".format(i))
    new_timestamp.append(pose[0])
    index.append(i)
    i+=1

df = pd.DataFrame({'index':index, 'timestamp' : new_timestamp, 'x' : new_posex, 'y' : new_posey, 'z' : new_posez, 'qx' : new_poseqx, 'qy' : new_poseqy, 'qz': new_poseqz ,'qw' : new_poseqw})
df.to_csv(csvfile, index=False, sep=',')

# pcd_file_path_1 = os.path.join( "./bongeunsa_result.pcd")
# pcd_file_path_2= os.path.join( "./points_top.pcd")

# point_cloud_1 = o3.io.read_point_cloud(pcd_file_path_1)
# point_cloud_2 = o3.io.read_point_cloud(pcd_file_path_2)
# points1 = np.array (point_cloud_1.points)
# points2 = np.array (point_cloud_2.points)

# N1 = points1.shape[0]
# N2 = points2.shape[0]
# points = np.zeros((N1+N2,3))
# points[:N1, :] = points1
# points[N1:N2+N1, :] = points2

# pcl = o3.PointCloud()
# pcl.points = o3.Vector3dVector(points)
# o3.write_point_cloud("./all_result.pcd", pcl)
