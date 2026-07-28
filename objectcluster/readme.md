###运行
+ roslaunch objectcluster ObjSegVoxelGrid.launch


###输入 修改yaml文件
+ 雷达话题 /zvision_lidar_points
+ 雷达高度 LidarHigh
+ 深度相机高度 DeepCameraHigh
+ 深度相机话题 /camera/depth/image_rect_raw 或 /camera/aligned_depth_to_color/image_raw
+ 深度相机内参 dfx，dfy，dcx，dcy
+ 深度相机与雷达之间的外参 dcamera2lidar_R，dcamera2lidar_T
+ 深度相机俯仰角 DCameraPitch


###输出
+ 障碍物点云 /objseg_gridclass/ObjPointCloud sensor_msgs::PointCloud2
+ 障碍物包围框 /objseg_gridclass/LidarObjBox visualization_msgs::MarkerArray
+ 雷达加深度相机完整点云 /objseg_gridclass/fullPointCloud sensor_msgs::PointCloud2


###launch
+ AllinOne.launch 同时启动障碍物检测，2D检测，检测结果融合
+ ObjSegVoxelGrid_lidar.launch 启动车前雷达障碍物检测
+ ObjSegVoxvelGrid_realsense.launch 启动车后深度相机障碍物检测