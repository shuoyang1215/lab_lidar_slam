/*该类的功能主要是点云的二维栅格处理
 edited by goldqiu -2022-04-9
*/
#include "map_conversion/pointcloud_process/pointcloud_2d_process.hpp"
#include "glog/logging.h"

namespace map_conversion {
  Pointcloud2dProcess::Pointcloud2dProcess(YAML::Node& config_node)
  : global_map_data(new CloudData::CLOUD())
  {

    Global_map_file = ros::package::getPath("livox_mapping") + "/PCD/scans_lc.pcd";
    local_cloud_topic = config_node["local_cloud_frame"].as<std::string>();
    topic_frame_id = config_node["frame_id"].as<std::string>();

    global_max_z_adjust = atof((config_node["global_max_z"].as<std::string>()).c_str());
    global_min_z_adjust = atof((config_node["global_min_z"].as<std::string>()).c_str());
    current_max_z_adjust = atof((config_node["current_max_z"].as<std::string>()).c_str());
    current_min_z_adjust = atof((config_node["current_min_z"].as<std::string>()).c_str());

    global_map_resolution = atof((config_node["2d_global_map_resolution"].as<std::string>()).c_str());
    current_map_resolution = atof((config_node["2d_current_map_resolution"].as<std::string>()).c_str());

    lidar_height = atof((config_node["lidar_height"].as<std::string>()).c_str());

    outlier_removal_method = atoi((config_node["outlier_removal_method"].as<std::string>()).c_str());
    mean_k = atoi((config_node["mean_k"].as<std::string>()).c_str());
    mul_thresh = atof((config_node["mul_thresh"].as<std::string>()).c_str());
    search_radius = atof((config_node["search_radius"].as<std::string>()).c_str());
    min_neighbors = atoi((config_node["min_neighbors"].as<std::string>()).c_str());

    // 读取体素滤波器参数，对点云先进行体素滤波去除杂点后再进行2D栅格转换
    std::vector<float> voxel_size(3);
    for (int i = 0; i < voxel_size.size(); i++) {
        voxel_size.at(i) = config_node["voxel_filter_size"][i].as<float>();
    }
    sor.setLeafSize(voxel_size[0], voxel_size[1], voxel_size[2]);  
  }

  void Pointcloud2dProcess::find_Z_value(CloudData::CLOUD_PTR cloud_data){
      std::cout << "初始点云数据点数：" << cloud_data->points.size() << std::endl;

      for(int i = 0; i < cloud_data->points.size() - 1; i++){
          if(cloud_data->points[i].z>max_z){
              max_z=cloud_data->points[i].z;
            }
          if(cloud_data->points[i].z<min_z){
              min_z=cloud_data->points[i].z;
            }
      }
      std::cout<<"orig max_z="<<max_z<<",min_z="<<min_z<<std::endl;
  }

  void Pointcloud2dProcess::global_map_init()
  {
    /* 从全局地图中生成栅格地图
    
    // if (pcl::io::loadPCDFile(Global_map_file, *(global_map_data)) == -1)
    // {
    //   PCL_ERROR ("Couldn't read file: %s \n", Global_map_file.c_str());
    //   return ;
    // } 
    // LOG(INFO) << "Load global map, size:" << global_map_data->points.size();

    // *************************************************************************************************************
    // ece01的包需要变化 因为初始时刻的IMUz轴不与重力对齐
    // // 采用静止初始化时的倾角来变换全局地图使得其与g方向垂直
    // Eigen::Matrix3d G_R_I;
    // G_R_I << 0.986839, 0.00411836, -0.161655,
    //         0, -0.999676, -0.0254679,
    //         -0.161708, 0.0251327, -0.986519;
    // cerr << "G_R_I: " << G_R_I << endl;
    // Eigen::Matrix4d G_T_I = Eigen::Matrix4d::Identity();
    // G_T_I.block<3, 3>(0, 0) = G_R_I;
    // pcl::transformPointCloud(*global_map_data, *global_map_data, G_T_I);

    // // DEBUG：存储与g对齐后的点云
    // global_map_data->height = 1;
    // global_map_data->width = global_map_data->points.size();
    // pcl::io::savePCDFileASCII(ros::package::getPath("map_conversion") + "/data/" + "transforemed_global_map_data.pcd", *global_map_data);
    // *************************************************************************************************************

    // find_Z_value(global_map_data);
    
    // PassThroughFilter(global_map_data,global_map_no_filter,false,max_z,min_z);

    ////// 直接对全局点云进行z轴截断
    // PassThroughFilter(global_map_data,global_map_after_filter,false,global_max_z_adjust,global_min_z_adjust);
    
    */

    ////// 对局部地图进行高度截断后，再根据位姿拼成全局地图
    PassThroughLocalMap(global_map_after_filter,false,global_max_z_adjust,global_min_z_adjust);

    // 采用体素滤波器对点云进行降采样
    std::cout << "Before VoxelGrid-global_map_after_filter.size: "<<global_map_after_filter.cloud_ptr->points.size() << std::endl;
    sor.setInputCloud(global_map_after_filter.cloud_ptr);
    sor.filter(*global_map_after_filter.cloud_ptr);
    std::cout << "After VoxelGrid-global_map_after_filter.size: "<<global_map_after_filter.cloud_ptr->points.size() << std::endl;

    return;
  }

  void Pointcloud2dProcess::outlier_removal(CloudData::CLOUD_PTR& cloudmap){
    if (outlier_removal_method == 1){
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(cloudmap);
      sor.setMeanK(mean_k);                  // 邻域点数
      sor.setStddevMulThresh(mul_thresh);       // 标准差倍数阈值
      sor.filter(*cloudmap);              
    }
    else if (outlier_removal_method == 2){
      pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
      ror.setInputCloud(cloudmap);
      ror.setRadiusSearch(search_radius);          // 半径搜索范围
      ror.setMinNeighborsInRadius(min_neighbors);   // 邻域内最小点数
      ror.filter(*cloudmap);             
    }
    else ROS_WARN("No method matched!!!");
  }

  CloudData::CLOUD_PTR Pointcloud2dProcess::local2global(const CloudData::CLOUD_PTR &cloudIn, const Pose6D &tf)
  {
      CloudData::CLOUD_PTR cloudOut(new CloudData::CLOUD());

      int cloudSize = cloudIn->size();
      cloudOut->resize(cloudSize);

      Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

      int numberOfCores = 16;
  #pragma omp parallel for num_threads(numberOfCores)
      for (int i = 0; i < cloudSize; ++i)
      {
          const auto &pointFrom = cloudIn->points[i];
          cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
          cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
          cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
          // cloudOut->points[i].intensity = pointFrom.intensity;
      }
      return cloudOut;
  }

  void Pointcloud2dProcess::PassThroughLocalMap(CloudData& cloud_after_PassThrough, const bool &flag_in, double z_max, double z_min)
  {
    // 读取局部地图对应的位姿
    ifstream inputFile(std::string(ros::package::getPath("livox_mapping") + "/PCD/lio_livox_slam.txt")); // 打开输入文件
    std::vector<Pose6D> local_map_poses;
    if (inputFile.is_open()) {
      std::string line;
      while (getline(inputFile, line)) { // 逐行读取文件内容
        double time, val_x, val_y, val_z, val_roll, val_pitch, val_yaw;
        sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf", &time, &val_x, &val_y, &val_z, &val_roll, &val_pitch, &val_yaw); // 使用sscanf函数解析每行数据
        // cout << std::fixed << "Read from pose file: " << val_x << "," << val_y << "," << val_z << "," << val_roll << "," << val_pitch << "," << val_yaw << endl;
        Pose6D pose6d{val_x, val_y, val_z, val_roll, val_pitch, val_yaw};
        local_map_poses.push_back(pose6d);
      }
      inputFile.close(); 
    }
    // debug
    // cout << "local_map_poses.size(): " << local_map_poses.size() << endl;

    std::string local_map_dir(std::string(ros::package::getPath("livox_mapping") + "/PCD/pcds_ori/"));
    cout << "local_map_dir: " << local_map_dir << endl;
    CloudData::CLOUD_PTR laserCloudMapPGO(new CloudData::CLOUD());
    for (int idx = 0; idx < local_map_poses.size(); ++idx){
      // 加载局部地图
      std::stringstream idx_;
      idx_ << std::setw(6) << std::setfill('0') << idx;
      std::string res = idx_.str();
      std::string local_map_file(local_map_dir + res + ".pcd");
      CloudData::CLOUD_PTR local_map(new CloudData::CLOUD());
      if (pcl::io::loadPCDFile(local_map_file, *local_map) == -1)
      {
          PCL_ERROR("Read file fail!\n");
      }
      // cout << "local_map->points.size(): " << local_map->points.size() << endl;
      // 根据对应位姿的z轴高度和设定的高度截断阈值对其进行z轴截断后再变化到世界坐标系中   
      pcl::PassThrough<pcl::PointXYZ> passthrough;
      CloudData::CLOUD_PTR local_map_filtered(new CloudData::CLOUD());
      passthrough.setInputCloud(local_map);
      passthrough.setFilterFieldName("z");
      // 根据雷达距离地面的安装高度来确定障碍物高度范围，距离地面z_min~z_max高度的点会被保留
      passthrough.setFilterLimits(-lidar_height + z_min, -lidar_height + z_max);
      passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
      passthrough.filter(*local_map_filtered);
      ///////////////////////////////////////
      // 只裁剪局部点云中小范围的点，防止远处的杂点对建图造成的影响
      // passthrough.setInputCloud(local_map_filtered);
      // passthrough.setFilterFieldName("x");
      // passthrough.setFilterLimits(-10, 10);
      // passthrough.filter(*local_map_filtered);
      // passthrough.setInputCloud(local_map_filtered);
      // passthrough.setFilterFieldName("y");
      // passthrough.setFilterLimits(-10, 10);
      // passthrough.filter(*local_map_filtered);
      ///////////////////////////////////////
      // /// 体素滤波
      // sor.setInputCloud(local_map_filtered);
      // sor.filter(*local_map_filtered);        
      *laserCloudMapPGO += *local2global(local_map_filtered, local_map_poses[idx]);   
    }
    // 对3D地图点云进行杂点去除
    if (outlier_removal_method) {
      std::cout << "Before outliers_removal size: "<< laserCloudMapPGO->points.size() << std::endl;
      outlier_removal(laserCloudMapPGO);
      std::cout << "After outliers_removal size: "<< laserCloudMapPGO->points.size() << std::endl;      
    }

    *cloud_after_PassThrough.cloud_ptr = *laserCloudMapPGO;
  }

  void Pointcloud2dProcess::PassThroughFilter(CloudData::CLOUD_PTR pcd_cloud,CloudData& cloud_after_PassThrough, const bool &flag_in,double z_max, double z_min)
  {
      /*方法一：直通滤波器对点云进行处理。*/
      pcl::PassThrough<pcl::PointXYZ> passthrough;
      passthrough.setInputCloud(pcd_cloud);
      passthrough.setFilterFieldName("z");
      passthrough.setFilterLimits(z_min, z_max);
      std::cout << "The range along the z axis: (" << z_min << ", " << z_max << ")." << std::endl;
      passthrough.setFilterLimitsNegative(flag_in);//true表示保留范围外，false表示保留范围内
      passthrough.filter(*cloud_after_PassThrough.cloud_ptr);
      std::cout << "直通滤波后点云数据点数：" << cloud_after_PassThrough.cloud_ptr->points.size() << std::endl;
  }

  /* 直接对点云取x、y值，生成2D点云 */
  void Pointcloud2dProcess::Pointcloud_to_2d_grid(const CloudData& pcd_cloud, nav_msgs::OccupancyGrid& msg, double map_resolution)
  {
    msg.header.seq = 0;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";

    msg.info.map_load_time = ros::Time::now();
    msg.info.resolution = map_resolution;

    double x_min, x_max, y_min, y_max;
    double z_max_grey_rate = 0.05;
    double z_min_grey_rate = 0.95;
    double k_line = (z_max_grey_rate - z_min_grey_rate) / (max_z - min_z);
    double b_line = (max_z * z_min_grey_rate - min_z * z_max_grey_rate) / (max_z - min_z);

    if(pcd_cloud.cloud_ptr->points.empty())
    {
      ROS_WARN("pcd is empty!\n");
      return;
    }

    for(int i = 0; i < pcd_cloud.cloud_ptr->points.size(); i++)
    {
      if(i == 0)
      {
        x_min = x_max = pcd_cloud.cloud_ptr->points[i].x;
        y_min = y_max = pcd_cloud.cloud_ptr->points[i].y;
      }

      double x = pcd_cloud.cloud_ptr->points[i].x;
      double y = pcd_cloud.cloud_ptr->points[i].y;

      if(x < x_min) x_min = x;
      if(x > x_max) x_max = x;

      if(y < y_min) y_min = y;
      if(y > y_max) y_max = y;
    }

    msg.info.origin.position.x = x_min;
    msg.info.origin.position.y = y_min;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation.x = 0.0;
    msg.info.origin.orientation.y = 0.0;
    msg.info.origin.orientation.z = 0.0;
    msg.info.origin.orientation.w = 1.0;

    msg.info.width = int((x_max - x_min) / map_resolution);
    msg.info.height = int((y_max - y_min) / map_resolution);

    msg.data.resize(msg.info.width * msg.info.height);
    msg.data.assign(msg.info.width * msg.info.height, 0);

    cout << "data size = " << msg.data.size() << endl;

    for(int iter = 0; iter < pcd_cloud.cloud_ptr->points.size(); iter++)
    {
      int i = int((pcd_cloud.cloud_ptr->points[iter].x - x_min) / map_resolution);
      if(i < 0 || i >= msg.info.width) continue;

      int j = int((pcd_cloud.cloud_ptr->points[iter].y - y_min) / map_resolution);
      if(j < 0 || j >= msg.info.height - 1) continue;

      msg.data[i + j * msg.info.width] = 100;
  //    msg.data[i + j * msg.info.width] = int(255 * (pcd_cloud.cloud_ptr->points[iter].z * k_line + b_line)) % 255;
    }
  }
} // namespace data_input