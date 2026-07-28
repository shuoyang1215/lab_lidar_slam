/*该类的功能主要是点云的二维栅格处理
 edited by goldqiu -2022-04-9
*/
#ifndef MAP_CONVERSION_POINTCLOUD_2D_PROCESS_HPP_
#define MAP_CONVERSION_POINTCLOUD_2D_PROCESS_HPP_
#include "map_conversion/ros_topic_interface/cloud_data.hpp"
#include <yaml-cpp/yaml.h>
#include "map_conversion/utility.hpp"
#include "map_conversion/cloud_filter/cloud_filter_interface.hpp"
#include "map_conversion/cloud_filter/voxel_filter.hpp"

#include <vector>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <ros/package.h>
#include <omp.h>

namespace map_conversion {
class Pointcloud2dProcess {
  private:
      struct Pose6D {
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
      };
  public:
    Pointcloud2dProcess(YAML::Node& config_node);
    Pointcloud2dProcess() = default;

    void find_Z_value(CloudData::CLOUD_PTR cloud_data); //计算点云中Z轴的最大和最小值
    void global_map_init(); //初始化全局地图
    //直通滤波器
    void PassThroughFilter(CloudData::CLOUD_PTR  pcd_cloud,CloudData& cloud_after_PassThrough, const bool &flag_in,double z_max, double z_min);
    // 3D点云杂点去除
    void outlier_removal(CloudData::CLOUD_PTR& cloudmap);
    // 先在局部地图上截断再拼接
    void PassThroughLocalMap(CloudData& cloud_after_PassThrough, const bool &flag_in, double z_max, double z_min);
    //将三维点云转换为三维栅格
    void Pointcloud_to_2d_grid(const CloudData& pcd_cloud, nav_msgs::OccupancyGrid& msg, double map_resolution);
    // localmap变换到globalmap
    CloudData::CLOUD_PTR local2global(const CloudData::CLOUD_PTR &cloudIn, const Pose6D &tf);

    double max_z;
    double min_z;
    std::string Global_map_file;
    std::string local_cloud_topic;
    std::string topic_frame_id;

    CloudData::CLOUD_PTR global_map_data;
    CloudData global_map_after_filter;
    CloudData global_map_no_filter;

    double grid_x = 0.1;
    double grid_y = 0.1;
    double grid_z = 0.1;

    double global_map_resolution = 1;
    double current_map_resolution = 0.1;

    double thre_radius = 0.5; 

    double global_max_z_adjust;
    double global_min_z_adjust;
    double current_max_z_adjust;
    double current_min_z_adjust;

    double lidar_height;
    int outlier_removal_method;
    int mean_k;
    double mul_thresh;
    double search_radius;
    int min_neighbors;   
  private:
      std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
      pcl::VoxelGrid<CloudData::POINT> sor;
};
}

#endif