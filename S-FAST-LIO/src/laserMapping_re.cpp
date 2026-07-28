#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#include "IMU_Processing.hpp"

#include <pcl/registration/icp.h>
#include <ros/package.h>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
int add_point_size = 0, kdtree_delete_counter = 0;
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true, map_filter_enable = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0, map_filter_range = 10.0;

mutex mtx_buffer;
condition_variable sig_buffer;
ros::Publisher pubLaserCloudMap, pubFilteredCloudMap;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int scan_count = 0, publish_count = 0;
int feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_index = 0;

bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited, has_init_pose = false;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
int init_method = 1;

vector<BoxPointType> cub_needrm;
vector<PointVector> Nearest_Points;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
vector<double> init_pos(3, 0.0);
vector<double> init_rot{0, 0, 0, 1};
deque<double> time_buffer;
deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI()); //畸变纠正后降采样的单帧点云，W系
PointCloudXYZI::Ptr cloud(new PointCloudXYZI()), cloud_filtered(new PointCloudXYZI()), currentcloud_filtered(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;
std::string save_path;
ofstream fout_rtk, fout_lio;

KD_TREE<PointType> ikdtree;

V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;

esekfom::esekf kf;

state_ikfom state_point;
Eigen::Vector3d pos_lid; //估计的W系下的位置
Eigen::Matrix4f coarse_init_pose = Eigen::Matrix4f::Identity(); // rviz发布的2D pose estimate得到的位姿

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());

// 高频位姿
struct high_freq_state{
    M3D R;
    V3D p;
    V3D v;
    sensor_msgs::Imu imu_msg;
    high_freq_state()
    {
        R = M3D::Identity();
        p = V3D(0, 0, 0);
        v = V3D(0, 0, 0);
    }
};

double cur_lidar_time = 0;
// 储存的上一次融合时刻的 lidar 状态
high_freq_state last_lidar_high_freq_state;
// 高频位姿对应的队列
std::deque<high_freq_state> high_freq_state_queue;
// 雷达发布的位姿数量
int lidar_pose_count = 0, last_lidar_pose_count = 0;
// 高频位姿发布-odometry
ros::Publisher pubOdomHighFreq;

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void pose_cbk(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    coarse_init_pose.block<3, 1>(0, 3) = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Quaternionf q_(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    coarse_init_pose.block<3, 3>(0, 0) = q_.toRotationMatrix();

    has_init_pose = true;
    cout << "Get RVIZ 2D pose!!!" << endl;
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp =
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    // ************************** 利用IMU测量值实现高频位姿发布 ****************************
    // 高频位姿积累buffer
    high_freq_state cur_imu_state;
    cur_imu_state.imu_msg = *msg;

    // 累计imu buffer
    high_freq_state_queue.push_back(cur_imu_state);

    // 发生了雷达位姿更新，矫正 lidar buffer里面的所有imu高频位姿, 同时进行 RTK 全局坐标系下位姿的更新。
    if(lidar_pose_count > last_lidar_pose_count){
        high_freq_state delta_lidar_state;

        //清空buffer里面所有老的imu帧
        while(high_freq_state_queue.size() > 1 && high_freq_state_queue.front().imu_msg.header.stamp.toSec() < cur_lidar_time){
            high_freq_state_queue.pop_front();
        }
        for(int i = 0; i < high_freq_state_queue.size(); i++){
            if(i == 0){
                V3D cur_w = V3D(high_freq_state_queue[i].imu_msg.angular_velocity.x, 
                                high_freq_state_queue[i].imu_msg.angular_velocity.y,
                                high_freq_state_queue[i].imu_msg.angular_velocity.z)
                                - state_point.bg;
                V3D cur_a = V3D(high_freq_state_queue[i].imu_msg.linear_acceleration.x * G_m_s2 ,
                                high_freq_state_queue[i].imu_msg.linear_acceleration.y * G_m_s2 ,
                                high_freq_state_queue[i].imu_msg.linear_acceleration.z * G_m_s2 )
                                - state_point.ba;
                // 转换到世界坐标系下
                cur_a = state_point.rot.matrix() * cur_a;
                // 叠加重力
                for(int j = 0; j < 3; j++){
                    cur_a[j] += state_point.grav[j];
                }
                // 积分时间
                double delta_t = high_freq_state_queue[i].imu_msg.header.stamp.toSec() - cur_lidar_time;
                // high_freq_state_queue[i].R = state_point.rot.matrix() * SO3::exp(cur_w * delta_t);
                high_freq_state_queue[i].R = (state_point.rot * Sophus::SO3d::exp(cur_w * delta_t)).matrix();
                high_freq_state_queue[i].p = state_point.pos + state_point.vel * delta_t;
                high_freq_state_queue[i].v = state_point.vel + cur_a * delta_t;
            }

            else{
                V3D cur_w = 0.5 * (V3D(high_freq_state_queue[i].imu_msg.angular_velocity.x, 
                                high_freq_state_queue[i].imu_msg.angular_velocity.y,
                                high_freq_state_queue[i].imu_msg.angular_velocity.z) + 
                            V3D(high_freq_state_queue[i-1].imu_msg.angular_velocity.x, 
                                high_freq_state_queue[i-1].imu_msg.angular_velocity.y,
                                high_freq_state_queue[i-1].imu_msg.angular_velocity.z)) - state_point.bg;
                                
                V3D cur_a = 0.5 * (V3D(high_freq_state_queue[i].imu_msg.linear_acceleration.x * G_m_s2,
                                high_freq_state_queue[i].imu_msg.linear_acceleration.y * G_m_s2,
                                high_freq_state_queue[i].imu_msg.linear_acceleration.z * G_m_s2) + 
                            V3D(high_freq_state_queue[i-1].imu_msg.linear_acceleration.x * G_m_s2,
                                high_freq_state_queue[i-1].imu_msg.linear_acceleration.y * G_m_s2,
                                high_freq_state_queue[i-1].imu_msg.linear_acceleration.z * G_m_s2)) - state_point.ba;
                // 转换到世界坐标系下
                cur_a = high_freq_state_queue[i-1].R * cur_a;
                // 叠加重力
                for(int j = 0; j < 3; j++){
                    cur_a[j] += state_point.grav[j];
                }
                // 积分时间
                double delta_t = high_freq_state_queue[i].imu_msg.header.stamp.toSec() - high_freq_state_queue[i-1].imu_msg.header.stamp.toSec();
                // high_freq_state_queue[i].R = high_freq_state_queue[i-1].R * SO3::exp(cur_w * delta_t);
                high_freq_state_queue[i].R = (Sophus::SO3d(high_freq_state_queue[i-1].R) * Sophus::SO3d::exp(cur_w * delta_t)).matrix();
                high_freq_state_queue[i].p = high_freq_state_queue[i-1].p + high_freq_state_queue[i-1].v * delta_t;
                high_freq_state_queue[i].v = high_freq_state_queue[i-1].v + cur_a * delta_t;
            }
        }

        // 计算两次lidar融合帧位置的高频位置之差
        delta_lidar_state.R = last_lidar_high_freq_state.R.inverse() * high_freq_state_queue.back().R;
        delta_lidar_state.p = last_lidar_high_freq_state.R.inverse() * (high_freq_state_queue.back().p - last_lidar_high_freq_state.p);
        delta_lidar_state.v = last_lidar_high_freq_state.R.inverse() * (high_freq_state_queue.back().v - last_lidar_high_freq_state.v);

        // last 状态更新, flag_rtk 状态更新
        last_lidar_high_freq_state = high_freq_state_queue.back();
        last_lidar_pose_count = lidar_pose_count;
    }

    // 正常imu积分模式
    else if(lidar_pose_count == last_lidar_pose_count && lidar_pose_count >= 1 
                && high_freq_state_queue.size() >= 2){ //  && high_freq_rtk_queue.size() >= 2
        int imu_que_size = high_freq_state_queue.size();

        V3D cur_w = 0.5 * (V3D(high_freq_state_queue.back().imu_msg.angular_velocity.x, 
                        high_freq_state_queue.back().imu_msg.angular_velocity.y,
                        high_freq_state_queue.back().imu_msg.angular_velocity.z) + 
                    V3D(high_freq_state_queue[imu_que_size-2].imu_msg.angular_velocity.x, 
                        high_freq_state_queue[imu_que_size-2].imu_msg.angular_velocity.y,
                        high_freq_state_queue[imu_que_size-2].imu_msg.angular_velocity.z)) - state_point.bg;
                        
        V3D cur_a = 0.5 * (V3D(high_freq_state_queue.back().imu_msg.linear_acceleration.x * G_m_s2,
                        high_freq_state_queue.back().imu_msg.linear_acceleration.y * G_m_s2,
                        high_freq_state_queue.back().imu_msg.linear_acceleration.z * G_m_s2) + 
                    V3D(high_freq_state_queue[imu_que_size-2].imu_msg.linear_acceleration.x * G_m_s2,
                        high_freq_state_queue[imu_que_size-2].imu_msg.linear_acceleration.y * G_m_s2,
                        high_freq_state_queue[imu_que_size-2].imu_msg.linear_acceleration.z * G_m_s2)) - state_point.ba;
        // 转换到世界坐标系下
        cur_a = high_freq_state_queue[imu_que_size-2].R * cur_a;
        // 叠加重力
        for(int j = 0; j < 3; j++){
            cur_a[j] += state_point.grav[j];
        }
        // 积分lidar坐标系下的局部位姿
        double delta_t = high_freq_state_queue.back().imu_msg.header.stamp.toSec() - high_freq_state_queue[imu_que_size-2].imu_msg.header.stamp.toSec();
        // high_freq_state_queue.back().R = high_freq_state_queue[imu_que_size-2].R * SO3::exp(cur_w * delta_t);
        high_freq_state_queue.back().R = (Sophus::SO3d(high_freq_state_queue[imu_que_size-2].R) * Sophus::SO3d::exp(cur_w * delta_t)).matrix();
        high_freq_state_queue.back().p = high_freq_state_queue[imu_que_size-2].p + high_freq_state_queue[imu_que_size-2].v * delta_t;
        high_freq_state_queue.back().v = high_freq_state_queue[imu_que_size-2].v + cur_a * delta_t;
    }

    // 发布imu数据
    nav_msgs::Odometry high_odom;
    high_odom.header.frame_id = "camera_init";
    high_odom.child_frame_id = "body";
    high_odom.header.stamp = high_freq_state_queue.back().imu_msg.header.stamp;
    // ***************** 发布导航需要的车体中心的位置 *****************
    // T^I0_B = T^I0_I * T^I_B 右乘一个外参
    // 以IMU坐标系为基准，机器人中心的位置为(-224, -23.29, 0)mm
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = high_freq_state_queue.back().R;
    pose.block<3, 1>(0, 3) = high_freq_state_queue.back().p;
    // 对T^I0_I进行变换
    Eigen::Matrix4d B_2_I = Eigen::Matrix4d::Identity();
    B_2_I(0 ,3) = -0.224;
    B_2_I(1 ,3) = -0.02329;
    pose = pose * B_2_I;
    // ***************************************************************
    high_odom.pose.pose.position.x = pose(0, 3);
    high_odom.pose.pose.position.y = pose(1, 3);
    high_odom.pose.pose.position.z = pose(2, 3);
    Eigen::Quaterniond HighQuat(pose.block<3, 3>(0, 0));
    high_odom.pose.pose.orientation.x = HighQuat.x();
    high_odom.pose.pose.orientation.y = HighQuat.y();
    high_odom.pose.pose.orientation.z = HighQuat.z();
    high_odom.pose.pose.orientation.w = HighQuat.w();
    // high_odom.pose.pose.position.x = high_freq_state_queue.back().p(0);
    // high_odom.pose.pose.position.y = high_freq_state_queue.back().p(1);
    // high_odom.pose.pose.position.z = high_freq_state_queue.back().p(2);
    // Eigen::Quaterniond HighQuat(high_freq_state_queue.back().R);
    // high_odom.pose.pose.orientation.x = HighQuat.x();
    // high_odom.pose.pose.orientation.y = HighQuat.y();
    // high_odom.pose.pose.orientation.z = HighQuat.z();
    // high_odom.pose.pose.orientation.w = HighQuat.w();
    pubOdomHighFreq.publish(high_odom);

    // // 发布 path 路径数据
    // if(lidar_pose_count > 0){
    //     geometry_msgs::PoseStamped cur_pose_path_rtk;
    //     cur_pose_path_rtk.header.stamp = high_freq_rtk_queue.back().imu_msg.header.stamp;
    //     cur_pose_path_rtk.header.frame_id = "camera_init";
    //     cur_pose_path_rtk.pose.position.x = high_freq_rtk_queue.back().p(0);
    //     cur_pose_path_rtk.pose.position.y = high_freq_rtk_queue.back().p(1);
    //     cur_pose_path_rtk.pose.position.z = high_freq_rtk_queue.back().p(2);
    //     cur_pose_path_rtk.pose.orientation.x = HighQuatRtk.x();
    //     cur_pose_path_rtk.pose.orientation.y = HighQuatRtk.y();
    //     cur_pose_path_rtk.pose.orientation.z = HighQuatRtk.z();
    //     cur_pose_path_rtk.pose.orientation.w = HighQuatRtk.w();
    //     path_high_freq_out.poses.push_back(cur_pose_path_rtk);
    //     path_high_freq_out.header.stamp    = ros::Time::now();
    //     path_high_freq_out.header.frame_id ="camera_init";

    //     // cur_pose_path_rtk.header.stamp = high_freq_rtk_queue.back().imu_msg.header.stamp;
    //     // cur_pose_path_rtk.header.frame_id = "camera_init";
    //     // cur_pose_path_rtk.pose.position.x = T_cur_state_rtk.block<3, 1>(0, 3)(0);
    //     // cur_pose_path_rtk.pose.position.y = T_cur_state_rtk.block<3, 1>(0, 3)(1);
    //     // cur_pose_path_rtk.pose.position.z = T_cur_state_rtk.block<3, 1>(0, 3)(2);
    //     // cur_pose_path_rtk.pose.orientation.x = HighQuatOrin.x();
    //     // cur_pose_path_rtk.pose.orientation.y = HighQuatOrin.y();
    //     // cur_pose_path_rtk.pose.orientation.z = HighQuatOrin.z();
    //     // cur_pose_path_rtk.pose.orientation.w = HighQuatOrin.w();
    //     // path_high_freq_rtk.poses.push_back(cur_pose_path_rtk);
    //     // path_high_freq_rtk.header.stamp    = ros::Time::now();
    //     // path_high_freq_rtk.header.frame_id ="camera_init";

    //     // pub_high_freq_path_rtk.publish(path_high_freq_rtk);
    //     pub_high_freq_path_out.publish(path_high_freq_out);
    // }

    // *********************************************************************************


    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int scan_num = 0;
//把当前要处理的LIDAR和IMU数据打包到meas
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 5) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();                                                                                                                                                    
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

BoxPointType LocalMap_Points;      // ikd-tree地图立方体的2个角点
bool Localmap_Initialized = false; // 局部地图是否初始化
void lasermap_fov_segment()
{
    cub_needrm.clear(); // 清空需要移除的区域
    kdtree_delete_counter = 0;

    V3D pos_LiD = pos_lid; // W系下位置
    //初始化局部地图范围，以pos_LiD为中心,长宽高均为cube_len
    if (!Localmap_Initialized)
    {
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }

    //各个方向上pos_LiD与局部地图边界的距离
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        // 与某个方向上的边界距离（1.5*300m）太小，标记需要移除need_move(FAST-LIO2论文Fig.3)
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return; //如果不需要，直接返回，不更改局部地图

    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    //需要移动的距离
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
        else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);

    if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); //删除指定范围内的点
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I.matrix() * p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

//根据最新估计位姿  增量添加点云到map
void init_ikdtree()
{
    //加载读取点云数据到cloud中
    string all_points_dir(string(ros::package::getPath("livox_mapping") + "/PCD/") + "scans_lc.pcd");
    if (pcl::io::loadPCDFile<PointType>(all_points_dir, *cloud) == -1)
    {
        PCL_ERROR("Read file fail!\n");
    }

    ikdtree.set_downsample_param(0.1);
    ikdtree.Build(cloud->points);
    std::cout << "---- ikdtree size: " << ikdtree.size() << std::endl;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher &pubLaserCloudFull_)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i],
                             &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull_.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(
            new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&feats_undistort->points[i],
                             &laserCloudWorld->points[i]);
        }

        static int scan_wait_num = 0;
        scan_wait_num++;

        if (scan_wait_num % 4 == 0)
            *pcl_wait_save += *laserCloudWorld;
    }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_map(const ros::Publisher &pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template <typename T>
void set_posestamp(T &out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    auto q_ = Eigen::Quaterniond(state_point.rot.matrix());
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped, const ros::Publisher &pubOdomAftMapped_robot)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);

    auto P = kf.get_P();
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                    odomAftMapped.pose.pose.position.y,
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));

    fout_lio << odomAftMapped.header.stamp << " "
      << odomAftMapped.pose.pose.position.x << " "
      << odomAftMapped.pose.pose.position.y << " "
      << odomAftMapped.pose.pose.position.z << " "
      << q.x() << " "
      << q.y() << " "
      << q.z() << " "
      << q.w() << std::endl;

    // 发布导航需要的车体中心的位置
    // T^I0_B = T^I0_I * T^I_B 右乘一个外参
    // 以IMU坐标系为基准，机器人中心的位置为(-224, -23.29, 0)mm
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = state_point.rot.matrix();
    pose.block<3, 1>(0, 3) = state_point.pos;
    // 对T^I0_I进行变换
    Eigen::Matrix4d B_2_I = Eigen::Matrix4d::Identity();
    B_2_I(0 ,3) = -0.224;
    B_2_I(1 ,3) = -0.02329;
    pose = pose * B_2_I;
    // 修改odometry的位姿
    set_posestamp(odomAftMapped.pose);

    odomAftMapped.pose.pose.position.x = pose(0, 3);
    odomAftMapped.pose.pose.position.y = pose(1, 3);
    odomAftMapped.pose.pose.position.z = pose(2, 3);

    auto q_ = Eigen::Quaterniond(pose.block<3, 3>(0, 0));
    odomAftMapped.pose.pose.orientation.x = q_.coeffs()[0];
    odomAftMapped.pose.pose.orientation.y = q_.coeffs()[1];
    odomAftMapped.pose.pose.orientation.z = q_.coeffs()[2];
    odomAftMapped.pose.pose.orientation.w = q_.coeffs()[3];

    pubOdomAftMapped_robot.publish(odomAftMapped);


}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);            // 是否发布当前正在扫描的点云的topic
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);          // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true); // 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic，需要该变量和上一个变量同时为true才发布
    nh.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);                   // 卡尔曼滤波的最大迭代次数
    nh.param<string>("map_file_path", map_file_path, "");                    // 地图保存路径
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");         // 雷达点云topic名称
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");           // IMU的topic名称
    nh.param<bool>("common/time_sync_en", time_sync_en, false);              // 是否需要时间同步，只有当外部未进行时间同步时设为true
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5); // VoxelGrid降采样时的体素大小
    nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cube_len, 200);    // 地图的局部区域的长度（FastLio2论文中有解释）
    nh.param<float>("mapping/det_range", DET_RANGE, 300.f); // 激光雷达的最大探测范围
    nh.param<double>("mapping/fov_degree", fov_deg, 180);
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);               // IMU陀螺仪的协方差
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);               // IMU加速度计的协方差
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);        // IMU陀螺仪偏置的协方差
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);        // IMU加速度计偏置的协方差
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);        // 最小距离阈值，即过滤掉0～blind范围内的点云
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA); // 激光雷达的类型
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);       // 激光雷达扫描的线数（livox avia为6线）
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);           // 采样间隔，即每隔point_filter_num个点取1个点
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false); // 是否提取特征点（FAST_LIO2默认不进行特征点提取）
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false); // 是否将点云地图保存到PCD文件
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); // 雷达相对于IMU的外参R

    nh.param<int>("mapping/init_method", init_method, 1); // 初始化方法 1 for prior, 2 for ICP
    nh.param<vector<double>>("mapping/init_pos", init_pos, vector<double>()); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
    nh.param<vector<double>>("mapping/init_rot", init_rot, vector<double>()); // 雷达相对于IMU的外参R
    nh.param<std::string>("save_path", save_path, "/");

    nh.param<bool>("map_filter_enable", map_filter_enable, false); // 是否对ICP初始化匹配时的点云进行直通滤波
    nh.param<double>("map_filter_range", map_filter_range, 10.0);        // 点云地图直通滤波范围

    cout << "Lidar_type: " << p_pre->lidar_type << endl;
    // 初始化path的header（包括时间戳和帧id），path用于保存odemetry的路径
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    // 从绝对路径改为工程绝对路径自动获得
    fout_lio.open(ros::package::getPath("sfast_lio") + "/log/lio_livox_slam_re.txt",ios::out);

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_pose = nh.subscribe("/initialpose", 100, pose_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    // pubFilteredCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Filtered_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
    ros::Publisher pubOdomAftMapped_robot = nh.advertise<nav_msgs::Odometry>("/Odometry_robot", 100000);
    ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("/path", 100000);
    // 高频位姿
    pubOdomHighFreq = nh.advertise<nav_msgs::Odometry>("/HighFreqOdometry_robot", 100000);

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

    shared_ptr<ImuProcess> p_imu1(new ImuProcess());
    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
    p_imu1->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
                      V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    double t_init1 = omp_get_wtime();
    init_ikdtree(); //读取点云文件 初始化ikdtree
    std::cout << std::endl << "  Map loading time(ms):  " << (omp_get_wtime() - t_init1) * 1000 << std::endl
                << std::endl;
    fout_lio << "  Map loading time(ms):  " << (omp_get_wtime() - t_init1) * 1000 << std::endl
                << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> icp;

    // 采用先验对初始状态值直接更改
    if (init_method == 1 ){
        cout << "Prior initialization!!!" << endl;
        state_point = kf.get_x();
        state_point.pos = Eigen::Vector3d(init_pos[0], init_pos[1], init_pos[2]);
        Eigen::Quaterniond q(init_rot[3], init_rot[0], init_rot[1], init_rot[2]);
        Sophus::SO3d SO3_q(q);
        state_point.rot = SO3_q;
        kf.change_x(state_point);        
    }
    else if (init_method == 2 || init_method == 3){
        if (init_method == 2) {
            cout << "ICP initialization!!!" << endl;
            // 采用yaml中的参数作为ICP初值
            coarse_init_pose.block<3, 1>(0, 3) = Eigen::Vector3f(init_pos[0], init_pos[1], init_pos[2]);
            Eigen::Quaternionf q_(init_rot[3], init_rot[0], init_rot[1], init_rot[2]);
            coarse_init_pose.block<3, 3>(0, 0) = q_.toRotationMatrix();
        }
        else cout << "RVIZ assisted initialization!!!" << endl;
        // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);
    }
    else ROS_ERROR("No matched initialization method!!!");

    int total_frame = 0;
    double total_time = 0;

    while (ros::ok())
    {
        if (flg_exit)
            break;
        ros::spinOnce();

        if (flg_first_scan && init_method == 3){
            // 从rviz交互中拿到初始位姿数据
            if (!has_init_pose){
                rate.sleep();
                continue;
            }
            // cout << "RVIZ initial pose: " << coarse_init_pose << endl;      
        }

        if (sync_packages(Measures)) //把一次的IMU和LIDAR数据打包到Measures
        {
            double t00 = omp_get_wtime();

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu1->first_lidar_time = first_lidar_time;

                sensor_msgs::PointCloud2 laserCloudMap;
                pcl::toROSMsg(*cloud, laserCloudMap);
                laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
                laserCloudMap.header.frame_id = "camera_init";
                pubLaserCloudMap.publish(laserCloudMap);   

                // 如果采用非先验之外的方法进行初始化，需要在第一帧点云数据拿到时进行操作
                if (init_method == 2 || init_method == 3){
                    t_init1 = omp_get_wtime();
                    // icp.setInputSource(Measures.lidar);                  
                    if (map_filter_enable){
                        // 对先验地图进行一定区域裁剪，使得初始化匹配速度更快
                        // 创建滤波器对象
                        // cout << "original point cloud size: " << cloud->points.size() << endl;
                        pcl::PassThrough<pcl::PointXYZINormal> pass;//创建滤波器对象
                        pass.setInputCloud(cloud);			//设置待滤波的点云
                        pass.setFilterFieldName("x");		//设置在x轴方向上进行滤波
                        pass.setFilterLimits(-map_filter_range + coarse_init_pose(0, 3), map_filter_range + coarse_init_pose(0, 3));		//设置滤波范围,在范围之外的点会被剪除
                        pass.filter(*cloud_filtered);		
                        // 对当前点云也进行滤波，否则可能匹配出问题      
                        pass.setInputCloud(Measures.lidar);
                        pass.setFilterFieldName("x");		//设置在x轴方向上进行滤波
                        pass.setFilterLimits(-map_filter_range, map_filter_range);		//设置滤波范围,在范围之外的点会被剪除
                        pass.filter(*currentcloud_filtered);		
                        // cout << "x filtered point cloud size: " << cloud_filtered->points.size() << endl;   

                        pass.setInputCloud(cloud_filtered);			//设置待滤波的点云
                        pass.setFilterFieldName("y");		//设置在x轴方向上进行滤波
                        pass.setFilterLimits(-map_filter_range + coarse_init_pose(1, 3), map_filter_range + coarse_init_pose(1, 3));		//设置滤波范围,在范围之外的点会被剪除
                        pass.filter(*cloud_filtered);		
                        // 当前点云也进行滤波，否则可能匹配出问题   
                        pass.setInputCloud(currentcloud_filtered);
                        pass.setFilterFieldName("y");		//设置在x轴方向上进行滤波
                        pass.setFilterLimits(-map_filter_range, map_filter_range);	
                        pass.filter(*currentcloud_filtered);		
                        cout << "xy filtered map size: " << cloud_filtered->points.size() << endl;
                        cout << "xy filtered point cloud size: " << currentcloud_filtered->points.size() << endl;

                        // // debug: 发布直通滤波后的地图点云 看是否是滤波出了问题
                        // sensor_msgs::PointCloud2 CloudMap;
                        // pcl::toROSMsg(*cloud_filtered, CloudMap);
                        // CloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
                        // CloudMap.header.frame_id = "camera_init";
                        // pubFilteredCloudMap.publish(CloudMap);   

                        // 对原始点云也进行直通滤波器 防止室外大场景下初始化漂移
                        icp.setInputSource(currentcloud_filtered); 
                        icp.setInputTarget(cloud_filtered);     
                    }
                    else{
                        icp.setInputSource(Measures.lidar); 
                        icp.setInputTarget(cloud);  
                    }     
                    pcl::PointCloud<pcl::PointXYZINormal>::Ptr trans_source (new pcl::PointCloud<pcl::PointXYZINormal>());
                    icp.align(*trans_source, coarse_init_pose); //                     

                    cout << "ICP has converged? " << (icp.hasConverged() == true) << endl;
                    cout << "ICP FitnessScore: " << icp.getFitnessScore() << endl;
                    Eigen::Matrix4f init_T = icp.getFinalTransformation();
                    cout << "ICP result: " << init_T << endl;
                    // 修改状态变量中的位姿
                    state_point = kf.get_x();
                    state_point.pos = Eigen::Vector3d(init_T(0, 3), init_T(1, 3), init_T(2, 3));
                    Eigen::Quaterniond q(init_T.block<3, 3>(0, 0).cast<double>());
                    Sophus::SO3d SO3_q(q);
                    state_point.rot = SO3_q;
                    kf.change_x(state_point);  
                    std::cout << std::endl << "  ICP initialization time(ms):  " << (omp_get_wtime() - t_init1) * 1000 << std::endl
                                << std::endl;
                    fout_lio << "  ICP initialization time(ms):  " << (omp_get_wtime() - t_init1) * 1000 << std::endl
                                << std::endl;
                }


                flg_first_scan = false;
                continue;
            }

            p_imu1->Process(Measures, kf, feats_undistort);

            //如果feats_undistort为空 ROS_WARN
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            lasermap_fov_segment(); //更新localmap边界，然后降采样当前帧点云

            //点云下采样
            std::cout << "feats_size: " << feats_undistort->points.size() << std::endl;
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();

            std::cout << "feats_down_size :" << feats_down_size << std::endl;
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            if (0) // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
                std::cout << "ikdtree size: " << featsFromMap->points.size() << std::endl;
            }

            /*** iterated state estimation ***/
            Nearest_Points.resize(feats_down_size); //存储近邻点的vector
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en);

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, pubOdomAftMapped_robot);

            // 雷达状态更新，记录状态时刻，标志高频位置需要在状态更新后重积分
            lidar_pose_count ++;
            cur_lidar_time = Measures.lidar_end_time;

            /*** add the feature points to map kdtree ***/
            feats_down_world->resize(feats_down_size);
            // map_incremental();

            /******* Publish points *******/
            if (path_en)
                publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body);

            double t11 = omp_get_wtime();
            total_frame ++;
            total_time += (t11 - t00) * 1000;
            std::cout << "feats_down_size: " << feats_down_size <<  "  avg loc time(ms):  " << total_time / total_frame <<  "  cur loc time(ms):  " << (t11 - t00) * 1000 << std::endl
                      << std::endl;
            fout_lio << "feats_down_size: " << feats_down_size << "  avg loc time(ms):  " << total_time / total_frame << "  cur loc time(ms):  " << (t11 - t00) * 1000 << std::endl
                      << std::endl;
        }

        rate.sleep();
    }

    return 0;
}
