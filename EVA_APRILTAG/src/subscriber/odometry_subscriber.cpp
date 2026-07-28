
#include "subscriber/odometry_subscriber.hpp"
#include "glog/logging.h"
#include "tools/print_info.hpp"

namespace eva_april {
    OdometrySubscriber::OdometrySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
            : nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
        is_start_sub = false;
    }

    void OdometrySubscriber::StartSubscribe() {
        is_start_sub = true;
    }

    void OdometrySubscriber::StopSubscribe() {
        is_start_sub = false;
    }

    void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr &odom_msg_ptr) {
        buff_mutex_.lock();
        PoseData pose_data;
        pose_data.time = odom_msg_ptr->header.stamp.toSec();

        //set the position
        pose_data.pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
        pose_data.pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
        pose_data.pose(2, 3) = odom_msg_ptr->pose.pose.position.z;

        Eigen::Quaterniond q;
        q.x() = odom_msg_ptr->pose.pose.orientation.x;
        q.y() = odom_msg_ptr->pose.pose.orientation.y;
        q.z() = odom_msg_ptr->pose.pose.orientation.z;
        q.w() = odom_msg_ptr->pose.pose.orientation.w;
        Eigen::Vector3d eulerAngle_odo =q.matrix().eulerAngles(2,1,0);
        pose_data.pose.block<3, 3>(0, 0) = q.matrix();

        // ground constraint
//        pose_data.ground_plane_h = odom_msg_ptr->twist.twist.linear.z;
//        Eigen::Vector3f eulerAngle(eulerAngle_odo[0],odom_msg_ptr->twist.twist.linear.y,odom_msg_ptr->twist.twist.linear.x);
//        Eigen::AngleAxisf rollAngle(Eigen::AngleAxisf(eulerAngle(2), Eigen::Vector3f::UnitX()));
//        Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisf(eulerAngle(1), Eigen::Vector3f::UnitY()));
//        Eigen::AngleAxisf yawAngle(Eigen::AngleAxisf(eulerAngle(0), Eigen::Vector3f::UnitZ()));
//        Eigen::Matrix3f rotation_matrix;
//        rotation_matrix = yawAngle * pitchAngle * rollAngle;
//        pose_data.ground_pose.setIdentity();
//        pose_data.ground_pose.block<3, 3>(0, 0) = rotation_matrix;
//        pose_data.ground_pose(0, 3) = odom_msg_ptr->pose.pose.position.x;
//        pose_data.ground_pose(1, 3) = odom_msg_ptr->pose.pose.position.y;
//        pose_data.ground_pose(2, 3) = odom_msg_ptr->twist.twist.linear.z;

        if(odom_msg_ptr->twist.twist.angular.x > -0.1){
            pose_data.is_reliable = true; // 判断当前pose是否可信赖
        }
        else{
            pose_data.is_reliable = false; // 判断当前pose是否可信赖
        }
        new_pose_data_.push_back(pose_data);
        if(!is_start_sub){
            new_pose_data_.pop_front();
        }
        buff_mutex_.unlock();
    }

    std::deque<PoseData> OdometrySubscriber::ParseData(std::deque<PoseData> &pose_data_buff) {
        buff_mutex_.lock();
        std::deque<PoseData> copy_data;
        if (new_pose_data_.size() > 0) {
            pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
            copy_data = new_pose_data_;
            new_pose_data_.clear();
        }
        buff_mutex_.unlock();
        return copy_data;
    }

    void OdometrySubscriber::Reset(){
        buff_mutex_.lock();
        new_pose_data_.clear();
        buff_mutex_.unlock();

    }
}