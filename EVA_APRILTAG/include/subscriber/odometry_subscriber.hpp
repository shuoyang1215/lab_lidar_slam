
#ifndef GLOBAL_FUSION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define GLOBAL_FUSION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "sensor_data/pose_data.hpp"

namespace eva_april {
    class OdometrySubscriber {
    public:
        OdometrySubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);

        OdometrySubscriber() = default;

        std::deque<PoseData> ParseData(std::deque<PoseData> &deque_pose_data);
        void StartSubscribe();
        void StopSubscribe();
        void Reset();

    private:
        void msg_callback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<PoseData> new_pose_data_;

        std::mutex buff_mutex_;
        bool is_start_sub;
    };
}
#endif