/*
 * @Description: 订阅cam数据
 */
#ifndef GLOBAL_FUSION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define GLOBAL_FUSION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace eva_april {
    class CamSubscriber {
    public:
        CamSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);

        CamSubscriber() = default;

        void ParseData(bool init, bool init_lio, bool init_viw, bool stop_flag);
        void StartSubscribe();
        void StopSubscribe();

    private:
        void msg_callback(const sensor_msgs::ImageConstPtr &cam_msg_ptr);

        cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);
    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<sensor_msgs::ImageConstPtr> new_cam_data_;

        std::mutex buff_mutex_;
        bool is_start_sub;
    };
}
#endif