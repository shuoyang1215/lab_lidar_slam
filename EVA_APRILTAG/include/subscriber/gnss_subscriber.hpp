
#ifndef GLOBAL_FUSION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define GLOBAL_FUSION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "sensor_data/gnss_data.hpp"

namespace eva_april {
    class GNSSSubscriber {
    public:
        GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, std::string rtk_init_path_input);

        GNSSSubscriber() = default;
        double last_gnss_height = 0.0;
        double last_gnss_time = 0.0;
        std::string rtk_init_path;

        std::deque<GNSSData> ParseData(std::deque<GNSSData> &deque_gnss_data);
        void StartSubscribe();
        void StopSubscribe();
        void Reset();


    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GNSSData> new_gnss_data_;

        std::mutex buff_mutex_;
        bool is_start_sub;
    };
}
#endif