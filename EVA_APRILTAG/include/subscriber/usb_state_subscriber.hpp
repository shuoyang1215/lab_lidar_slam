//
// Created by garry on 2022/9/26.
//

#ifndef SRC_USB_STATE_SUBSCRIBER_H
#define SRC_USB_STATE_SUBSCRIBER_H


#include <mutex>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float32MultiArray.h"


namespace eva_april {
    class USBStateSubscriber {
    public:
        USBStateSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
        USBStateSubscriber() = default;
        void clear_state();
        int is_reset_all, is_reset_viw, is_DownCar, is_calib_rtk;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::mutex buff_mutex_;

        void msg_callback(const std_msgs::UInt8MultiArray::ConstPtr& msg_usb_state);
        int alg_reset_sub_state, heading_cal_req_sub_state, Updowncar_req_sub_state;
        int last_alg_reset_sub_state, last_heading_cal_req_sub_state, last_Updowncar_req_sub_state;
    };
}




#endif //SRC_USB_STATE_SUBSCRIBER_H
