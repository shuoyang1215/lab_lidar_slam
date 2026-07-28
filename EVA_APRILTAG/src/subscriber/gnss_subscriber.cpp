
#include "subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

int i_test = 0;

namespace eva_april {
    GNSSSubscriber::GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size, std::string rtk_init_path_input)
            : nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
        last_gnss_height = 0.0;
        last_gnss_time = 0.0;
        is_start_sub = false;
        rtk_init_path = rtk_init_path_input;
    }

    void GNSSSubscriber::StartSubscribe() {
        is_start_sub = true;
    }

    void GNSSSubscriber::StopSubscribe() {
        is_start_sub = false;
    }

    void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr) {
        buff_mutex_.lock();
        GNSSData gnss_data;

        if (nav_sat_fix_ptr->status.status != 0)
        {
            if (abs(last_gnss_height - nav_sat_fix_ptr->altitude) > 1e-10 ){ // 新来的RTK值是新的, RTK 信号坐标有变化
                gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
                gnss_data.latitude = nav_sat_fix_ptr->latitude;
                gnss_data.longitude = nav_sat_fix_ptr->longitude;
                gnss_data.altitude = nav_sat_fix_ptr->altitude;
                gnss_data.status = nav_sat_fix_ptr->status.status;
                gnss_data.service = nav_sat_fix_ptr->status.service;
                gnss_data.pos_cov = nav_sat_fix_ptr->position_covariance;

                //  for test 手动生成 service 数据
//                ++ i_test;
//
//                gnss_data.service = i_test / 200;



                if(gnss_data.pos_cov[0] + gnss_data.pos_cov[4] > 0.0025){ // RTK 噪声过大，认为是不可信的 fixme:是否存在中间的临界状态？
                    gnss_data.status = 2;
                }

                else if(gnss_data.status == 4){  // 当前RTK信号status以及协方差都是绝对可信的

                    if(!gnss_data.IsInit()){   // 当前仍然没有选择参考点, 并且处于自动选择初始点模式
                        gnss_data.InitOriginPosition();   // 选择第一次接收到的RTK信息, 并且当前RTK可靠的点作为参考点
                    }

                    if(gnss_data.service != gnss_data.origin_service){ // RTK service 变化，进行文件读取，更新参考点
                        std::ifstream rtk_file; // fixme: 顺序是否为  纬度、经度、高度
                        rtk_file.open(rtk_init_path, std::ios::in);
                        double cur_rtk_init[3] = {0};
                        rtk_file >> cur_rtk_init[0] >> cur_rtk_init[1] >> cur_rtk_init[2];
                        rtk_file.close();

                        gnss_data.ForceInitOriginPosition(cur_rtk_init[1], cur_rtk_init[0], cur_rtk_init[2]);
                        gnss_data.UpdateService();
                    }
                }

                gnss_data.UpdateXYZ();   //  更新在当前参考点下的 ENU 坐标系坐标

                gnss_data.gnss_pose.setIdentity();
                gnss_data.gnss_pose(0, 3) = gnss_data.local_E;
                gnss_data.gnss_pose(1, 3) = gnss_data.local_N;
                gnss_data.gnss_pose(2, 3) = gnss_data.local_U;

                LOG_IF(INFO, Config::use_glog) << gnss_data.local_E << "  " << gnss_data.local_N << "  " << gnss_data.local_U << "  当前 RTK 转换后的xyz" << std::endl;

                new_gnss_data_.push_back(gnss_data);
                if(!is_start_sub){
                    new_gnss_data_.pop_front();
                }
                last_gnss_height = nav_sat_fix_ptr->altitude;
                last_gnss_time = nav_sat_fix_ptr->header.stamp.toSec();
            }
        }

        else{
            if (nav_sat_fix_ptr->header.stamp.toSec() - last_gnss_time >= 0.02){
                gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
                gnss_data.latitude = nav_sat_fix_ptr->latitude;
                gnss_data.longitude = nav_sat_fix_ptr->longitude;
                gnss_data.altitude = nav_sat_fix_ptr->altitude;
                gnss_data.status = nav_sat_fix_ptr->status.status;
                gnss_data.service = nav_sat_fix_ptr->status.service;
                gnss_data.pos_cov = nav_sat_fix_ptr->position_covariance;

                if(gnss_data.pos_cov[0] + gnss_data.pos_cov[4] > 0.0025 && gnss_data.status!=0){
                    gnss_data.status = 2;
                }

                gnss_data.UpdateXYZ();   //  更新在当前参考点下的 ENU 坐标系坐标

                gnss_data.gnss_pose.setIdentity();
                gnss_data.gnss_pose(0, 3) = gnss_data.local_E;
                gnss_data.gnss_pose(1, 3) = gnss_data.local_N;
                gnss_data.gnss_pose(2, 3) = gnss_data.local_U;

                new_gnss_data_.push_back(gnss_data);
                if(!is_start_sub){
                    new_gnss_data_.pop_front();
                }
                last_gnss_height = nav_sat_fix_ptr->altitude;
                last_gnss_time = nav_sat_fix_ptr->header.stamp.toSec();
            }
        }

        buff_mutex_.unlock();
    }

    std::deque<GNSSData> GNSSSubscriber::ParseData(std::deque<GNSSData> &gnss_data_buff) {
        buff_mutex_.lock();
        std::deque<GNSSData> copy_gnss_data;
        if (new_gnss_data_.size() > 0) {
            gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
            copy_gnss_data = new_gnss_data_;
            new_gnss_data_.clear();
        }
        buff_mutex_.unlock();
        return copy_gnss_data;
    }

    void GNSSSubscriber::Reset() {
        buff_mutex_.lock();
        new_gnss_data_.clear();
        GNSSData gnss_reset;
        gnss_reset.origin_longitude = 0.0;
        gnss_reset.origin_latitude = 0.0;
        gnss_reset.origin_altitude = 0.0;
        gnss_reset.origin_service = 0;
        gnss_reset.origin_position_inited = false;
        last_gnss_height = 0.0;
        last_gnss_time = 0.0;
        buff_mutex_.unlock();
    }
}