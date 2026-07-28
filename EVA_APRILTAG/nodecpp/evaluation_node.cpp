
#include <ros/ros.h>
#include "global_definition/global_definition.h"
#include "config.h"
#include "CommonFunc.h"
#include "evo_apriltag/evo_apriltag.h"
#include "evo_apriltag/cam_subscriber.hpp"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "subscriber/odometry_subscriber.hpp"
#include "subscriber/pose_source_subscriber.hpp"

using namespace eva_april;

std::mutex buff_mutex_, eva_mutex_;
double length = 0;
int cur_signal = 0, last_signal = 0;

void doubleCallback(const std_msgs::Float64::ConstPtr& msg)
{
    buff_mutex_.lock();
    length = msg->data;
    buff_mutex_.unlock();
}

void signal_callback(const std_msgs::Int8::ConstPtr &sig_msg) {
    eva_mutex_.lock();
    cur_signal = sig_msg->data;
    eva_mutex_.unlock();
}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "evaluation_node");
    ros::NodeHandle nh("~");

    Config::readConfig();

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = Config::save_dir;
    FLAGS_alsologtostderr = false;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;

    // Subscriber
    std::shared_ptr<PoseSourceSubscriber> high_freq_pose_odo_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> lio_pose_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> viw_pose_sub_ptr_;
    std::shared_ptr<CamSubscriber> cam_sub_ptr_;

    Eigen::Vector3d last_pos_lio(0,0,0), last_pos_viw(0,0,0);

    high_freq_pose_odo_sub_ptr_ = std::make_shared<PoseSourceSubscriber>(nh, Config::high_freq_pose_topic, 1000);
    lio_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, Config::lio_odo_topic, 1000);
    viw_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, Config::viw_topic, 1000);
    cam_sub_ptr_ = std::make_shared<CamSubscriber>(nh, Config::cam_topic, 1000);
    ros::Subscriber sub = nh.subscribe(Config::length_topic, 1000, doubleCallback);

    ros::Subscriber sub_signal = nh.subscribe(Config::start_trigger_topic, 2, signal_callback);

    std::deque<PoseData> lio_pose_buffer_;
    std::deque<PoseData> viw_pose_buffer_;
    std::vector<std::pair<double, Eigen::Matrix4d>> FusedPath;
    std::vector<std::pair<double, Eigen::Matrix4d>> LIOPath;
    std::vector<std::pair<double, Eigen::Matrix4d>> VIWPath;

    Eigen::Matrix4d lio_pose, viw_pose;
    lio_pose = Eigen::Matrix4d::Identity();
    viw_pose = Eigen::Matrix4d::Identity();

    double lio_length = 0, viw_length = 0;
    double first_lio_time = 0;
    bool start_tag_flag = false, stop_tag_flag = false, lio_start = false;
    init();

    YAML::Node config_node_ = Config::config_node_;

    int eva_times = config_node_["eva_times"].as<int>();  // 需要评估的次数
    std::vector<float> first_tag_time_list(eva_times);
    std::vector<float> last_tag_time_list(eva_times);
    for (int i = 0; i < eva_times; i++) {
        first_tag_time_list.at(i) = config_node_["first_tag_time_list"][i].as<int>();
        last_tag_time_list.at(i) = config_node_["last_tag_time_list"][i].as<int>();
    }
    // int first_tag_time = config_node_["first_tag_time"].as<int>();  // 第一次遇到april tag 的时间点
    // int last_tag_time = config_node_["last_tag_time"].as<int>();  // 第二次遇到april tag 的时间点

    high_freq_pose_odo_sub_ptr_->StartSubscribe();
    viw_pose_sub_ptr_->StartSubscribe();
    lio_pose_sub_ptr_->StartSubscribe();
    cam_sub_ptr_->StartSubscribe();

    ros::Rate rate(1000);
    for (int i = 0; i < eva_times; i++){
        std::cerr << "第" << i << "次评估开始..." << std::endl;
        // 清零设置
        init();
        LIOPath.clear();
        // first_lio_time = 0;
        start_tag_flag = false;
        stop_tag_flag = false;
        lio_pose_buffer_.clear();
        while (ros::ok()) {
            ros::spinOnce();

            if(LIOPath.size() > 1 && LIOPath.back().first - first_lio_time >= first_tag_time_list[i] && LIOPath.back().first - first_lio_time <= first_tag_time_list[i] + 0.5){
                start_tag_flag = true;
            }
            else{
                start_tag_flag = false;
            }

            if(LIOPath.size() > 1 && LIOPath.back().first - first_lio_time >= last_tag_time_list[i] && last_tag_time_list[i] > 0){
                stop_tag_flag = true;
            }

            cam_sub_ptr_->ParseData(start_tag_flag, start_tag_flag, start_tag_flag, stop_tag_flag);

            if (Config::eva_lio){
                lio_pose_sub_ptr_->ParseData(lio_pose_buffer_);
                while (!lio_pose_buffer_.empty() && !stop_tag_flag){
                    std::pair<double, Eigen::Matrix4d> item;
                    item.first = lio_pose_buffer_.front().time;
                    lio_pose = lio_pose_buffer_.front().pose.cast<double>();
                    item.second = lio_pose;
                    lio_pose_buffer_.pop_front();
                    // continue没有意义 上面的item也不会被保存
                    // if(stop_tag_flag)
                    //     continue;
                    Eigen::Vector3d pos = lio_pose.block<3,1>(0,3);
                    double cur_length = sqrt(
                            (pos[0] - last_pos_lio[0]) * (pos[0] - last_pos_lio[0]) +
                            (pos[1] - last_pos_lio[1]) * (pos[1] - last_pos_lio[1]));
                    // 多个apriltag板测精度时输出的长度为开始时刻到结束时刻的总长
                    lio_length += cur_length;
                    last_pos_lio = pos;
                    // 只需要获取一次包开始的时间
                    if(LIOPath.size() == 0 && i == 0){
                        first_lio_time = item.first;
                    }
                    LIOPath.push_back(item);
                }
            }
            if(stop_tag_flag)
                break;            
            rate.sleep();
        }
        // 评估一次的
        bool succc = EVO(lio_length, LIOPath, 1);
    }    

    // while (ros::ok()) {
    //     ros::spinOnce();

    //     // high_freq_pose_odo_sub_ptr_->StartSubscribe();
    //     // viw_pose_sub_ptr_->StartSubscribe();
    //     // lio_pose_sub_ptr_->StartSubscribe();
    //     // cam_sub_ptr_->StartSubscribe();



    //     if(LIOPath.size() > 1 && LIOPath.back().first - first_lio_time >= first_tag_time && LIOPath.back().first - first_lio_time <= first_tag_time + 0.5){
    //         start_tag_flag = true;
    //     }
    //     else{
    //         start_tag_flag = false;
    //     }

    //     if(LIOPath.size() > 1 && LIOPath.back().first - first_lio_time >= last_tag_time && last_tag_time > 0){
    //         stop_tag_flag = true;
    //     }

    //     cam_sub_ptr_->ParseData(start_tag_flag, start_tag_flag, start_tag_flag, stop_tag_flag);

    //     if (Config::eva_lio){
    //         lio_pose_sub_ptr_->ParseData(lio_pose_buffer_);
    //         while (!lio_pose_buffer_.empty()){

    //             std::pair<double, Eigen::Matrix4d> item;
    //             item.first = lio_pose_buffer_.front().time;
    //             lio_pose = lio_pose_buffer_.front().pose.cast<double>();
    //             item.second = lio_pose;
    //             lio_pose_buffer_.pop_front();
    //             if(stop_tag_flag)
    //                 continue;
    //             Eigen::Vector3d pos = lio_pose.block<3,1>(0,3);
    //             double cur_length = sqrt(
    //                     (pos[0] - last_pos_lio[0]) * (pos[0] - last_pos_lio[0]) +
    //                     (pos[1] - last_pos_lio[1]) * (pos[1] - last_pos_lio[1]));
    //             lio_length += cur_length;
    //             last_pos_lio = pos;
    //             if(LIOPath.size() == 0){
    //                 first_lio_time = item.first;
    //             }
    //             LIOPath.push_back(item);
    //         }
    //     }
    //     if (Config::eva_viw){
    //         viw_pose_sub_ptr_->ParseData(viw_pose_buffer_);
    //         while (!viw_pose_buffer_.empty()){


    //             std::pair<double, Eigen::Matrix4d> item;
    //             item.first = viw_pose_buffer_.front().time;
    //             item.second = viw_pose_buffer_.front().pose.cast<double>();
    //             VIWPath.push_back(item);
    //             viw_pose_buffer_.pop_front();
    //             if(stop_tag_flag)
    //                 continue;
    //             ///
    //             Eigen::Vector3d pos = item.second.block<3,1>(0,3);
    //             double cur_length = sqrt(
    //                     (pos[0] - last_pos_viw[0]) * (pos[0] - last_pos_viw[0]) +
    //                     (pos[1] - last_pos_viw[1]) * (pos[1] - last_pos_viw[1]));
    //             viw_length += cur_length;
    //             last_pos_viw = pos;
    //             ///
    //         }
    //     }
    //     rate.sleep();
    // }

    // bool succc = EVO(lio_length, LIOPath, 1);

    return 0;
}