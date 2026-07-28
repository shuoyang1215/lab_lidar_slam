//
// Created by qzj on 2021/4/25.
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include <yaml-cpp/yaml.h>
#include "global_definition/global_definition.h"
#include <Eigen/Core>
#include <vector>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>      // std::ifstream
#include <stdio.h>
#include <unistd.h>
#include "tools/file_manager.hpp"
#include <ros/package.h>


namespace eva_april{
    class Config {

    public:

        Config(){}
        static void readConfig();

    public:
        // Map files
        static std::string cam_topic;
        static std::string save_dir;
        //       for use
        static std::string dr_odo_tf_topic;
        static std::string dr_odo_path_topic;
        static std::string dr_odo_topic;
        static std::string absolute_pose_topic;
        static std::string vio_odo_topic;
        static std::string lio_odo_topic;
        static std::string start_trigger_topic;
        static std::string usb_sub_state_topic;
        static bool use_trigger;
        //       for rviz
        // sensor fusion
        //    for use
        static std::string rtk_init_path;
        static std::string log_save_path;
        static std::string output_pose_topic;
        static std::string location_state_topic;
        static std::string output_pose_odometry_topic;
        static std::string high_freq_pose_topic;
        static std::string fused_pose_topic;
        //    for rviz
        static std::string high_freq_pose_tf_topic;
        static std::string high_freq_pose_path_topic;
        static std::string fused_pose_tf_topic;
        static std::string fused_pose_path_topic;
        static std::string gnss_pose_path_topic;
        static std::string viw_pose_path_topic;
        static std::string fused_pose_cur_scan_topic;



        static float FREQ;
        static float FREQ_viw;
        static int apriltag_size;
        static float td;
        static Eigen::Matrix4d imu_T_wheel;
        static Eigen::Matrix4d rtk_T_wheel;
        static Eigen::Matrix3d ric;
        static Eigen::Vector3d tic;
        static std::string cam_param;
        static std::string length_topic;
        static std::string reset_topic;
        static std::string viw_topic;
        static bool set_log_save_path;
        static bool eva_lio;
        static int use_glog;
        static bool eva_viw;
        static YAML::Node config_node_;
    };
} // namespace eva_april

#endif //SRC_CONFIG_H
