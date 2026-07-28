//
// Created by qzj on 2021/4/25.
//

#include "config.h"
#include "glog/logging.h"

namespace eva_april{

    // Topics
    // raw sensor data
    YAML::Node Config::config_node_ = YAML::Node();


    std::string Config::save_dir;
    std::string Config::cam_topic;
    // map

    // pose source (pose graph edge)
    // for use
    std::string Config::dr_odo_path_topic;
    std::string Config::dr_odo_tf_topic;
    std::string Config::dr_odo_topic;
    std::string Config::absolute_pose_topic;
    std::string Config::vio_odo_topic;
    std::string Config::lio_odo_topic;
    // for rviz
    // sensor fusion
    // for use
    std::string Config::fused_pose_topic;
    std::string Config::high_freq_pose_topic;
    std::string Config::output_pose_topic;
    std::string Config::rtk_init_path;
    std::string Config::log_save_path;
    std::string Config::location_state_topic;
    std::string Config::output_pose_odometry_topic;
    std::string Config::start_trigger_topic;
    std::string Config::usb_sub_state_topic;
    bool Config::use_trigger;
    // for rviz
    std::string Config::fused_pose_tf_topic;
    std::string Config::fused_pose_path_topic;
    std::string Config::gnss_pose_path_topic;
    std::string Config::viw_pose_path_topic;
    std::string Config::high_freq_pose_tf_topic;
    std::string Config::high_freq_pose_path_topic;
    std::string Config::fused_pose_cur_scan_topic;



    float Config::FREQ;
    float Config::FREQ_viw;
    int Config::apriltag_size;
    Eigen::Matrix4d Config::imu_T_wheel;
    Eigen::Matrix4d Config::rtk_T_wheel;
    Eigen::Matrix3d Config::ric;
    Eigen::Vector3d Config::tic;
    float Config::td;
    std::string Config::cam_param;

    std::string Config::length_topic;
    std::string Config::reset_topic;
    std::string Config::viw_topic;

    bool Config::set_log_save_path;
    bool Config::eva_lio;
    bool Config::eva_viw;
    int Config::use_glog;

    void Config::readConfig() {

        config_node_ = YAML::LoadFile(eva_april::MATCH_YAML_PATH);

        // Topics
        // raw sensor data
        cam_topic = config_node_["cam_topic"].as<std::string>();

        ///创建评估结果文件
        time_t now_time = time(NULL);
        tm *T_tm = localtime(&now_time);
        //转换为年月日星期时分秒结果，如图：
        std::string timeDetail = asctime(T_tm);
        int n=9;
        while(n-->0)
            timeDetail.pop_back();

        set_log_save_path = config_node_["set_log_save_path"].as<bool>();
        log_save_path = config_node_["log_save_path"].as<std::string>();


        eva_april::FileManager::replace_str(timeDetail," ","-");
        eva_april::FileManager::replace_str(timeDetail,":","-");

        if(set_log_save_path){
            save_dir = log_save_path + "/SLAM_LOG_fusion/" + timeDetail + '/';
        }
        else{
            std::string PROJECT_DIR = ros::package::getPath("eva_april");
            save_dir = PROJECT_DIR + "/SLAM_LOG_fusion/" + timeDetail + '/';
        }

        eva_april::FileManager::createDirectory(save_dir);

        std::string log_path = save_dir + "fusion_log.log";

        // pose source (pose graph edge)
        // for use
        dr_odo_topic = config_node_["dr_odo_topic"].as<std::string>();
        absolute_pose_topic = config_node_["absolute_pose_topic"].as<std::string>();
        vio_odo_topic = config_node_["vio_odo_topic"].as<std::string>();
        lio_odo_topic = config_node_["lio_odo_topic"].as<std::string>();
        // for rviz
        dr_odo_tf_topic = config_node_["dr_odo_tf_topic"].as<std::string>();
        dr_odo_path_topic = config_node_["dr_odo_path_topic"].as<std::string>();

        // sensor fusion
        // for use
        output_pose_topic = config_node_["output_pose_topic"].as<std::string>();
        rtk_init_path = config_node_["rtk_init_path"].as<std::string>();
        location_state_topic = config_node_["location_state_topic"].as<std::string>();
        output_pose_odometry_topic = config_node_["output_pose_odometry_topic"].as<std::string>();
        high_freq_pose_topic = config_node_["high_freq_pose_topic"].as<std::string>();
        fused_pose_topic = config_node_["fused_pose_topic"].as<std::string>();
        use_trigger = config_node_["use_trigger"].as<bool>();
        start_trigger_topic = config_node_["start_trigger_topic"].as<std::string>();
        usb_sub_state_topic = config_node_["usb_sub_state_topic"].as<std::string>();
        // for rviz
        high_freq_pose_tf_topic = config_node_["high_freq_pose_tf_topic"].as<std::string>();
        high_freq_pose_path_topic = config_node_["high_freq_pose_path_topic"].as<std::string>();
        fused_pose_tf_topic = config_node_["fused_pose_tf_topic"].as<std::string>();
        fused_pose_path_topic = config_node_["fused_pose_path_topic"].as<std::string>();
        gnss_pose_path_topic = config_node_["gnss_pose_path_topic"].as<std::string>();
        viw_pose_path_topic = config_node_["viw_pose_path_topic"].as<std::string>();
        fused_pose_cur_scan_topic = config_node_["fused_pose_cur_scan_topic"].as<std::string>();

        for (size_t i = 0; i < 4; i++){
            for (size_t j = 0; j < 4; j++)
                imu_T_wheel(i, j) = config_node_["imu_T_wheel"][4 * i + j].as<double>();
        }

        for (size_t i = 0; i < 4; i++){
            for (size_t j = 0; j < 4; j++)
                rtk_T_wheel(i, j) = config_node_["rtk_T_wheel"][4 * i + j].as<double>();
        }

        // for apriltag
        FREQ = config_node_["freq_fused"].as<float>();
        FREQ_viw = config_node_["freq_viw"].as<float>();
        apriltag_size = config_node_["apriltag_size"].as<int>();
        for (size_t i = 0; i < 3; i++){
            for (size_t j = 0; j < 3; j++)
                ric(i, j) = config_node_["Cam2Imu_R"][3 * i + j].as<double>();
        }
        for (size_t j = 0; j < 3; j++)
            tic(j) = config_node_["Cam2Imu_T"][j].as<double>();
        td = config_node_["td"].as<float>();
        cam_param = config_node_["cam_param"].as<std::string>();

        length_topic = config_node_["length_topic"].as<std::string>();
        reset_topic = config_node_["reset_topic"].as<std::string>();
        viw_topic = config_node_["viw_topic"].as<std::string>();

        eva_lio = config_node_["eva_lio"].as<bool>();
        eva_viw = config_node_["eva_viw"].as<bool>();
        use_glog = config_node_["use_glog"].as<int>();

    }
} // namespace eva_april
