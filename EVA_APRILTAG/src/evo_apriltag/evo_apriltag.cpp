
#include "evo_apriltag/evo_apriltag.h"
#include "config.h"
#include "camodocal/camera_models/CameraFactory.h"
#include <ros/package.h>
#include "tools/file_manager.hpp"
#include "glog/logging.h"

deque<pair<double, cv::Mat>> AprilTag_buf_start, AprilTag_buf_latest, AprilTag_lio_buf_start, AprilTag_viw_buf_start;
int AprilTagSize;
std::string EVO_RESULT_PATH;

void init(){
    AprilTagSize = eva_april::Config::apriltag_size;
    AprilTag_buf_start.clear();
    AprilTag_buf_latest.clear();
    AprilTag_lio_buf_start.clear();
    AprilTag_viw_buf_start.clear();
    ///创建评估结果文件
//    time_t now_time = time(NULL);
//    tm *T_tm = localtime(&now_time);
//    //转换为年月日星期时分秒结果，如图：
//    string timeDetail = asctime(T_tm);
//    int n=9;
//    while(n-->0)
//        timeDetail.pop_back();
//    eva_april::FileManager::replace_str(timeDetail," ","-");
//    eva_april::FileManager::replace_str(timeDetail,":","-");
//    std::string PROJECT_DIR = ros::package::getPath("eva_april");
//    std::string OUTPUT_FOLDER = PROJECT_DIR + "/fusion_result/" + timeDetail + '/';
//    eva_april::FileManager::createDirectory(OUTPUT_FOLDER);
//    EVO_RESULT_PATH = OUTPUT_FOLDER + "fusion_evo_result.txt";
//    std::ofstream fou3(EVO_RESULT_PATH, std::ios::out);
//    fou3.close();
}

/**
 * @brief 将开始原始图像帧保存到buf中，只保留开始和结尾的图像
 */
void AprilTag_buf_push(std::pair<double, cv::Mat> img, bool init, bool init_lio, bool init_viw)
{
    if(init){///estimator.solver_flag == SolverFlag::INITIAL
        while(AprilTag_buf_start.size() >= AprilTagSize)
            AprilTag_buf_start.pop_front();
        AprilTag_buf_start.emplace_back(img);
    }
    else{
        while(AprilTag_buf_latest.size() >= AprilTagSize)
            AprilTag_buf_latest.pop_front();
        AprilTag_buf_latest.emplace_back(img);
    }
    if (eva_april::Config::eva_lio && init_lio){
        while(AprilTag_lio_buf_start.size() >= AprilTagSize)
            AprilTag_lio_buf_start.pop_front();
        AprilTag_lio_buf_start.emplace_back(img);
    }
    if (eva_april::Config::eva_viw && init_viw){
        while(AprilTag_viw_buf_start.size() >= AprilTagSize)
            AprilTag_viw_buf_start.pop_front();
        AprilTag_viw_buf_start.emplace_back(img);
    }

}

/**
 * @brief 当初始化完成之后将VIO轨迹第一帧之前保存在buf里面的图像过滤掉，并将剩余buf的后一部分删掉，只保留3帧
 * @param first_time VIO轨迹第一帧的时间
 */
void AprilTag_buf_filter(const double &first_time, deque<pair<double, cv::Mat>> &buf_start, deque<pair<double, cv::Mat>> &buf_latest)
{
    while (buf_start.front().first < first_time - 0.001 && buf_start.size() > 0)
        buf_start.pop_front();
    while (buf_start.size() > AprilTagSize)
        buf_start.pop_back();
}

/**
 * @brief 利用buf里面的AprilTag图像和估计的轨迹进行轨迹评估
 */
bool EVO(double &total_length, std::vector<std::pair<double, Eigen::Matrix4d>> &FusedPath, int choice)
{
    double dt;
    deque<pair<double, cv::Mat>> buf_start;
    switch(choice){
        case 1:
            buf_start = AprilTag_lio_buf_start; dt = 0.1; break;
        case 2:
            buf_start = AprilTag_viw_buf_start; dt = 1.0/eva_april::Config::FREQ_viw; break;
        default:
            buf_start = AprilTag_buf_start; dt = 1.0/eva_april::Config::FREQ;
    }

    LOG_IF(INFO, eva_april::Config::use_glog) << "buf_size_start--------------- " <<  AprilTag_buf_start.size()<< endl;


    std::string camPath = eva_april::WORK_SPACE_PATH + "/config/" + eva_april::Config::cam_param;
    camodocal::CameraPtr m_camera;
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(camPath);
    cv::Mat image;
    std_msgs::Header header;
    double time_evo = 0, time_est;
    Eigen::Matrix4d evo_Twc_start = Eigen::Matrix4d::Identity(),
            evo_Twc_latest = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d est_Tbw_start = Eigen::Matrix4d::Identity(),
            est_Tbw_latest = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d evo_relative, est_relative;
    bool start_isCalOK = false, latest_isCalOK = false;
    auto iter_start = FusedPath.begin(),
            iter_latest = FusedPath.end() - 1;
    vector<pair<Eigen::Matrix4d, Eigen::Matrix4d>> evo_est_poses_start, evo_est_poses_latest;
    //while (!AprilTag_buf_start.empty())
    for (deque<pair<double, cv::Mat>>::iterator it = buf_start.begin(); it != buf_start.end(); ++it)
    {
        if (evo_est_poses_start.size()>2)
            break;
        time_evo = it->first;
        time_est = iter_start->first;

        while (time_est < (time_evo - dt))
        {
            iter_start++;
            time_est = iter_start->first;

            LOG_IF(INFO, eva_april::Config::use_glog) << "时间差-----" <<  abs(time_evo - time_est) << "--- dt: "<< dt << endl;
        }
        if(abs(time_evo - time_est) < dt)
        {
            LOG_IF(INFO, eva_april::Config::use_glog) << " abs(time_evo - time_est) < dt ------" << endl;

            start_isCalOK = calcCamPose(time_evo,
                                        it->second,
                                        m_camera,
                                        evo_Twc_start);

            LOG_IF(INFO, eva_april::Config::use_glog) << " start_is_cal_ok ------" << start_isCalOK << endl;

            if(start_isCalOK)
            {
                est_Tbw_start = iter_start->second;
                evo_est_poses_start.emplace_back(make_pair(evo_Twc_start, est_Tbw_start));
                LOG_IF(INFO, eva_april::Config::use_glog) << " evo_est_poses_start.emplace_back(make_pair(evo_Twc_start, est_Tbw_start)); " << endl;
            }
            iter_start++;
        }
    }

    for (deque<pair<double, cv::Mat>>::iterator it = AprilTag_buf_latest.end() - 1; it != AprilTag_buf_latest.begin() - 1; --it)
    {
        if (evo_est_poses_latest.size()>2)
            break;
        time_evo = it->first;
        time_est = iter_latest->first;
        while(time_est > (time_evo + dt))
        {
            iter_latest--;
            time_est = iter_latest->first;
            LOG_IF(INFO, eva_april::Config::use_glog) << "时间差-----" <<  abs(time_est - time_evo) << "--- dt: "<< dt << endl;
        }
        if(abs(time_evo - time_est) < dt)
        {
            LOG_IF(INFO, eva_april::Config::use_glog) << " abs(time_evo - time_est) < dt ------" << endl;
            latest_isCalOK = calcCamPose(time_evo,
                                         it->second,
                                         m_camera,
                                         evo_Twc_latest);
            LOG_IF(INFO, eva_april::Config::use_glog) << " latest_isCalOK ------" << start_isCalOK << endl;

            if(latest_isCalOK)
            {
                est_Tbw_latest = iter_latest->second;
                evo_est_poses_latest.emplace_back(make_pair(evo_Twc_latest, est_Tbw_latest));
                LOG_IF(INFO, eva_april::Config::use_glog) << " evo_est_poses_latest.emplace_back   " << endl;
            }
            iter_latest--;
        }
    }

    if(evo_est_poses_start.empty())
    {
        LOG_IF(INFO, eva_april::Config::use_glog) << "START_APRILTAG_FAIL!!!!!!!!!!" <<endl;
        return false;
    }

    if(evo_est_poses_latest.empty())
    {
        LOG_IF(INFO, eva_april::Config::use_glog) << "END_APRILTAG_FAIL!!!!!!!!!!" <<endl;
        return false;
    }
    //TODO：若有viw轨迹 取在线估计的外参
    Eigen::Matrix4d body_T_cam0 = Eigen::Matrix4d::Identity();
    body_T_cam0.block<3, 3>(0, 0) = eva_april::Config::ric;
    body_T_cam0.block<3, 1>(0, 3) = eva_april::Config::tic;

    Eigen::Matrix4d Error = Eigen::Matrix4d::Identity();
    vector<Eigen::Matrix4d> Error_all;
    for(auto & it1 : evo_est_poses_start)
    {
        for(auto & it2 : evo_est_poses_latest)
        {
//            evo_relative = (body_T_cam0 * it1.first) *(body_T_cam0 * it2.first).inverse();
            evo_relative = (it1.first * body_T_cam0.inverse()).inverse() *(it2.first * body_T_cam0.inverse());

            est_relative = it1.second.inverse() * it2.second;
            Error = evo_relative.inverse() * est_relative;
            Error_all.emplace_back(Error);
        }
    }
    for(auto & it : Error_all)
    {
        Eigen::Vector3d Error_rot = R2ypr(Error.block<3, 3>(0,0));
        Eigen::Vector3d Error_rot_i = R2ypr(it.block<3, 3>(0,0));
        if(Error_rot_i.norm() < Error_rot.norm())
            Error = it;
    }

    Eigen::Vector3d Error_rot = R2ypr(Error.block<3, 3>(0,0)); // Yaw-Pitch-Roll  (deg)
    Eigen::AngleAxisd Error_Angle(Error.block<3, 3>(0,0));
    Eigen::Vector3d Error_tran(Error.block<3,1>(0,3));

    ofstream fout(eva_april::Config::save_dir + "fusion_evo_result.txt", ios::app);
    fout.setf(ios::fixed, ios::floatfield);
    switch(choice){
        case 1:
            fout << "\n---------------------Evaluation for LIO---------------------\n";
            cerr << "---------------------Evaluation for LIO---------------------\n"; break;
        case 2:
            fout << "\n---------------------Evaluation for VIW---------------------\n";
            cerr << "---------------------Evaluation for VIW---------------------\n"; break;
        default :
            fout << "---------------------Evaluation for fusion---------------------\n";
            cerr << "---------------------Evaluation for fusion---------------------\n";
    }
    fout << "total dist (m) : " << total_length << " m\n";
    fout << "rot error (deg) : Yaw-Pitch-Roll" << "\n"
         << "           " << Error_rot.transpose() << endl;
    fout << "euler error (deg/m) : Yaw-Pitch-Roll" << "\n"
         << "             " << Error_rot.transpose() / total_length << endl;
    fout << "Yaw error (deg/m) : " <<  Error_rot(0) / total_length << "\n";
    fout << "RP error (deg/m) : " <<  Error_rot.block<2,1>(1,0).norm() / total_length << endl;
    fout << "tran error (m) : x-y-z" << "\n"
         << "           " << Error_tran.transpose() << endl;
    fout << "tran error (%) : x-y-z" << "\n"
         << "              " << Error_tran.transpose() *100.0 / total_length << endl;
    fout << "tran norm error (%) : " << Error_tran.block<2,1>(0,0).norm() * 100.0 / total_length << endl;
    fout.close();

    cerr << "total dist (m) : " << total_length << " m\n";
    cerr << "rot error (deg) : Yaw-Pitch-Roll" << "\n"
         << "           " << Error_rot.transpose() << endl;
    cerr << "euler error (deg/m) : Yaw-Pitch-Roll" << "\n"
         << "             " << Error_rot.transpose() / total_length << endl;
    cerr << "Yaw error (deg/m) : " <<  Error_rot(0) / total_length << "\n";
    cerr << "RP error (deg/m) : " <<  Error_rot.block<2,1>(1,0).norm() / total_length << endl;
    cerr << "tran error (m) : x-y-z" << "\n"
         << "           " << Error_tran.transpose() << endl;
    cerr << "tran error (%) : x-y-z" << "\n"
         << "              " << Error_tran.transpose() *100.0 / total_length << endl;
    cerr << "tran norm error (%) : " << Error_tran.block<2,1>(0,0).norm() * 100.0 / total_length << endl;
    return true;
}

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}
