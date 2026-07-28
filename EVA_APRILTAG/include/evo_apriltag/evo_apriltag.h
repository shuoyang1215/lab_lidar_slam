//
// Created by guo on 2022/2/18.
//

#ifndef SRC_EVO_APRILTAG_H
#define SRC_EVO_APRILTAG_H

#include <stdio.h>
#include <deque>
#include <queue>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "calc_cam_pose/calcCamPose.h"

using namespace std;

//const int WINDOW_SIZE = 10;
//std::vector<std::pair<double, Eigen::Matrix4d>> FusedPath;///for apriltag

void init();
void AprilTag_buf_push(std::pair<double, cv::Mat> img, bool init, bool init_lio, bool init_viw);
void AprilTag_buf_filter(const double &first_time, deque<pair<double, cv::Mat>> &buf_start, deque<pair<double, cv::Mat>> &buf_latest);
bool EVO(double &total_length, std::vector<std::pair<double, Eigen::Matrix4d>> &FusedPath, int choice=0);
static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);
#endif //SRC_EVO_APRILTAG_H
