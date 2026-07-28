#ifndef YOLOX_TENSORRT_CPP_ROS_KALMAN_H
#define YOLOX_TENSORRT_CPP_ROS_KALMAN_H


#include <Eigen/Dense>
#include "ros/ros.h"

class KalmanFilter
{
public:
//private:
    int stateSize; //state variable's dimenssion
    int measSize; //measurement variable's dimession
    int uSize; //control variables's dimenssion
    Eigen::VectorXd x;
    Eigen::VectorXd z;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd u;
    Eigen::MatrixXd P;//coveriance
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;//measurement noise covariance
    Eigen::MatrixXd Q;//process noise covariance
    float time_now;
    bool is_tracked;
    float tracked_count;
    float live_count;
//public:
    KalmanFilter();
    KalmanFilter(Eigen::VectorXd &x_);
    ~KalmanFilter(){}
    void init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_,Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_);
    Eigen::VectorXd predict();
    Eigen::VectorXd predict(Eigen::MatrixXd& A_);
    Eigen::VectorXd predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_);
    Eigen::VectorXd update(Eigen::VectorXd z_meas);
    Eigen::VectorXd update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas);
};

#endif //YOLOX_TENSORRT_CPP_ROS_KALMAN_H
