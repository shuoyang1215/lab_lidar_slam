#include "../../include/sort/Kalman.h"
#include <iostream>

using namespace std;

//KalmanFilter::KalmanFilter(int stateSize_ = 4, int measSize_ = 2, int uSize_=0) :stateSize(stateSize_), measSize(measSize_), uSize(uSize_)
KalmanFilter::KalmanFilter(Eigen::VectorXd &x_):x(x_)
{
    stateSize = 4;
    measSize = 2;
    uSize = 0;

    // 状态矩阵
    x.resize(stateSize);
    // x.setZero();

    // 状态转移矩阵
    A.resize(stateSize, stateSize);
    // A.setIdentity();
    A <<    1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    u.resize(uSize);
    u.transpose();
    u.setZero();

    B.resize(stateSize, uSize);
    B.setZero();

    P.resize(stateSize, stateSize);
    P.setIdentity();

    // 状态值到测量值
    H.resize(measSize, stateSize);
    // H.setZero();
    H <<    1, 0, 0, 0,
            0, 1, 0, 0;

    z.resize(measSize);
    z.setZero();

    Q.resize(stateSize, stateSize);
    // Q.setZero();
    Q.setIdentity()*0.001;

    R.resize(measSize, measSize);
    // R.setZero();
    R.setIdentity()*0.01;
    time_now = ros::Time::now().toSec();
    is_tracked = false;
    tracked_count = 1;
    live_count = 1;

}

void KalmanFilter::init(Eigen::VectorXd &x_, Eigen::MatrixXd& P_, Eigen::MatrixXd& R_, Eigen::MatrixXd& Q_)
{
    x = x_;
    P = P_;
    R = R_;
    Q = Q_;
}

// 预测 返回预测cxcy
Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_, Eigen::MatrixXd &B_, Eigen::VectorXd &u_)
{
    A = A_;
    B = B_;
    u = u_;
    x = A*x + B*u;
    Eigen::MatrixXd A_T = A.transpose();
    P = A*P*A_T + Q;
    return H*x;
}

// 预测 返回预测cxcy
Eigen::VectorXd KalmanFilter::predict(Eigen::MatrixXd& A_)
{
    A = A_;
    // x = x 预测值（加上速度）
    x = A*x;
    Eigen::MatrixXd A_T = A.transpose();
    // P = P矩阵预测值
    P = A*P*A_T + Q;
    return H*x;
}

// 预测 返回预测cxcy
Eigen::VectorXd KalmanFilter::predict()
{
    // x = x 预测值（加上速度）
    x = A*x;
    Eigen::MatrixXd A_T = A.transpose();
    // P = P矩阵预测值
    P = A*P*A_T + Q;
    return H*x;
}

// 更新与校正 返回校正cxcy
Eigen::VectorXd KalmanFilter::update(Eigen::MatrixXd& H_,Eigen::VectorXd z_meas)
{
    H = H_;
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
    temp1 = H*P*Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    // Kalman gain 校正
    Eigen::MatrixXd K = P*Ht*temp2;
    // 根据预测状态x更新预测z
    z = H*x;
    // 根据测量值校正状态值
    x = x + K*(z_meas-z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    // P矩阵校正
    P = (I - K*H)*P;
    // 根据校正状态值返回校正z
    return H*x;
}

// 更新与校正 返回校正cxcy
Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd z_meas)
{
    Eigen::MatrixXd temp1, temp2,Ht;
    Ht = H.transpose();
    temp1 = H*P*Ht + R;
    temp2 = temp1.inverse();//(H*P*H'+R)^(-1)
    // Kalman gain 校正
    Eigen::MatrixXd K = P*Ht*temp2;
    // 根据预测状态x更新预测z
    z = H*x;
    // 根据测量值校正状态值
    x = x + K*(z_meas-z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    // P矩阵校正
    P = (I - K*H)*P;
    // 根据校正状态值返回校正z
    return H*x;
}

