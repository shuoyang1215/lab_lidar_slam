
#ifndef GLOBAL_FUSION_SENSOR_DATA_POSE_DATA_HPP_
#define GLOBAL_FUSION_SENSOR_DATA_POSE_DATA_HPP_

#include <Eigen/Dense>

namespace eva_april {
    class PoseData {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseData(){}

        PoseData(const Eigen::Matrix<double,4,4> &pose_){
            pose = pose_;
        }

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d ground_pose = Eigen::Matrix4d::Identity();
        double time = 0.0, ground_plane_h = 0.0;
        bool is_reliable = true;
    public:
        Eigen::Quaterniond GetQuaternion();
    };
}

#endif