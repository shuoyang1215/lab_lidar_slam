
#include "sensor_data/pose_data.hpp"

namespace eva_april {
    Eigen::Quaterniond PoseData::GetQuaternion() {
        Eigen::Quaterniond q;
        q = pose.block<3, 3>(0, 0);

        return q;
    }
}