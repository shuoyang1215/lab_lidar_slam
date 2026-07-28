
#ifndef GLOBAL_FUSION_TOOLS_PRINT_INFO_HPP_
#define GLOBAL_FUSION_TOOLS_PRINT_INFO_HPP_

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include "pcl/common/eigen.h"

namespace eva_april {
    class PrintInfo {
    public:
        static void PrintPose(std::string head, Eigen::Matrix4f pose);
    };
}
#endif