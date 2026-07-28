#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

template<typename T>
inline Eigen::Matrix<T,4,4> interpolateSE3(const Eigen::Matrix<T,4,4> & source, const Eigen::Matrix<T,4,4> & target, const T alpha){
//    if(alpha<0 || alpha>1)
//    {
//        std::cerr << "warning: alpha < 0 or alpha > 1" <<std::endl;
//    }
    if(alpha < 1e-6)
        return Eigen::Matrix<T,4,4>::Identity();
    else{
        Sophus::SE3d<T> SE1(source);
        Sophus::SE3d<T> SE2(target);
        if(alpha > (1-1e-6))
            return (SE1.inverse() * SE2).matrix();
        else{
            Eigen::Matrix<T, 6, 1> se1 = SE1.log();
            Eigen::Matrix<T, 6, 1> se2 = SE2.log();

            Sophus::SE3d<T> SE3t = SE1 * Sophus::SE3d<T>::exp(alpha * (se2-se1));

            return SE3t.matrix();
        }
    }

}

template<typename T>
inline Eigen::Matrix<T,4,4> scaleSE3(const Eigen::Matrix<T,4,4>& source, const T alpha){

//    if(alpha<0 || alpha>1)
//    {
//        std::cerr << "warning: alpha < 0 or alpha > 1" <<std::endl;
//    }
    if(alpha < 1e-6)
        return Eigen::Matrix<T,4,4>::Identity();
    else if(alpha > (1-1e-6))
        return source;
    else{
        Eigen::Matrix<T,6,1> se3 = Sophus::SE3d<T>(source).log();
        Sophus::SE3d<T> result = Sophus::SE3d<T>::exp(alpha * se3);
        return result.matrix();
    }
}