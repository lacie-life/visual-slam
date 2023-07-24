//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/se3.hpp>


int main(){

    //create a rotation matrix using axis-angle
    // Rotation matrix with 90 degrees along Z axis
    Eigen::Matrix3d R(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)));
    //turn to quaternion
    Eigen::Quaterniond q(R);
    //create a sophus SO3
    Sophus::SO3d SO3_R(R);
    //from quaternion to rotation matrix
    Sophus::SO3d SO3_q(q);
    //print SO3_R
    std::cout << "SO3_R: " << std::endl << SO3_R.matrix() << std::endl;
    //print SO3_q
    std::cout << "SO3_q: " << std::endl << SO3_q.matrix() << std::endl;
    //they are equal
    std::cout << "they are equal " << std::endl;

    //get lie algebra representation of SO3
    auto so3 = SO3_R.log();
    //print lie_SO3_R
    std::cout << "so3: " << std::endl << so3.transpose() << std::endl;
    //convert so3 to skew-symmetric matrix
    auto so3_skew = Sophus::SO3d::hat(so3);
    //print so3_skew
    std::cout << "so3_skew: " << std::endl << so3_skew << std::endl;
    //take vee operator of so3_skew
    auto so3_skew_vee = Sophus::SO3d::vee(so3_skew);
    //print so3_skew_vee
    std::cout << "so3_skew_vee: " << std::endl << so3_skew_vee.transpose() << std::endl;

    //update by perturbation model
    Eigen::Vector3d so3_perturbation(1e-4,0.0,0.0); //this is a small perturbation
    Sophus::SO3d SO3_updated = Sophus::SO3d::exp(so3_perturbation) * SO3_R;
    //print SO3_updated
    std::cout << "SO3_updated: " << std::endl << SO3_updated.matrix() << std::endl;
    std::cout << "************************************************************"<< std::endl;

    //create for SE3
    Eigen::Vector3d t(1.0,2.0,3.0);
    Sophus::SE3d SE3_Rt(SO3_R,t);
    //SE3 with quaternion
    Sophus::SE3d SE3_qt(q,t);
    //print SE3_Rt
    std::cout << "SE3_Rt: " << std::endl << SE3_Rt.matrix() << std::endl;
    //print SE3_qt
    std::cout << "SE3_qt: " << std::endl << SE3_qt.matrix() << std::endl;
    //compute lie algebra representation of SE3
    auto se3 = SE3_Rt.log();
    //print lie_SE3_Rt
    std::cout << "se3: " << std::endl << se3.transpose() << std::endl;

    //compute hat operator of se3
    auto se3_hat = Sophus::SE3d::hat(se3);
    //print se3_hat
    std::cout << "se3_hat: " << std::endl << se3_hat << std::endl;
    //take vee operator of se3_hat
    auto se3_hat_vee = Sophus::SE3d::vee(se3_hat);
    //print se3_hat_vee
    std::cout << "se3_hat_vee: " << std::endl << se3_hat_vee.transpose() << std::endl;

    //take SO3 from SE3
    Sophus::SO3d SO3_SE3 = SE3_Rt.so3();
    //print SO3_SE3
    std::cout << "SO3_SE3: " << std::endl << SO3_SE3.matrix() << std::endl;
    auto t_se3 = SE3_Rt.translation();
    //print t_se3
    std::cout << "t_se3: " << std::endl << t_se3.transpose() << std::endl;
    //update by perturbation model
    Eigen::VectorXd update_se3; //this is a small perturbation
    update_se3.resize(6);
    update_se3.setZero();
    update_se3(0) = 1e-4;
    //update SE3
    Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt;
    //print SE3_updated
    std::cout << "SE3_updated: " << std::endl << SE3_updated.matrix() << std::endl;



    return 0;

}


