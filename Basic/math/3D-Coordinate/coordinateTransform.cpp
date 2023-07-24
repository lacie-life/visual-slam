//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(){

    Eigen::Quaternion<double> q1(.35, 0.2, 0.3, 0.1);
    Eigen::Quaternion<double> q2(-0.5, 0.4, -0.1, 0.2);

    q1.normalize();
    q2.normalize();


    Eigen::Vector3d t1(0.3, 0.1, 0.1);
    Eigen::Vector3d t2(-0.1, 0.5, 0.3 );

    //World to robot 1
    Eigen::Isometry3d T_R1W = Eigen::Isometry3d(q1);
    T_R1W.pretranslate(t1);

    //World to robot 2
    Eigen::Isometry3d T_R2W = Eigen::Isometry3d(q2);
    T_R2W.pretranslate(t2);

    //coordinate of point to R1
    Eigen::Vector3d p_R1(0.5, 0.0, 0.2);
    //point to R2
    Eigen::Vector3d p_R2 = T_R2W * T_R1W.inverse() * p_R1;

    //set precision
    std::cout.precision(5);
    std::cout << "p_R2 = " << p_R2.transpose() << std::endl;


    return 0;

}

