//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tictoc.h"


template <typename T>
void skew_symmetric_matrix(Eigen::Matrix<T, 3, 3> &matrix, const Eigen::Matrix<T, 3, 1> &vector) {
    matrix << 0, -vector(2), vector(1),
            vector(2), 0, -vector(0),
            -vector(1), vector(0), 0;
}

int main(){

    TicToc tt;

    std::cout << "--------------------------VECTOR -------------------------------" << std::endl;

    //create a vector
    Eigen::Vector3d v(1,2,3);
    std::cout << "v: " << v.transpose() << std::endl;

    //create a dynamic vector
    Eigen::VectorXd vd;
    //set the size of the vector to 10
    vd.resize(10);
    vd << 1,2,3,4,5,6,7,8,9,10;
    std::cout << "vd: " << vd.transpose() << std::endl;
    //set the size of the vector to 20
    vd.resize(20);
    vd << 1,2,3,4,5,6,7,8,9,10, 11,12,13,14,15,16,17,18,19,20;
    std::cout << "vd: " << vd.transpose() << std::endl;

    vd.resize(15);
    //set linespace 1 to 100 with 15 elements
    vd.setLinSpaced(15,1,100);
    std::cout << "vd.setLinSpaced(15,1,100) " << vd.transpose() << std::endl;
    vd.setLinSpaced(5, 50);
    std::cout << "vd.setLinSpaced(5, 50) " << vd.transpose() << std::endl;

    //set random values from 1 to 100
    vd.setRandom();
    std::cout << "vd transpose: " << vd.transpose() << std::endl;
    //find the max value
    std::cout << "maxCoeff: " << vd.maxCoeff() << std::endl;
    //find the min value
    std::cout << "minCoeff: " << vd.minCoeff() << std::endl;
    //find the mean value
    std::cout << "mean: " << vd.mean() << std::endl;
    //take value form 5 to 10
    std::cout << "vd.segment(5,5): " << vd.segment(5,5).transpose() << std::endl;

    //create a vector with size 10
    Eigen::VectorXd vd2(10);
    vd2.resize(10);
    //set the values from 1 to 10
    vd2 << 1,2,3,4,5,6,7,8,9,10;
    //create a vector with size 10
    Eigen::VectorXd vd3(10);
    vd3.resize(10);
    //set the values from 10 to 1
    vd3 << 10,9,8,7,6,5,4,3,2,1;
    //size of vd3
    std::cout << "vd3 size: " << vd3.size() << std::endl;

    //add vd2 and vd3
    std::cout << "vd2 + vd3: " << (vd2 + vd3).transpose() << std::endl;
    //subtract vd2 and vd3
    std::cout << "vd2 - vd3: " << (vd2 - vd3).transpose() << std::endl;
    //dot product of vd2 and vd3
    std::cout << "vd2 dot vd3: " << vd2.dot(vd3) << std::endl;

    //create a 3d vector
    Eigen::Vector3d x1(1,2,3);
    //create a 3d vector
    Eigen::Vector3d x2(4,5,6);

    //cross product of x1 and x2
    std::cout << "x1 cross x2: " << (x1.cross(x2)).transpose() << std::endl;

    //create skew symmetric matrix of vector x1
    Eigen::Matrix3d S;
    skew_symmetric_matrix(S, x1);
    std::cout << "S: " << S << std::endl;
    std::cout << "cross product x1 x2 " <<(S*x2).transpose()<< std::endl;

    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "--------------------------MATRIX -------------------------------" << std::endl;
    Eigen::MatrixXd mat1;
    mat1.resize(5,5);
    mat1.setRandom();
    std::cout << "mat1: " << mat1 << std::endl;
    std::cout <<"the second row is: " << mat1.row(1) << std::endl;
    std::cout <<"the mat(1,2) is: " << mat1(1,2) << std::endl;
    std::cout <<"the mat(1,2) is: " << mat1.coeff(1,2) << std::endl;
    //cast mat to float matrix
    Eigen::MatrixXf mat2 = mat1.cast<float>();
    //matrix transpose
    std::cout << "mat1 transpose: " << mat1.transpose() << std::endl;
    //compute rank of mat1
    std::cout << "rank of mat1: " << mat1.colPivHouseholderQr().rank() << std::endl;
    //if rank is equal than max of cols and rows, then the matrix is full rank

    if(mat1.colPivHouseholderQr().rank() == std::max(mat1.rows(), mat1.cols()))
        std::cout << "mat1 is full rank" << std::endl;
    else
        std::cout << "mat1 is not full rank" << std::endl;

    //determine if mat1 is invertible
    if(mat1.colPivHouseholderQr().isInvertible())
        std::cout << "mat1 is invertible" << std::endl;
    else
        std::cout << "mat1 is not invertible" << std::endl;

    //inverse of mat1
    std::cout << "inverse of mat1: " << mat1.colPivHouseholderQr().inverse() << std::endl;
    //compute determine of mat1
    std::cout << "determine of mat1: " << mat1.determinant() << std::endl;

    std::cout << "sum of mat1: " << mat1.sum() << std::endl;
    //max of mat1
    std::cout << "max of mat1: " << mat1.maxCoeff() << std::endl;
    //min of mat1
    std::cout << "min of mat1: " << mat1.minCoeff() << std::endl;
    //mean of mat1
    std::cout << "mean of mat1: " << mat1.mean() << std::endl;
    //sum of each column
    std::cout << "sum of each column: " << mat1.colwise().sum().transpose() << std::endl;
    //find eigenvalues, vectors and eigenvectors of mat1
    Eigen::EigenSolver<Eigen::MatrixXd> es(mat1);
    std::cout << "eigenvalues: " << es.eigenvalues().transpose() << std::endl;
    std::cout << "eigenvectors: " << es.eigenvectors().transpose() << std::endl;

    //create a matrix with size 50x50
    Eigen::MatrixXd mat2d(50,50);
    mat2d.setRandom();
    //vector 50x1
    Eigen::VectorXd vec(50);
    vec.setRandom();
    //solve mat2d*X = vec
     tt.tic();
     auto Xn = mat2d.inverse()*vec;
     //tt.toc_ms();
     std::cout << "time NN: " << tt.toc_us() << std::endl;
     //std::cout << "Xn: " << Xn.transpose() << std::endl;

     tt.tic();
     auto Xn2 = mat2d.colPivHouseholderQr().solve(vec);
     std::cout << "time QR: " << tt.toc_us() << std::endl;

     tt.tic();
     //solve using Cholesky dltl
     auto Xn3 = mat2d.ldlt().solve(vec);
     std::cout << "time Cholesky: " << tt.toc_us() << std::endl;

     std::cout << "-------------------------------QUATERNION---------------------------------" << std::endl;

     //create a unit quaternion
     Eigen::Quaterniond q1(1,1,2,1);
     //normalize quaternion
     q1.normalize();

     //print quaternion
     std::cout << "q: " << q1.coeffs().transpose() << std::endl;

     //create unit quaternion q2
     Eigen::Quaterniond q2(1,1.8,2,2.6);
     q2.normalize();

     //q1 multiplied by q2
     auto qm = q1*q2;
     std::cout << "q1*q2: " << qm.coeffs().transpose() << std::endl;

     //conjugate of q1
     auto qc = q1.conjugate();
     //q1 multiplied by qc
     auto qm2 = q1*qc;
     std::cout << "q1*qc: " << qm2.coeffs().w() << std::endl;

     //inverse of q2
     auto qi = q2.inverse();

     Eigen::Vector3d v1(1,2,3);
     //rotate v1 by q2
     auto vr = q2.toRotationMatrix()*v1;
     std::cout << "rotated v1: " << vr.transpose() << std::endl;
     //convert vector v1 to quaternion
     auto qv = Eigen::Quaterniond(0, v1[0], v1[1], v1[2]);
     //qv.normalize();
     //rotate v1 by qv, Check the result with vr
     auto vr2 = q2*qv*q2.inverse();
     std::cout << "rotated v1: " << vr2.coeffs().transpose()<< std::endl;

     //quaternion to angle axis
     Eigen::AngleAxisd aa(q2);
     //print aa
     std::cout << "aa:axis  " << aa.axis().transpose() << " angle " << aa.angle() << std::endl;

    return 0;

}

