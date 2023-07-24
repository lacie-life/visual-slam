//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

int main(int argc, char **argv) {
    cv::Mat image = cv::imread(
            argv[1], CV_8UC1);
    //check if image is loaded fine
    if (!image.data) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

    //radial distortion parameters
    double k1 = -0.28340811, k2 = 0.07395907, p1 = 0.00019359, p2 = 1.76187114e-05;
    //intrinsic parameters 458.654, 457.296, 367.215, 248.375
    double fx = 458.654, fy = 457.296, cx = 367.215, cy = 248.375;

    //create the distortion parameters
    double k11 = -0.38340811, k12 = 0.09395907, k13 = 0.00059359, k14 = 6.76187114e-05;

    int rows = image.rows;
    int cols = image.cols;

    //create image_undistorted
    cv::Mat image_undistorted(rows, cols, CV_8UC1);

    // the process is shown as,
    // Point in world coordinates--> Point in camera coordinates
    // --> Point in normalized coordinates--> Point in distorted coordinates
    // --> point in image coordinates

    //compute the pixel in the undistorted image
    for (int v = 0; v < rows; v++) {
        for (int u = 0; u < cols; u++) {
            // transform the point from image coordinates to normalized coordinates
            double x = (u - cx) / fx; //x coordinate in the normalized plane
            double y = (v - cy) / fy; //y coordinate in the normalized plane

            //compute normalized coordinates to distorted coordinates
            //compute x_d and y_d
            double r2 = x * x + y * y; // compute the radius of the pixel in the normalized plane
            double x_d = x * (1 + k1 * r2 + k2 * r2 * r2) + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            double y_d = y * (1 + k1 * r2 + k2 * r2 * r2) + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

            //distorted coordinates to image coordinates, again
            //compute the pixel in the undistorted image, using cast
            int u_d = cvRound(fx * x_d + cx);
            int v_d = cvRound(fy * y_d + cy);
            //now, map the pixel in the undistorted image to the pixel in the distorted image
            //check if the point is in the image u_d and v_d
            if (u_d >= 0 && u_d < cols && v_d >= 0 && v_d < rows) {
                image_undistorted.at<uchar>(v, u) = image.at<uchar>((v_d), (u_d));
            } else {
                image_undistorted.at<uchar>(v, u) = 0;
            }
        }
    }
    //using OPENCV undistort function
    //undistorted image new
    cv::Mat image_undistorted_new(rows, cols, CV_8UC1);

    cv::Mat image_undistorted_new2(rows, cols, CV_8UC1);

    //camera matrix
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    //distortion coefficients
    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, 0);
    //print the camera matrix
    std::cout << "Camera Matrix: " << std::endl << cameraMatrix << std::endl;
    //print the distortion coefficients
    std::cout << "Distortion Coefficients: " << std::endl << distCoeffs << std::endl;

    //distortion coefficients
    cv::Mat distCoeffs_new = (cv::Mat_<double>(5, 1) << k11, k12, k13, k14, 0);


    //transform the image to undistorted image
    cv::undistort(image, image_undistorted_new, cameraMatrix, distCoeffs);
    //transfrom undistorted image to distorted image
    cv::undistort(image_undistorted_new, image_undistorted_new2, cameraMatrix, distCoeffs_new);

    //show the image
    // the result is shown in the same window using opencv function and the manual method
    cv::imshow("image_distorted", image);
    cv::imshow("image_undistorted", image_undistorted);
    cv::imshow("image_undistorted_new", image_undistorted_new);
    cv::imshow("image_undistorted_new2", image_undistorted_new2);

    cv::waitKey(0);


    return 0;
}
