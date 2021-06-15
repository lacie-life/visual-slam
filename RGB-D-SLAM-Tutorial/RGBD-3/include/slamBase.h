//
// Created by nguyen on 15/5/2021.
//

/************************************************* ************************
> File Name: rgbd-slam-tutorial-gx/part III/code/include/slamBase.h
> Author: xiang gao
> Mail: gaoxiang12@mails.tsinghua.edu.cn
> Created Time: Saturday, July 18, 2015 15:14:22
    > Description: Basic functions used in rgbd-slam tutorial (C style)
 ************************************************** **********************/
# pragma once

// Various header files
// C++ Standard Library
#include <fstream>
#include <vector>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Type definition
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// Camera internal parameter structure
struct CAMERA_INTRINSIC_PARAMETERS
{
    double cx, cy, fx, fy, scale;
};

// Function interface
// image2PonitCloud converts rgb image to point cloud
PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d converts a single point from image coordinates to space coordinates
// input: 3-dimensional point Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );
