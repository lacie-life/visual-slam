//
// Created by nguyen on 15/5/2021.
//

/************************************************* ************************
> File Name: src/slamBase.cpp
> Author: xiang gao
> Mail: gaoxiang12@mails.tsinghua.edu.cn
    > Implementation of slamBase.h
> Created Time: Saturday, July 18, 2015 15:31:49
 ************************************************** **********************/

#include "../include/slamBase.h"

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    PointCloud::Ptr cloud (new PointCloud );

    for (int m = 0; m <depth.rows; m++)
        for (int n=0; n <depth.cols; n++)
        {
            // Get the value at (m, n) in the depth map
            ushort d = depth.ptr<ushort>(m)[n];
            // d may have no value, if so, skip this point
            if (d == 0)
                continue;
            // If d has a value, add a point to the point cloud
            PointT p;

            // Calculate the space coordinates of this point
            p.z = double(d) / camera.scale;
            p.x = (n-camera.cx) * p.z / camera.fx;
            p.y = (m-camera.cy) * p.z / camera.fy;

            // Get its color from the rgb image
            // rgb is a three-channel BGR format picture, so get the colors in the following order
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // add p to the point cloud
            cloud->points.push_back( p );
        }
    // Set and save the point cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    cv::Point3f p; // 3D point
    p.z = double( point.z) / camera.scale;
    p.x = (point.x-camera.cx) * p.z / camera.fx;
    p.y = (point.y-camera.cy) * p.z / camera.fy;
    return p;
}