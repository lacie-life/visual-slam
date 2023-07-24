//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <pangolin/pangolin.h>
#include <unistd.h>
#include <open3d/Open3D.h>


typedef std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud);

int main(int argc, char **argv) {

    //create vector of colorImgs and depthImgs
    std::vector<cv::Mat> colorImgs;
    std::vector<cv::Mat> depthImgs;
    //create trajectory
    TrajectoryType poses;
    //read pose.txt file
    std::ifstream ifs(argv[1]);
    //check is file is open
    if (!ifs.is_open()) {
        std::cerr << "File is not open" << std::endl;
        return -1;
    }
    //read file
    while (!ifs.eof()) {
        //create data[7]
        double data[7] = {0};
        //read file
        ifs >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
        //create pose
        Sophus::SE3d pose(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2]));
        //push pose to poses
        poses.push_back(pose);
    }
    //close file
    ifs.close();
    //show trajectory size
    std::cout << "poses size: " << poses.size() << std::endl;

    //read colorImg and depthImg
    for (int i = 0; i < poses.size(); i++) {
        //create colorImg and depthImg
        cv::Mat colorImg, depthImg;
        //read colorImg
        colorImg = cv::imread(argv[2] +
                              std::to_string(i + 1) + ".png");
        //read depthImg
        depthImg = cv::imread(argv[2] +
                              std::to_string(i + 1) + ".pgm", -1);

        //check is colorImg and depthImg is empty
        if (colorImg.empty() || depthImg.empty()) {
            std::cerr << "colorImg or depthImg is empty" << std::endl;
            return -1;
        }

        colorImgs.push_back(colorImg);
        depthImgs.push_back(depthImg);
    }
    //size of colorImgs and depthImgs
    std::cout << "colorImgs size: " << colorImgs.size() << std::endl;
    std::cout << "depthImgs size: " << depthImgs.size() << std::endl;

    //show size of image[0]
    std::cout << "colorImgs[0].size(): " << colorImgs[0].size() << std::endl;

    //set intrinsic parameters
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0; // to convert from millimeters to meters

    //create point-cloud
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    //pointcloud.reserve(colorImgs.size()*colorImgs[0].rows*colorImgs[0].cols);
    pointcloud.reserve(1000000);

    //compute pointcloud
    for (auto i = 0; i < 5; i++) {
        std::cout << "pose: " << i + 1 << std::endl;
        //create colorImg and depthImg
        cv::Mat colorImg = colorImgs[i];
        cv::Mat depthImg = depthImgs[i];
        //transform se3
        Sophus::SE3d T = poses[i];
        //create pointcloud
        for (auto v = 0; v < colorImg.rows; v++) {
            for (auto u = 0; u < colorImg.cols; u++) {
                //get depth at u, v
                double d = depthImg.ptr<unsigned short>(v)[u]; // depth in millimeters < 65535
                if (d == 0) continue; // no depth
                //create point
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale; // depth in meters
                point[0] = (u - cx) * point[2] / fx; // x in meters, u = x * fx / z + cx
                point[1] = (v - cy) * point[2] / fy; // y in meters, v = y * fy / z + cy
                //transform point
                Eigen::Vector3d pointW = T * point; // transform point from camera frame to world frame

                Vector6d p;
                p.head(3) = pointW; // point coordinate in world frame
                //get color BGR at u, v
                p[5] = colorImg.data[v * colorImg.step + u * colorImg.channels() + 0]; // blue
                p[4] = colorImg.data[v * colorImg.step + u * colorImg.channels() + 1]; // green
                p[3] = colorImg.data[v * colorImg.step + u * colorImg.channels() + 2]; // red
                pointcloud.push_back(p);

            }
        }

    }
    //show pointcloud size
    std::cout << "point-cloud size: " << pointcloud.size() << std::endl;
    showPointCloud(pointcloud); //using pangolin

    open3d::geometry::PointCloud pcd;
    //copy pointcloud to pcd from eigen vector
    for (auto & i : pointcloud) {
        pcd.points_.emplace_back(i.head(3));
        pcd.colors_.emplace_back(i.tail(3)/255.0);
    }

    std::shared_ptr<open3d::geometry::PointCloud> pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(pcd);
    //rotation matrix
    //rotate point cloud
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1.0, 0.0, 0.0,
            0, -1, 0,
            0, 0, -1;

    pcd_ptr->Rotate(rotation_matrix, Eigen::Vector3d(0, 0, 1));


    //draw point cloud
    open3d::visualization::DrawGeometries({pcd_ptr}, "Point Cloud"); //open3d

    return 0;
}

void showPointCloud(const std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        std::cerr << "Point cloud is empty!" << std::endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}