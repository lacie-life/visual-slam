//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

#include <open3d/Open3D.h>

void show3DpointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &point_cloud);


int main(int argc, char **argv) {
    // intrinsic parameters
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    double b = 0.573;

    //read left and right image
    cv::Mat left = cv::imread(argv[1],
                              CV_8UC1);
    cv::Mat right = cv::imread(argv[2],
                               CV_8UC1);
    //check if read successfully
    if (left.empty() || right.empty()) {
        std::cout << "Can't read image" << std::endl;
        return -1;
    }
    //check the size of left and right image
    if (left.size() != right.size()) {
        std::cout << "The size of left and right image is not equal" << std::endl;
        return -1;
    } else {
        std::cout << "left img size: " << left.size() << std::endl;
        std::cout << "right img size: " << right.size() << std::endl;
        //show the number channel of left and right image
        std::cout << "left img channel: " << left.channels() << std::endl;
        std::cout << "right img channel: " << right.channels() << std::endl;
    }
    //create disparity image
    //create stereo matcher left and right
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
    cv::Mat disparity_sgbm, disparity;

    //compute disparity_sgbm
    sgbm->compute(left, right, disparity_sgbm);
    //convert to disparity image
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0);

    //COMPUTE MAXIMUM AND MIN DEPTH
    double max_depth = fx * b / 10; //10 is the minimum disparity
    double min_depth = fx * b / 96; //96 is the maximum disparity

    //show max and min depth
    std::cout << "max depth [m]: " << max_depth << std::endl;
    std::cout << "min depth [m]: " << min_depth << std::endl;


    //create 3D point cloud
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> point_cloud;
    //compute 3D point cloud form disparity image
    for (int v = 0; v < left.rows; v++) {
        for (int u = 0; u < left.cols; u++) {
            //check if the pixel is valid
            if (disparity.at<float>(v, u) <= 10 || disparity.at<float>(v, u) >= 96)
                continue;

            //compute 3D point
            Eigen::Vector4d point(0, 0, 0, 0);
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));

            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;
            point[3] = (double(left.at<uchar>(v, u))) / 255.0;
            //print point[3]
            //std::cout << "point[3]: " << point[3] << std::endl;


            point_cloud.push_back(point);
        }
    }
    //imshow disparity image
    cv::imshow("left", left);
    cv::imshow("disparity", disparity / 96.0);
    cv::waitKey(0);
    //show 3D point cloud
    show3DpointCloud(point_cloud);

    open3d::geometry::PointCloud pcl_open3d;
    for (auto & p : point_cloud) {
        pcl_open3d.points_.emplace_back(p.head(3));
        pcl_open3d.colors_.emplace_back(Eigen::Vector3d(p[3], p[3], p[3]));
    }

    auto pcl_ptr = std::make_shared<open3d::geometry::PointCloud>(pcl_open3d);
    Eigen::Matrix3d rotation_matrix;
    //set rotation matrix to pitch 180
    rotation_matrix << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;

    pcl_ptr->Rotate(rotation_matrix, Eigen::Vector3d(0, 0, 1));
    //set viewpoint closer to the point cloud
    pcl_ptr->Translate(Eigen::Vector3d(0, 0, -5));

    open3d::visualization::DrawGeometries({pcl_ptr});


    return 0;
}

//create show 3D point cloud function
void show3DpointCloud(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &point_cloud) {

    //check if the point cloud is empty
    if (point_cloud.empty()) {
        std::cout << "Point cloud is empty" << std::endl;
        return;
    }
    //create pangolin viewer
    pangolin::CreateWindowAndBind("3D Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );
    pangolin::View &d_cam =
            pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0,
                                                -1024.0f / 768.0f).SetHandler(new pangolin::Handler3D(s_cam));


    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (const auto &point: point_cloud) {
            glColor3f(float(point[3]), float(point[3]), float(point[3]));
            glVertex3d(point[0], point[1], point[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }

}


