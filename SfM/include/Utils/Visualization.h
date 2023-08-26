//
// Created by lacie on 27/08/2023.
//

#ifndef SIMPLESFM_VISUALIZATION_H
#define SIMPLESFM_VISUALIZATION_H


#include <iostream>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

#include "SfM/Image.h"

namespace SimpleSfM
{
    /**
     *
     * Implementation principle:
     * The approach of ShowPointCloud and ShowCameras of AsyncVisualization is to copy the incoming data to its own members.
     * So there will be no data race issues
     *
     *
     * Usage:
     * AsyncVisualization async_visualization;
     * async_visualization.RunVisualizationThread();
     *
     * do something ...
     * async_visualization.ShowPointCloud(...);
     * async_visualization.ShowCameras(...);
     *
     * async_visualization.WaitForVisualizationThread();
     *
     */
    class AsyncVisualization
    {
    public:
        AsyncVisualization();

        void RunVisualizationThread();
        void WaitForVisualizationThread();
        void RunVisualizationOnly();

        void ShowPointCloud(std::vector<cv::Point3f> &point_cloud,
                            std::vector<cv::Vec3b> &colors);

        void ShowCameras(std::vector<cv::Mat> &Rs,
                         std::vector<cv::Mat> &ts);

        void Close();

    private:
        cv::viz::Viz3d window_;
        std::vector<cv::Point3f> point_cloud_;
        std::vector<cv::Vec3b> colors_;
        size_t point_cloud_change_count_ = 0;

        std::vector<cv::Mat> Rs_;
        std::vector<cv::Mat> ts_;

        size_t camera_count_ = 0;

        bool is_camera_update_ = false;
        bool is_point_cloud_update_ = false;

        std::thread *thread_ = nullptr;
    };
}

#endif //SIMPLESFM_VISUALIZATION_H
