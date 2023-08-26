//
// Created by lacie on 23/08/2023.
//

#ifndef SIMPLESFM_TRIANGULATOR_H
#define SIMPLESFM_TRIANGULATOR_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace SimpleSfM
{
    class Triangulator
    {
    public:
        struct Parameters
        {
            double regis_tri_max_error = 2.0;
            double regis_tri_min_angle = 1.5;
        };

        struct Statistics
        {
            bool is_succeed = false;
            cv::Vec3d point3D;
            double ave_residual = 0;
            std::vector<double> residuals;
        };

        Triangulator(const Parameters &params, const cv::Mat &K);

        Statistics Triangulate(const std::vector<cv::Mat> &Rs,
                               const std::vector<cv::Mat> &ts,
                               const std::vector<cv::Vec2d> &points2D);

        cv::Vec3d TriangulateMultiviewPoint(const std::vector<cv::Mat> &Rs,
                                            const std::vector<cv::Mat> &ts,
                                            const std::vector<cv::Vec2d> &points2D);

    private:
        Parameters params_;
        Statistics statistics_;
        cv::Mat K_;
    };
}


#endif //SIMPLESFM_TRIANGULATOR_H
