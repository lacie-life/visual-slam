//
// Created by lacie on 23/08/2023.
//

#ifndef SIMPLESFM_REGISTRANT_H
#define SIMPLESFM_REGISTRANT_H

#include <cstddef>
#include <vector>
#include <opencv2/opencv.hpp>

namespace SimpleSfM
{
    class Registrant
    {
    public:
        enum class PnPMethod
        {
            P3P = 0,
            AP3P = 1,
            EPNP = 2,
            UPNP = 3
        };
        struct Parameters
        {
            size_t abs_pose_min_num_inliers = 15;           // After PnP, the number corresponding to 2D-3D is greater than the threshold, it is considered successful
            PnPMethod pnp_method = PnPMethod::EPNP;         // PnP method
            size_t abs_pose_num_iterative_optimize = 10000; //
            double abs_pose_ransac_confidence = 0.9999;     // PnP ransac confidence
            double abs_pose_max_error = 4.0;                // In PnP, the points whose error is less than the threshold are considered as interior points
        };

        struct Statistics
        {
            bool is_succeed = false;
            size_t num_point2D_3D_correspondences = 0;
            size_t num_inliers = 0;

            double ave_residual = 0;

            cv::Mat R;
            cv::Mat t;
            std::vector<double> residuals;
            std::vector<bool> inlier_mask;
        };

    public:
        Registrant(const Parameters &params, const cv::Mat &K);

        /**
         * For the incoming 3D-2D point correspondence, use the PnP algorithm to solve the pose
         */
        Statistics Register(const std::vector<cv::Vec3d> &points3D,
                            const std::vector<cv::Vec2d> &points2D);

        void PrintStatistics(const Statistics &statistics);

    private:
        Parameters params_;
        Statistics statistics_;

        cv::Mat K_;
    };

}

#endif //SIMPLESFM_REGISTRANT_H
