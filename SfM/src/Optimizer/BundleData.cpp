//
// Created by lacie on 27/08/2023.
//

#include "Optimizer/BundleData.h"
#include "SfM/Projection.h"

namespace SimpleSfM
{
    double BundleData::Debug()
    {
        double sum_residuals = 0;
        double num = 0;
        for (auto &landmark_el : landmarks)
        {
            const Landmark &landmark = landmark_el.second;
            const cv::Vec3d &point3D = landmark.point3D;

            double temp_sum_error = 0;
            for (const Measurement &meas : landmark.measurements)
            {
                const image_t &image_id = meas.image_id;
                const cv::Vec2d &point2D = meas.point2D;

                cv::Mat R;

                cv::Rodrigues(camera_poses[image_id].rvec, R);
                const cv::Mat &t = camera_poses[image_id].tvec;

                double error = Projection::CalculateReprojectionError(point3D, point2D, R, t, K);
                temp_sum_error += error;
            }
            sum_residuals += temp_sum_error / landmark.measurements.size();
            num += 1;
        }

        return sum_residuals / num;
    }
}