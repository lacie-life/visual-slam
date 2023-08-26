//
// Created by lacie on 23/08/2023.
//

#include "SfM/Registrant.h"
#include "SfM/Projection.h"
#include "Utils/Utils.h"

namespace SimpleSfM
{
    Registrant::Registrant(const Parameters &params, const cv::Mat &K)
            : params_(params), K_(K)
    {
        assert(K_.type() == CV_64F);
    }

    Registrant::Statistics Registrant::Register(const std::vector<cv::Vec3d> &points3D,
                                                const std::vector<cv::Vec2d> &points2D)
    {
        assert(points2D.size() == points3D.size());

        statistics_.is_succeed = false;
        statistics_.num_point2D_3D_correspondences = points2D.size();
        statistics_.num_inliers = 0;
        statistics_.ave_residual = 0;

        if (points2D.size() < params_.abs_pose_min_num_inliers)
        {
            return statistics_;
        }
        cv::Mat inlier_idxs;
        cv::Mat rvec;
        cv::Mat t;

        /// PnP solve pose
        /// It should be noted that the inlier_idxs output of solvePnPRansac
        /// inlier_idx = inlier_idxs[i] means inlier_idx points are inlier points,
        /// The value of the mask at this time is no longer 0 and 1
        bool kUseExtrinsicGuess = false;

        // TODO: Find out whether the running time of solvePnPRansac has anything to do with abs_pose_num_iterative_optimize
        // TODO: The parameter abs_pose_num_iterative_optimize only works when cv::SOLVEPNP_ITERATIVE is selected
        switch (params_.pnp_method)
        {
            case PnPMethod::P3P:
                cv::solvePnPRansac(Utils::Vector3dToPoint3f(points3D), Utils::Vector2dToPoint2f(points2D),
                                   K_, cv::Mat(), rvec, t, kUseExtrinsicGuess,
                                   params_.abs_pose_num_iterative_optimize, params_.abs_pose_max_error,
                                   params_.abs_pose_ransac_confidence, inlier_idxs, cv::SOLVEPNP_P3P);
                break;
            case PnPMethod::AP3P:
                cv::solvePnPRansac(Utils::Vector3dToPoint3f(points3D), Utils::Vector2dToPoint2f(points2D),
                                   K_, cv::Mat(), rvec, t, kUseExtrinsicGuess,
                                   params_.abs_pose_num_iterative_optimize, params_.abs_pose_max_error,
                                   params_.abs_pose_ransac_confidence, inlier_idxs, cv::SOLVEPNP_AP3P);
                break;
            case PnPMethod::EPNP:
                cv::solvePnPRansac(Utils::Vector3dToPoint3f(points3D), Utils::Vector2dToPoint2f(points2D),
                                   K_, cv::Mat(), rvec, t, kUseExtrinsicGuess,
                                   params_.abs_pose_num_iterative_optimize, params_.abs_pose_max_error,
                                   params_.abs_pose_ransac_confidence, inlier_idxs, cv::SOLVEPNP_UPNP);
                break;
            case PnPMethod::UPNP:
                cv::solvePnPRansac(Utils::Vector3dToPoint3f(points3D), Utils::Vector2dToPoint2f(points2D),
                                   K_, cv::Mat(), rvec, t, kUseExtrinsicGuess,
                                   params_.abs_pose_num_iterative_optimize, params_.abs_pose_max_error,
                                   params_.abs_pose_ransac_confidence, inlier_idxs, cv::SOLVEPNP_UPNP);
                break;
            default:
                assert(false);
        }

        // The inlier_idxs.rows here does not represent the number of real 2D-3D inlier
        // This is because since a 2D point has multiple matching points, these matching points may correspond to different 3D points
        // As a result, when passing 2D-2D-3D, the same 2D point corresponds to multiple 3D points
        // So if there are multiple identical 2D points in inlier_idxs, only one can be kept
        // So inlier_idxs.rows does not represent the number of real 2D-3D inlier
        // TODO : Improvement: Get the number of real inliers
        if (inlier_idxs.rows < params_.abs_pose_min_num_inliers)
        {
            return statistics_;
        }

        std::vector<bool> inlier_mask(points2D.size(), false);

        for (int i = 0; i < inlier_idxs.rows; ++i)
        {
            int idx = inlier_idxs.at<int>(i, 0);
            inlier_mask[idx] = true;
        }

        assert(rvec.type() == CV_64F);
        assert(t.type() == CV_64F);

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        double sum_residual = 0;
        std::vector<double> residuals(inlier_mask.size());
        for (size_t i = 0; i < inlier_mask.size(); ++i)
        {
            double error = Projection::CalculateReprojectionError(points3D[i], points2D[i], R, t, K_);
            if (!inlier_mask[i])
                continue;
            sum_residual += error;
            residuals[i] = error;
        }

        statistics_.is_succeed = true;
        statistics_.num_inliers = static_cast<size_t>(inlier_idxs.rows);
        statistics_.ave_residual = sum_residual / statistics_.num_inliers;

        statistics_.R = R;
        statistics_.t = t;
        statistics_.residuals = std::move(residuals);
        statistics_.inlier_mask = std::move(inlier_mask);

        return statistics_;
    }

    void Registrant::PrintStatistics(const Statistics &statistics)
    {
        const size_t kWidth = 30;
        std::cout.flags(std::ios::left);
        std::cout << std::endl;
        std::cout << "--------------- Register Summary Start ---------------" << std::endl;
        std::cout << std::setw(kWidth) << "Initialize status"
                  << " : " << (statistics.is_succeed ? "true" : "false") << std::endl;
        std::cout << std::setw(kWidth) << "Num 2D 3D correspondences"
                  << " : " << statistics.num_point2D_3D_correspondences << std::endl;
        std::cout << std::setw(kWidth) << "Num inliers "
                  << " : " << statistics.num_inliers << std::endl;
        std::cout << std::setw(kWidth) << "Ave residual "
                  << " : " << statistics.ave_residual << std::endl;
        std::cout << "--------------- Register Summary End ---------------" << std::endl;
        std::cout << std::endl;
    }
}



