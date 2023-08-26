//
// Created by lacie on 26/08/2023.
//

#ifndef SIMPLESFM_INITIALIZER_H
#define SIMPLESFM_INITIALIZER_H

#include <cstddef>
#include <vector>
#include <opencv2/opencv.hpp>

namespace SimpleSfM
{

    class Initializer
    {
    public:
        struct Parameters
        {

            size_t rel_pose_min_num_inlier = 100; // 2D-2D The threshold of the number of inliers corresponding to a point

            double rel_pose_ransac_confidence = 0.9999; // Confidence of ransac when seeking matrix (H, E)

            double rel_pose_essential_error = 4.0; // Find the error threshold of the solution matrix E

            double rel_pose_homography_error = 12.0; // Find the error threshold of solving matrix H

            double init_tri_max_error = 2.0; // When triangulating, the reprojection error threshold

            double init_tri_min_angle = 4.0; // When triangulating, the angle threshold
        };
        struct Statistics
        {
            bool is_succeed = false;     // Whether the initialization is successful
            std::string method = "None"; // What method was used for initialization
            std::string fail_reason = "None";

            size_t num_inliers_H = 0; // When estimating the homography matrix, the number of interior points that fit the homography matrix
            size_t num_inliers_F = 0; // When estimating the fundamental matrix, the number of inliers that fit the fundamental matrix
            double H_F_ratio = 0;     // The number of interior points of the homography matrix divided by the number of interior points of the fundamental matrix

            size_t num_inliers = 0;          // Number of 3D points successfully triangulated (reprojection error less than threshold)
            double median_tri_angle = 0;     // Median of 3D point angles for successful triangulation
            double ave_tri_angle = 0;        // Average of 3D point angles for successful triangulation
            double ave_residual = 0;         // average reprojection error
            cv::Mat R1;                      // Rotation matrix 1 (identity matrix)
            cv::Mat t1;                      // translation vector 1 (zero vector)
            cv::Mat R2;                      // Rotation Matrix 2
            cv::Mat t2;                      // translation vector 2
            std::vector<cv::Vec3d> points3D; // 3D points measured by all 2D points, including inlier and outlier
            std::vector<double> tri_angles;  // The angle of each 3D point
            std::vector<double> residuals;   // Reprojection error for each 3D point
            std::vector<bool> inlier_mask;   // Mark which 3D points are interior points
        };

    public:
        Initializer(const Parameters &params, const cv::Mat &K);

        /**
         * For the aligned feature points of the incoming two images, try to initialize
         * Return initialized statistics
         */
        Statistics Initialize(const std::vector<cv::Vec2d> &points2D1,
                              const std::vector<cv::Vec2d> &points2D2);

        void PrintStatistics(const Statistics &statistics);

    private:
        /**
         * Use RANSAC to find the homography matrix H that satisfies the correspondence between the two sets of points2D1 and points2D2
         * @param points2D1 : point set 1
         * @param points2D2 : point set 2
         * @param H : [output] homography matrix
         * @param inlier_mask : [output] mark which is the inner point
         * @param num_inliers : [output] how many inliers there are
         */
        void FindHomography(const std::vector<cv::Vec2d> &points2D1,
                            const std::vector<cv::Vec2d> &points2D2,
                            cv::Mat &H,
                            std::vector<bool> &inlier_mask,
                            size_t &num_inliers);

        /**
         * Use RANSAC to find the fundamental matrix F that satisfies the correspondence between the two sets of points2D1 and points2D2
         * @param points2D1 : point set 1
         * @param points2D2 : point set 2
         * @param H : [output] Fundamental matrix
         * @param inlier_mask : [output] mark which is the inner point
         * @param num_inliers : [output] how many inliers there are
         */
        void FindFundanmental(const std::vector<cv::Vec2d> &points2D1,
                              const std::vector<cv::Vec2d> &points2D2,
                              cv::Mat &F,
                              std::vector<bool> &inlier_mask,
                              size_t &num_inliers);

        /**
         * Decompose the homography matrix H to obtain the initial pose, and perform triangulation and save it in statistics_
         * @param H : homography matrix
         * @param points2D1 : point set 1
         * @param points2D2 : point set 2
         * @param inlier_mask_H : The inlier_mask obtained when calling FindHomography
         * @return : true, initialized successfully; false, failed
         */
        bool RecoverPoseFromHomography(const cv::Mat &H,
                                       const std::vector<cv::Vec2d> &points2D1,
                                       const std::vector<cv::Vec2d> &points2D2,
                                       const std::vector<bool> &inlier_mask_H);

        /**
         * Decompose the basic matrix F (actually decompose the essential matrix E) to obtain the initial pose, and perform triangulation and save it in statistics_
         * @param F : homography matrix
         * @param points2D1 : point set 1
         * @param points2D2 : point set 2
         * @param inlier_mask_F : The inlier_mask obtained when calling FindFundanmental
         * @return : true, initialized successfully; false, failed
         */
        bool RecoverPoseFromFundanmental(const cv::Mat &F,
                                         const std::vector<cv::Vec2d> &points2D1,
                                         const std::vector<cv::Vec2d> &points2D2,
                                         const std::vector<bool> &inlier_mask_F);

        cv::Vec3d Triangulate(const cv::Mat &P1,
                              const cv::Mat &P2,
                              const cv::Vec2d &point2D1,
                              const cv::Vec2d &point2D2);

        /**
         * According to statistics_, get the information of initialization failure
         * [note] : only when statistics_.is_succeed == false
         *          and statistics_.num_inliers
         *             statistics_.median_tri_angle
         *             statistics_.ave_tri_angle
         *             statistics_.ave_residual
         *          This function can only be called when it is not empty
         */
        std::string GetFailReason();

        //    bool CheckCheirality(std::vector<cv::Vec3d>& points3D,
        //                         const std::vector<cv::Vec2d>& points2D1,
        //                         const std::vector<cv::Vec2d>& points2D2,
        //                         const cv::Mat& R1,
        //                         const cv::Mat& t1,
        //                         const cv::Mat& R2,
        //                         const cv::Mat& t2);

    private:
        Parameters params_;
        Statistics statistics_;

        cv::Mat K_;
    };

}

#endif //SIMPLESFM_INITIALIZER_H
