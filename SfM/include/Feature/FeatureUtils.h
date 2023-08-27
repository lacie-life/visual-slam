//
// Created by lacie on 21/08/2023.
//

#ifndef SIMPLESFM_FEATUREUTILS_H
#define SIMPLESFM_FEATUREUTILS_H

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

namespace SimpleSfM {

    class FeatureUtils
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////
        // Feature
        ////////////////////////////////////////////////////////////////////////////////
        /**
         * Calculate the SIFT feature points of the image, and the descriptor
         *
         * @param image : image
         * @param pts : [output] extracted feature points
         * @param desc : [output] extracted feature descriptor
         */
        static void ExtractFeature(const cv::Mat &image,
                                   std::vector<cv::KeyPoint> &kpts,
                                   cv::Mat &desc,
                                   int num_features = 8024);

        ////////////////////////////////////////////////////////////////////////////////
        // Feature
        ////////////////////////////////////////////////////////////////////////////////
        /**
         * Calculate the SIFTGPU feature points of the image, and the descriptor
         *
         * @param image : image
         * @param pts : [output] extracted feature points
         * @param desc : [output] extracted feature descriptor
         */
        static void ExtractFeatureGPU(const cv::Mat &image,
                                   std::vector<cv::KeyPoint> &kpts,
                                   cv::Mat &desc,
                                   int num_features = 8024);

        /**
         * Sort the feature points according to the scale from large to small,
         * and extract the num_features feature points with the first scale
         *
         * @param kpts : feature points
         * @param num_features : Extract num_features feature points with the first scale
         * @param top_scale_kpts : [output] num_features feature points before the scale
         */
        static void ExtractTopScaleKeyPoints(const std::vector<cv::KeyPoint> kpts,
                                             const int &num_features,
                                             std::vector<cv::KeyPoint> &top_scale_kpts);

        /**
         * Sort the feature points according to the scale from large to small,
         * and extract the feature descriptors of the num_features feature points with the first scale
         *
         * @param kpts : Feature points
         * @param descriptors : feature descriptor
         * @param num_features : Extract num_features feature points with the first scale
         * @param top_scale_descriptors : [output] Feature descriptors corresponding to the num_features feature points at the top of the scale
         */
        static void ExtractTopScaleDescriptors(const std::vector<cv::KeyPoint> kpts,
                                               const cv::Mat &descriptors,
                                               const int &num_features,
                                               cv::Mat &top_scale_descriptors);

        /**
         * Remove distortion of feature points
         * @param image : image
         * @param K : camera memory
         * @param dist_coef : camera distortion parameter
         * @param pts : Feature points
         * @param undistort_pts : [output] Feature points after removing distortion.
         *                        Note that pts.size() >= undistort.pts.size()
         */
        static void UndistortFeature(const cv::Mat &image,
                                     const cv::Mat &K,
                                     const cv::Mat &dist_coef,
                                     const std::vector<cv::Point2f> &pts,
                                     std::vector<cv::Point2f> &undistort_pts,
                                     std::vector<size_t> &index);

        ////////////////////////////////////////////////////////////////////////////////
        // Matches
        ////////////////////////////////////////////////////////////////////////////////
        /**
         * Calculate feature point matching
         * 1NN < distance_ratio * 2NN matches will be retained
         * @param desc1 : Feature descriptor on the first image
         * @param desc2 : The feature descriptor on the second image
         * @param matches : [output] Feature matching of the first image and the second image
         * @param distance_ratio : the distance threshold between 1NN and 2NN
         */
        static void ComputeMatches(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<cv::DMatch> &matches, const float distance_ratio = 0.8);

        /**
         * Calculate the matching of feature points
         * 1NN < distance_ratio * 2NN matches will be retained, and cross-validation will be turned on
         * If the i-th feature point of the first image matches the j-th feature point of the second image
         * Then the j-th feature point of the second image matches the i-th feature point of the first image
         * @param desc1 : Feature descriptor on the first image
         * @param desc2 : The feature descriptor on the second image
         * @param matches : [output] Feature matching of the first image and the second image
         * @param distance_ratio : the distance threshold between 1NN and 2NN
         */
        static void ComputeCrossMatches(const cv::Mat &desc1, const cv::Mat &desc2, std::vector<cv::DMatch> &matches, const float distance_ratio = 0.8);

        /**
         * Use the fundamental matrix to filter feature matches
         * @param pts1 : Feature points on the first image
         * @param pts2 : Feature points on the second image
         * @param matches : feature matching of the first image and the second image
         * @param prune_matches : [output] Feature matching after using RANSAC + fundamental matrix purification
         */
        static void FilterMatches(const std::vector<cv::Point2f> &pts1,
                                  const std::vector<cv::Point2f> &pts2,
                                  const std::vector<cv::DMatch> &matches,
                                  std::vector<cv::DMatch> &prune_matches);

        /**
         * Use the maximum distance to filter the matches.
         * If the distance between feature descriptors is greater than max_distance, it is not considered a correct match.
         * It is required that the feature descriptor has been L1Root normalized or L2 normalized
         * @param matches : matches to be filtered
         * @param prune_matches : [output] filtered matches
         * @param max_distance : maximum distance
         */
        static void FilterMatchesByDistance(const std::vector<cv::DMatch> &matches,
                                            std::vector<cv::DMatch> &prune_matches,
                                            const double &max_distance = 0.7);

        /**
         * show matches
         * @param image1 : the first image
         * @param image2 : the second image
         * @param pts1 : Feature points on the first image
         * @param pts2 : Feature points on the second image
         * @param matches : feature matching of the first image and the second image
         * @param window_name : Display the name of the image window
         * @param duration : duration
         */

        static void ShowMatches(const cv::Mat &image1,
                                const cv::Mat &image2,
                                const std::vector<cv::Point2f> &pts1,
                                const std::vector<cv::Point2f> &pts2,
                                const std::vector<cv::DMatch> &matches,
                                const std::string &window_name,
                                const time_t duration = 1000);

        /**
         * show matches
         * @param image1 : the name of the first image
         * @param image2 : the name of the second image
         * @param pts1 : Feature points on the first image
         * @param pts2 : Feature points on the second image
         * @param matches : feature matching of the first image and the second image
         * @param window_name : Display the name of the image window
         * @param duration : duration
         */
        static void ShowMatches(const std::string &image_name1,
                                const std::string &image_name2,
                                const std::vector<cv::Point2f> &pts1,
                                const std::vector<cv::Point2f> &pts2,
                                const std::vector<cv::DMatch> &matches,
                                const std::string &window_name,
                                const time_t duration = 1000);
        /**
         * According to the matching relationship, get the feature points after alignment
         * That is, aligned_pts1[i] and aligned_pts2[i] are matching points
         * @param pts1 : Feature points on the first image
         * @param pts2 : Feature points on the second image
         * @param matches : feature matching of the first image and the second image
         * @param aligned_pts1 : [output] The feature points of the first image after alignment
         * @param aligned_pts2 : [output] The feature points of the second image after alignment
         */
        static void GetAlignedPointsFromMatches(const std::vector<cv::Point2f> &pts1,
                                                const std::vector<cv::Point2f> &pts2,
                                                const std::vector<cv::DMatch> &matches,
                                                std::vector<cv::Point2f> &aligned_pts1,
                                                std::vector<cv::Point2f> &aligned_pts2);

    private:
        /**
         * Cross-validate the matching to get the matching relationship after cross-validation
         * @param matches12 : matches between the first image and the second image
         * @param matches21 : matches between the second image and the first image
         * @param prune_matches : [output] The matches between the first image and the second image after proposed
         */
        static void CrossCheck(const std::vector<cv::DMatch> &matches12,
                               const std::vector<cv::DMatch> &matches21,
                               std::vector<cv::DMatch> &prune_matches);
    };

}


#endif //SIMPLESFM_FEATUREUTILS_H
