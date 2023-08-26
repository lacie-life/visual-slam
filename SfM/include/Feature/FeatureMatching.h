//
// Created by lacie on 20/08/2023.
//

#ifndef SIMPLESFM_FEATUREMATCHING_H
#define SIMPLESFM_FEATUREMATCHING_H

#include <string>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "Utils/Types.h"
#include "Utils/Database.h"

namespace SimpleSfM
{
    class FeatureMatcher
    {
    public:
        /**
         * @param database_path         :   database path
         * @param max_num_matches       :   maximum number of matches
         * @param max_distance          :   The maximum distance between feature descriptions.
         *                                  If it exceeds, it is not considered to be a correct match
         *                                  (the feature descriptor has been normalized when matching is required)
         * @param distance_ratio        :   1NN < 2NN * distance_ratio Then it is considered a correct match
         * @param cross_check           :   Whether to enable cross-validation
         */
        FeatureMatcher(const std::string &database_path,
                       const int &max_num_matches = 10240,
                       const double &max_distance = 0.7,
                       const double &distance_ratio = 0.8,
                       const bool &cross_check = true)
                : database_path_(database_path),
                  max_num_matches_(max_num_matches),
                  max_distance_(max_distance),
                  distance_ratio_(distance_ratio),
                  cross_check_(cross_check) {}

        /**
         * @brief MatchImagePairs   ：   Match the given image pair and store the matching result in the database
         * @param image_pairs       ：   image pair
         */
        void MatchImagePairs(const std::vector<std::pair<image_t, image_t>> &image_pairs);
        virtual void RunMatching() = 0;

    protected:
        std::string database_path_;
        int max_num_matches_;
        double max_distance_;
        double distance_ratio_;
        bool cross_check_;
        cv::Ptr<Database> database_;
    };

    class SequentialFeatureMatcher : public FeatureMatcher
    {
    public:
        /**
         * @param database_path : database path
         * @param overlap : Each picture is matched with the previous pictures
         * @param max_num_matches : maximum number of matches
         * @param max_distance : The maximum distance between feature descriptions,
         *                       if it exceeds, it is not considered a correct match
         *                       (the feature descriptor has been normalized when matching is required)
         * @param distance_ratio : 1NN < 2NN * distance_ratio is considered a correct match
         * @param cross_check : Whether to enable cross-validation
         */
        SequentialFeatureMatcher(const std::string &database_path,
                                 const int &overlap = 3,
                                 const int &max_num_matches = 10240,
                                 const double &max_distance = 0.7,
                                 const double &distance_ratio = 0.8,
                                 const bool &cross_check = true)
                : FeatureMatcher(database_path, max_num_matches, max_distance, distance_ratio, cross_check),
                  overlap_(overlap) {}
        void RunMatching();

    private:
        int overlap_;
    };

    class BruteFeatureMatcher : public FeatureMatcher
    {
    public:
        /**
         * @param database_path : database path
         * @param max_pairs_size : Up to max_pairs_size for simultaneous loading of image pairs in memory
         * @param max_num_matches : maximum number of matches
         * @param max_distance : The maximum distance between feature descriptions,
         *                       if it exceeds, it is not considered a correct match
         *                       (the feature descriptor has been normalized when matching is required)
         * @param distance_ratio : 1NN < 2NN * distance_ratio is considered a correct match
         * @param cross_check : Whether to enable cross-validation
         */
        BruteFeatureMatcher(const std::string &database_path,
                            const int &max_pairs_size = 100,
                            const bool &is_preemtive = true,
                            const int &preemtive_num_features = 100,
                            const int &preemtive_min_num_matches = 4,
                            const int &max_num_matches = 10240,
                            const double &max_distance = 0.7,
                            const double &distance_ratio = 0.8,
                            const bool &cross_check = true)
                : FeatureMatcher(database_path, max_num_matches, max_distance, distance_ratio, cross_check),
                  max_pairs_size_(max_pairs_size),
                  is_preemtive_(is_preemtive),
                  preemtive_num_features_(preemtive_num_features),
                  preemtive_min_num_matches_(preemtive_min_num_matches) {}
        void RunMatching();

    private:
        /**
         * Wu C. Towards Linear-Time Incremental Structure from Motion[C]//
         * International Conference on 3d Vision. IEEE Computer Society, 2013:127-134.
         *
         * @brief PreemptivelyFilterImagePairs  :   Use preemptive matching to filter out impossible image pairs
         * @param image_pairs                   :   image pair
         */
        std::vector<std::pair<image_t, image_t>> PreemptivelyFilterImagePairs(std::vector<std::pair<image_t, image_t>> image_pairs);

        cv::Mat GetTopScaleDescriptors(const image_t &image_id);
        bool HasTopScaleDescriptorsCache(const image_t &image_id);

        int max_pairs_size_;
        bool is_preemtive_;
        int preemtive_num_features_;
        int preemtive_min_num_matches_;

        std::unordered_map<image_t, cv::Mat> top_scale_descriptors_cache_;
    };

    // TODO : Use Vocab Tree for matching, mainly I don’t know how to train to get a general Vocab Tree
    class VocaburaryTreeFeatureMatcher : public FeatureMatcher
    {
    public:
        void RunMatching();
    };
}

#endif //SIMPLESFM_FEATUREMATCHING_H
