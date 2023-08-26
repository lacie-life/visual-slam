//
// Created by lacie on 20/08/2023.
//

#ifndef SIMPLESFM_FEATUREEXTRACTION_H
#define SIMPLESFM_FEATUREEXTRACTION_H

#include <iostream>
#include <string>

namespace SimpleSfM {
    class FeatureExtractor
    {
    public:
        /// Normalization method of feature descriptor
        enum class Normalization
        {
            L1_ROOT,
            L2,
            ROOT_SIFT
        };

    public:
        /**
         * @param database_path         :	database path
         * @param images_path           :   The folder where the images are located
         * @param max_image_size        :   The maximum image size, if it exceeds this size,
         *                                  the maximum size of the image will be reduced to max_image_size,
         *                                  and then feature extraction will be performed on the downsampled image
         * @param max_num_features      :   The maximum number of extracted feature points
         * @param normalization         :   Normalization method of feature descriptor
         */
        FeatureExtractor(const std::string &database_path,
                         const std::string &images_path,
                         const int &max_image_size = 3200,
                         const int &max_num_features = 10240,
                         const Normalization &normalization = Normalization::L1_ROOT)
                : database_path_(database_path),
                  images_path_(images_path),
                  max_image_size_(max_image_size),
                  max_num_features_(max_num_features),
                  normalization_(normalization) {}

        virtual void RunExtraction() = 0;

    protected:
        std::string database_path_;
        std::string images_path_;
        int max_image_size_;
        int max_num_features_;
        Normalization normalization_;
    };

    class FeatureExtractorCPU : public FeatureExtractor
    {
    public:
        using FeatureExtractor::FeatureExtractor;
        void RunExtraction();
    };

    // TODO : Feature extraction on GPU using SiftGPU/SuftGPU
    class FeatureExtractorGPU : public FeatureExtractor
    {
    public:
        using FeatureExtractor::FeatureExtractor;
        virtual void RunExtraction() = 0;
    };
}

#endif //SIMPLESFM_FEATUREEXTRACTION_H
