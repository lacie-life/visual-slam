//
// Created by lacie on 24/08/2023.
//

#ifndef SIMPLESFM_SCENEGRAPH_H
#define SIMPLESFM_SCENEGRAPH_H

#include "Utils/Types.h"
#include "Utils/Database.h"

namespace SimpleSfM
{

    /**
     * SceneGraph is used to quickly
     * (1) Find each picture has multiple observation points (num_observations)
     * (2) Find how many corresponding points each picture has with all other pictures (num_correspondences)
     * (3) find matching points between two images
     * (4) All matching points of a certain feature point of an image (distributed on multiple images)
     */

    class SceneGraph
    {
    public:
        struct Correspondence
        {
            Correspondence() : image_id(INVALID), point2D_idx(INVALID) {}
            Correspondence(const image_t image_id, point2D_t point2D_idx)
                    : image_id(image_id), point2D_idx(point2D_idx) {}
            image_t image_id;
            point2D_t point2D_idx;
        };

        SceneGraph() {}

        void Load(const cv::Ptr<Database> database, const size_t min_num_matches);
        void Finalize();

        size_t NumImages() const;

        /**
         * Determine whether the image_id exists in SceneGraph
         * @param image_id : image id
         * @return : exists, returns true; otherwise, returns false.
         */
        bool ExistsImage(const image_t image_id) const;

        /**
         * Find how many observations each image has (num_observations)
         * @param image_id : image id
         * @return : num_observations
         */
        point2D_t NumObservationsForImage(image_t image_id) const;

        /**
         * Find how many correspondences each picture has with all other pictures (num_correspondences)
         * @param image_id : image id
         * @return : num_correspondences
         */
        point2D_t NumCorrespondencesForImage(image_t image_id) const;

        /**
         * Find how many corresponding points there are in total between the two images
         * @param image_id1 : image id1
         * @param image_id2 : image id2
         * @return : The number of matching feature points between image_id1 and image_id2
         */
        point2D_t NumCorrespondencesBetweenImages(const image_t image_id1, const image_t image_id2) const;

        /**
         * Add image to SceneGraph
         * @param image_id : image id
         * @param num_points2D : The number of feature points of the image
         */
        void AddImage(const image_t image_id, const size_t num_points2D);

        /**
         * Add feature point matching of two images to SceneGraph
         * have to be aware of is,
         * (1) The AddImage function has been called, and image_id1 and image_id2 have been added to SceneGraph
         * (2) matches are cross-validated (when calculating matches, they are verified by default)
         * @param image_id1 : the id of the first image
         * @param image_id2 : the id of the second image
         * @param matches : cross-validated feature matches
         */
        void AddCorrespondences(const image_t image_id1,
                                const image_t image_id2,
                                const std::vector<cv::DMatch> &matches);

        /**
         * Find all matching points of the point2D_idx feature point of the image_id1 image (distributed on multiple images)
         * @param image_id : image id
         * @param point2D_idx : feature point serial number
         * @return : Return all matching points of this feature point on other images
         */
        const std::vector<typename SceneGraph::Correspondence> FindCorrespondences(
                const image_t image_id, const point2D_t point2D_idx) const;

        /**
         * Get the match between two images
         * @param image_id1 : the id of the first image
         * @param image_id2 : the id of the second image
         * @return : returns the match between the two images
         */
        std::vector<cv::DMatch> FindCorrespondencesBetweenImages(
                const image_t image_id1, const image_t image_id2) const;

        /**
         * Determine whether there is a matching point in the point2D_idx feature point of the image_id1 image
         * @param image_id : image id
         * @param point2D_idx : feature point serial number
         * @return : If there is a matching point, return true; otherwise, return false.
         */
        bool HasCorrespondences(const image_t image_id, const point2D_t point2D_idx) const;

        /**
         * Determine whether the point2D_idx feature point of the image_id1 image is TwoViewObservation
         * The meaning of TwoViewObservation is:
         * The point2D_idx feature point of the image_id1 image only matches the feature point of one image (set to corr_image_id and corr_point2D_idx)
         * And the corr_point2D_idx feature point of the corr_image_id picture only matches the point2D_idx feature point of the image_id1 picture
         * The Track generated by TwoViewObservation has only two elements
         * Cannot provide 2D-3D correspondence for subsequent pictures
         *
         * @param image_id : image id
         * @param point2D_idx : feature point serial number
         * @return : If it is TwoViewObservation, return true; otherwise, return false.
         */
        bool IsTwoViewObservation(const image_t image_id, const point2D_t point2D_idx) const;

        std::vector<image_t> GetAllImageIds() const;

        const std::unordered_map<image_pair_t, point2D_t> ImagePairs();

    private:
        struct Image
        {
            // num_observations indicates how many points can find matching points among the feature points of the image
            point2D_t num_observations = 0;
            // Accumulate the matching points with other images, how many matching points in total
            point2D_t num_correspondences = 0;

            /// corrs[i] Indicates which feature points of the i-th feature point are matching points with other pictures
            std::vector<std::vector<Correspondence>> corrs;
        };

        // nodes of the scene graph
        std::unordered_map<image_t, typename SceneGraph::Image> images_;

        // The number of valid matching points between image pairs
        std::unordered_map<image_pair_t, point2D_t> image_pairs_;
    };

}

#endif //SIMPLESFM_SCENEGRAPH_H
