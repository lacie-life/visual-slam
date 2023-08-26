//
// Created by lacie on 20/08/2023.
//

#ifndef SIMPLESFM_IMAGE_H
#define SIMPLESFM_IMAGE_H

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "Utils/Types.h"
#include "SfM/Point2D.h"

namespace SimpleSfM
{
    class Image
    {
    public:
        Image();
        Image(const image_t &image_id, const std::string &image_name);
        Image(const image_t &image_id,
              const std::string &image_name,
              const std::vector<Point2D> &points2D);

        const image_t &ImageId() const;
        image_t ImageId();
        void SetImageId(const image_t &image_id);

        const std::string &ImageName() const;
        std::string ImageName();
        void SetImageName(const std::string &image_name);

        point2D_t NumPoints2D() const;
        point2D_t NumPoints3D() const;

        const cv::Mat &Rotation() const;
        cv::Mat Rotation();
        void SetRotation(const cv::Mat &R);

        const cv::Mat &Translation() const;
        cv::Mat Translation();
        void SetTranslation(const cv::Mat &t);

        void SetPoints2D(const std::vector<Point2D> &points2D);
        const Point2D &GetPoint2D(const point2D_t &point2D_idx) const;
        Point2D GetPoint2D(const point2D_t &point2D_idx);

        ////////////////////////////////////////////////////////////////////////////////
        // Set the 3D point corresponding to the 2D point
        ////////////////////////////////////////////////////////////////////////////////
        void SetPoint2DForPoint3D(const point2D_t &point2D_idx,
                                  const point3D_t &point3D_idx);

        void ResetPoint2DForPoint3D(const point2D_t &point2D_idx);

        bool Point2DHasPoint3D(const point2D_t &point2D_idx);

    private:
        image_t image_id_;
        std::string image_name_;

        // Number of 2D points in the track
        // SetPoint2DForPoint3D and ResetPoint2DForPoint3D will update this variable, +1 or -1
        point2D_t num_points3D_;

        cv::Mat R_;
        cv::Mat t_;

        std::vector<Point2D> points2D_;
    };
}

#endif //SIMPLESFM_IMAGE_H
