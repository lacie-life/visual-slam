//
// Created by lacie on 20/08/2023.
//

#ifndef SIMPLESFM_POINT2D_H
#define SIMPLESFM_POINT2D_H

#include <opencv2/opencv.hpp>

#include "Utils/Types.h"

namespace SimpleSfM
{
    class Point2D
    {
    public:
        Point2D();
        Point2D(const cv::Vec2d &xy, const cv::Vec3b &color, const point3D_t &point3D_id = INVALID);

        const cv::Vec2d &XY() const;
        cv::Vec2d XY();
        void SetXY(const cv::Vec2d &xy);

        const cv::Vec3b &Color() const;
        cv::Vec3b Color();
        void SetColor(const cv::Vec3b &color);

        const point3D_t &Point3DId() const;
        point3D_t Point3DId();

        void SetPoint3D(const point3D_t &point3D_id);
        void ResetPoint3D();
        bool HasPoint3D() const;

    private:
        cv::Vec2d xy_;
        cv::Vec3b color_;
        point3D_t point3D_id_;
    };
}




#endif //SIMPLESFM_POINT2D_H
