//
// Created by lacie on 20/08/2023.
//

#ifndef SIMPLESFM_POINT3D_H
#define SIMPLESFM_POINT3D_H

#include <opencv2/opencv.hpp>

#include "SfM/Track.h"

namespace SimpleSfM
{
    class Point3D
    {
    public:
        Point3D();
        Point3D(const cv::Vec3d &xyz, const cv::Vec3b &color, const double &error = -1.0);

        const cv::Vec3d &XYZ() const;
        cv::Vec3d XYZ();
        void SetXYZ(const cv::Vec3d &xyz);

        const cv::Vec3b Color() const;
        cv::Vec3b Color();
        void SetColor(const cv::Vec3b &color);

        double Error() const;
        bool HasError() const;
        void SetError(const double &error);

        const class Track &Track() const;
        class Track Track();
        void SetTrack(const class Track &track);

        ////////////////////////////////////////////////////////////////////////////////
        // Adding elements to the track is actually calling the member function of the track
        ////////////////////////////////////////////////////////////////////////////////
        void AddElement(const TrackElement &element);
        void AddElement(const image_t image_id, const point2D_t point2D_idx);
        void AddElements(const std::vector<TrackElement> &elements);

        ////////////////////////////////////////////////////////////////////////////////
        // Delete elements from track, actually call the member function of track
        ////////////////////////////////////////////////////////////////////////////////
        void DeleteElement(const size_t idx);
        void DeleteElement(const TrackElement &element);
        void DeleteElement(const image_t image_id, const point2D_t point2D_idx);

    private:
        cv::Vec3d xyz_;
        cv::Vec3b color_;
        double error_;
        // track
        class Track track_;
    };
}

#endif //SIMPLESFM_POINT3D_H
