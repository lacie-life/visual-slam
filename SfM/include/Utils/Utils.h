//
// Created by lacie on 20/08/2023.
//

#ifndef SIMPLESFM_UTILS_H
#define SIMPLESFM_UTILS_H

#include <vector>
#include <opencv2/opencv.hpp>

namespace SimpleSfM {
    class Utils {
    public:
        // points
        static std::vector<cv::Point2f> Vector2dToPoint2f(const std::vector<cv::Vec2d> &points2D);
        static std::vector<cv::Vec2d> Point2fToVector2d(const std::vector<cv::Point2f> &points2D);

        static std::vector<cv::Point3f> Vector3dToPoint3f(const std::vector<cv::Vec3d> &points3D);
        static std::vector<cv::Vec3d> Point3fToVector3d(const std::vector<cv::Point3f> &points3D);

        // files
        static cv::String UnionPath(const std::string &directory, const std::string &filename);

        static void SplitPath(const std::string &path, std::string &directory, std::string &filename);
    };
}
#endif //SIMPLESFM_UTILS_H
