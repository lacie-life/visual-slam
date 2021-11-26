/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include <pangolin/pangolin.h>

#include <mutex>
#include <opencv2/opencv.hpp>

#include "Eigen/Dense"
#include <Eigen/Geometry>

namespace ORB_SLAM2
{

class MapPoint;
class MapObject;
class Map;
class KeyFrame;

class MapDrawer
{
public:
    MapDrawer(Map *pMap, const std::string &strSettingPath);

    Map *mpMap;

    void DrawMapPoints();
    void DrawMapCuboids();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    Eigen::MatrixXd truth_poses; // n*3 xyz pose

private:
    std::vector<Eigen::Vector3f> box_colors;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

    Eigen::MatrixXd all_edge_pt_ids; // for object drawing
    Eigen::MatrixXd front_edge_pt_ids;
};

} // namespace ORB_SLAM2

#endif // MAPDRAWER_H
