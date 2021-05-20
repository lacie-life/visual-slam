#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {
    // Forward declare
    struct MapPoint;
    struct Feature;
    
    /**
     * Frame
     * Each frame is assigned an independent id, and the key frame is assigned a key frame ID
     */
     
    struct Frame
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            typedef std::shared_ptr<Frame> Ptr;

            unsigned long id_ = 0;           // id of this frame
            unsigned long keyframe_id_ = 0;  // id of key frame
            bool is_keyframe_ = false;       // Is it a key frame
            double time_stamp_;              // Timestamp, not used temporarily
            SE3 pose_;                       // Tcw pose form
            std::mutex pose_mutex_;          // pose data lock
            cv::Mat left_img_, right_img_;   // stereo images

             // extracted features in left image
            std::vector<std::shared_ptr<Feature>> features_left_;
            // corresponding features in right image, set to nullptr if no corresponding
            std::vector<std::shared_ptr<Feature>> features_right_;

        public:  // data members
            Frame() {}

            Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
                const Mat &right);

            // set and get pose, thread safe
            SE3 Pose() {
                std::unique_lock<std::mutex> lck(pose_mutex_);
                return pose_;
            }

            void SetPose(const SE3 &pose) {
                std::unique_lock<std::mutex> lck(pose_mutex_);
                pose_ = pose;
            }

            /// Set the key frame and assign the key frame id
            void SetKeyFrame();

            /// Factory construction mode, assign id
            static std::shared_ptr<Frame> CreateFrame();
    };
    
}

#endif