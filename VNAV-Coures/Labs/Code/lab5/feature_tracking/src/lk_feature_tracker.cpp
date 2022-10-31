#include "lk_feature_tracker.h"

#include <numeric>
#include <vector>

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <ros/ros.h>

using namespace cv;
using namespace cv::xfeatures2d;

/**
   LK feature tracker Constructor.
*/
LKFeatureTracker::LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

LKFeatureTracker::~LKFeatureTracker() {
  // Feel free to modify if you want!
  cv::destroyWindow(window_name_);
}

/** TODO This is the main tracking function, given two images, it detects,
 * describes and matches features.
 * We will be modifying this function incrementally to plot different figures
 * and compute different statistics.
 @param[in] frame Current image frame
*/
void LKFeatureTracker::trackFeatures(const cv::Mat& frame) {

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  DELIVERABLE 7 | Feature Tracking: Lucas-Kanade Tracker
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // For this part, you will need to:
  //
  //   1. Using OpenCV’s documentation and the C++ API for the LK tracker, track
  //   features for the video sequences we provided you by using the Harris
  //   corner detector (like here). Show the feature tracks at a given frame
  //   extracted when using the Harris corners (consider using the 'show'
  //   function below)
  //
  //   Hint 1: take a look at cv::goodFeaturesToTrack and cv::calcOpticalFlowPyrLK
  //
  //   2. Add an extra entry in the table you made previously for the Harris +
  //   LK tracker
  //
  //   Note: LKFeatureTracker does not inherit from the base tracker like other
  //   feature trackers, so you need to also implement the statistics gathering
  //   code right here.
  //
  // ~~~~ begin solution
  //

  // Read first frame and find corners in it
  if(prev_frame_.empty()||prev_corners_.size()==0){
    prev_frame_=frame;
    cv::cvtColor(prev_frame_, greyMat_prev_, CV_BGR2GRAY);
    goodFeaturesToTrack(greyMat_prev_, prev_corners_, 100, 0.3, 7, Mat(), 7, false, 0.04);
    // Create a mask image for drawing purposes
    mask = Mat::zeros(prev_frame_.size(), prev_frame_.type());
    return;
  }
  //clean up the mask
  if(counter==10){
    mask = Mat::zeros(prev_frame_.size(), prev_frame_.type());
    counter = 0;
  } 

  ROS_INFO_STREAM("number of corners to track " << prev_corners_.size());
  std::vector<uchar> status;
  std::vector<float> err;

  std::vector< cv::Point2f > corners;
  cv::Mat greyMat;
  cv::cvtColor(frame, greyMat, CV_BGR2GRAY);

  goodFeaturesToTrack(greyMat_prev_, prev_corners_, 100, 0.3, 7, Mat(), 7, false, 0.04);
  cv::calcOpticalFlowPyrLK(greyMat_prev_, greyMat, prev_corners_, corners, status, err);
  std::vector< cv::Point2f > good_curr_corners, good_prev_corners;
  for (uint i =0; i<prev_corners_.size(); i++){
    if(status[i]){
      good_prev_corners.push_back(prev_corners_[i]);
      good_curr_corners.push_back(corners[i]);
    }
  }
  show(frame, good_prev_corners, good_curr_corners);

  
  prev_frame_= frame;
  greyMat_prev_ = greyMat;
  counter ++;

  std::vector<uchar> inlier_mask;
  inlierMaskComputation(good_curr_corners, good_prev_corners, &inlier_mask);

  unsigned int num_inliers = 0;
  for (auto& inlier : inlier_mask)
    if (inlier == 1)
      num_inliers ++;

  //   Calculate the statistics to fill the table in the handout.

  float const new_num_samples = static_cast<float>(num_samples_) + 1.0f;
  float const old_num_samples = static_cast<float>(num_samples_);
  avg_num_keypoints_img1_ = (avg_num_keypoints_img1_ * old_num_samples + static_cast<float>(corners.size())) / new_num_samples;
  avg_num_keypoints_img2_ = (avg_num_keypoints_img2_ * old_num_samples + static_cast<float>(prev_corners_.size())) / new_num_samples;
  avg_num_matches_ = (avg_num_matches_ * old_num_samples + static_cast<float>(good_curr_corners.size())) / new_num_samples;
  avg_num_inliers_ = (avg_num_inliers_ * old_num_samples + static_cast<float>(num_inliers)) / new_num_samples;
  avg_inlier_ratio_ =
      (avg_inlier_ratio_ * old_num_samples + (static_cast<float>(num_inliers) / static_cast<float>(good_curr_corners.size()))) / new_num_samples;
  ++num_samples_;

  //
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                             end deliverable 7
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

}

/** TODO Display image with tracked features from prev to curr on the image
 * corresponding to 'frame'
 * @param[in] frame The current image frame, to draw the feature track on
 * @param[in] prev The previous set of keypoints
 * @param[in] curr The set of keypoints for the current frame
 */
void LKFeatureTracker::show(const cv::Mat& frame, std::vector<cv::Point2f>& prev,
                            std::vector<cv::Point2f>& curr) {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // ~~~~ begin solution
  //

  std::vector< cv::Point2f > good_corners = std::vector< cv::Point2f >();
  for (uint i =0; i<prev.size(); i++){
    line(
            mask,                        // Draw onto this image
            prev[i],                 // Starting here
            curr[i],                 // Ending here
            cv::Scalar(0, 255, 0),       // This color
            2,                           // This many pixels wide
            cv::FILLED                  // Draw line in this style
        );
    circle( frame, curr[i], 2, cv::Scalar(0, 0, 255), 2, cv::FILLED);
    
  }
  
  // Display the result
  cv::Mat img;
  add(frame, mask, img);
  imshow(window_name_, img);
  waitKey(10);

  //
  //     Hint: look at cv::line and cv::cirle functions.
  //     Hint 2: use imshow to display the image
  //
  // ~~~~ end solution
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

/** Compute Inlier Mask out of the given matched keypoints.
 @param[in] pts1 List of keypoints detected on the first image.
 @param[in] pts2 List of keypoints detected on the second image.
 @param[out] inlier_mask Mask indicating inliers (1) from outliers (0).
*/
void LKFeatureTracker::inlierMaskComputation(const std::vector<cv::Point2f>& pts1,
                                             const std::vector<cv::Point2f>& pts2,
                                             std::vector<uchar>* inlier_mask) const {
  CHECK_NOTNULL(inlier_mask);

  static constexpr double max_dist_from_epi_line_in_px = 3.0;
  static constexpr double confidence_prob = 0.99;
  try {
    findFundamentalMat(pts1, pts2, CV_FM_RANSAC,
                       max_dist_from_epi_line_in_px, confidence_prob,
                       *inlier_mask);
  } catch(...) {
    ROS_WARN("Inlier Mask could not be computed, this can happen if there"
             "are not enough features tracked.");
  }
}

void LKFeatureTracker::printStats() const {
  std::cout << "Avg. Keypoints 1 Size: " << avg_num_keypoints_img1_ << std::endl;
  std::cout << "Avg. Keypoints 2 Size: " << avg_num_keypoints_img2_ << std::endl;
  std::cout << "Avg. Number of matches: " << avg_num_matches_ << std::endl;
  std::cout << "Avg. Number of Inliers: " << avg_num_inliers_ << std::endl;
  std::cout << "Avg. Inliers ratio: " << avg_inlier_ratio_ << std::endl;
  std::cout << "Num. of samples: " << num_samples_ << std::endl;
}
