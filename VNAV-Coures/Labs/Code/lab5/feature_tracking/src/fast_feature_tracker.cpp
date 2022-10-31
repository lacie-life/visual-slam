// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  16.485 - Fall 2019  - Lab 5 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement an ORB feature tracker derived class
//  that inherits from your FeatureTracker base class.
//
// NOTE: Deliverables for the TEAM portion of this assignment start at number 3
// and end at number 7. If you have completed the parts labeled Deliverable 3-7,
// you are done with the TEAM portion of the lab. Deliverables 1-2 are
// individual.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  DELIVERABLE 6 (continued) | Comparing Feature Matching on Real Data
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
// For this part, you will need to implement the same functions you've just
// implemented in the case of SIFT, SURF, and ORB, but now for FAST+BRIEF.
//
// NOTE: We do not provide much template code for this section or the corresponding
// header file (fast_feature_tracker.h), but you should define and implement
// the same set of functions as for SIFT and SURF (see the template code for
// those cases for reference as needed)
//
// NOTE: Don't forget to modify the CMakeLists.txt to add these files!
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
#include "fast_feature_tracker.h"
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace cv::xfeatures2d;

FastFeatureTracker::FastFeatureTracker()
    : detector(FastFeatureDetector::create()) {}

void FastFeatureTracker::detectKeypoints(const cv::Mat &img,
                                         std::vector<cv::KeyPoint> *keypoints) const {
// ~~~~ begin solution
//
this->detector->detect(img, *keypoints);
//
// ~~~~ end solution
}

void FastFeatureTracker::describeKeypoints(const cv::Mat &img,
                                           std::vector<cv::KeyPoint> *keypoints,
                                           cv::Mat *descriptors) const {
// ~~~~ begin solution
//
cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> descriptor = cv::xfeatures2d::BriefDescriptorExtractor::create();
descriptor-> compute(img, *keypoints, *descriptors);
//
// ~~~~ end solution
}

void FastFeatureTracker::matchDescriptors(const cv::Mat &descriptors_1,
                                          const cv::Mat &descriptors_2,
                                          std::vector<std::vector<cv::DMatch>> *matches,
                                          std::vector<cv::DMatch> *good_matches) const {
// ~~~~ begin solution
//
FlannBasedMatcher matcher(new flann::LshIndexParams(20, 10, 2));
matcher.knnMatch(descriptors_1,descriptors_2, *matches, 2);

const float ratio_thresh = 0.8f;

for (auto& match : *matches){
  if(match.size()==1 || match[0].distance< ratio_thresh * match[1].distance)
    good_matches->push_back(match[0]);
}
//
// ~~~~ end solution
}
