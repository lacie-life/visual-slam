//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include "tictoc.h"

int main(int argc, char** argv){
    TicToc tt;

    //read image color
    cv::Mat img_1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img_2 = cv::imread(argv[2], cv::IMREAD_COLOR);

    //check images are loaded
    if(!img_1.data || !img_2.data){
        std::cout << "Image is not loaded" << std::endl;
        return -1;
    }
    //access pixel at rows = 20, cols = 50
    //std::cout << "Pixel at row = 20, col = 50 is " << img_1.at<cv::Vec3b>(20, 50) << std::endl;

    //print the size of images
    std::cout << "Size of image 1: " << img_1.size() << std::endl;
    std::cout << "Size of image 2: " << img_2.size() << std::endl;
    //channel of image
    std::cout << "Channel of image 1: " << img_1.channels() << std::endl;
    std::cout << "Channel of image 2: " << img_2.channels() << std::endl;
    //print the type of images
    std::cout << "Type of image 1: " << img_1.type() << std::endl;
    std::cout << "Type of image 2: " << img_2.type() << std::endl;
    // detect ORB features and compute descriptors
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    std::vector<cv::KeyPoint> keypoints_1, keypoints_2;
    cv::Mat descriptors_1, descriptors_2;
    tt.tic();
    orb->detectAndCompute(img_1, cv::noArray(), keypoints_1, descriptors_1);
    //tt.toc_ms();
    std::cout << "Time of ORB detection and compute: " << tt.toc_ms() << " ms" << std::endl;
    orb->detectAndCompute(img_2, cv::noArray(), keypoints_2, descriptors_2);
    //print the number of keypoints
    std::cout << "Number of keypoints in image 1: " << keypoints_1.size() << std::endl;
    std::cout << "Number of keypoints in image 2: " << keypoints_2.size() << std::endl;
    //draw keypoints
    cv::Mat img_keypoints_1;
    cv::drawKeypoints(img_1, keypoints_1, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("Keypoints in image 1", img_keypoints_1);
    cv::waitKey(0);

    //match descriptors using brute force matcher
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector<cv::DMatch> matches;
    tt.tic();
    matcher.match(descriptors_1, descriptors_2, matches);
    //tt.toc_ms();
    std::cout << "Time of matching: " << tt.toc_us() << " us" << std::endl;

    //print the number of matches
    std::cout << "Number of matches: " << matches.size() << std::endl;
    //draw matches
    cv::Mat img_matches;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
    cv::imshow("Matches", img_matches);
    cv::waitKey(0);

    //find the max and min distance using minmax_element function
    double min_dist, max_dist;
    std::vector<cv::DMatch>::iterator min_it, max_it;
    tt.tic();
    std::tie(min_it, max_it) = std::minmax_element(matches.begin(), matches.end(),
                                                   [](const cv::DMatch& m1, const cv::DMatch& m2){
                                                       return m1.distance < m2.distance;
                                                   });
    //tt.toc_ms();
    std::cout << "Time of finding min and max distance: " << tt.toc_us() << " us" << std::endl;
    min_dist = min_it->distance;
    max_dist = max_it->distance;

    //print the max and min distance
    std::cout << "Min distance: " << min_dist << std::endl;
    std::cout << "Max distance: " << max_dist << std::endl;
    //remove matches with distance < max (2*min_dist, 30)
    std::vector<cv::DMatch> good_matches;
    auto mmd = std::max(2*min_dist, 30.0);
    for(auto m: matches){
        if(m.distance < mmd){
            good_matches.push_back(m);
        }
    }
    //find second keypoints in first image that correspond to second image
    /*
    std::vector<cv::Point2f> points_1, points_2;
    for(auto m: good_matches){
        points_1.push_back(keypoints_1[m.queryIdx].pt);
        points_2.push_back(keypoints_2[m.trainIdx].pt);
    }
     */

    //print the number of good matches
    std::cout << "Number of good matches: " << good_matches.size() << std::endl;
    //draw good matches
    cv::Mat img_good_matches;
    cv::drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_good_matches);
    cv::imshow("Good matches", img_good_matches);
    cv::waitKey(0);


    return 0;
}
