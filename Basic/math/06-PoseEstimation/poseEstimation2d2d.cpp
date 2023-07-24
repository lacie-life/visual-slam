//
// Created by lacie-life on 2023/24/07.   
//

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <random>

void find_and_match_keypoints(const cv::Mat &img1, const cv::Mat &img2,
                              std::vector<cv::KeyPoint> &keypoints1,
                              std::vector<cv::KeyPoint> &keypoints2,
                              cv::Mat &des1, cv::Mat &des2,
                              std::vector<cv::DMatch> &matches) {

    //detect keypoints and compute descriptors
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    detector->detectAndCompute(img1, cv::noArray(), keypoints1, des1);
    detector->detectAndCompute(img2, cv::noArray(), keypoints2, des2);
    //match descriptors
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    std::vector<cv::DMatch> matches_temp;
    matcher->match(des1, des2, matches_temp);
    //filter matches
    double max_dist = 0;
    double min_dist = 1000;
    for (int i = 0; i < des1.rows; i++) {
        double dist = matches_temp[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    for (int i = 0; i < des1.rows; i++) {
        if (matches_temp[i].distance <= fmax(2 * min_dist, 30)) {
            matches.push_back(matches_temp[i]);
        }
    }

}

//create function to calculate the fundamental matrix, essential matrix and pose
void pose_estimation_2d2d(const std::vector<cv::KeyPoint> &keypoints1,
                          const std::vector<cv::KeyPoint> &keypoints2,
                          const std::vector<cv::DMatch> &matches,
                          cv::Mat &R, cv::Mat &t,
                          cv::Mat& fundamental_matrix) {
    //create two sets of points
    std::vector<cv::Point2f> points1, points2;
    for (auto match: matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    //compute fundamental matrix with 8-point algorithm with RANSAC
    fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC);
    //print fundamental matrix
    std::cout << "Fundamental matrix: " << std::endl << fundamental_matrix << std::endl;
    //define intrinsic camera matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    //compute essential matrix cv::findEssentialMat(points1, points2, K, cv::RANSAC, 0.999, 1.0);
    //cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, K, cv::RANSAC, 0.999, 1.0);
    //camera principal point
    cv::Point2d pp(325.1, 249.7);
    //camera focal length
    double f = 521.0;
    //compute essential matrix with the focal length
    //cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, f, pp, cv::RANSAC, 0.999, 1.0);
    //compute essential matrix with K
    cv::Mat essential_matrix = cv::findEssentialMat(points1, points2, K, cv::RANSAC, 0.999, 1.0);
    //print essential matrix
    std::cout << "Essential matrix: " << std::endl << essential_matrix << std::endl;
    //compute homography matrix
    cv::Mat homography_matrix = cv::findHomography(points1, points2, cv::RANSAC, 3);
    //print homography matrix
    std::cout << "Homography matrix: " << std::endl << homography_matrix << std::endl;
    //compute pose from essential matrix
    cv::recoverPose(essential_matrix, points1, points2, R, t, f);
    //print rotation matrix
    std::cout << "Rotation matrix: " << std::endl << R << std::endl;
    //print translation matrix
    std::cout << "Translation matrix: " << std::endl << t << std::endl;

}

//function pixel to camera coordinate
// u = cx + fx*x
// v = cy + fy*y
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K) {
    return {
            (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
            (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    };
}


int main(int argc, char **argv) {

    cv::Mat img1 = cv::imread(argv[1],
                              cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(argv[2],
                              cv::IMREAD_COLOR);
    //check images are loaded
    if (!img1.data || !img2.data) {
        std::cout << "Could not open or find the image" << std::endl;
        return -1;
    } else {
        //size of image
        std::cout << "size of image 1: " << img1.size() << std::endl;
        std::cout << "size of image 2: " << img2.size() << std::endl;
    }
    //create a function to detect keypoints and descriptors and match them
    std::vector<cv::KeyPoint> keypoints1, keypoints2;
    cv::Mat descriptors1, descriptors2;
    std::vector<cv::DMatch> matches;



    find_and_match_keypoints(img1, img2, keypoints1, keypoints2, descriptors1, descriptors2, matches);


    //show keypoints and matches size
    std::cout << "number of keypoints in image 1: " << keypoints1.size() << std::endl;
    std::cout << "number of keypoints in image 2: " << keypoints2.size() << std::endl;
    std::cout << "number of matches: " << matches.size() << std::endl;

    //compute pose
    cv::Mat R, t;
    cv::Mat fundamental_matrix;
    pose_estimation_2d2d(keypoints1, keypoints2, matches, R, t, fundamental_matrix);

    //compute fundamental matrix = t^R
    //define t_skew
    cv::Mat t_skew = (cv::Mat_<double>(3, 3) << 0, -t.at<double>(2), t.at<double>(1),
            t.at<double>(2), 0, -t.at<double>(0),
            -t.at<double>(1), t.at<double>(0), 0);
    //compute essential matrix
    cv::Mat essential_matrix = t_skew * R;
    //check essential matrix
    std::cout << "essential matrix: " << std::endl << essential_matrix << std::endl;


    //define intrinsic camera matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    //check epipolar constraint
    for (const auto &match: matches) {
        //get keypoints
        cv::Point2f p1 = keypoints1[match.queryIdx].pt;
        cv::Point2f p2 = keypoints2[match.trainIdx].pt;
        //convert to camera coordinate
        cv::Point2d p1_cam = pixel2cam(p1, K);
        cv::Point2d p2_cam = pixel2cam(p2, K);
        //transform p1_cam to homogeneous coordinates
        cv::Mat p1_cam_h = (cv::Mat_<double>(3, 1) << p1_cam.x, p1_cam.y, 1);
        //transform p2_cam to homogeneous coordinates
        cv::Mat p2_cam_h = (cv::Mat_<double>(3, 1) << p2_cam.x, p2_cam.y, 1);
        //compute epipolar constraint
        cv::Mat d = p2_cam_h.t() * essential_matrix * p1_cam_h;
        //print epipolar constraint
        std::cout << "epipolar constraint: " << d << std::endl;

    }
    //compute epipolar lines
    std::vector<cv::Vec3f> epilines1, epilines2;
    std::vector<cv::Point2f> points1, points2;
    for(auto match: matches){
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    cv::computeCorrespondEpilines(points2, 2, fundamental_matrix, epilines1);
    cv::computeCorrespondEpilines(points1, 1, fundamental_matrix, epilines2);
    //draw epipolar lines
    cv::Mat img_epi_lines1, img_epi_lines2;
    img_epi_lines1 = img1.clone();
    img_epi_lines2 = img2.clone();
    //draw epipolar lines with cofficients ax+by+c=0
    //r = lines{i};
    //p1 = uint32([0, -r(3)/r(2)]);
    //p2 = uint32([c, -(r(3)+r(1)*c)/r(2)]);
    //draw line
    for(int i=0; i<epilines1.size(); i++){
        cv::Point p1, p2;
        p1.x = 0;
        p1.y = -epilines1[i][2]/epilines1[i][1];
        p2.x = img1.cols;
        p2.y = -(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1];
        //define random color random_device
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 255);
        cv::Scalar color(dis(gen), dis(gen), dis(gen));

        //draw line and points
        cv::line(img_epi_lines1, p1, p2, color, 1);
        cv::circle(img_epi_lines1, points1[i], 5, color, -1);
        }
    for(int i=0; i<epilines2.size(); i++){
        cv::Point p1, p2;
        p1.x = 0;
        p1.y = -epilines2[i][2]/epilines2[i][1];
        p2.x = img2.cols;
        p2.y = -(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1];

        //define random int number without cv
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 255);
        cv::Scalar color = cv::Scalar(dis(gen), dis(gen), dis(gen));


        //draw line and points
        cv::line(img_epi_lines2, p1, p2, color, 1);
        cv::circle(img_epi_lines2, points2[i], 5, color, -1);
        }

    //show epipolar lines
    cv::imshow("epipolar lines 1", img_epi_lines1);
    cv::imshow("epipolar lines 2", img_epi_lines2);
    cv::waitKey(0);


    //draw matches
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
    cv::imshow("matches", img_matches);
    cv::waitKey(0);


    //draw descriptors1
    cv::imshow("img_matches", descriptors1);
    cv::waitKey(0);

}


