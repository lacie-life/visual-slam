//
// Created by lacie-life on 2023/24/07.   
//

#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    //read image
    cv::Mat image = cv::imread(argv[1]);
    
    //check image is loaded or not
    if (image.empty()) {
        std::cout << "Can not load image" << std::endl;
        return -1;
    }

    //show w , h, channel
    std::cout << "w: " << image.cols << std::endl;
    std::cout << "h: " << image.rows << std::endl;
    std::cout << "channel: " << image.channels() << std::endl;

    //check image type
    if(image.type() == CV_8UC1) {
        std::cout << "image type: CV_8UC1" << std::endl;
    } else if(image.type() == CV_8UC3) {
        std::cout << "image type: CV_8UC3" << std::endl;
    } else {
        std::cout << "image type: unknown" << std::endl;
        return -1;
    }

    //access pixel at 100, 100, red channel
    std::cout << "red channel: " << (int)image.at<cv::Vec3b>(100, 100)[2] << std::endl;

    //clone image
    cv::Mat image2 = image.clone();
    //set block pixel 100, 100, size 20x20 to red
    cv::Rect rect(100, 100, 20, 20);
    image2(rect).setTo(cv::Scalar(0, 0, 255));

    //draw a triangle on image
    cv::Point pt1(100, 100);
    cv::Point pt2(200, 200);
    cv::Point pt3(300, 100);
    std::vector<cv::Point> points;
    points.push_back(pt1);
    points.push_back(pt2);
    points.push_back(pt3);
    cv::fillConvexPoly(image2, points, cv::Scalar(0, 255, 0));

    //draw a line on image p1(100, 300) -> p2(400, 500)
    cv::line(image2, cv::Point(100, 300), cv::Point(400, 500), cv::Scalar(0, 0, 255), 3);
    //add text on image
    cv::putText(image2, "this is an angle", cv::Point(100, 80), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 100, 255), 2);

    //show image
    cv::imshow("image pre", image);
    cv::imshow("image af", image2);
    cv::waitKey(0);

    return 0;
}

