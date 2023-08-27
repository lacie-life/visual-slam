#include <iostream>
#include <opencv2/opencv.hpp>

#include "Utils/Timer.h"
#include "Utils/Database.h"
#include "Feature/FeatureMatching.h"

using namespace std;
using namespace cv;
using namespace SimpleSfM;

int main(int argc, char **argv)
{

    // Step 2 : Match the pictures and store them in the database

    if (argc != 2)
    {
        std::cout << "You need specify the YAML file path!" << std::endl;
        exit(-1);
    }

    cv::FileStorage fs(argv[1], FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout << "YAML file : " << argv[1] << " can't not open!" << std::endl;
        exit(-1);
    }

    string database_path;
    int match_type = 1;
    double max_distance = 0.7;
    double distance_ratio = 0.8;
    bool cross_check = true;

    fs["database_path"] >> database_path;
    fs["SIFTmatch.match_type"] >> match_type;
    fs["SIFTmatch.max_distance"] >> max_distance;
    fs["SIFTmatch.distance_ratio"] >> distance_ratio;
    fs["SIFTmatch.cross_check"] >> cross_check;

    assert(match_type == 0 || match_type == 1);

    cv::Ptr<FeatureMatcher> matcher;
    if (match_type == 0)
    {
        matcher = cv::Ptr<SequentialFeatureMatcher>(new SequentialFeatureMatcher(database_path));
    }
    else
    {
        matcher = cv::Ptr<BruteFeatureMatcher>(new BruteFeatureMatcher(database_path));
    }

    Timer timer;
    timer.Start();

    matcher->RunMatching();

    timer.PrintMinutes();

    return 0;
}
