#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "../include/slamBase.h"

// Given index, read a frame of data
FRAME readFrame( int index, ParameterReader& pd );
// measure the size of the movement
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main( int argc, char** argv)
{
    ParameterReader pd;
    int startIndex = atoi( pd.getData( "start_index" ).c_str() );
    int endIndex = atoi( pd.getData( "end_index" ).c_str() );

    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // current index is currIndex
    FRAME lastFrame = readFrame( currIndex, pd ); // last frame of data
    // We are always comparing currFrame and lastFrame
    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp( lastFrame, detector, descriptor );
    PointCloud::Ptr cloud = image2PointCloud( lastFrame.rgb, lastFrame.depth, camera );

    pcl::visualization::CloudViewer viewer("viewer");

    // Whether to display the point cloud
    bool visualize = pd.getData("visualize_pointcloud")==string("yes");

    int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    double max_norm = atof( pd.getData("max_norm").c_str() );

    for (currIndex=startIndex+1; currIndex<endIndex; currIndex++)
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // read currFrame
        computeKeyPointsAndDesp( currFrame, detector, descriptor );
        // Compare currFrame and lastFrame
        RESULT_OF_PNP result = estimateMotion( lastFrame, currFrame, camera );
        if (result.inliers <min_inliers) //inliers are not enough, give up the frame
            continue;
        // Calculate whether the range of motion is too large
        double norm = normofTransform(result.rvec, result.tvec);
        cout<<"norm = "<<norm<<endl;
        if (norm >= max_norm)
            continue;
        Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
        cout<<"T="<<T.matrix()<<endl;

        cloud = joinPointCloud( cloud, currFrame, T, camera );

        if (visualize == true)
            viewer.showCloud( cloud );

        lastFrame = currFrame;
    }

    pcl::io::savePCDFile( "../data/result.pcd", *cloud );
    return 0;
}

FRAME readFrame( int index, ParameterReader& pd)
{
    FRAME f;
    string rgbDir = pd.getData("rgb_dir");
    string depthDir = pd.getData("depth_dir");

    string rgbExt = pd.getData("rgb_extension");
    string depthExt = pd.getData("depth_extension");

    stringstream ss;
    ss<<rgbDir<<index<<rgbExt;
    string filename;
    ss>>filename;
    std:cout << filename << std::endl;
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}