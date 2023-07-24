#include "../include/slamBase.h"

bool findRtRANSAC(std::vector<cv::Point3f> objectPoints, vector<cv::Point2f> imagePoints, cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Mat &rvec, cv::Mat &tvec, bool useExtrinsicGuess, int iterationsCount, float reprojectionError, int minInliersCount, cv::Mat &inliers, int flags) {

    float confidence_factor = minInliersCount / (imagePoints.size()/2);
    float confidence = confidence_factor < 0.001 ? 0.001 : confidence_factor > 0.999 ? 0.999 : confidence_factor;

    solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess,
                   iterationsCount, reprojectionError, confidence, inliers, flags);
    return true; // for consistency with the other methods
}

PointCloud::Ptr image2PointCloud( cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    PointCloud::Ptr cloud (new PointCloud );

    for (int m = 0; m <depth.rows; m+=2)
        for (int n=0; n <depth.cols; n+=2)
        {
            // Get the value at (m, n) in the depth map
            ushort d = depth.ptr<ushort>(m)[n];
            // d may have no value, if so, skip this point
            if (d == 0)
                continue;
            // If d has a value, add a point to the point cloud
            PointT p;

            // Calculate the space coordinates of this point
            p.z = double(d) / camera.scale;
            p.x = (n-camera.cx) * p.z / camera.fx;
            p.y = (m-camera.cy) * p.z / camera.fy;
            
            // Get its color from the rgb image
            // rgb is a three-channel BGR format picture, so get the colors in the following order
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // add p to the point cloud
            cloud->points.push_back( p );
        }
    // Set and save the point cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    return cloud;
}

cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    cv::Point3f p; // 3D point
    p.z = double( point.z) / camera.scale;
    p.x = (point.x-camera.cx) * p.z / camera.fx;
    p.y = (point.y-camera.cy) * p.z / camera.fy;
    return p;
}

// computeKeyPointsAndDesp extracts key points and feature descriptors at the same time
void computeKeyPointsAndDesp( FRAME& frame, string detector, string descriptor)
{
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detector = cv::ORB::create();
    _descriptor = cv::ORB::create();

    if (!_detector || !_descriptor)
    {
        cerr<<"Unknown detector or discriptor type !"<<detector<<","<<descriptor<<endl;
        return;
    }

    _detector->detect( frame.rgb, frame.kp );
    _descriptor->compute( frame.rgb, frame.kp, frame.desp );

    return;
}

// estimateMotion calculates the motion between two frames
// Input: frame 1 and frame 2
// Output: rvec and tvec
RESULT_OF_PNP estimateMotion( FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;
    vector< cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );
   
    RESULT_OF_PNP result;
    vector< cv::DMatch> goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    for (size_t i=0; i<matches.size(); i++)
    {
        if (matches[i].distance <minDis)
            minDis = matches[i].distance;
    }
    
    cout<<"min dis = "<<minDis<<endl;
    if (minDis <10)
        minDis = 10;

    for (size_t i=0; i<matches.size(); i++)
    {
        if (matches[i].distance <good_match_threshold*minDis)
            goodMatches.push_back( matches[i] );
    }

    cout<<"good matches: "<<goodMatches.size()<<endl;

    if (goodMatches.size() <= 5)
    {
        result.inliers = -1;
        return result;
    }
    // The 3D point of the first frame
    vector<cv::Point3f> pts_obj;
    // The image point of the second frame
    vector< cv::Point2f> pts_img;

    // Camera internal parameters
    for (size_t i=0; i<goodMatches.size(); i++)
    {
        // query is the first, train is the second
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;
        // Be careful to get d! x is to the right and y is down, so y is the row and x is the column!
        ushort d = frame1.depth.ptr<ushort>( int(p.y) )[ int(p.x) ];
        if (d == 0)
            continue;
        pts_img.push_back( cv::Point2f( frame2.kp[goodMatches[i].trainIdx].pt) );

        // Convert (u,v,d) to (x,y,z)
        cv::Point3f pt (p.x, p.y, d );
        cv::Point3f pd = point2dTo3d( pt, camera );
        pts_obj.push_back( pd );
    }

    if (pts_obj.size() ==0 || pts_img.size()==0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0, 0, 1}
    };

    // Build the camera matrix
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
    cv::Mat rvec, tvec, inliers;
    // Solve for pnp
    findRtRANSAC( pts_obj, pts_img,
                  cameraMatrix, cv::Mat(),
                  rvec, tvec, false,
                  100, 1.0,
                  100, inliers, cv::SOLVEPNP_ITERATIVE);
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    return result;
}


// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues( rvec, R );
    Eigen::Matrix3d r;
    for (int i=0; i<3; i++)
        for (int j=0; j<3; j++)
            r(i,j) = R.at<double>(i,j);
  
    // Convert the translation vector and rotation matrix into a transformation matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

// joinPointCloud
// Input: the original point cloud, the new frame and its pose
// Output: the image after adding the new frame to the original frame
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera)
{
     PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

     // merge point clouds
     PointCloud::Ptr output (new PointCloud());
     pcl::transformPointCloud( *original, *output, T.matrix() );
     *newCloud += *output;

     // Voxel grid filtering and downsampling
     static pcl::VoxelGrid<PointT> voxel;
     static ParameterReader pd;
     double gridsize = atof( pd.getData("voxel_grid").c_str() );
     voxel.setLeafSize( gridsize, gridsize, gridsize );
     voxel.setInputCloud( newCloud );
     PointCloud::Ptr tmp( new PointCloud() );
     voxel.filter( *tmp );
     return tmp;
}