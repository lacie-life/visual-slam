#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "../include/slamBase.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/eigen_types.h>

// Put the definition of g2o to the front
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

// Given index, read a frame of data
FRAME readFrame( int index, ParameterReader& pd );
// Estimate the size of a movement
double normofTransform( cv::Mat rvec, cv::Mat tvec );

// detect two frames and define the result
enum CHECK_RESULT {NOT_MATCHED=0, TOO_FAR_AWAY, TOO_CLOSE, KEYFRAME};
// Function declaration
CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops=false );
// Detect short-range loops
void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );
// Random detection loop
void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti );

int main( int argc, char** argv)
{
    // The previous part is the same as vo
    ParameterReader pd;
    int startIndex = atoi( pd.getData( "start_index" ).c_str() );
    int endIndex = atoi( pd.getData( "end_index" ).c_str() );

    // All the key frames are placed here
    vector< FRAME> keyframes;
    // initialize
    cout<<"Initializing ..."<<endl;
    int currIndex = startIndex; // current index is currIndex
    FRAME currFrame = readFrame( currIndex, pd ); // previous frame data

    string detector = pd.getData( "detector" );
    string descriptor = pd.getData( "descriptor" );
    //std::cout << "hello " << std::endl;
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    computeKeyPointsAndDesp( currFrame, detector, descriptor );
    PointCloud::Ptr cloud = image2PointCloud( currFrame.rgb, currFrame.depth, camera );
    //std::cout << "hello " << std::endl;
    /*******************************
    // New: About the initialization of g2o
    *******************************/
    // Initialize the solver
    // Choose an optimization method
    // fix with std::unique_ptr, linear solver:
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver
            (new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());

    // Initialize the solver
    // linearSolver->setBlockOrdering( false );
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr (new g2o::BlockSolver_6_3(std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));


    g2o::SparseOptimizer globalOptimizer; // The last thing I use is this stuff
    globalOptimizer.setAlgorithm( solver );
    // Do not output debugging information
    globalOptimizer.setVerbose( false );

    // Add the first vertex to globalOptimizer
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId( currIndex );
    v->setEstimate( Eigen::Isometry3d::Identity() ); //estimated as identity matrix
    v->setFixed( true ); //The first vertex is fixed, no optimization
    globalOptimizer.addVertex( v );

    keyframes.push_back( currFrame );

    double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    bool check_loop_closure = pd.getData("check_loop_closure")==string("yes");
    //std::cout << "hello " << std::endl;
    for (currIndex=startIndex+1; currIndex<endIndex; currIndex++)
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // read currFrame
        computeKeyPointsAndDesp( currFrame, detector, descriptor ); //Extract features
        std::cout << currIndex << std::endl;
        CHECK_RESULT result = checkKeyframes( keyframes.back(), currFrame, globalOptimizer ); //match this frame with the last frame in keyframes
        std::cout << "Hello" << std::endl;
        switch (result) // adopt different strategies according to different matching results
        {
            case NOT_MATCHED:
                //No match, skip directly
                cout<<RED"Not enough inliers."<<endl;
                break;
            case TOO_FAR_AWAY:
                // It???s too close, just jump
                cout<<RED"Too far away, may be an error."<<endl;
                break;
            case TOO_CLOSE:
                // too far, maybe something went wrong
                cout<<RESET"Too close, not a keyframe"<<endl;
                break;
            case KEYFRAME:
                cout<<GREEN"This is a new keyframe"<<endl;
                // Not far, not near, just right
                /**
                 * This is important!!
                 * This is important!!
                 * This is important!!
                 * (very important so I've said three times!)
                 */
                // detect loopback
                if (check_loop_closure)
                {
                    checkNearbyLoops( keyframes, currFrame, globalOptimizer );
                    checkRandomLoops( keyframes, currFrame, globalOptimizer );
                }
                keyframes.push_back( currFrame );

                break;
            default:
                break;
        }
        

    }

    // optimize
    cout<<RESET"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("../result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //You can specify the number of optimization steps
    globalOptimizer.save( "../result_after.g2o" );
    cout<<"Optimization done."<<endl;

    // Splicing point cloud map
    cout<<"saving the point cloud map..."<<endl;
    PointCloud::Ptr output (new PointCloud()); //Global map
    PointCloud::Ptr tmp (new PointCloud() );

    pcl::VoxelGrid<PointT> voxel; // Grid filter, adjust the map resolution
    pcl::PassThrough<PointT> pass; // z-direction interval filter, because the effective depth interval of the rgbd camera is limited, remove those that are too far away
    pass.setFilterFieldName("z");
    pass.setFilterLimits( 0.0, 4.0 ); //Don't need more than 4m

    double gridsize = atof( pd.getData( "voxel_grid" ).c_str() ); //The resolution map can be adjusted in parameters.txt
    voxel.setLeafSize( gridsize, gridsize, gridsize );

    for (size_t i=0; i<keyframes.size(); i++)
    {
        // Take a frame from g2o
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex( keyframes[i].frameID ));
        Eigen::Isometry3d pose = vertex->estimate(); //The optimized pose of this frame
        PointCloud::Ptr newCloud = image2PointCloud( keyframes[i].rgb, keyframes[i].depth, camera ); //Convert to point cloud
        // The following is filtering
        voxel.setInputCloud( newCloud );
        voxel.filter( *tmp );
        pass.setInputCloud( tmp );
        pass.filter( *newCloud );
        // Transform the point cloud and add it to the global map
        pcl::transformPointCloud( *newCloud, *tmp, pose.matrix() );
        *output += *tmp;
        tmp->clear();
        newCloud->clear();
    }

    voxel.setInputCloud( output );
    voxel.filter( *tmp );
    //storage
    pcl::io::savePCDFile( "../result.pcd", *tmp );

    cout<<"Final map is saved."<<endl;
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
    //std::cout << filename << std::endl;
    f.rgb = cv::imread( filename );
    //std::cout << filename << std::endl;

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    //std::cout << filename << std::endl;
    f.depth = cv::imread( filename, -1 );
    //std::cout << filename << std::endl;
    f.frameID = index;
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes( FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();
    // ??????f1 ??? f2
    std::cout << "check : checkKeyframes" << std::endl;
    RESULT_OF_PNP result = estimateMotion( f1, f2, camera );
    //std::cout << "check" << std::endl;
    if ( result.inliers < min_inliers ) //inliers?????????????????????
        return NOT_MATCHED;
    // ??????????????????????????????
    double norm = normofTransform(result.rvec, result.tvec);
    if ( is_loops == false )
    {
        if ( norm >= max_norm )
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if ( norm >= max_norm_lp)
            return TOO_FAR_AWAY;
    }

    if ( norm <= keyframe_threshold )
        return TOO_CLOSE;   // too adjacent frame
    // ???g2o?????????????????????????????????????????????
    // ????????????
    // ??????????????????id??????
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( f2.frameID );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        opti.addVertex(v);
    }
    // ?????????
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // ???????????????????????????id
    edge->setVertex( 0, opti.vertex(f1.frameID ));
    edge->setVertex( 1, opti.vertex(f2.frameID ));
    edge->setRobustKernel( new g2o::RobustKernelHuber() );
    // ????????????
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();
    // ?????????????????????????????????????????????????????????????????????????????????
    // ??????pose???6D?????????????????????6*6???????????????????????????????????????????????????0.1???????????????
    // ??????????????????????????????0.01???????????????????????????100?????????
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;
    // ?????????????????????????????????????????????????????????????????????
    edge->setInformation( information );
    // ??????????????????pnp???????????????
    Eigen::Isometry3d T = cvMat2Eigen( result.rvec, result.tvec );
    // edge->setMeasurement( T );
    edge->setMeasurement( T.inverse() );
    // ?????????????????????
    opti.addEdge(edge);
    return KEYFRAME;
}

void checkNearbyLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int nearby_loops = atoi( pd.getData("nearby_loops").c_str() );
    
    // ?????????currFrame??? frames????????????????????????
    if ( frames.size() <= nearby_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
}

void checkRandomLoops( vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // ??????????????????????????????
    
    if ( frames.size() <= random_loops )
    {
        // no enough keyframes, check everyone
        for (size_t i=0; i<frames.size(); i++)
        {
            checkKeyframes( frames[i], currFrame, opti, true );
        }
    }
    else
    {
        // randomly check loops
        for (int i=0; i<random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes( frames[index], currFrame, opti, true );
        }
    }
}
