#include <iostream>
#include <fstream>
#include <sstream>


#include "../include/slamBase.h"
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/eigen_types.h>

using namespace std;


// Given index, read a frame of data
FRAME readFrame( int index, ParameterReader& pd );
// Estimate the size of a movement
double normofTransform( cv::Mat rvec, cv::Mat tvec );

int main( int argc, char** argv)
{
    // The previous part is the same as vo
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

    /*******************************
    // New: About the initialization of g2o
    *******************************/
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

    int lastIndex = currIndex; // the id of the previous frame

    for (currIndex=startIndex+1; currIndex<endIndex; currIndex++)
    {
        cout<<"Reading files "<<currIndex<<endl;
        FRAME currFrame = readFrame( currIndex,pd ); // read currFrame
        computeKeyPointsAndDesp( currFrame, detector, descriptor );
        // compare currFrame and lastFrame
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

        // It will be faster if you remove the visualization
        if (visualize == true)
        {
            cloud = joinPointCloud( cloud, currFrame, T, camera );
            viewer.showCloud( cloud );
        }

        // Add the edge of this vertex to the previous frame to g2o
        // Vertex part
        // Vertex only needs to set id
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId( currIndex );
        v->setEstimate( Eigen::Isometry3d::Identity() );
        globalOptimizer.addVertex(v);
        // edge part
        g2o::EdgeSE3* edge = new g2o::EdgeSE3();
        // The id of the two vertices connecting this edge
        edge->vertices() [0] = globalOptimizer.vertex( lastIndex );
        edge->vertices() [1] = globalOptimizer.vertex( currIndex );
        // Information matrix
        Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6, 6 >::Identity();
        // The information matrix is ​​the inverse of the covariance matrix, which represents our pre-estimation of the accuracy of the edges
        // Because the pose is 6D, the information matrix is ​​a 6*6 matrix, assuming that the estimation accuracy of position and angle are both 0.1 and independent of each other
        // Then the covariance is a matrix of 0.01 diagonal, and the information matrix is ​​a matrix of 100
        information(0,0) = information(1,1) = information(2,2) = 100;
        information(3,3) = information(4,4) = information(5,5) = 100;
        // You can also set the angle larger, which means that the estimation of the angle is more accurate
        edge->setInformation( information );
        // The estimation of the edge is the result of the pnp solution
        edge->setMeasurement( T );
        // add this edge to the graph
        globalOptimizer.addEdge(edge);

        lastFrame = currFrame;
        lastIndex = currIndex;

    }

    // optimize all edges
    cout<<"optimizing pose graph, vertices: "<<globalOptimizer.vertices().size()<<endl;
    globalOptimizer.save("../data/result_before.g2o");
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize( 100 ); //You can specify the optimization step
    globalOptimizer.save( "../data/result_after.g2o" );
    cout<<"Optimization done."<<endl;

    globalOptimizer.clear();

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
    f.rgb = cv::imread( filename );

    ss.clear();
    filename.clear();
    ss<<depthDir<<index<<depthExt;
    ss>>filename;

    f.depth = cv::imread( filename, -1 );
    f.frameID = index;
    return f;
}

double normofTransform( cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+ fabs(cv::norm(tvec));
}