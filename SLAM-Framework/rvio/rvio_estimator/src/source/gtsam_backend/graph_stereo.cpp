#include "gtsam_backend/graph.h"
#include "estimator/parameters.h"

void graph_solver::addStereoMeas(int f_id, Eigen::Vector2d point0, Eigen::Vector2d point1, Eigen::Vector3d point3d)
{

    if(!system_initializied_)
        return;

    //const gtsam::noiseModel::Isotropic::shared_ptr model = gtsam::noiseModel::Isotropic::Sigma(3,1);

    //if landmark doesnt exist add the node
    //    if(!values_prev_.exists(L(f_id)))
    //    {
    //        gtsam::Pose3 cam_pose     = values_curr_.at<gtsam::Pose3>(X(cur_sc_));
    //        gtsam::Point3 world_point = cam_pose.transform_from(gtsam::Point3(point3d(0),point3d(1),point3d(2)));
    //        values_curr_.insert(L(f_id), world_point);
    //        values_prev_.insert(L(f_id), world_point);
    //    }

    double uL, uR, v;
    uL = point0(0);
    uR = point1(0);
    v  = point0(1);

    //std::cout << "frame id" << f_id << std::endl;
    //std::cout << "landmark id " << L(f_id) << std::endl;
    //std::cout << "uL " << uL << std::endl << "uR " << uR << std::endl << "v " << v << std::endl;

    if(stereo_smart_factor_.count(L(f_id)) == 0)
    {
       //std::cout << "adding stereo meas" << std::endl;
       auto gaussian = gtsam::noiseModel::Isotropic::Sigma(3, 0.01);
       gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(0 ,0, 0));

       gtsam::SmartProjectionParams params(gtsam::HESSIAN, gtsam::ZERO_ON_DEGENERACY);
       stereo_smart_factor_[L(f_id)] =  gtsam::SmartStereoProjectionPoseFactor::shared_ptr(
                   new gtsam::SmartStereoProjectionPoseFactor(gaussian, params, sensor_P_body));
       graph->push_back(stereo_smart_factor_[L(f_id)]);
    }

    stereo_smart_factor_[L(f_id)]->add(gtsam::StereoPoint2(uL, uR, v), X(cur_sc_), K);

    //graph->push_back(gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>(gtsam::StereoPoint2(uL, uR, v),
    //                                                                         model, X(cur_sc_), L(f_id), K));
}
