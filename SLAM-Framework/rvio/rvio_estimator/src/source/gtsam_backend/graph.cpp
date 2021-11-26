#include "gtsam_backend/graph.h"


graph_solver::graph_solver() : graph_path("/home/hb/graph.dot")
{
    this->graph = new gtsam::NonlinearFactorGraph();
    // ISAM2 solver
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.1;
    isam_params.relinearizeSkip = 1;
    isam_params.cacheLinearizedFactors = false;
    isam_params.enableDetailedResults = true;
    isam_params.print();
    this->isam2 = new gtsam::ISAM2(isam_params);
}

graph_solver::~graph_solver()
{

}

void graph_solver::initVariables()
{
    cur_sc_ = 0;
}

void graph_solver::initialize(Eigen::Vector3d Ps, Eigen::Matrix3d Rs, Eigen::Vector3d Vs,
                              Eigen::Vector3d Bas, Eigen::Vector3d Bgs)
{

    if(system_initializied_)
        return;

    initVariables();

    Eigen::Quaterniond q(Rs);
    gtsam::Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
    gtsam::Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

    gtsam::State init_state = gtsam::State(gtsam::Pose3(gtsam::Quaternion(q.w(), q.x(), q.y(),q.z()), gtsam::Vector3(Ps)),
                                           gtsam::Vector3(Vs), gtsam::Bias(acc_bias, gyro_bias));


    auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(1e-4),
                                                           gtsam::Vector3::Constant(1e-4)).finished());

    auto v_noise = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    auto b_noise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);

    std::cout << "initial rotation for gtsam:" << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;
    std::cout << "init p:" << init_state.p() << std::endl;
    std::cout << "init v:" << init_state.v() << std::endl;
    std::cout << "init b:" << init_state.b() << std::endl;

    // Add initial state to the graph
    values_curr_.insert(    X(cur_sc_), init_state.pose());
    values_curr_.insert(    V(cur_sc_), init_state.v());
    values_curr_.insert(    B(cur_sc_), init_state.b());
    values_prev_.insert(X(cur_sc_), init_state.pose());
    values_prev_.insert(V(cur_sc_), init_state.v());
    values_prev_.insert(B(cur_sc_), init_state.b());

    //create graph
    graph->add(gtsam::PriorFactor<gtsam::Pose3>(X(cur_sc_), init_state.pose(), pose_noise));
    graph->add(gtsam::PriorFactor<gtsam::Vector3>(V(cur_sc_), init_state.v(),  v_noise));
    graph->add(gtsam::PriorFactor<gtsam::Bias>(   B(cur_sc_), init_state.b(), b_noise));

    //add graph values

    //    //fix the first pose
    //    gtsam::Pose3 first_pose = values_curr_.at<gtsam::Pose3>(X(cur_sc_));
    //    graph->add(gtsam::NonlinearEquality<gtsam::Pose3>(X(cur_sc_), first_pose));

     K.reset(new gtsam::Cal3_S2Stereo(395.80426777047927,398.1361799271774,0,321.77659844943565,245.86312789272498, 0.05)); //fx,fy,s,u0,v0,b
    //K.reset(new gtsam::Cal3_S2Stereo(4.481008985853343e+02,4.481008985853343e+02,0,3.765e+02,2.405e+02, 0.055)); //fx,fy,s,u0,v0,b
    //K.reset(new gtsam::Cal3_S2Stereo(822.37,822.37,0,538.73,579.10, 0.372)); //fx,fy,s,u0,v0,b


    //    cur_sc_lookup_[t] = cur_sc_;
    //    timestamp_lookup_[cur_sc_] = t;

    if(set_imu_preintegration(init_state))
        system_initializied_ = true;

    return;
}

void graph_solver::addIMUMeas(std::vector<pair<double, Eigen::Vector3d> > acc_vec,
                              std::vector<pair<double, Eigen::Vector3d> > ang_vel_vec)
{
    imu_lock_.lock();
    for(size_t i= 0; i < acc_vec.size(); ++i)
        acc_vec_.push_back(acc_vec.at(i));
    for(size_t i = 0; i < ang_vel_vec.size(); ++i)
        ang_vel_vec_.push_back(ang_vel_vec.at(i));
    imu_lock_.unlock();

    return;
}

void graph_solver::progateWithIMU(double timestamp, Eigen::Matrix3d& Rs, Eigen::Vector3d& Ps)
{
    if(!system_initializied_)
        return;

    if(acc_vec_.size() < 2)
        return;

    gtsam::CombinedImuFactor imu_factor = createIMUFactor(timestamp);
    graph->add(imu_factor);

    gtsam::State predicted_state = getPredictedState(values_prev_);

    //increasing the node counter
    cur_sc_++;

    values_curr_.insert(    X(cur_sc_), predicted_state.pose());
    values_curr_.insert(    V(cur_sc_), predicted_state.v());
    values_curr_.insert(    B(cur_sc_), predicted_state.b());

    values_prev_.insert(    X(cur_sc_), predicted_state.pose());
    values_prev_.insert(    V(cur_sc_), predicted_state.v());
    values_prev_.insert(    B(cur_sc_), predicted_state.b());

    //returng the Ps and Rs to the estimator
    Ps(0) = predicted_state.pose().x();
    Ps(1) = predicted_state.pose().y();
    Ps(2) = predicted_state.pose().z();
    Rs    = predicted_state.pose().rotation().matrix();
}


void graph_solver::optimize()
{
    if(!system_initializied_ && cur_sc_ < 2)
        return;

    try {
        printf("Optimizing Graph\n");
        gtsam::ISAM2Result result = isam2->update(*graph, values_curr_);
        values_prev_ = isam2->calculateEstimate();
        graph->saveGraph(graph_path);
    } catch (gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("FORSTER2 gtsam indeterminate linear system exception!");
        std::cerr << e.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Current propaged pose:" << values_prev_.at<gtsam::Pose3>( X(cur_sc_)) << std::endl;
    std::cout << "Current propaged vel:" << values_prev_.at<gtsam::Vector3>( V(cur_sc_)) << std::endl;
    std::cout << "Current propaged bias:" << values_prev_.at<gtsam::Bias>( B(cur_sc_)) << std::endl;

    //clearing the old values
    values_curr_.clear();
    graph->resize(0);
    resetIMUIntegration();
}

