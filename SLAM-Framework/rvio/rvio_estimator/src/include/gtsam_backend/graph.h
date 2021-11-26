#include <stdio.h>
#include <mutex>
#include <thread>
#include "eigen3/Eigen/Eigen"
#include <opencv2/opencv.hpp>

#include "utility/tic_toc.h"
#include "estimator/feature_manager.h"
#include "featureTracker/feature_tracker.h"
#include "visualization/visualization.h"
#include "estimator/parameters.h"

// Graphs
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
// Factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <gtsam/slam/StereoFactor.h>

#include "utils/State.h"

using gtsam::symbol_shorthand::X; // Pose3     (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel       (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias      (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::L; // Landmark  (X, Y, Z)

class graph_solver
{
public:
  graph_solver();
  ~graph_solver();


  void initialize(Eigen::Vector3d Ps, Eigen::Matrix3d Rs, Eigen::Vector3d Vs,
                  Eigen::Vector3d Bas, Eigen::Vector3d Bgs);
  void addIMUMeas(std::vector<pair<double, Vector3d> > acc_vec, std::vector<pair<double, Vector3d> > ang_vel_vec);
  void progateWithIMU(double timestamp, Eigen::Matrix3d& Rs, Eigen::Vector3d& Ps);
  void addStereoMeas(int f_id, Eigen::Vector2d point0, Eigen::Vector2d point1, Eigen::Vector3d point3d);
  void optimize();
  void save_graph() { graph->saveGraph(graph_path);
                    }

private:
  void initVariables();
  std::mutex imu_lock_;

private:

  gtsam::Cal3_S2Stereo::shared_ptr K;
  size_t cur_sc_;
  bool system_initializied_;

  // All created nodes
  gtsam::Values values_prev_;
  gtsam::Values values_curr_;

  std::unordered_map<double, size_t> cur_sc_lookup_; // curr state based on timestamp
  std::unordered_map<size_t, double> timestamp_lookup_;

  gtsam::NonlinearFactorGraph* graph;

  // ISAM2 solvers
  gtsam::ISAM2* isam2;

  // imu preintegration
  std::vector<std::pair<double, Eigen::Vector3d> > acc_vec_, ang_vel_vec_;
  gtsam::PreintegratedCombinedMeasurements* preint_gtsam;
  bool set_imu_preintegration(const gtsam::State& init_state);
  gtsam::CombinedImuFactor createIMUFactor(double update_time);
  gtsam::State getPredictedState(gtsam::Values prev_values);
  void resetIMUIntegration();

  //stereo smartfactors
  std::map<size_t, gtsam::SmartStereoProjectionPoseFactor::shared_ptr> stereo_smart_factor_;

  //save graph path
  std::ofstream graph_path;

};
