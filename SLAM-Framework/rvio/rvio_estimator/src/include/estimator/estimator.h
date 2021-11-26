#include <stdio.h>
#include <mutex>
#include <thread>
#include "eigen3/Eigen/Eigen"
#include <opencv2/opencv.hpp>

#include "parameters.h"
#include "utility/tic_toc.h"
#include "estimator/feature_manager.h"
#include "featureTracker/feature_tracker.h"
#include "visualization/visualization.h"

#include "gtsam_backend/graph.h"

class estimator
{
public:

  estimator();
  ~estimator();

  void initEstimator();
  void inputImages(double t, const cv::Mat& img0, const cv::Mat img1);
  void setParameters();
  void inputIMU(double t, const Eigen::Vector3d linear_acc, const Eigen::Vector3d ang_vel);
  void save_graph() { graph_obj_.save_graph();  }

protected:
  FeatureTracker feature_tracker_obj_;
  FeatureManager feature_manager_;

  std::thread process_thread_;
  std::mutex buf_lock_;
  std::mutex process_lock_;

  std::queue<pair<double, map<int, vector<pair<int, Eigen::Matrix<double, 7, 1> > > > > > feature_buf_;
  std::queue<std::pair<double, Eigen::Vector3d>> acc_buf_;
  std::queue<std::pair<double, Eigen::Vector3d>> ang_vel_buf_;

  //config params
private:
  double time_delay_;
  Eigen::Vector3d g_;

private:

  Eigen::Vector3d        Ps[(WINDOW_SIZE + 1)];
  Eigen::Vector3d        Vs[(WINDOW_SIZE + 1)];
  Eigen::Matrix3d        Rs[(WINDOW_SIZE + 1)];
  Eigen::Vector3d        Bas[(WINDOW_SIZE + 1)];
  Eigen::Vector3d        Bgs[(WINDOW_SIZE + 1)];


  Eigen::Matrix3d ric[2];
  Eigen::Vector3d tic[2];

  Eigen::Vector3d prev_acc_, prev_ang_vel_;
private:
  bool IMUAvailable(double t);
  void processMeasurements();
  bool getIMUInterval(double t0, double t1, std::vector<pair<double, Eigen::Vector3d>> &acc_vector,
                      std::vector<pair<double, Eigen::Vector3d>> &ang_vel_vector);
  void initFirstIMUPose(std::vector<pair<double, Eigen::Vector3d>> &acc_vector);
  void processIMU(std::vector<pair<double, Eigen::Vector3d> > &acc_vector,
                  std::vector<pair<double, Eigen::Vector3d> > &ang_vel_vector);

  void predictWithIMU(double timestamp);
  void processImages(const map<int,vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image,
                     const double header);
  Eigen::Matrix<double, 3, 4> currLeftCamPose(Eigen::Vector3d Ps, Eigen::Matrix3d Rs);
  Eigen::Matrix<double, 3, 4> currRightCamPose(Eigen::Vector3d Ps, Eigen::Matrix3d Rs);

  int input_img_cnt_;
  double prev_time, cur_time;
  bool init_first_pose_flag_;
  bool first_imu_;

  int frame_count;
  int counter_;

  //graph
private:
    graph_solver graph_obj_;

};
