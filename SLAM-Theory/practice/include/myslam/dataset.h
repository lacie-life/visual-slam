#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

/**
  * Data set reading
  * Pass in the configuration file path when constructing, the dataset_dir of the configuration file is the dataset path
  * After Init, the camera and the next image can be obtained
  */
class Dataset {
    public:
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
     typedef std::shared_ptr<Dataset> Ptr;
     Dataset(const std::string& dataset_path);

     /// Initialization, return whether it is successful
     bool Init();

     /// create and return the next frame containing the stereo images
     Frame::Ptr NextFrame();

     /// get camera by id
     Camera::Ptr GetCamera(int camera_id) const {
         return cameras_.at(camera_id);
     }

    private:
     std::string dataset_path_;
     int current_image_index_ = 0;

     std::vector<Camera::Ptr> cameras_;
};
} // namespace myslam

#endif