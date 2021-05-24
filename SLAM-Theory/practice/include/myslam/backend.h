#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {
class Map;

/**
 * rear end
 * There is a separate optimization thread to start optimization when the Map is updated
 * Map update is triggered by the front end
 */
class Backend {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;

    /// Start the optimization thread in the constructor and hang
    Backend();

    // Set the left and right cameras to obtain internal and external parameters
    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        cam_left_ = left;
        cam_right_ = right;
    }

    /// Set up the map
    void SetMap(std::shared_ptr<Map> map) {map_ = map;}

    /// Trigger map update, start optimization
    void UpdateMap();

    /// Close the backend thread
    void Stop();

   private:
    /// Backend thread
    void BackendLoop();

    /// Optimize the given keyframes and landmark points
    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

} // namespace myslam

#endif // MYSLAM_BACKEND_H