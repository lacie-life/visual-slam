
#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "config.yaml", "config file path");

int main(int argc, char **argv) {
    //google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    std::cout << FLAGS_config_file << std::endl;
    assert(vo->Init() == true);

    vo->Run();

    return 0;
}
