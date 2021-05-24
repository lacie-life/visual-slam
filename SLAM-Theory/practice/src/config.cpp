#include "myslam/config.h"

namespace myslam {
bool Config::SetParameterFile(const std::string &filename) {

    if (config_ == nullptr){
        config_ = std::shared_ptr<Config>(new Config);
    }

    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    std::cout << filename << std::endl;

    if (config_->file_.isOpened() == false) {
        std::cout << filename << std::endl;
        LOG(ERROR) << "parameter file " << filename << " does not exist.";
        config_->file_.release();
        return false;
    }
    std::cout << Config::Get<std::string>("dataset_dir") << std::endl;
    return true;
}

Config::~Config() {
    if (file_.isOpened())
        file_.release();
}

std::shared_ptr<Config> Config::config_ = nullptr;

}