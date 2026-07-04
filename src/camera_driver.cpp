#include "hikcamera_ros_driver/camera_driver.hpp"
#include "yaml-cpp/yaml.h"
#include <ctime>
#include <expected>
#include <hikcamera/capturer.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <sys/shm.h>
#include <thread>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
namespace hikcamera_ros_driver {
std::expected<void, std::string> ConfigsLoader(
    std::string& yaml_config_path, hikcamera::Config& config, int image_width, int image_height) {
    YAML::Node yaml_config     = YAML::Load(yaml_config_path);
    config.white_balance_blue  = yaml_config["Hikcamera"]["white_balance_blue"].as<unsigned int>();
    config.white_balance_red   = yaml_config["Hikcamera"]["white_balance_red"].as<unsigned int>();
    config.white_balance_green = yaml_config["Hikcamera"]["white_balance_green"].as<unsigned int>();
    config.auto_white_balance  = yaml_config["Hikcamera"]["white_balance_blue"].as<unsigned int>();
    config.brightness          = yaml_config["Hikcamera"]["brightness"].as<unsigned int>();
    config.exposure_us         = yaml_config["Hikcamera"]["exposure_time"].as<unsigned int>();
    config.fixed_framerate     = yaml_config["Hikcamera"]["fps"].as<unsigned int>();
    config.sharpness           = yaml_config["Hikcamera"]["sharpness"].as<unsigned int>();
    config.gain                = yaml_config["Hikcamera"]["gain"].as<unsigned int>();
    image_height               = yaml_config["Hikcamera"]["image_height"].as<unsigned int>();
    image_width                = yaml_config["Hikcamera"]["image_width"].as<unsigned int>();
}
std::expected<bool, std::string> CameraInit(const hikcamera::Config& config) {
    auto camera = std::make_shared<hikcamera::Camera>();
    camera->initialize(config);
    camera->connect();
}
std::expected<void, std::string> CameraThread() { }

}
