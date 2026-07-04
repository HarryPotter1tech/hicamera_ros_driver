#include "hikcamera/capturer.hpp"
#include <ctime>
#include <expected>
#include <iostream>
#include <string>
#include <vector>
namespace hikcamera_ros_driver {
std::expected<void, std::string> ConfigsLoader(
    std::string& yaml_config_path, hikcamera::Config config);
std::expected<void, std::string> CameraThread(const hikcamera::Config& config);
std::expected<bool, std::string> CameraInit();
}