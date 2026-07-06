#include "hikcamera_ros_driver/camera_driver.hpp"
#include <hikcamera/capturer.hpp>

#include <iostream>
#include <rclcpp/rclcpp.hpp>
int main() {
    rclcpp::init(0, nullptr);
    std::string yaml_config_path = "/home/ubuntu/ros_ws/src/hikcamera_ros_driver/config/"
                                   "hikcamera_config.yaml";
    auto camera                  = std::make_shared<hikcamera::Camera>();
    auto config                  = hikcamera::Config();
    auto image_width             = 0;
    auto image_height            = 0;
    auto is_camera_running       = std::atomic<bool>(true);
    auto camera_thread           = std::thread();
    if (auto result = hikcamera_ros_driver::ConfigsLoader(
            yaml_config_path, config, image_width, image_height);
        !result) {
        return EXIT_FAILURE;
    }
    if (auto result = hikcamera_ros_driver::CameraThreadStart(config, is_camera_running); !result) {
        return EXIT_FAILURE;
    } else {
        camera_thread = std::move(result.value());
    }
    rclcpp::spin(std::make_shared<rclcpp::Node>("hikcamera_ros_driver_node"));
    hikcamera_ros_driver::CameraThreadStop(camera_thread, config, is_camera_running);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
