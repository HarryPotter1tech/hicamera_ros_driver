#include "hikcamera_ros_driver/camera_driver.hpp"
#include <hikcamera/capturer.hpp>
#include <rclcpp/rclcpp.hpp>
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("hikcamera_ros_driver_node");

    node->declare_parameter<std::string>("config_file",
        "/workspace/ros_ws/third-party/hikcamera_ros_driver/config/hikcamera_config.yaml");
    auto yaml_config_path = node->get_parameter("config_file").as_string();

    auto camera            = std::make_shared<hikcamera::Camera>();
    auto config            = hikcamera::Config();
    auto image_width       = 0;
    auto image_height      = 0;
    std::string shm_name;
    auto is_camera_running = std::atomic<bool>(true);
    auto camera_thread     = std::thread();
    if (auto result = hikcamera_ros_driver::ConfigsLoader(
            yaml_config_path, config, image_width, image_height, shm_name);
        !result) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load config: %s", result.error().c_str());
        return EXIT_FAILURE;
    }
    if (auto result = hikcamera_ros_driver::CameraThreadStart(config, is_camera_running, shm_name);
        !result) {
        RCLCPP_ERROR(node->get_logger(), "Failed to start camera: %s", result.error().c_str());
        return EXIT_FAILURE;
    } else {
        camera_thread = std::move(result.value());
    }
    rclcpp::spin(node);
    hikcamera_ros_driver::CameraThreadStop(camera_thread, is_camera_running);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
