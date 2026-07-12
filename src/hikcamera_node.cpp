#include "hikcamera_ros_driver/hikcamera_node.hpp"
#include "hikcamera_ros_driver/camera_driver.hpp"
#include <hikcamera/capturer.hpp>
#include <rclcpp/rclcpp.hpp>

namespace hikcamera_ros_driver {

HikCameraNode::HikCameraNode()
    : Node("hikcamera_ros_driver_node") {
    auto config       = hikcamera::Config();
    int image_width   = 0;
    int image_height  = 0;
    std::string shm_name;

    if (auto result = ConfigsLoader(*this, config, image_width, image_height, shm_name);
        !result) {
        RCLCPP_ERROR(this->get_logger(), "ConfigsLoader failed: %s", result.error().c_str());
        return;
    }

    if (auto result = CameraThreadStart(config, is_camera_running_, shm_name);
        !result) {
        RCLCPP_ERROR(this->get_logger(), "CameraThreadStart failed: %s", result.error().c_str());
    } else {
        camera_thread_ = std::move(result.value());
    }
}

HikCameraNode::~HikCameraNode() {
    CameraThreadStop(camera_thread_, is_camera_running_);
}

} // namespace hikcamera_ros_driver
