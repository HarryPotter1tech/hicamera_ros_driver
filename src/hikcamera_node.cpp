#include "hikcamera_ros_driver/hikcamera_node.hpp"

#include <cv_bridge/cv_bridge.hpp>

namespace hikcamera_ros_driver {

HikCameraNode::HikCameraNode()
    : Node("hikcamera_ros_driver_node") {
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_shm_bridge_.image_topic_, 10);

    auto ret = camera_shm_bridge_.load_config(*this);
    if (!ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "load_config failed: %s", ret.error().c_str());
        return;
    }

    ret = camera_shm_bridge_.camera_init();
    if (!ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "camera_init failed: %s", ret.error().c_str());
        return;
    }

    ret = camera_shm_bridge_.camera_connect();
    if (!ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "camera_connect failed: %s", ret.error().c_str());
        return;
    }

    ret = camera_shm_bridge_.shm_init();
    if (!ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "shm_init failed: %s", ret.error().c_str());
        return;
    }

    ret = camera_shm_bridge_.camera_shm_thread(*image_pub_);
    if (!ret.has_value()) {
        RCLCPP_ERROR(this->get_logger(), "camera_shm_thread failed: %s", ret.error().c_str());
    }
}

HikCameraNode::~HikCameraNode() = default;

} // namespace hikcamera_ros_driver
