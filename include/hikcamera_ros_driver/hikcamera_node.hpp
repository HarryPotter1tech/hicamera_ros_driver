#pragma once

#include "hikcamera_ros_driver/camera_bridge.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hikcamera_ros_driver {

class HikCameraNode final : public rclcpp::Node {
public:
    HikCameraNode();
    ~HikCameraNode() override;

private:
    camera_bridge::CameraBridge camera_shm_bridge_ { };
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

} // namespace hikcamera_ros_driver
