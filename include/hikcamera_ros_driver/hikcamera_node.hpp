#pragma once

#include <atomic>
#include <thread>

#include <rclcpp/rclcpp.hpp>

namespace hikcamera_ros_driver {

class HikCameraNode final : public rclcpp::Node {
public:
    HikCameraNode();
    ~HikCameraNode() override;

private:
    std::atomic<bool> is_camera_running_{true};
    std::thread camera_thread_;
};

} // namespace hikcamera_ros_driver
