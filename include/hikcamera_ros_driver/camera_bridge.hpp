#pragma once

#include <hikcamera/shm.hpp>
#include <atomic>
#include <expected>
#include <memory>
#include <string>
#include <thread>

#include <hikcamera/capturer.hpp>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hikcamera_ros_driver::camera_bridge {

class CameraBridge final {
public:
    CameraBridge() = default;
    ~CameraBridge();

    auto load_config(rclcpp::Node& node) -> std::expected<void, std::string>;
    auto camera_init()                  -> std::expected<void, std::string>;
    auto camera_connect()               -> std::expected<void, std::string>;
    auto shm_init()                     -> std::expected<void, std::string>;
    auto camera_shm_thread(rclcpp::Publisher<sensor_msgs::msg::Image>& image_pub)
        -> std::expected<void, std::string>;
    auto camera_shm_thread_stop()       -> std::expected<void, std::string>;

    std::string image_topic_ = "/hikcamera_image";

private:
    hikcamera::Config hik_config_;
    std::string shm_name_;
    int shm_fd_ = -1;
    std::shared_ptr<hikcamera::Camera> camera_ = nullptr;
    std::atomic<bool> is_camera_running_{false};
    std::thread camera_thread_;
};

} // namespace hikcamera_ros_driver::camera_bridge
