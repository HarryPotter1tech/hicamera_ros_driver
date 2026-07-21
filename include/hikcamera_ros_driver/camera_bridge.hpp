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

namespace hikcamera_ros_driver::camera_bridge {

class CameraBridge final {
public:
    CameraBridge() = default;
    ~CameraBridge();

    auto load_config(rclcpp::Node& node) -> std::expected<void, std::string>;
    auto camera_init()                  -> std::expected<void, std::string>;
    auto camera_connect()               -> std::expected<void, std::string>;
    auto shm_init()                     -> std::expected<void, std::string>;
    auto camera_shm_thread()            -> std::expected<void, std::string>;
    auto camera_shm_thread_stop()       -> std::expected<void, std::string>;

private:
    hikcamera::Config hik_config_;
    std::string shm_name_;
    int shm_fd_ = -1;
    hikcamera::imageSHM* shm_ptr_ = nullptr;
    std::shared_ptr<hikcamera::Camera> camera_ = nullptr;
    std::atomic<bool> is_camera_running_{false};
    std::thread camera_thread_;
};

} // namespace hikcamera_ros_driver::camera_bridge
