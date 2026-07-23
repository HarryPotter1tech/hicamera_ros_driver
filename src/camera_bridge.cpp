#include "hikcamera_ros_driver/camera_bridge.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace hikcamera_ros_driver::camera_bridge {

using namespace hikcamera;

CameraBridge::~CameraBridge() { auto _ = camera_shm_thread_stop(); }

auto CameraBridge::load_config(rclcpp::Node& node) -> std::expected<void, std::string> {
    node.declare_parameter("white_balance_blue", hik_config_.white_balance_blue);
    node.declare_parameter("white_balance_red", hik_config_.white_balance_red);
    node.declare_parameter("white_balance_green", hik_config_.white_balance_green);
    node.declare_parameter("auto_white_balance", hik_config_.auto_white_balance);
    node.declare_parameter("timeout_ms", hik_config_.timeout_ms);
    node.declare_parameter("exposure_us", hik_config_.exposure_us);
    node.declare_parameter("framerate", hik_config_.framerate);
    node.declare_parameter("width", hik_config_.width);
    node.declare_parameter("height", hik_config_.height);
    node.declare_parameter("invert_image", hik_config_.invert_image);
    node.declare_parameter("software_sync", hik_config_.software_sync);
    node.declare_parameter("trigger_mode", hik_config_.trigger_mode);
    node.declare_parameter("fixed_framerate", hik_config_.fixed_framerate);
    node.declare_parameter("gain", hik_config_.gain);
    node.declare_parameter("shm_name", "/hikcamera_shm");

    node.get_parameter("white_balance_blue", hik_config_.white_balance_blue);
    node.get_parameter("white_balance_red", hik_config_.white_balance_red);
    node.get_parameter("white_balance_green", hik_config_.white_balance_green);
    node.get_parameter("auto_white_balance", hik_config_.auto_white_balance);
    node.get_parameter("timeout_ms", hik_config_.timeout_ms);
    node.get_parameter("exposure_us", hik_config_.exposure_us);
    node.get_parameter("framerate", hik_config_.framerate);
    node.get_parameter("width", hik_config_.width);
    node.get_parameter("height", hik_config_.height);
    node.get_parameter("invert_image", hik_config_.invert_image);
    node.get_parameter("software_sync", hik_config_.software_sync);
    node.get_parameter("trigger_mode", hik_config_.trigger_mode);
    node.get_parameter("fixed_framerate", hik_config_.fixed_framerate);
    node.get_parameter("gain", hik_config_.gain);
    shm_name_ = node.get_parameter("shm_name").as_string();
    return { };
}

auto CameraBridge::camera_init() -> std::expected<void, std::string> {
    camera_ = std::make_shared<hikcamera::Camera>();
    camera_->configure(hik_config_);
    return { };
}

auto CameraBridge::camera_connect() -> std::expected<void, std::string> {
    return camera_->connect();
}

auto CameraBridge::shm_init() -> std::expected<void, std::string> {
    auto fd_ret = SHMInit(shm_name_, sizeof(imageSHM));
    if (!fd_ret.has_value()) {
        return std::unexpected(fd_ret.error());
    }
    shm_fd_ = fd_ret.value();

    auto ptr_ret = SHMGetPtr(shm_fd_);
    if (!ptr_ret.has_value()) {
        SHMClose(shm_fd_);
        shm_fd_ = -1;
        return std::unexpected(ptr_ret.error());
    }
    shm_ptr_ = ptr_ret.value();
    return { };
}

auto CameraBridge::camera_shm_thread()
    -> std::expected<void, std::string> {
    if (!camera_ || shm_fd_ == -1) {
        return std::unexpected("camera or shm not initialized");
    }

    is_camera_running_ = true;
    camera_thread_ = std::thread([this]() {
        while (is_camera_running_.load(std::memory_order_acquire)
            && camera_->connected()) {
            auto t0 = std::chrono::steady_clock::now();

            auto write_ret = SHMWrite(shm_ptr_, *camera_);
            if (!write_ret.has_value()) {
                RCLCPP_WARN(rclcpp::get_logger("CameraBridge"),
                    "SHMWrite failed (retrying): %s", write_ret.error().c_str());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            auto loop_time = std::chrono::steady_clock::now() - t0;
            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_time).count();
            auto frame = shm_ptr_->frame_counter.load(std::memory_order_relaxed);
            RCLCPP_INFO(rclcpp::get_logger("CameraBridge"),
                "TIMING: frame=%lu loop=%ldus (%.1ffps)",
                (unsigned long)frame, (long)elapsed_us, 1e6 / elapsed_us);
        }

        if (!camera_->connected()) {
            RCLCPP_ERROR(rclcpp::get_logger("CameraBridge"), "camera disconnected");
            rclcpp::shutdown();
        }
    });
    return { };
}

auto CameraBridge::camera_shm_thread_stop() -> std::expected<void, std::string> {
    is_camera_running_ = false;
    if (camera_thread_.joinable()) camera_thread_.join();
    if (shm_ptr_) SHMReleasePtr(shm_ptr_);
    shm_ptr_ = nullptr;
    if (shm_fd_ != -1) SHMClose(shm_fd_);
    shm_fd_ = -1;
    camera_.reset();
    return { };
}

} // namespace hikcamera_ros_driver::camera_bridge
