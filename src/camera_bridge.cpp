#include "hikcamera_ros_driver/camera_bridge.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <fcntl.h>
#include <std_msgs/msg/header.hpp>
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
    node.declare_parameter("brightness", hik_config_.brightness);
    node.declare_parameter("sharpness", hik_config_.sharpness);
    node.declare_parameter("timeout_ms", hik_config_.timeout_ms);
    node.declare_parameter("exposure_us", hik_config_.exposure_us);
    node.declare_parameter("framerate", hik_config_.framerate);
    node.declare_parameter("invert_image", hik_config_.invert_image);
    node.declare_parameter("software_sync", hik_config_.software_sync);
    node.declare_parameter("trigger_mode", hik_config_.trigger_mode);
    node.declare_parameter("fixed_framerate", hik_config_.fixed_framerate);
    node.declare_parameter("gain", hik_config_.gain);
    node.declare_parameter("shm_name", "/hikcamera_shm");
    node.declare_parameter("image_topic", "/hikcamera_image");

    node.get_parameter("white_balance_blue", hik_config_.white_balance_blue);
    node.get_parameter("white_balance_red", hik_config_.white_balance_red);
    node.get_parameter("white_balance_green", hik_config_.white_balance_green);
    node.get_parameter("auto_white_balance", hik_config_.auto_white_balance);
    node.get_parameter("brightness", hik_config_.brightness);
    node.get_parameter("sharpness", hik_config_.sharpness);
    node.get_parameter("timeout_ms", hik_config_.timeout_ms);
    node.get_parameter("exposure_us", hik_config_.exposure_us);
    node.get_parameter("framerate", hik_config_.framerate);
    node.get_parameter("invert_image", hik_config_.invert_image);
    node.get_parameter("software_sync", hik_config_.software_sync);
    node.get_parameter("trigger_mode", hik_config_.trigger_mode);
    node.get_parameter("fixed_framerate", hik_config_.fixed_framerate);
    node.get_parameter("gain", hik_config_.gain);
    shm_name_    = node.get_parameter("shm_name").as_string();
    image_topic_ = node.get_parameter("image_topic").as_string();
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
    auto ret = SHMInit(shm_name_, sizeof(imageSHM));
    if (!ret.has_value()) {
        return std::unexpected(ret.error());
    }
    shm_fd_ = ret.value();
    return { };
}

auto CameraBridge::camera_shm_thread(rclcpp::Publisher<sensor_msgs::msg::Image>& image_pub)
    -> std::expected<void, std::string> {
    if (!camera_ || shm_fd_ == -1) {
        return std::unexpected("camera or shm not initialized");
    }

    is_camera_running_ = true;
    camera_thread_ = std::thread([this, &image_pub]() {
        while (is_camera_running_.load(std::memory_order_acquire)
            && camera_->connected()) {
            auto imagedata = camera_->read_image_with_timestamp();
            if (!imagedata) break;

            // ROS publish first (real-time priority)
            builtin_interfaces::msg::Time stamp;
            stamp.sec = static_cast<int32_t>(
                std::chrono::duration_cast<std::chrono::seconds>(
                    imagedata->timestamp.time_since_epoch()).count());
            stamp.nanosec = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    imagedata->timestamp.time_since_epoch()).count() % 1000000000);
            auto msg = cv_bridge::CvImage(
                std_msgs::msg::Header(), "bgr8", imagedata->mat).toImageMsg();
            msg->header.stamp = stamp;
            image_pub.publish(*msg);

            // SHM write second
            auto write_ret = SHMWrite(shm_fd_, imagedata.value());
            if (!write_ret.has_value()) {
                RCLCPP_ERROR(rclcpp::get_logger("CameraBridge"),
                    "SHMWrite failed: %s", write_ret.error().c_str());
                break;
            }
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
    if (shm_fd_ != -1) SHMClose(shm_fd_);
    shm_fd_ = -1;
    camera_.reset();
    return { };
}

} // namespace hikcamera_ros_driver::camera_bridge
