#include "hikcamera_ros_driver/hikcamera_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<hikcamera_ros_driver::HikCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
