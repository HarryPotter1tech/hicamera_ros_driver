#include "hikcamera/capturer.hpp"
#include <atomic>
#include <chrono>
#include <expected>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <semaphore.h>
#include <string>
#include <thread>

#define SLOT_NUM 4
#define MAX_IMAGE_SIZE 36864000
namespace hikcamera_ros_driver {
typedef struct imageSHM {
    sem_t sem;
    pthread_mutex_t mutex;
    std::atomic<bool> is_shm_initialized;
    unsigned int read_index  = -1;
    unsigned int write_index = -1;
    unsigned int counter     = 0;
    std::chrono::steady_clock::time_point timestamp[SLOT_NUM];
    char imagedata[SLOT_NUM][MAX_IMAGE_SIZE];
} imageSHM;
auto ConfigsLoader(rclcpp::Node& node, hikcamera::Config& config, int& image_width,
    int& image_height, std::string& shm_name) -> std::expected<void, std::string>;
auto CameraThreadStart(const hikcamera::Config& config, std::atomic<bool>& is_camera_running,
    const std::string& shm_name) -> std::expected<std::thread, std::string>;
auto CameraThreadStop(std::thread& camera_thread, std::atomic<bool>& is_camera_running)
    -> std::expected<void, std::string>;
auto CameraInit(const hikcamera::Config& config, std::shared_ptr<hikcamera::Camera> camera)
    -> std::expected<bool, std::string>;
auto SHMInit(const std::string& shm_path_name, size_t shm_size) -> std::expected<int, std::string>;
auto SHMWrite(int shm_fd, const hikcamera::Camera::Image& data) -> std::expected<void, std::string>;
auto SHMRead(int shm_fd, cv::Mat& out_mat, std::chrono::steady_clock::time_point& out_ts,
    int width, int height) -> std::expected<void, std::string>;
auto SHMUnlink(const std::string& shm_path_name) -> std::expected<bool, std::string>;
auto SHMClose(int shm_fd) -> std::expected<bool, std::string>;

class HikCameraNode final : public rclcpp::Node {
public:
    HikCameraNode();
    ~HikCameraNode() override;

private:
    std::atomic<bool> is_camera_running_{true};
    std::thread camera_thread_;
};

} // namespace hikcamera_ros_driver