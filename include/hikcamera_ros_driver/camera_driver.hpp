#include "hikcamera/capturer.hpp"
#include <atomic>
#include <expected>
#include <memory>
#include <queue>
#include <semaphore.h>
#include <string>
#include <thread>
#include <vector>
#define SLOT_NUM 4
namespace hikcamera_ros_driver {
typedef struct imageSHM {
    hikcamera::Camera::Image data[SLOT_NUM];
    sem_t sem;
    pthread_mutex_t mutex;
    std::atomic<bool> is_shm_initialized;
    unsigned int read_index  = -1;
    unsigned int write_index = -1;
    unsigned int counter     = 0;
} imageSHM;
auto ConfigsLoader(std::string& yaml_config_path, hikcamera::Config& config, int& image_width,
    int& image_height, std::string& shm_name) -> std::expected<void, std::string>;
auto CameraThreadStart(const hikcamera::Config& config, std::atomic<bool>& is_camera_running,
    const std::string& shm_name) -> std::expected<std::thread, std::string>;
auto CameraThreadStop(std::thread& camera_thread,
    std::atomic<bool>& is_camera_running) -> std::expected<void, std::string>;
auto CameraInit(const hikcamera::Config& config, std::shared_ptr<hikcamera::Camera> camera)
    -> std::expected<bool, std::string>;
auto SHMInit(const std::string& shm_path_name, size_t shm_size) -> std::expected<int, std::string>;
auto SHMWrite(int shm_fd, const hikcamera::Camera::Image& data) -> std::expected<void, std::string>;
auto SHMRead(int shm_fd, hikcamera::Camera::Image& data) -> std::expected<void, std::string>;
auto SHMUnlink(const std::string& shm_path_name) -> std::expected<bool, std::string>;
auto SHMClose(int shm_fd) -> std::expected<bool, std::string>;

} // namespace hikcamera_ros_driver