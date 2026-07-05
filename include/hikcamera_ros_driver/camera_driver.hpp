#include "hikcamera/capturer.hpp"
#include <atomic>
#include <expected>
#include <memory>
#include <semaphore.h>
#include <string>
#include <vector>
namespace hikcamera_ros_driver {
typedef struct imageSHM {
    unsigned char data[1920 * 1080][3];
    sem_t sem;
    pthread_mutex_t mutex;
    std::atomic<bool> is_shm_initialized;
    std::atomic<unsigned int> read_index;
    std::atomic<unsigned int> write_index;
} imageSHM;
auto ConfigsLoader(std::string& yaml_config_path, hikcamera::Config& config, int& image_width,
    int& image_height) -> std::expected<void, std::string>;
auto CameraThread(const hikcamera::Config& config) -> std::expected<void, std::string>;
auto CameraInit(const hikcamera::Config& config, std::shared_ptr<hikcamera::Camera> camera)
    -> std::expected<bool, std::string>;
}
auto SHMInit(const std::string& shm_path_name, size_t shm_size, size_t sem_num = 1)
    -> std::expected<int, std::string>;
auto SHMWrite(int shm_fd, const std::vector<unsigned char>& data)
    -> std::expected<void, std::string>;
auto SHMRead(int shm_fd, unsigned char (&data)[1920 * 1080][3]) -> std::expected<void, std::string>;
auto SHMUnlink(const std::string& shm_path_name) -> std::expected<bool, std::string>;
auto SHMClose(int shm_fd) -> std::expected<bool, std::string>;