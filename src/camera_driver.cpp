#include "hikcamera_ros_driver/camera_driver.hpp"
#include "yaml-cpp/yaml.h"
#include <expected>
#include <fcntl.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>
namespace hikcamera_ros_driver {

// ── Config ──

auto ConfigsLoader(std::string& yaml_config_path, hikcamera::Config& config, int& image_width,
    int& image_height) -> std::expected<void, std::string> {
    auto yaml_config           = YAML::LoadFile(yaml_config_path);
    config.white_balance_blue  = yaml_config["Hikcamera"]["white_balance_blue"].as<unsigned int>();
    config.white_balance_red   = yaml_config["Hikcamera"]["white_balance_red"].as<unsigned int>();
    config.white_balance_green = yaml_config["Hikcamera"]["white_balance_green"].as<unsigned int>();
    config.auto_white_balance  = yaml_config["Hikcamera"]["auto_white_balance"].as<unsigned int>();
    config.brightness          = yaml_config["Hikcamera"]["brightness"].as<unsigned int>();
    config.exposure_us         = yaml_config["Hikcamera"]["exposure_time"].as<float>();
    config.framerate           = yaml_config["Hikcamera"]["fps"].as<float>();
    config.sharpness           = yaml_config["Hikcamera"]["sharpness"].as<unsigned int>();
    config.timeout_ms          = yaml_config["Hikcamera"]["timeout_ms"].as<unsigned int>();
    config.invert_image        = yaml_config["Hikcamera"]["invert_image"].as<bool>();
    config.software_sync       = yaml_config["Hikcamera"]["software_sync"].as<bool>();
    config.trigger_mode        = yaml_config["Hikcamera"]["trigger_mode"].as<bool>();
    config.fixed_framerate     = yaml_config["Hikcamera"]["fixed_framerate"].as<bool>();
    config.gain                = yaml_config["Hikcamera"]["gain"].as<float>();
    image_height               = yaml_config["Hikcamera"]["image_height"].as<int>();
    image_width                = yaml_config["Hikcamera"]["image_width"].as<int>();
    return { };
}

// ── Camera ──

auto CameraInit(const hikcamera::Config& config, std::shared_ptr<hikcamera::Camera> camera)
    -> std::expected<bool, std::string> {
    if (auto result = camera->initialize(config); !result) return std::unexpected(result.error());
    return { true };
}

auto CameraThreadStart(const hikcamera::Config& config, std::atomic<bool>& is_camera_running)
    -> std::expected<std::thread, std::string> {
    auto camera = std::make_shared<hikcamera::Camera>();
    std::expected<bool, std::string> camera_init_result;
    std::expected<int, std::string> shm_init_result;
    std::expected<hikcamera::Camera::Image, std::string> imagedata;
    if (camera_init_result = CameraInit(config, camera); !camera_init_result) {
        return std::unexpected(camera_init_result.error());
    }
    if (shm_init_result = SHMInit("/hikcamera_shm", sizeof(imageSHM)); !shm_init_result) {
        return std::unexpected(shm_init_result.error());
    }
    std::thread camera_thread([&]() {
        while (is_camera_running.load(std::memory_order_acquire)) {
            if (imagedata = camera->read_image_with_timestamp(); !imagedata) {
                break;
            }
            if (auto result = SHMWrite(shm_init_result.value(), imagedata.value()); !result) {
                break;
            }
        }
    });
    return camera_thread;
}
auto CameraThreadStop(std::thread& camera_thread,
    std::atomic<bool>& is_camera_running) -> std::expected<void, std::string> {
    is_camera_running.store(false, std::memory_order_release);
    if (camera_thread.joinable()) {
        camera_thread.join();
    }
    return { };
}

// ── SHM ──

auto SHMInit(const std::string& shm_path_name, size_t shm_size) -> std::expected<int, std::string> {
    int shm_fd = shm_open(shm_path_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        return std::unexpected("Failed to create shared memory object");
    }
    if (ftruncate(shm_fd, shm_size) == -1) {
        return std::unexpected("Failed to set size of shared memory object");
    }
    auto image_shm = reinterpret_cast<hikcamera_ros_driver::imageSHM*>(
        mmap(nullptr, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    pthread_mutexattr_t mutex_attr;
    pthread_mutexattr_init(&mutex_attr);
    pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&image_shm->mutex, &mutex_attr);
    sem_init(&image_shm->sem, 1, 0);
    image_shm->read_index         = -1;
    image_shm->write_index        = -1;
    image_shm->counter            = 1;
    image_shm->is_shm_initialized = true;
    return { shm_fd };
}

auto SHMWrite(int shm_fd, const hikcamera::Camera::Image& data)
    -> std::expected<void, std::string> {
    auto image_shm = reinterpret_cast<hikcamera_ros_driver::imageSHM*>(mmap(nullptr,
        sizeof(hikcamera_ros_driver::imageSHM), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (image_shm == MAP_FAILED) {
        return std::unexpected("Failed to map shared memory object");
    }
    pthread_mutex_lock(&image_shm->mutex);
    // std::memcpy(image_shm->data, data.data(), data.size());
    image_shm->write_index                  = (image_shm->write_index + 1) % SLOT_NUM;
    image_shm->data[image_shm->write_index] = data;
    pthread_mutex_unlock(&image_shm->mutex);
    sem_post(&image_shm->sem);
    munmap(image_shm, sizeof(hikcamera_ros_driver::imageSHM));
    return { };
}

auto SHMRead(int shm_fd, hikcamera::Camera::Image& data) -> std::expected<void, std::string> {
    auto image_shm = reinterpret_cast<hikcamera_ros_driver::imageSHM*>(mmap(nullptr,
        sizeof(hikcamera_ros_driver::imageSHM), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0));
    if (image_shm == MAP_FAILED) {
        return std::unexpected("Failed to map shared memory object");
    }
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 1; // Wait for 1 second
    sem_timedwait(&image_shm->sem, &timeout);
    pthread_mutex_lock(&image_shm->mutex);
    if (image_shm->is_shm_initialized.load(std::memory_order_acquire) == 1) {
        if (image_shm->read_index >= image_shm->write_index) {
            image_shm->read_index = image_shm->write_index;
        } else {
            image_shm->read_index++;
        }
        data = image_shm->data[image_shm->read_index];
    } else {
        pthread_mutex_unlock(&image_shm->mutex);
        munmap(image_shm, sizeof(hikcamera_ros_driver::imageSHM));
        return std::unexpected("Shared memory is not initialized");
    }
    pthread_mutex_unlock(&image_shm->mutex);
    munmap(image_shm, sizeof(hikcamera_ros_driver::imageSHM));
    return { };
}

auto SHMClose(int shm_fd) -> std::expected<bool, std::string> {
    if (close(shm_fd) == -1) {
        return std::unexpected("Failed to close shared memory object");
    }
    return { true };
}

auto SHMUnlink(const std::string& shm_path_name) -> std::expected<bool, std::string> {
    if (shm_unlink(shm_path_name.c_str()) == -1) {
        return std::unexpected("Failed to unlink shared memory object");
    }
    return { true };
}
}