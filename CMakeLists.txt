cmake_minimum_required(VERSION 3.14)
project(hikcamera_ros_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(hikcamera  REQUIRED)
find_package(yaml-cpp REQUIRED)

file(GLOB SOURCES "src/*.cpp")

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    hikcamera
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
