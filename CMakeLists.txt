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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(hikcamera  REQUIRED)
find_package(cv_bridge REQUIRED)

file(GLOB SOURCES "src/*.cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(hikcamera_ros_driver ${SOURCES})
target_link_libraries(hikcamera_ros_driver 
    ${OpenCV_LIBS}
    Eigen3::Eigen
)
ament_target_dependencies(hikcamera_ros_driver 
    rclcpp
    std_msgs
    hikcamera
    cv_bridge
)
install (TARGETS hikcamera_ros_driver DESTINATION lib/${PROJECT_NAME})
ament_package()
