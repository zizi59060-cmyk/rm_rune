cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

# 为了避免 fastcdr/fastrtps 的 target 在生成阶段丢失
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

add_library(io STATIC
  simboard.cpp
  ros2camera.cpp
)

target_include_directories(io PUBLIC
  ${PROJECT_SOURCE_DIR}
)

target_link_libraries(io
  ${OpenCV_LIBS}
  yaml-cpp
  fmt::fmt
  tools
)

ament_target_dependencies(io
  rclcpp
  std_msgs
  sensor_msgs
  tf2_msgs
  builtin_interfaces
)