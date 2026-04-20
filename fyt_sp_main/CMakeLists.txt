cmake_minimum_required(VERSION 3.16.3)

project(sp_vision_25 CXX CUDA)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CUDA_ARCHITECTURES 87)

message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

# OpenCV
find_package(OpenCV 4 REQUIRED
  COMPONENTS
    core
    imgproc
    highgui
    dnn
    calib3d
    cudawarping
    cudaarithm
    cudaimgproc
)

# Common deps
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Ceres REQUIRED)

# CUDA
set(CUDAToolkit_ROOT "/usr/local/cuda")
find_package(CUDA REQUIRED)

# ROS2 / DDS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

# TensorRT
set(TENSORRT_ROOT /usr)
include_directories(${TENSORRT_ROOT}/include)

find_library(NVINFER_LIB nvinfer HINTS ${TENSORRT_ROOT}/lib ${TENSORRT_ROOT}/lib/aarch64-linux-gnu)
find_library(NVINFER_PLUGIN_LIB nvinfer_plugin HINTS ${TENSORRT_ROOT}/lib ${TENSORRT_ROOT}/lib/aarch64-linux-gnu)
find_library(NVONNXPARSER_LIB nvonnxparser HINTS ${TENSORRT_ROOT}/lib ${TENSORRT_ROOT}/lib/aarch64-linux-gnu)
find_library(NVINFER_BUILDER_LIB nvinfer_builder_resource HINTS ${TENSORRT_ROOT}/lib ${TENSORRT_ROOT}/lib/aarch64-linux-gnu)
find_library(NVINFER_RUNTIME_LIB nvinfer_runtime HINTS ${TENSORRT_ROOT}/lib ${TENSORRT_ROOT}/lib/aarch64-linux-gnu)

set(TENSORRT_LIBS
    ${NVINFER_LIB}
    ${NVINFER_PLUGIN_LIB}
    ${NVONNXPARSER_LIB}
    ${NVINFER_BUILDER_LIB}
    ${NVINFER_RUNTIME_LIB}
)

message(STATUS "TensorRT libs: ${TENSORRT_LIBS}")

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)

# 离线视频测试
add_executable(auto_buff_test tests/auto_buff_test.cpp)
target_link_libraries(auto_buff_test
    ${OpenCV_LIBS}
    fmt::fmt
    yaml-cpp
    auto_buff
    tools
    io
)

# 模拟器联调
add_executable(auto_buff_sim src/auto_buff_sim.cpp)
target_link_libraries(auto_buff_sim
    ${OpenCV_LIBS}
    fmt::fmt
    yaml-cpp
    auto_buff
    tools
    io
)
ament_target_dependencies(auto_buff_sim
    rclcpp
    std_msgs
    sensor_msgs
    tf2_msgs
    builtin_interfaces
)

# 只看模型输出 / 关键点 / 选 target / solvePnP，不发控制命令
add_executable(auto_buff_debug_view src/auto_buff_debug_view.cpp)
target_link_libraries(auto_buff_debug_view
    ${OpenCV_LIBS}
    fmt::fmt
    yaml-cpp
    auto_buff
    tools
    io
)
ament_target_dependencies(auto_buff_debug_view
    rclcpp
    std_msgs
    sensor_msgs
    tf2_msgs
    builtin_interfaces
)

# 如果你后面还想临时验证 auto_aim，也可以打开这个
# add_executable(auto_aim_test tests/auto_aim_test.cpp)
# target_link_libraries(auto_aim_test
#     ${OpenCV_LIBS}
#     fmt::fmt
#     yaml-cpp
#     tools
#     auto_aim
# )