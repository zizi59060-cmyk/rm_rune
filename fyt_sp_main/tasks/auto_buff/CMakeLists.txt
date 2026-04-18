cmake_minimum_required(VERSION 3.16)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Ceres REQUIRED)
enable_language(CUDA)

add_library(auto_buff SHARED
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolox_buff.cu
)

target_link_libraries(auto_buff PRIVATE
    ${CERES_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    fmt::fmt
    tools
    ${NVINFER_LIB}
    ${NVINFER_PLUGIN_LIB}
    ${NVONNXPARSER_LIB}
)

target_include_directories(auto_buff PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${TENSORRT_ROOT}/include
)

set_target_properties(auto_buff PROPERTIES
    CXX_STANDARD 17
    CUDA_STANDARD 17
)