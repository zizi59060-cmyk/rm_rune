cmake_minimum_required(VERSION 3.16)

add_subdirectory(planner/tinympc)

enable_language(CUDA)
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} --expt-relaxed-constexpr")
set_source_files_properties(tasks/auto_aim/multithread/mt_detector.cu PROPERTIES LANGUAGE CUDA)

add_library(auto_aim OBJECT
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cu
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

set(TENSORRT_LIB_DIR "/usr/lib/aarch64-linux-gnu")
set(TENSORRT_INCLUDE_DIR "/usr/include/aarch64-linux-gnu")

target_include_directories(auto_aim PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${TENSORRT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
)

target_link_libraries(auto_aim PUBLIC
    tinympcstatic
    ${OpenCV_LIBS}
    nvinfer
    cudart
    ${CUDA_LIBRARIES}
    ${TENSORRT_LIB_DIR}/libnvinfer.so
    ${TENSORRT_LIB_DIR}/libnvinfer_plugin.so
    ${TENSORRT_LIB_DIR}/libnvonnxparser.so
)

set_target_properties(auto_aim PROPERTIES
    CXX_STANDARD 17
    CUDA_STANDARD 17
)