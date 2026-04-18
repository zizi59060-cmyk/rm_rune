/* tasks/auto_buff/yolox_buff.hpp */
#pragma once

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Dense> 

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>

#include "trt_logger.hpp"
#include "trt_utils.hpp"
#include "tools/logger.hpp"

namespace auto_buff
{
struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

struct RuneObject
{
  cv::Rect2f box;
  int color;
  int type;
  float prob;
  std::vector<cv::Point2f> pts; 
};


class YoloXBuff
{
public:
  struct Object
  {
    cv::Rect_<float> rect;
    int label;
    float prob;
    std::vector<cv::Point2f> kpt; 
  };

  YoloXBuff(const std::string & config);
  ~YoloXBuff();

  std::vector<Object> get_multicandidateboxes(cv::Mat & image);
  std::vector<Object> get_onecandidatebox(cv::Mat & image);


private:
  // --- [!!! 修复 !!!] ---
  // 移除硬编码的阈值，改为成员变量
  float m_conf_threshold = 0.25f; // (fyt 默认值)
  float m_nms_threshold = 0.3f;   // (fyt 默认值)
  // -------------------------

  TrtLogger mLogger_;
  TrtUniquePtr<nvinfer1::IRuntime> mRuntime;
  TrtUniquePtr<nvinfer1::ICudaEngine> mEngine;
  TrtUniquePtr<nvinfer1::IExecutionContext> mContext;
  cudaStream_t mStream = nullptr;

  std::string mInputTensorName_;
  std::string mOutputTensorName_;
  nvinfer1::Dims mInputDims_{};
  nvinfer1::Dims mOutputDims_{};
  size_t mInputBytes_ = 0;
  size_t mOutputBytes_ = 0;

  void * mGpuBuffers_[2] = {nullptr, nullptr};
  std::vector<float> mCpuOutputBuffer_;

  std::vector<int> strides_;
  std::vector<GridAndStride> grid_strides_;
  
  const int NUM_POINTS = 5; 

  void allocateBindings();
  void freeBindings();

  cv::Mat letterbox(
    const cv::Mat & img, Eigen::Matrix3f & transform_matrix,
    std::vector<int> new_shape);

  void generateGridsAndStride(
    const int target_w, const int target_h, std::vector<int> & strides,
    std::vector<GridAndStride> & grid_strides);
  
  static inline float intersectionArea(const RuneObject & a, const RuneObject & b);
    
  void save(const std::string & programName, const cv::Mat & image);
};
}  // namespace auto_buff