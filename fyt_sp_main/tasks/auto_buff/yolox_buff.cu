/* tasks/auto_buff/yolox_buff.cu */
#include "yolox_buff.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <numeric> 
#include <unordered_map> 

#include <opencv2/dnn.hpp> 
#include <opencv2/imgproc.hpp>

#include "tools/logger.hpp" 

// ==================================================================
// YOLOX (fyt) 解析逻辑所需的常量
// [!!! 修复 !!!] 我们使用 480x480
// ==================================================================
static constexpr int INPUT_W = 480;   // <-- [修复]
static constexpr int INPUT_H = 480;   // <-- [修复]
static constexpr int NUM_CLASSES = 2; 
static constexpr int NUM_COLORS = 2;  
static constexpr int NUM_POINTS_STATIC = 5; 
static constexpr int NUM_POINTS_2_STATIC = 2 * NUM_POINTS_STATIC; 

static std::unordered_map<int, int> DNN_COLOR_TO_ENEMY_COLOR = {
    {0, 1}, {1, 0}}; 

namespace auto_buff
{

// ... (letterbox, generateGridsAndStride, intersectionArea 函数保持不变) ...
cv::Mat YoloXBuff::letterbox(
  const cv::Mat & img, Eigen::Matrix3f & transform_matrix,
  std::vector<int> new_shape)
{
  int img_h = img.rows;
  int img_w = img.cols;

  float scale =
    std::min(new_shape[1] * 1.0 / img_h, new_shape[0] * 1.0 / img_w);
  int resize_h = static_cast<int>(round(img_h * scale));
  int resize_w = static_cast<int>(round(img_w * scale));

  int pad_h = new_shape[1] - resize_h;
  int pad_w = new_shape[0] - resize_w;

  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(resize_w, resize_h));

  float half_h = pad_h * 1.0 / 2;
  float half_w = pad_w * 1.0 / 2;

  int top = static_cast<int>(round(half_h - 0.1));
  int bottom = static_cast<int>(round(half_h + 0.1));
  int left = static_cast<int>(round(half_w - 0.1));
  int right = static_cast<int>(round(half_w + 0.1));

  /* clang-format off */
  transform_matrix << 1.0 / scale, 0, -half_w / scale,
                      0, 1.0 / scale, -half_h / scale,
                      0, 0, 1;
  /* clang-format on */

  cv::copyMakeBorder(
    resized_img, resized_img, top, bottom, left, right, cv::BORDER_CONSTANT,
    cv::Scalar(114, 114, 114));

  return resized_img;
}

void YoloXBuff::generateGridsAndStride(
  const int target_w, const int target_h, std::vector<int> & strides,
  std::vector<GridAndStride> & grid_strides)
{
  grid_strides.clear();
  for (auto stride : strides) {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;

    for (int g1 = 0; g1 < num_grid_h; g1++) {
      for (int g0 = 0; g0 < num_grid_w; g0++) {
        grid_strides.emplace_back(GridAndStride{g0, g1, stride});
      }
    }
  }
}

inline float YoloXBuff::intersectionArea(const RuneObject & a, const RuneObject & b)
{
  cv::Rect_<float> inter = a.box & b.box;
  return inter.area();
}
// ... (以上函数保持不变) ...


// ==================================================================
// TRT 构造/析构/内存
// ==================================================================
YoloXBuff::YoloXBuff(const std::string & config)
{
  try {
    auto yaml = YAML::LoadFile(config);
    std::string model_path = yaml["buff_model_path"].as<std::string>();

    if (yaml["auto_buff"]["conf_threshold"]) {
      m_conf_threshold = yaml["auto_buff"]["conf_threshold"].as<float>();
    }
    if (yaml["auto_buff"]["nms_threshold"]) {
      m_nms_threshold = yaml["auto_buff"]["nms_threshold"].as<float>();
    }
    tools::logger()->info("YoloXBuff (TRT): Conf_threshold loaded: {}", m_conf_threshold);
    tools::logger()->info("YoloXBuff (TRT): NMS_threshold loaded: {}", m_nms_threshold);

    std::ifstream engineFile(model_path, std::ios::binary);
    if (!engineFile.is_open()) {
      throw std::runtime_error("Failed to open engine file: " + model_path);
    }
    engineFile.seekg(0, std::ios::end);
    std::streamsize fsize = engineFile.tellg();
    engineFile.seekg(0, std::ios::beg);
    std::vector<char> engineData(static_cast<size_t>(fsize));
    if (!engineFile.read(engineData.data(), fsize)) {
      throw std::runtime_error("Failed to read engine data from: " + model_path);
    }
    engineFile.close();

    mRuntime.reset(nvinfer1::createInferRuntime(mLogger_));
    if (!mRuntime) { throw std::runtime_error("createInferRuntime failed."); }
    mEngine.reset(mRuntime->deserializeCudaEngine(engineData.data(), engineData.size()));
    if (!mEngine) { throw std::runtime_error("deserializeCudaEngine failed."); }

    mContext.reset(mEngine->createExecutionContext());
    if (!mContext) { throw std::runtime_error("createExecutionContext failed."); }
    CHECK_CUDA_BUFF(cudaStreamCreate(&mStream));

    allocateBindings();

    strides_ = {8, 16, 32}; 
    generateGridsAndStride(INPUT_W, INPUT_H, strides_, grid_strides_);

    tools::logger()->info("YoloXBuff (TRT): Engine loaded. Grids generated: {}", grid_strides_.size());

  } catch (const std::exception & e) {
    tools::logger()->error("YoloXBuff (TRT): Exception in ctor: {}", e.what());
    throw;
  }
}

YoloXBuff::~YoloXBuff()
{
  try {
    tools::logger()->info("YoloXBuff (TRT): Releasing resources...");
    freeBindings();
    if (mStream) CHECK_CUDA_BUFF(cudaStreamDestroy(mStream));
  } catch (...) {}
}

void YoloXBuff::allocateBindings()
{
  if (!mEngine) return;
  int nbIOTensors = mEngine->getNbIOTensors();

  for (int i = 0; i < nbIOTensors; ++i) {
    const char * tname = mEngine->getIOTensorName(i);
    std::string name(tname);
    nvinfer1::DataType dtype = mEngine->getTensorDataType(name.c_str());
    nvinfer1::TensorIOMode mode = mEngine->getTensorIOMode(name.c_str());
    nvinfer1::Dims dims = mEngine->getTensorShape(name.c_str());
    size_t elems = getTensorVolume(dims);
    size_t bytes = elems * getElementSize(dtype);

    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      mInputTensorName_ = name;
      mInputDims_ = dims;
      mInputBytes_ = bytes;
      CHECK_CUDA_BUFF(cudaMalloc(&mGpuBuffers_[0], bytes));
    } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
      mOutputTensorName_ = name;
      mOutputDims_ = dims;
      mOutputBytes_ = bytes;
      CHECK_CUDA_BUFF(cudaMalloc(&mGpuBuffers_[1], bytes));
      mCpuOutputBuffer_.resize(elems); 
    }
  }
}

void YoloXBuff::freeBindings()
{
  if (mGpuBuffers_[0]) CHECK_CUDA_BUFF(cudaFree(mGpuBuffers_[0]));
  if (mGpuBuffers_[1]) CHECK_CUDA_BUFF(cudaFree(mGpuBuffers_[1]));
  mGpuBuffers_[0] = nullptr;
  mGpuBuffers_[1] = nullptr;
  mCpuOutputBuffer_.clear();
}


// ==================================================================
// YOLOX 解析 (来自 fyt) 和 推理 (来自 sp)
// ==================================================================

static void generateProposals_fyt(
  std::vector<RuneObject> & output_objs, std::vector<cv::Rect> & rects,
  std::vector<float> & scores,
  const cv::Mat & output_buffer,
  const Eigen::Matrix<float, 3, 3> & transform_matrix, float conf_threshold,
  const std::vector<GridAndStride>& grid_strides) 
{
  const int num_anchors = grid_strides.size();

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {
    float confidence = output_buffer.at<float>(anchor_idx, NUM_POINTS_2_STATIC);
    if (confidence < conf_threshold) { 
      continue;
    }

    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    double color_score, class_score;
    cv::Point color_id, class_id;
    cv::Mat color_scores =
      output_buffer.row(anchor_idx)
        .colRange(NUM_POINTS_2_STATIC + 1, NUM_POINTS_2_STATIC + 1 + NUM_COLORS);
    cv::Mat num_scores =
      output_buffer.row(anchor_idx)
        .colRange(NUM_POINTS_2_STATIC + 1 + NUM_COLORS,
                  NUM_POINTS_2_STATIC + 1 + NUM_COLORS + NUM_CLASSES);
    
    cv::minMaxLoc(color_scores, NULL, &color_score, NULL, &color_id);
    cv::minMaxLoc(num_scores, NULL, &class_score, NULL, &class_id);

    Eigen::Matrix<float, 3, 5> apex_norm;
    Eigen::Matrix<float, 3, 5> apex_dst;

    for(int i = 0; i < NUM_POINTS_STATIC; ++i) {
        apex_norm(0, i) = (output_buffer.at<float>(anchor_idx, i * 2 + 0) + grid0) * stride;
        apex_norm(1, i) = (output_buffer.at<float>(anchor_idx, i * 2 + 1) + grid1) * stride;
        apex_norm(2, i) = 1.0f;
    }

    apex_dst = transform_matrix * apex_norm;

    RuneObject obj;
    obj.pts.resize(NUM_POINTS_STATIC);
    for(int i = 0; i < NUM_POINTS_STATIC; ++i) {
        obj.pts[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
    }

    auto rect = cv::boundingRect(obj.pts);

    obj.box = rect;
    obj.color = DNN_COLOR_TO_ENEMY_COLOR.at(color_id.x);
    obj.type = class_id.x; // 0 or 1
    obj.prob = confidence;

    rects.push_back(rect);
    scores.push_back(confidence);
    output_objs.push_back(std::move(obj));
  }
}


std::vector<YoloXBuff::Object> YoloXBuff::get_multicandidateboxes(cv::Mat & image)
{
  const int64 start = cv::getTickCount();

  if (image.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return std::vector<YoloXBuff::Object>();
  }

  Eigen::Matrix3f transform_matrix;
  cv::Mat resized_img = letterbox(image, transform_matrix, {INPUT_W, INPUT_H});

  // [!!! 关键修复 !!!]
  // fyt 的模型
  // 使用 1.0 作为缩放因子 (即 [0-255] 范围)，而不是 1./255.0 ([0-1] 范围)
  cv::Mat blob = cv::dnn::blobFromImage(
    resized_img, 1.0, cv::Size(INPUT_W, INPUT_H), cv::Scalar(0, 0, 0), true);
  // [!!! 修复结束 !!!]


  CHECK_CUDA_BUFF(cudaMemcpyAsync(
    mGpuBuffers_[0], blob.data, mInputBytes_, cudaMemcpyHostToDevice, mStream));

  mContext->setTensorAddress(mInputTensorName_.c_str(), mGpuBuffers_[0]);
  mContext->setTensorAddress(mOutputTensorName_.c_str(), mGpuBuffers_[1]);
  if (!mContext->enqueueV3(mStream)) {
    tools::logger()->error("YoloXBuff (TRT): enqueueV3 failed");
    return {};
  }

  CHECK_CUDA_BUFF(cudaMemcpyAsync(
    mCpuOutputBuffer_.data(), mGpuBuffers_[1], mOutputBytes_, cudaMemcpyDeviceToHost,
    mStream));

  CHECK_CUDA_BUFF(cudaStreamSynchronize(mStream));

  int expected_channels = NUM_POINTS_2_STATIC + 1 + NUM_COLORS + NUM_CLASSES;
  if (mOutputDims_.nbDims != 3 || mOutputDims_.d[2] != expected_channels) {
      tools::logger()->error("Model output mismatch! Expected [N_Anchors, {}], but got [{}, {}, {}]", 
                             expected_channels, mOutputDims_.d[0], mOutputDims_.d[1], mOutputDims_.d[2]);
      return {};
  }
  
  if (grid_strides_.size() != mOutputDims_.d[1]) {
      tools::logger()->warn("Grid size ({}) does not match model output dim ({})! Check strides.", grid_strides_.size(), mOutputDims_.d[1]);
  }
  
  cv::Mat output_buffer(mOutputDims_.d[1], mOutputDims_.d[2], CV_32F, mCpuOutputBuffer_.data());
  
  std::vector<RuneObject> objs_tmp; 
  std::vector<cv::Rect> rects;
  std::vector<float> scores;
  std::vector<int> indices;

  generateProposals_fyt(
    objs_tmp, rects, scores, output_buffer, transform_matrix, 
    m_conf_threshold, this->grid_strides_);

  cv::dnn::NMSBoxes(rects, scores, m_conf_threshold, m_nms_threshold, indices);

  std::vector<Object> object_result; 
  
  for (size_t i = 0; i < indices.size(); ++i) {
    const int index = indices[i];
    RuneObject & fyt_obj = objs_tmp[index];

    Object obj; 
    obj.rect = fyt_obj.box;
    obj.prob = fyt_obj.prob;
    obj.label = fyt_obj.type; 
    obj.kpt = fyt_obj.pts; 

    object_result.push_back(obj);

    cv::rectangle(image, obj.rect, cv::Scalar(255, 255, 255), 1, 8);
    const std::string label = "buff:" + std::to_string(obj.prob).substr(0, 4);
    const cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, nullptr);
    const cv::Rect textBox(
      obj.rect.tl().x, obj.rect.tl().y - 15, textSize.width, textSize.height + 5);
    cv::rectangle(image, textBox, cv::Scalar(0, 255, 255), cv::FILLED);
    cv::putText(
      image, label, cv::Point(obj.rect.tl().x, obj.rect.tl().y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5,
      cv::Scalar(0, 0, 0));
    
    const int radius = 2;
    for (int k = 0; k < NUM_POINTS; ++k) {
      if (k < obj.kpt.size()) {
          cv::circle(image, obj.kpt[k], radius, cv::Scalar(255, 0, 0), -1, cv::LINE_AA);
      }
    }
  }

  const float t = (cv::getTickCount() - start) / static_cast<float>(cv::getTickFrequency());
  cv::putText(
    image, cv::format("FPS (TRT): %.2f", 1.0 / t), cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0,
    cv::Scalar(255, 0, 0), 2, 8);

  return object_result;
}

std::vector<YoloXBuff::Object> YoloXBuff::get_onecandidatebox(cv::Mat & image)
{
  auto results = get_multicandidateboxes(image);
  
  if (results.empty()) {
    return {};
  }
  
  auto best_it = std::max_element(results.begin(), results.end(), 
    [](const Object& a, const Object& b) {
        return a.prob < b.prob;
    });

  const int radius = 2;
  for (int i = 0; i < NUM_POINTS; ++i) {
    if (i < best_it->kpt.size()) {
        cv::putText(
          image, std::to_string(i), best_it->kpt[i] + cv::Point2f(5, -5), cv::FONT_HERSHEY_SIMPLEX,
          0.5, cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
    }
  }

  return {*best_it};
}

void YoloXBuff::save(const std::string & programName, const cv::Mat & image)
{
  const std::filesystem::path saveDir = "../result/"; 
  if (!std::filesystem::exists(saveDir)) {
    std::filesystem::create_directories(saveDir);
  }
  const std::filesystem::path savePath = saveDir / (programName + ".jpg");
  cv::imwrite(savePath.string(), image);
}

}  // namespace auto_buff