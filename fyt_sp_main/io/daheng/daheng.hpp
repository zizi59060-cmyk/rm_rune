#ifndef IO__DAHENG_HPP
#define IO__DAHENG_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "include/GxIAPI.h"
#include "include/DxImageProc.h"

#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// -------------------- 大恒相机类 --------------------
class DaHeng : public CameraBase
{
public:
  // 构造与析构
  DaHeng(double exposure_ms, double gain, const std::string &vid_pid);
  ~DaHeng() override;

  // 读取一帧图像（带缓存防止空帧）
  void read(cv::Mat &img, std::chrono::steady_clock::time_point &timestamp) override;

  // SDK 回调接口
  void onFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrame);

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
  };

  // 参数
  double exposure_us_;
  double gain_;
  GX_DEV_HANDLE handle_;
  bool is_color_camera_;

  // 队列与线程状态
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  tools::ThreadSafeQueue<CameraData> queue_;

  // 图像缓存（防止空帧）
  cv::Mat last_frame_;
  std::chrono::steady_clock::time_point last_timestamp_;

  // 色彩滤镜
  int64_t pixel_color_filter_;

  // FPS 统计
  int frame_count_;
  std::chrono::steady_clock::time_point last_fps_time_;
};

} // namespace io

#endif // IO__DAHENG_HPP
