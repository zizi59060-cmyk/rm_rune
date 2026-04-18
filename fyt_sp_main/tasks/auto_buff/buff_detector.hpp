/* tasks/auto_buff/buff_detector.hpp */
#pragma once

#include <optional>

#include "buff_type.hpp"
// #include "yolo11_buff.hpp" // <-- 删除
#include "yolox_buff.hpp"  // <-- 包含新的

namespace auto_buff
{
class Buff_Detector
{
public:
  Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);  // detect_one

  std::optional<PowerRune> detect_24(cv::Mat & bgr_img);  // detect_multi

  std::optional<PowerRune> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

private:
  void handle_img(const cv::Mat & bgr_img, cv::Mat & dilated_img);

  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);

  void handle_lose();

  enum Status
  {
    LOSE,
    TEM_LOSE,
    TRACK,
  } status_;

  int lose_;
  const int LOSE_MAX = 10;
  std::optional<PowerRune> last_powerrune_;
  // YOLO11_BUFF MODE_; // <-- 删除
  YoloXBuff MODE_;   // <-- 使用新的类
};
}  // namespace auto_buff

