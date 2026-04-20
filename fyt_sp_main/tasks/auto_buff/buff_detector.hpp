#pragma once

#include <optional>

#include "buff_type.hpp"
#include "yolox_buff.hpp"

namespace auto_buff
{

class Buff_Detector
{
public:
  explicit Buff_Detector(const std::string & config);

  std::optional<PowerRune> detect(cv::Mat & bgr_img);
  std::optional<PowerRune> detect_24(cv::Mat & bgr_img);
  std::optional<PowerRune> detect_debug(cv::Mat & bgr_img, cv::Point2f v);

private:
  cv::Point2f get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img);
  void handle_lose();

  // 统一四角点顺序：
  // 输入：YOLOX 的 5 个点（[0] 为 R 中心 / [1..4] 为 corners）
  // 输出：稳定顺序 [r_center, p1, p2, p3, p4]
  std::vector<cv::Point2f> normalize_kpt_order(const std::vector<cv::Point2f> & raw_kpt) const;

  enum Status
  {
    LOSE,
    TEM_LOSE,
    TRACK,
  } status_;

  int lose_;
  const int LOSE_MAX = 10;
  std::optional<PowerRune> last_powerrune_;
  YoloXBuff MODE_;
};

}  // namespace auto_buff