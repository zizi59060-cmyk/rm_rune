/* tasks/auto_buff/buff_detector.cpp */
#include "buff_detector.hpp"
#include "tools/logger.hpp"
#include "tools/img_tools.hpp"
#include <numeric>

namespace auto_buff
{

Buff_Detector::Buff_Detector(const std::string & config)
: status_(LOSE), lose_(0), MODE_(config)
{
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat & bgr_img)
{
  if (fanblades.empty()) return {0, 0};

  cv::Point2f r_center_t = {0, 0};
  int valid_count = 0;

  for (auto & fanblade : fanblades) {
    if (!fanblade.points.empty()) {
      r_center_t += fanblade.points[0];
      valid_count++;
    }
  }

  if (valid_count == 0) return {0, 0};
  r_center_t /= static_cast<float>(valid_count);

  return r_center_t;
}

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
  }
  status_ = TEM_LOSE;
}

std::optional<PowerRune> Buff_Detector::detect_24(cv::Mat & bgr_img)
{
  std::vector<YoloXBuff::Object> results = MODE_.get_multicandidateboxes(bgr_img);

  if (results.empty()) {
    handle_lose();
    return std::nullopt;
  }

  std::vector<FanBlade> fanblades;
  fanblades.reserve(results.size());

  for (auto & result : results) {
    // 根据 YOLO 输出的类别 label 判断扇叶当前状态：
    // label == 0 -> 未激活；label == 1 -> 已激活
    FanBlade_type fb_type = _unlight;
    if (result.label == 1) {
      fb_type = _light;
    }

    cv::Point2f armor_center(0, 0);
    if (result.kpt.size() >= 5) {
      // 1~4 是装甲板四个角点
      for (int i = 1; i <= 4; i++) {
        armor_center += result.kpt[i];
      }
      armor_center /= 4.0f;
    } else {
      continue;
    }

    fanblades.emplace_back(FanBlade(result.kpt, armor_center, fb_type));
  }

  if (fanblades.empty()) {
    handle_lose();
    return std::nullopt;
  }

  auto r_center = get_r_center(fanblades, bgr_img);
  PowerRune powerrune(fanblades, r_center, last_powerrune_);

  if (powerrune.is_unsolve()) {
    handle_lose();
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  return P;
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  return detect_24(bgr_img);
}

std::optional<PowerRune> Buff_Detector::detect_debug(cv::Mat & bgr_img, cv::Point2f v)
{
  std::vector<YoloXBuff::Object> results = MODE_.get_multicandidateboxes(bgr_img);
  if (results.empty()) return std::nullopt;

  std::vector<FanBlade> fanblades_t;
  fanblades_t.reserve(results.size());

  for (auto & result : results) {
    FanBlade_type fb_type = _unlight;
    if (result.label == 1) {
      fb_type = _light;
    }

    cv::Point2f armor_center(0, 0);
    if (result.kpt.size() >= 5) {
      for (int i = 1; i <= 4; i++) {
        armor_center += result.kpt[i];
      }
      armor_center /= 4.0f;
    } else {
      continue;
    }

    fanblades_t.emplace_back(FanBlade(result.kpt, armor_center, fb_type));
  }

  if (fanblades_t.empty()) return std::nullopt;

  auto r_center = get_r_center(fanblades_t, bgr_img);
  std::vector<FanBlade> fanblades;

  // debug 模式下，根据外部传入的偏移向量 v 选取一个扇叶
  for (auto & fanblade : fanblades_t) {
    if (cv::norm((fanblade.center - r_center) - v) < 50 || results.size() == 1) {
      fanblades.emplace_back(fanblade);
      break;
    }
  }
  if (fanblades.empty()) return std::nullopt;

  PowerRune powerrune(fanblades, r_center, std::nullopt);
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  return P;
}

}  // namespace auto_buff
