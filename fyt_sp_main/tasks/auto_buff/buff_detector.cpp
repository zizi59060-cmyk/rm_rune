#include "buff_detector.hpp"

#include "tools/logger.hpp"
#include "tools/img_tools.hpp"
#include <numeric>

namespace auto_buff
{

namespace
{

static cv::Point2f mean4(
  const cv::Point2f & a,
  const cv::Point2f & b,
  const cv::Point2f & c,
  const cv::Point2f & d)
{
  return cv::Point2f(
    0.25f * (a.x + b.x + c.x + d.x),
    0.25f * (a.y + b.y + c.y + d.y));
}

}  // namespace

Buff_Detector::Buff_Detector(const std::string & config)
: status_(LOSE), lose_(0), MODE_(config)
{
}

std::vector<cv::Point2f> Buff_Detector::normalize_kpt_order(const std::vector<cv::Point2f> & raw_kpt) const
{
  std::vector<cv::Point2f> out;
  if (raw_kpt.size() < 5) {
    return out;
  }

  const cv::Point2f r_center = raw_kpt[0];

  std::vector<cv::Point2f> corners(raw_kpt.begin() + 1, raw_kpt.begin() + 5);
  const cv::Point2f c = mean4(corners[0], corners[1], corners[2], corners[3]);

  // 先按绕中心的极角排序
  std::sort(
    corners.begin(), corners.end(),
    [&](const cv::Point2f & a, const cv::Point2f & b) {
      double aa = std::atan2(a.y - c.y, a.x - c.x);
      double bb = std::atan2(b.y - c.y, b.x - c.x);
      return aa < bb;
    });

  // 找到最像“左上角”的点作为起点（x+y 最小）
  int start_idx = 0;
  float best = corners[0].x + corners[0].y;
  for (int i = 1; i < 4; ++i) {
    float s = corners[i].x + corners[i].y;
    if (s < best) {
      best = s;
      start_idx = i;
    }
  }

  std::vector<cv::Point2f> ordered(4);
  for (int i = 0; i < 4; ++i) {
    ordered[i] = corners[(start_idx + i) % 4];
  }

  out.reserve(5);
  out.push_back(r_center);
  out.insert(out.end(), ordered.begin(), ordered.end());
  return out;
}

cv::Point2f Buff_Detector::get_r_center(std::vector<FanBlade> & fanblades, cv::Mat &)
{
  if (fanblades.empty()) {
    return {0, 0};
  }

  cv::Point2f r_center_t = {0, 0};
  int valid_count = 0;

  for (auto & fanblade : fanblades) {
    if (!fanblade.points.empty()) {
      r_center_t += fanblade.points[0];
      valid_count++;
    }
  }

  if (valid_count == 0) {
    return {0, 0};
  }

  r_center_t *= (1.0f / static_cast<float>(valid_count));
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
    if (result.kpt.size() < 5) {
      continue;
    }

    auto kpt = normalize_kpt_order(result.kpt);
    if (kpt.size() < 5) {
      continue;
    }

    FanBlade_type fb_type = (result.label == 1) ? _light : _unlight;

    cv::Point2f armor_center(0, 0);
    for (int i = 1; i <= 4; ++i) {
      armor_center += kpt[i];
    }
    armor_center *= 0.25f;

    fanblades.emplace_back(FanBlade(kpt, armor_center, fb_type));
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
  if (results.empty()) {
    return std::nullopt;
  }

  std::vector<FanBlade> fanblades_t;
  fanblades_t.reserve(results.size());

  for (auto & result : results) {
    if (result.kpt.size() < 5) {
      continue;
    }

    auto kpt = normalize_kpt_order(result.kpt);
    if (kpt.size() < 5) {
      continue;
    }

    FanBlade_type fb_type = (result.label == 1) ? _light : _unlight;

    cv::Point2f armor_center(0, 0);
    for (int i = 1; i <= 4; ++i) {
      armor_center += kpt[i];
    }
    armor_center *= 0.25f;

    fanblades_t.emplace_back(FanBlade(kpt, armor_center, fb_type));
  }

  if (fanblades_t.empty()) {
    return std::nullopt;
  }

  auto r_center = get_r_center(fanblades_t, bgr_img);
  std::vector<FanBlade> fanblades;

  for (auto & fanblade : fanblades_t) {
    if (cv::norm((fanblade.center - r_center) - v) < 50 || results.size() == 1) {
      fanblades.emplace_back(fanblade);
      break;
    }
  }

  if (fanblades.empty()) {
    return std::nullopt;
  }

  PowerRune powerrune(fanblades, r_center, std::nullopt);
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  return P;
}

}  // namespace auto_buff