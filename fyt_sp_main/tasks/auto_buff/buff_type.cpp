#include "buff_type.hpp"

#include <limits>
#include "tools/logger.hpp"

namespace auto_buff
{

FanBlade::FanBlade(
  const std::vector<cv::Point2f> & kpt,
  cv::Point2f keypoints_center,
  FanBlade_type t)
: center(keypoints_center), type(t)
{
  points.insert(points.end(), kpt.begin(), kpt.end());
}

FanBlade::FanBlade(FanBlade_type t)
: type(t)
{
  if (t != _unlight) {
    std::exit(-1);
  }
}

PowerRune::PowerRune(
  std::vector<FanBlade> & ts,
  const cv::Point2f r_center_in,
  std::optional<PowerRune> last_powerrune)
: r_center(r_center_in)
{
  if (ts.empty()) {
    unsolvable_ = true;
    return;
  }

  light_num = std::count_if(
    ts.begin(), ts.end(),
    [](const FanBlade & f) { return f.type == _light; });

  std::vector<int> unlight_idxs;
  for (int i = 0; i < static_cast<int>(ts.size()); ++i) {
    if (ts[i].type == _unlight) {
      unlight_idxs.push_back(i);
    }
  }

  if (unlight_idxs.empty()) {
    unsolvable_ = true;
    return;
  }

  int target_idx = unlight_idxs[0];

  if (last_powerrune.has_value() && !last_powerrune->fanblades.empty()) {
    const auto & last_center = last_powerrune->fanblades[0].center;
    float best = std::numeric_limits<float>::max();

    for (int idx : unlight_idxs) {
      float d = cv::norm(ts[idx].center - last_center);
      if (d < best) {
        best = d;
        target_idx = idx;
      }
    }
  }

  ts[target_idx].type = _target;
  for (int i = 0; i < static_cast<int>(ts.size()); ++i) {
    if (i != target_idx) {
      ts[i].type = _light;
    }
  }

  std::swap(ts[0], ts[target_idx]);

  double base_angle = atan_angle(ts[0].center);
  for (auto & t : ts) {
    t.angle = atan_angle(t.center) - base_angle;
    if (t.angle < -1e-3) {
      t.angle += CV_2PI;
    }
  }

  std::sort(
    ts.begin() + 1, ts.end(),
    [](const FanBlade & a, const FanBlade & b) {
      return a.angle < b.angle;
    });

  fanblades.emplace_back(ts[0]);

  const std::vector<double> target_angles = {
    2.0 * CV_PI / 5.0,
    4.0 * CV_PI / 5.0,
    6.0 * CV_PI / 5.0,
    8.0 * CV_PI / 5.0
  };

  int ts_idx = 1;
  for (int i = 0; i < 4; ++i) {
    if (ts_idx < static_cast<int>(ts.size()) &&
        std::fabs(ts[ts_idx].angle - target_angles[i]) < CV_PI / 5.0) {
      fanblades.emplace_back(ts[ts_idx++]);
    } else {
      fanblades.emplace_back(FanBlade(_unlight));
    }
  }
}

double PowerRune::atan_angle(cv::Point2f p) const
{
  auto v = p - r_center;
  auto angle = std::atan2(v.y, v.x);
  return angle >= 0.0 ? angle : angle + CV_2PI;
}

}  // namespace auto_buff