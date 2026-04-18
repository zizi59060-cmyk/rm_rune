#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <vector>

#include "buff_target.hpp"
#include "buff_type.hpp"
#include "io/command.hpp"

namespace auto_buff
{
class Aimer
{
public:
  explicit Aimer(const std::string & config_path);

  io::Command aim(
    Target & target, std::chrono::steady_clock::time_point & timestamp, double bullet_speed,
    bool to_now = true);

  double angle = 0.0;
  double t_gap = 0.0;

private:
  SmallTarget target_;
  double yaw_offset_ = 0.0;
  double pitch_offset_ = 0.0;

  double fire_gap_time_ = 0.0;
  double predict_time_ = 0.0;

  int mistake_count_ = 0;
  bool switch_fanblade_ = false;

  double last_yaw_ = 0.0;
  double last_pitch_ = 0.0;

  std::chrono::steady_clock::time_point last_fire_t_;

  bool get_send_angle(
    auto_buff::Target & target, const double predict_time, const double bullet_speed,
    const bool to_now, double & yaw, double & pitch);
};
}  // namespace auto_buff

#endif  // AUTO_BUFF__AIMER_HPP