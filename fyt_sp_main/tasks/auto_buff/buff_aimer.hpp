#ifndef AUTO_BUFF__AIMER_HPP
#define AUTO_BUFF__AIMER_HPP

#include <chrono>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "io/command.hpp"
#include "tasks/auto_buff/buff_target.hpp"

namespace auto_buff
{

class Aimer
{
public:
  explicit Aimer(const std::string & config_path);

  io::Command aim(
    auto_buff::Target & target,
    std::chrono::steady_clock::time_point & timestamp,
    double bullet_speed,
    bool to_now = true);

  bool get_send_angle(
    auto_buff::Target & target,
    const double predict_time,
    const double bullet_speed,
    const bool to_now,
    double & yaw,
    double & pitch);

  // ===== 调试接口 =====
  double dbg_calc_cmd_yaw() const { return dbg_calc_cmd_yaw_; }
  double dbg_calc_cmd_pitch() const { return dbg_calc_cmd_pitch_; }
  double dbg_delta_cmd_yaw() const { return dbg_delta_cmd_yaw_; }
  double dbg_delta_cmd_pitch() const { return dbg_delta_cmd_pitch_; }
  bool dbg_switch_fanblade() const { return dbg_switch_fanblade_; }
  int dbg_mistake_count() const { return dbg_mistake_count_; }

public:
  double angle = 0.0;

private:
  double yaw_offset_ = 0.0;
  double pitch_offset_ = 0.0;
  double fire_gap_time_ = 0.08;
  double predict_time_ = 0.0;

  double last_yaw_ = 0.0;
  double last_pitch_ = 0.0;

  bool switch_fanblade_ = false;
  int mistake_count_ = 0;

  // 是否已经有上一帧有效命令
  bool has_last_cmd_ = false;

  std::chrono::steady_clock::time_point last_fire_t_;

  // ===== 调试量 =====
  double dbg_calc_cmd_yaw_ = 0.0;
  double dbg_calc_cmd_pitch_ = 0.0;
  double dbg_delta_cmd_yaw_ = 0.0;
  double dbg_delta_cmd_pitch_ = 0.0;
  bool dbg_switch_fanblade_ = false;
  int dbg_mistake_count_ = 0;
};

}  // namespace auto_buff

#endif