#include "buff_aimer.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_buff
{

Aimer::Aimer(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;
  fire_gap_time_ = yaml["fire_gap_time"].as<double>();
  predict_time_ = yaml["predict_time"].as<double>();

  last_fire_t_ = std::chrono::steady_clock::now();

  // 调试量初始化
  dbg_calc_cmd_yaw_ = 0.0;
  dbg_calc_cmd_pitch_ = 0.0;
  dbg_delta_cmd_yaw_ = 0.0;
  dbg_delta_cmd_pitch_ = 0.0;
  dbg_switch_fanblade_ = false;
  dbg_mistake_count_ = 0;
}

io::Command Aimer::aim(
  auto_buff::Target & target,
  std::chrono::steady_clock::time_point & timestamp,
  double bullet_speed,
  bool to_now)
{
  io::Command command = {false, false, 0, 0};

  if (target.is_unsolve()) {
    dbg_calc_cmd_yaw_ = last_yaw_;
    dbg_calc_cmd_pitch_ = -last_pitch_;
    dbg_delta_cmd_yaw_ = 0.0;
    dbg_delta_cmd_pitch_ = 0.0;
    dbg_switch_fanblade_ = false;
    dbg_mistake_count_ = mistake_count_;

    // 丢目标时不清 has_last_cmd_
    // 这样短暂丢帧后重新接上，不会又被当成“首次锁定”
    return command;
  }

  if (bullet_speed < 10) {
    bullet_speed = 24;
  }

  auto now = std::chrono::steady_clock::now();
  auto detect_now_gap = tools::delta_time(now, timestamp);
  auto future = to_now ? (detect_now_gap + predict_time_) : (0.1 + predict_time_);

  double yaw = last_yaw_;
  double pitch = last_pitch_;

  if (get_send_angle(target, future, bullet_speed, to_now, yaw, pitch)) {
    // 计算出的命令角（pitch 这里先记录下发给下游后的符号）
    dbg_calc_cmd_yaw_ = yaw;
    dbg_calc_cmd_pitch_ = -pitch;

    double delta_yaw = tools::limit_rad(yaw - last_yaw_);
    double delta_pitch = tools::limit_rad(pitch - last_pitch_);

    dbg_delta_cmd_yaw_ = delta_yaw;
    dbg_delta_cmd_pitch_ = delta_pitch;

    command.yaw = yaw;
    command.pitch = -pitch;

    // ========= 关键修改：首次锁定直接接管，不做大跳变判定 =========
    if (!has_last_cmd_) {
      last_yaw_ = yaw;
      last_pitch_ = pitch;
      has_last_cmd_ = true;

      switch_fanblade_ = false;
      mistake_count_ = 0;
      command.control = true;

      dbg_delta_cmd_yaw_ = 0.0;
      dbg_delta_cmd_pitch_ = 0.0;
      dbg_switch_fanblade_ = false;
      dbg_mistake_count_ = 0;

      tools::logger()->info(
        "[Aimer] first lock accepted: yaw={:.3f} deg pitch={:.3f} deg",
        yaw * 57.3, (-pitch) * 57.3);
    } else {
      // ========= 正常跟踪阶段 =========
      if (mistake_count_ > 3) {
        switch_fanblade_ = true;
        mistake_count_ = 0;
        command.control = true;
      } else if (std::abs(delta_yaw) > 5 / 57.3 || std::abs(delta_pitch) > 5 / 57.3) {
        switch_fanblade_ = true;
        mistake_count_++;
        command.control = false;
      } else {
        switch_fanblade_ = false;
        mistake_count_ = 0;
        command.control = true;
      }

      dbg_switch_fanblade_ = switch_fanblade_;
      dbg_mistake_count_ = mistake_count_;

      last_yaw_ = yaw;
      last_pitch_ = pitch;
    }
  } else {
    dbg_calc_cmd_yaw_ = last_yaw_;
    dbg_calc_cmd_pitch_ = -last_pitch_;
    dbg_delta_cmd_yaw_ = 0.0;
    dbg_delta_cmd_pitch_ = 0.0;
    dbg_switch_fanblade_ = false;
    dbg_mistake_count_ = mistake_count_;
    command.control = false;
  }

  if (switch_fanblade_) {
    command.shoot = false;
    last_fire_t_ = now;
  } else if (!switch_fanblade_ && tools::delta_time(now, last_fire_t_) > fire_gap_time_) {
    command.shoot = true;
    last_fire_t_ = now;
  }

  return command;
}

bool Aimer::get_send_angle(
  auto_buff::Target & target,
  const double predict_time,
  const double bullet_speed,
  const bool /*to_now*/,
  double & yaw,
  double & pitch)
{
  target.predict(predict_time);

  auto compute_traj = [&](const Eigen::Vector3d & aim_world) -> std::optional<tools::Trajectory> {
    double d = std::sqrt(aim_world[0] * aim_world[0] + aim_world[1] * aim_world[1]);
    double h = aim_world[2];
    tools::Trajectory traj(bullet_speed, d, h);
    if (traj.unsolvable) return std::nullopt;
    return traj;
  };

  // 与 buff_solver / buff_target 保持一致：扇叶点在 buff 系下为 (0, -0.7, 0)
  Eigen::Vector3d aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, -0.7, 0.0));
  auto traj_opt = compute_traj(aim_in_world);
  if (!traj_opt.has_value()) {
    tools::logger()->debug("[Aimer] Unsolvable trajectory: bs={:.2f}", bullet_speed);
    return false;
  }

  tools::Trajectory traj = traj_opt.value();
  constexpr int kMaxIter = 3;
  double last_fly = traj.fly_time;

  for (int iter = 0; iter < kMaxIter; ++iter) {
    target.predict(traj.fly_time);

    aim_in_world = target.point_buff2world(Eigen::Vector3d(0.0, -0.7, 0.0));
    auto traj2_opt = compute_traj(aim_in_world);
    if (!traj2_opt.has_value()) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory(it={}): bs={:.2f}", iter, bullet_speed);
      return false;
    }

    tools::Trajectory traj2 = traj2_opt.value();
    double time_error = traj2.fly_time - last_fly;

    traj = traj2;
    if (std::abs(time_error) < 0.03) {
      break;
    }
    last_fly = traj.fly_time;
  }

  yaw = std::atan2(aim_in_world[1], aim_in_world[0]) + yaw_offset_;
  pitch = traj.pitch + pitch_offset_;
  angle = target.ekf_x()[5];
  return true;
}

}  // namespace auto_buff