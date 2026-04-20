#include "buff_target.hpp"

namespace auto_buff
{

namespace
{

static bool compute_nowtime(
  bool & has_start_timestamp,
  std::chrono::steady_clock::time_point & start_timestamp,
  std::chrono::steady_clock::time_point & timestamp,
  double & nowtime_out)
{
  if (!has_start_timestamp) {
    start_timestamp = timestamp;
    has_start_timestamp = true;
    nowtime_out = 0.0;
    return false;
  }

  nowtime_out = tools::delta_time(timestamp, start_timestamp);
  if (nowtime_out < -1e-3) {
    start_timestamp = timestamp;
    nowtime_out = 0.0;
    return true;
  }
  return false;
}

}  // namespace

Target::Target()
{
  debug_x_.resize(7);
  debug_x_.setZero();
}

bool Target::is_unsolve() const
{
  return unsolvable_;
}

Eigen::VectorXd Target::ekf_x() const
{
  return debug_x_;
}

double Target::unwrap_to_near(double reference, double wrapped) const
{
  double k = std::round((reference - wrapped) / (2.0 * CV_PI));
  return wrapped + k * 2.0 * CV_PI;
}

double Target::clamp_dt(double dt) const
{
  if (dt < kMinUpdateDt_) {
    return kMinUpdateDt_;
  }
  if (dt > kMaxPredictDt_) {
    return kMaxPredictDt_;
  }
  return dt;
}

double Target::clamp_phase_delta(double dphase) const
{
  dphase = tools::limit_rad(dphase);
  if (dphase > kMaxPhaseStep_) {
    dphase = kMaxPhaseStep_;
  } else if (dphase < -kMaxPhaseStep_) {
    dphase = -kMaxPhaseStep_;
  }
  return dphase;
}

double Target::clamp_omega(double omega) const
{
  if (omega > kMaxOmegaBig_) {
    return kMaxOmegaBig_;
  }
  if (omega < -kMaxOmegaBig_) {
    return -kMaxOmegaBig_;
  }
  return omega;
}

double Target::clamp_predict_dt(double dt) const
{
  if (dt < 0.0) {
    return 0.0;
  }
  if (dt > kMaxPredictDt_) {
    return kMaxPredictDt_;
  }
  return dt;
}

void Target::init_common(double nowtime, const PowerRune & p)
{
  lasttime_ = nowtime;

  center_world_ = p.xyz_in_world;
  center_yaw_ = p.ypd_in_world[0];
  center_pitch_ = p.ypd_in_world[1];
  center_distance_ = p.ypd_in_world[2];

  phase_ = p.ypr_in_world[2];
  omega_ = 0.0;

  unsolvable_ = false;
  first_in_ = false;
  lost_cn_ = 0;

  debug_x_[0] = center_yaw_;
  debug_x_[1] = 0.0;
  debug_x_[2] = center_pitch_;
  debug_x_[3] = center_distance_;
  debug_x_[4] = center_yaw_;
  debug_x_[5] = phase_;
  debug_x_[6] = omega_;
}

void Target::update_common(double nowtime, const PowerRune & p, double omega_alpha)
{
  double raw_dt = nowtime - lasttime_;
  double dt = clamp_dt(raw_dt);

  const double meas_yaw = p.ypd_in_world[0];
  const double meas_pitch = p.ypd_in_world[1];
  const double meas_dis = p.ypd_in_world[2];
  const double meas_phase_wrapped = tools::limit_rad(p.ypr_in_world[2]);
  const double meas_phase_unwrapped = unwrap_to_near(phase_, meas_phase_wrapped);

  double dphase = meas_phase_unwrapped - phase_;
  dphase = clamp_phase_delta(dphase);

  voter_.vote(phase_, phase_ + dphase);

  // 关键修正：
  // 当前 phase 观测方向和模拟器真实转向相反，所以这里翻转符号
  double new_omega = kOmegaSign_ * (dphase / dt);
  new_omega = clamp_omega(new_omega);

  omega_ = (1.0 - omega_alpha) * omega_ + omega_alpha * new_omega;
  omega_ = clamp_omega(omega_);

  // center 低通
  constexpr double a_center = 0.25;
  center_world_ = (1.0 - a_center) * center_world_ + a_center * p.xyz_in_world;
  center_yaw_ = (1.0 - a_center) * center_yaw_ + a_center * meas_yaw;
  center_pitch_ = (1.0 - a_center) * center_pitch_ + a_center * meas_pitch;
  center_distance_ = (1.0 - a_center) * center_distance_ + a_center * meas_dis;

  // 当前 phase 仍然贴合观测本身，不翻
  constexpr double a_phase = 0.30;
  phase_ = phase_ + a_phase * dphase;

  lasttime_ = nowtime;
  lost_cn_ = 0;
  unsolvable_ = false;

  debug_x_[0] = center_yaw_;
  debug_x_[1] = 0.0;
  debug_x_[2] = center_pitch_;
  debug_x_[3] = center_distance_;
  debug_x_[4] = center_yaw_;
  debug_x_[5] = phase_;
  debug_x_[6] = omega_;
}

Eigen::Vector3d Target::point_buff2world(const Eigen::Vector3d & point_in_buff) const
{
  if (unsolvable_) {
    return Eigen::Vector3d::Zero();
  }

  // Aimer 主要取扇叶击打点
  if (std::abs(point_in_buff.x()) < 1e-6 &&
      std::abs(point_in_buff.y() + kBladeRadius_) < 1e-6 &&
      std::abs(point_in_buff.z()) < 1e-6) {
    return predict_position(0.0);
  }

  // rune center
  if (point_in_buff.norm() < 1e-6) {
    return center_world_;
  }

  // 其他点给统一近似映射
  Eigen::Vector3d u(-std::sin(center_yaw_), std::cos(center_yaw_), 0.0);
  Eigen::Vector3d v(0.0, 0.0, 1.0);
  Eigen::Vector3d w(std::cos(center_yaw_), std::sin(center_yaw_), 0.0);

  return center_world_ + point_in_buff.x() * w + point_in_buff.y() * u + point_in_buff.z() * v;
}

SmallTarget::SmallTarget() : Target()
{
}

void SmallTarget::get_target(
  const std::optional<PowerRune> & p,
  std::chrono::steady_clock::time_point & timestamp)
{
  double nowtime = 0.0;
  bool reset = compute_nowtime(
    has_start_timestamp_, start_timestamp_, timestamp, nowtime);

  if (reset) {
    first_in_ = true;
    unsolvable_ = true;
    lost_cn_ = 0;
    omega_ = 0.0;
    phase_ = 0.0;
  }

  if (!p.has_value() || p->is_unsolve()) {
    lost_cn_++;
    if (!unsolvable_ && lost_cn_ < 10) {
      predict(nowtime - lasttime_);
    } else {
      unsolvable_ = true;
      omega_ = 0.0;
    }

    debug_x_[5] = phase_;
    debug_x_[6] = omega_;
    return;
  }

  if (first_in_) {
    init_common(nowtime, p.value());
    return;
  }

  update_common(nowtime, p.value(), 0.35);

  // 小符速度额外收紧
  if (omega_ > kMaxOmegaSmall_) {
    omega_ = kMaxOmegaSmall_;
  } else if (omega_ < -kMaxOmegaSmall_) {
    omega_ = -kMaxOmegaSmall_;
  }

  debug_x_[6] = omega_;
}

void SmallTarget::predict(double dt)
{
  if (unsolvable_) {
    return;
  }

  dt = clamp_predict_dt(dt);
  omega_ = clamp_omega(omega_);
  if (omega_ > kMaxOmegaSmall_) {
    omega_ = kMaxOmegaSmall_;
  } else if (omega_ < -kMaxOmegaSmall_) {
    omega_ = -kMaxOmegaSmall_;
  }

  phase_ += omega_ * dt;
  lasttime_ += dt;

  debug_x_[5] = phase_;
  debug_x_[6] = omega_;
}

Eigen::Vector3d SmallTarget::predict_position(double dt) const
{
  if (unsolvable_) {
    return Eigen::Vector3d::Zero();
  }

  if (dt < 0.0) {
    dt = 0.0;
  } else if (dt > kMaxPredictDt_) {
    dt = kMaxPredictDt_;
  }

  double omega = omega_;
  if (omega > kMaxOmegaSmall_) {
    omega = kMaxOmegaSmall_;
  } else if (omega < -kMaxOmegaSmall_) {
    omega = -kMaxOmegaSmall_;
  }

  const double future_phase = phase_ + omega * dt;

  const Eigen::Vector3d u(-std::sin(center_yaw_), std::cos(center_yaw_), 0.0);
  const Eigen::Vector3d v(0.0, 0.0, 1.0);

  Eigen::Vector3d offset =
    kBladeRadius_ * (-std::cos(future_phase) * u + std::sin(future_phase) * v);

  return center_world_ + offset;
}

BigTarget::BigTarget() : Target()
{
}

void BigTarget::get_target(
  const std::optional<PowerRune> & p,
  std::chrono::steady_clock::time_point & timestamp)
{
  double nowtime = 0.0;
  bool reset = compute_nowtime(
    has_start_timestamp_, start_timestamp_, timestamp, nowtime);

  if (reset) {
    first_in_ = true;
    unsolvable_ = true;
    lost_cn_ = 0;
    omega_ = 0.0;
    phase_ = 0.0;
  }

  if (!p.has_value() || p->is_unsolve()) {
    lost_cn_++;
    if (!unsolvable_ && lost_cn_ < 20) {
      predict(nowtime - lasttime_);
    } else {
      unsolvable_ = true;
      omega_ = 0.0;
    }

    debug_x_[5] = phase_;
    debug_x_[6] = omega_;
    return;
  }

  if (first_in_) {
    init_common(nowtime, p.value());
    return;
  }

  update_common(nowtime, p.value(), 0.20);

  omega_ = clamp_omega(omega_);
  debug_x_[6] = omega_;
}

void BigTarget::predict(double dt)
{
  if (unsolvable_) {
    return;
  }

  dt = clamp_predict_dt(dt);
  omega_ = clamp_omega(omega_);

  phase_ += omega_ * dt;
  lasttime_ += dt;

  debug_x_[5] = phase_;
  debug_x_[6] = omega_;
}

Eigen::Vector3d BigTarget::predict_position(double dt) const
{
  if (unsolvable_) {
    return Eigen::Vector3d::Zero();
  }

  if (dt < 0.0) {
    dt = 0.0;
  } else if (dt > kMaxPredictDt_) {
    dt = kMaxPredictDt_;
  }

  double omega = clamp_omega(omega_);
  const double future_phase = phase_ + omega * dt;

  const Eigen::Vector3d u(-std::sin(center_yaw_), std::cos(center_yaw_), 0.0);
  const Eigen::Vector3d v(0.0, 0.0, 1.0);

  Eigen::Vector3d offset =
    kBladeRadius_ * (-std::cos(future_phase) * u + std::sin(future_phase) * v);

  return center_world_ + offset;
}

}  // namespace auto_buff