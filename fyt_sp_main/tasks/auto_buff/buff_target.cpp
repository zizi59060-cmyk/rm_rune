/* tasks/auto_buff/buff_target.cpp */
#include "buff_target.hpp"
#include <cmath>
#include <sstream>
#include <iomanip>

namespace auto_buff
{

#ifndef AUTO_BUFF_DEBUG
#define AUTO_BUFF_DEBUG 1
#endif

#ifndef AUTO_BUFF_DEBUG_STRIDE
#define AUTO_BUFF_DEBUG_STRIDE 10
#endif

static constexpr double kBladeRadius = 0.7;
static inline Eigen::Vector3d blade_point_in_buff()
{
  return Eigen::Vector3d(0.0, -kBladeRadius, 0.0);
}

// 把 angle 调整到最接近 reference 的 2π 等价角（unwrapped）
static inline double unwrap_to_near(double reference, double angle_wrapped)
{
  // angle_wrapped 一般在 [-pi, pi]，reference 可能是任意实数
  double a = angle_wrapped;
  double k = std::round((reference - a) / (2.0 * CV_PI));
  return a + k * 2.0 * CV_PI;
}

static inline std::string fmt_pt(const cv::Point2f & p)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1)
      << "(" << p.x << "," << p.y << ")";
  return oss.str();
}

// ---------------- Voter ----------------
Voter::Voter() : clockwise_(0) {}
void Voter::vote(const double angle_last, const double angle_now)
{
  if (std::abs(clockwise_) > 50) return;
  if (angle_last > angle_now) clockwise_--;
  else clockwise_++;
}
int Voter::clockwise() const { return clockwise_ > 0 ? 1 : -1; }

// ---------------- Target ----------------
Target::Target() : first_in_(true), unsolvable_(true) {}

Eigen::Vector3d Target::point_buff2world(const Eigen::Vector3d & point_in_buff) const
{
  if (unsolvable_) return Eigen::Vector3d(0, 0, 0);

  // 只在生成旋转矩阵时 wrap
  double yaw = tools::limit_rad(ekf_.x[4]);
  double phase = tools::limit_rad(ekf_.x[5]);

  Eigen::Matrix3d R_buff2world =
    tools::rotation_matrix(Eigen::Vector3d(yaw, 0.0, phase));

  double R_yaw = ekf_.x[0];
  double R_pitch = ekf_.x[2];
  double R_dis = ekf_.x[3];

  Eigen::Vector3d center_world(
    R_dis * std::cos(R_pitch) * std::cos(R_yaw),
    R_dis * std::cos(R_pitch) * std::sin(R_yaw),
    R_dis * std::sin(R_pitch));

  return R_buff2world * point_in_buff + center_world;
}

bool Target::is_unsolve() const { return unsolvable_; }
Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

// ---------------- SmallTarget ----------------
SmallTarget::SmallTarget() : Target() {}

void SmallTarget::get_target(
  const std::optional<PowerRune> & p,
  std::chrono::steady_clock::time_point & timestamp)
{
  double nowtime = 0.0;
  bool reset_happened = compute_nowtime(timestamp, nowtime);
  if (reset_happened) {
    first_in_ = true;
    lost_cn_ = 0;
    unsolvable_ = true;
    lasttime_ = 0.0;
    rotation_dir_ = 1;
    rotation_dir_locked_ = false;
  }

  if (!p.has_value()) {
    unsolvable_ = true;
    lost_cn_++;
    return;
  }

  if (first_in_) {
    unsolvable_ = true;
    init(nowtime, p.value());
    first_in_ = false;
  }

  if (lost_cn_ > 6) {
    unsolvable_ = true;
    lost_cn_ = 0;
    first_in_ = true;
    return;
  }

  unsolvable_ = false;
  update(nowtime, p.value());
}

void SmallTarget::predict(double) {}

Eigen::Vector3d SmallTarget::predict_position(double dt) const
{
  double current_phase = ekf_.x[5];
  double future_phase = current_phase + (voter.clockwise() > 0 ? SMALL_W : -SMALL_W) * dt;

  Eigen::Matrix3d R_buff2world =
    tools::rotation_matrix(Eigen::Vector3d(tools::limit_rad(ekf_.x[4]), 0.0, tools::limit_rad(future_phase)));

  Eigen::Vector3d p_buff(0.0, 0.0, 0.7);

  double R_yaw = ekf_.x[0];
  double R_pitch = ekf_.x[2];
  double R_dis = ekf_.x[3];

  Eigen::Vector3d center_world(
    R_dis * std::cos(R_pitch) * std::cos(R_yaw),
    R_dis * std::cos(R_pitch) * std::sin(R_yaw),
    R_dis * std::sin(R_pitch));

  return R_buff2world * p_buff + center_world;
}

void SmallTarget::init(double, const PowerRune &) {}
void SmallTarget::update(double, const PowerRune &) {}
Eigen::MatrixXd SmallTarget::h_jacobian() const { return Eigen::MatrixXd(); }

// ---------------- BigTarget ----------------
BigTarget::BigTarget()
: Target(),
  spd_fitter_(100, 0.5, 1.884, 2.000),
  fit_spd_(0.0),
  has_last_measurement_(false),
  debug_frame_(0),
  last_has_target_(false),
  track_age_(0),
  last_align_i_(0),
  last_align_err_(1e9),
  ypr_phase_unwrapped_(0.0),
  has_ypr_phase_unwrapped_(false)
{}

void BigTarget::get_target(
  const std::optional<PowerRune> & p,
  std::chrono::steady_clock::time_point & timestamp)
{
  double nowtime = 0.0;
  bool reset_happened = compute_nowtime(timestamp, nowtime);
  if (reset_happened) {
    tools::logger()->info("[AutoBuff][EDGE] timebase reset.");
    first_in_ = true;
    lost_cn_ = 0;
    unsolvable_ = true;
    lasttime_ = 0.0;

    rotation_dir_ = 1;
    rotation_dir_locked_ = false;

    has_last_measurement_ = false;
    debug_frame_ = 0;

    last_has_target_ = false;
    track_age_ = 0;
    last_align_i_ = 0;
    last_align_err_ = 1e9;
    has_ypr_phase_unwrapped_ = false;

    spd_fitter_ = tools::RansacSineFitter(100, 0.5, 1.884, 2.000);
    fit_spd_ = 0.0;
  }

  bool has_target_now = (p.has_value() && !p->is_unsolve());
  if (!last_has_target_ && has_target_now) {
    tools::logger()->info(
      "[AutoBuff][EDGE] TRACK_ENTER t={:.3f} light_num={} fanblades={} target_center={}",
      nowtime, p->light_num, (int)p->fanblades.size(),
      (p->fanblades.empty() ? std::string("(empty)") : fmt_pt(p->fanblades[0].center)));
    track_age_ = 0;
    last_align_i_ = 0;
    last_align_err_ = 1e9;
    has_ypr_phase_unwrapped_ = false;
  }
  if (last_has_target_ && !has_target_now) {
    tools::logger()->info("[AutoBuff][EDGE] TRACK_LOST t={:.3f}", nowtime);
  }
  last_has_target_ = has_target_now;

  if (!p.has_value() || p->is_unsolve()) {
    lost_cn_++;
    if (lost_cn_ <= 60 && !first_in_) {
      predict(nowtime - lasttime_);
      lasttime_ = nowtime;
      unsolvable_ = false;
    } else {
      unsolvable_ = true;
    }
    return;
  }

  if (std::isnan(p->ypd_in_world[0]) || std::isnan(p->ypr_in_world[2])) return;

  if (first_in_) {
    unsolvable_ = true;
    init(nowtime, p.value());
    first_in_ = false;
    lost_cn_ = 0;
    return;
  }

  if (lost_cn_ > 60) {
    first_in_ = true;
    lost_cn_ = 0;
    return;
  }

  lost_cn_ = 0;
  unsolvable_ = false;

  update(nowtime, p.value());

  if (std::isnan(ekf_.x[6])) {
    tools::logger()->warn("[AutoBuff] NaN detected. Reset.");
    first_in_ = true;
    return;
  }
}

void BigTarget::predict(double dt)
{
  double t = lasttime_ + dt;

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_next = x;
    double a_ = x[7];
    double w_ = x[8];
    double fi_ = x[9];

    if (std::abs(w_) < 1e-6) w_ = (w_ >= 0 ? 1e-6 : -1e-6);

    // yaw 可 wrap，但 phase 不要 wrap（保持连续）
    x_next[4] = tools::limit_rad(x_next[4]);

    double integration =
      (-a_ / w_ * std::cos(w_ * t + fi_) +
       a_ / w_ * std::cos(w_ * lasttime_ + fi_) +
       (2.09 - a_) * dt);

    x_next[5] = x_next[5] + rotation_dir_ * integration;  // ✅ 连续相位
    x_next[6] = a_ * std::sin(w_ * t + fi_) + 2.09 - a_;
    return x_next;
  };

  double epsilon = 1e-4;
  Eigen::VectorXd x_curr = ekf_.x;
  Eigen::VectorXd f_x = f(x_curr);

  if (A_.rows() != 10 || A_.cols() != 10) A_.resize(10, 10);

  for (int j = 0; j < 10; ++j) {
    Eigen::VectorXd x_plus = x_curr;
    x_plus[j] += epsilon;
    Eigen::VectorXd f_plus = f(x_plus);
    Eigen::VectorXd diff = f_plus - f_x;

    // 对 wrap 变量做差分 wrap；phase 不做（phase 应该是连续小量差）
    diff[0] = tools::limit_rad(diff[0]);
    diff[2] = tools::limit_rad(diff[2]);
    diff[4] = tools::limit_rad(diff[4]);

    A_.col(j) = diff / epsilon;
  }

  Q_.setZero();
  Q_(5, 5) = 0.09;
  Q_(6, 6) = 0.5;
  Q_(9, 9) = 1.0;

  ekf_.predict(A_, Q_, f);
  lasttime_ += dt;
}

Eigen::Vector3d BigTarget::predict_position(double dt) const
{
  if (unsolvable_ || !has_last_measurement_) return Eigen::Vector3d::Zero();

  double a_ = ekf_.x[7];
  double w_ = ekf_.x[8];
  double fi_ = ekf_.x[9];
  double current_phase = ekf_.x[5];  // 连续相位

  double t_start = lasttime_;
  double t_end = lasttime_ + dt;
  if (std::abs(w_) < 1e-6) w_ = (w_ >= 0 ? 1e-6 : -1e-6);

  double val_end = -a_ / w_ * std::cos(w_ * t_end + fi_) + (2.09 - a_) * t_end;
  double val_start = -a_ / w_ * std::cos(w_ * t_start + fi_) + (2.09 - a_) * t_start;
  double integration = val_end - val_start;

  double future_phase = current_phase + rotation_dir_ * integration;  // 连续相位

  // ✅ 关键：dphi 用“连续差”再 wrap 到最短弧，避免在 ±π 边界跳
  double dphi = tools::limit_rad(future_phase - current_phase);

  Eigen::Matrix3d R_delta = tools::rotation_matrix(Eigen::Vector3d(0.0, 0.0, dphi));
  Eigen::Matrix3d R_future = last_measurement_.R_buff2world * R_delta;

  Eigen::Vector3d center_world = last_measurement_.xyz_in_world;
  Eigen::Vector3d p_buff = blade_point_in_buff();

  return R_future * p_buff + center_world;
}

void BigTarget::init(double nowtime, const PowerRune & p)
{
  lasttime_ = nowtime;
  unsolvable_ = true;
  has_last_measurement_ = false;

  rotation_dir_ = 1;
  rotation_dir_locked_ = false;

  track_age_ = 0;
  last_align_i_ = 0;
  last_align_err_ = 1e9;
  has_ypr_phase_unwrapped_ = false;

  x0_.resize(10);
  P0_.resize(10, 10);
  A_.resize(10, 10);
  Q_.resize(10, 10);
  H_.resize(10, 10);
  R_.resize(7, 7);

  // phase(索引5) 初始化用观测的 wrap 值，但后面会变成连续相位
  x0_ << p.ypd_in_world[0], 0.0, p.ypd_in_world[1], p.ypd_in_world[2],
    p.ypr_in_world[0], p.ypr_in_world[2],
    1.1775, 0.9125, 1.942, 0.0;

  P0_.setIdentity();
  P0_(8, 8) = 10.0;
  P0_(9, 9) = 10.0;

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[0] = tools::limit_rad(c[0]);
    c[2] = tools::limit_rad(c[2]);
    c[4] = tools::limit_rad(c[4]);
    // ✅ phase(5) 不 wrap：保持连续
    c[9] = tools::limit_rad(c[9]);  // fi 可以 wrap
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0_, P0_, x_add);
}

void BigTarget::update(double nowtime, const PowerRune & p)
{
  debug_frame_++;
  track_age_++;

  const Eigen::VectorXd & R_ypd = p.ypd_in_world;
  const Eigen::VectorXd & ypr = p.ypr_in_world;
  const Eigen::VectorXd & B_ypd = p.blade_ypd_in_world;

  // --- 1) 观测相位做连续化（unwrapped），用于等价类对齐 ---
  double ypr_phase_wrapped = tools::limit_rad(ypr[2]);
  if (!has_ypr_phase_unwrapped_) {
    // 第一次：直接以 EKF 当前 phase 为参考解包
    ypr_phase_unwrapped_ = unwrap_to_near(ekf_.x[5], ypr_phase_wrapped);
    has_ypr_phase_unwrapped_ = true;
  } else {
    // 后续：以上一次 unwrapped 为参考，保持连续
    ypr_phase_unwrapped_ = unwrap_to_near(ypr_phase_unwrapped_, ypr_phase_wrapped);
  }

  // --- 2) 等价类对齐：带温启动 + 迟滞，避免第一片边界抖动 ---
  // 候选：ekf_phase + k*(2π/5)，但 ekf_phase 是连续的，所以候选也是连续的
  int best_i = 0;
  double best_err = 1e9;
  for (int i = -5; i <= 5; i++) {
    double cand = ekf_.x[5] + i * 2.0 * CV_PI / 5.0;  // 连续候选
    double err = std::abs(cand - ypr_phase_unwrapped_);
    if (err < best_err) {
      best_err = err;
      best_i = i;
    }
  }

  // 温启动：刚进入跟踪的前几帧，禁止大跳（最治“第一片更远”）
  const int kAlignWarmupFrames = 8;
  // 迟滞：只有“明显更好”才允许从上一档切换
  const double kAlignSwitchMargin = 0.03; // rad，大概 1.7 deg

  int use_i = best_i;
  double use_err = best_err;

  if (track_age_ <= kAlignWarmupFrames) {
    // 前几帧：固定使用上一帧（或 0），避免边界抖动触发 wrap 路径变化
    use_i = last_align_i_;
    double cand = ekf_.x[5] + use_i * 2.0 * CV_PI / 5.0;
    use_err = std::abs(cand - ypr_phase_unwrapped_);
  } else {
    // 迟滞：如果切换不能显著降低误差，就保持上一帧
    double cur_cand = ekf_.x[5] + last_align_i_ * 2.0 * CV_PI / 5.0;
    double cur_err = std::abs(cur_cand - ypr_phase_unwrapped_);
    if (best_i != last_align_i_) {
      if (!(best_err + kAlignSwitchMargin < cur_err)) {
        use_i = last_align_i_;
        use_err = cur_err;
      }
    }
  }

  // 应用对齐（不 wrap）
  ekf_.x[5] = ekf_.x[5] + use_i * 2.0 * CV_PI / 5.0;
  last_align_i_ = use_i;
  last_align_err_ = use_err;

#if AUTO_BUFF_DEBUG
  if (debug_frame_ % AUTO_BUFF_DEBUG_STRIDE == 0) {
    tools::logger()->debug(
      "[AutoBuff][DBG] ALIGN2 t={:.3f} track_age={} ypr_phase(w)={:.3f} ypr_phase(u)={:.3f} "
      "ekf_phase(u,after_align)={:.3f} best_i={} best_err={:.3f} use_i={} use_err={:.3f}",
      nowtime, track_age_, ypr_phase_wrapped, ypr_phase_unwrapped_,
      ekf_.x[5], best_i, best_err, use_i, use_err);
  }
#endif

  // voter（仅统计趋势）
  voter.vote(ekf_.x[5], ypr_phase_unwrapped_);

  // 预测到当前时刻
  predict(nowtime - lasttime_);

  // 更新1：圆心 ypd + phase（残差用 wrap 差）
  Eigen::MatrixXd H1 = Eigen::MatrixXd::Zero(4, 10);
  H1(0, 0) = 1.0;
  H1(1, 2) = 1.0;
  H1(2, 3) = 1.0;
  H1(3, 5) = 1.0;

  Eigen::MatrixXd R1 = Eigen::MatrixXd::Identity(4, 4);
  R1.diagonal() << 0.01, 0.01, 0.5, 0.1;

  auto z_subtract1 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    // phase 残差必须 wrap，避免 2π 误差
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  // 观测 phase 用 wrap 值即可（残差里会 wrap）
  Eigen::VectorXd z1{{R_ypd[0], R_ypd[1], R_ypd[2], ypr_phase_wrapped}};
  ekf_.update(z1, H1, R1, z_subtract1);

  // 更新2：扇叶末端点约束
  Eigen::MatrixXd H2 = h_jacobian();
  Eigen::MatrixXd R2 = Eigen::MatrixXd::Identity(3, 3);
  R2.diagonal() << 0.01, 0.01, 0.5;

  auto h2 = [&](const Eigen::VectorXd & x) -> Eigen::Vector3d {
    Eigen::Matrix3d R_buff2world =
      tools::rotation_matrix(Eigen::Vector3d(tools::limit_rad(x[4]), 0.0, tools::limit_rad(x[5])));
    Eigen::Vector3d p_buff = blade_point_in_buff();

    double R_yaw = x[0];
    double R_pitch = x[2];
    double R_dis = x[3];

    Eigen::Vector3d center_world(
      R_dis * std::cos(R_pitch) * std::cos(R_yaw),
      R_dis * std::cos(R_pitch) * std::sin(R_yaw),
      R_dis * std::sin(R_pitch));

    Eigen::Vector3d point_in_world = R_buff2world * p_buff + center_world;
    return tools::xyz2ypd(point_in_world);
  };

  auto z_subtract2 = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    return c;
  };

  Eigen::VectorXd z2{{B_ypd[0], B_ypd[1], B_ypd[2]}};
  ekf_.update(z2, H2, R2, h2, z_subtract2);

  // 拟合速度（仅做日志/诊断，不影响预测主链路）
  if (ekf_.x[6] < 2.1 && ekf_.x[6] >= 0) spd_fitter_.add_data(nowtime, ekf_.x[6]);
  spd_fitter_.fit();
  fit_spd_ = spd_fitter_.sine_function(
    nowtime,
    spd_fitter_.best_result_.A,
    spd_fitter_.best_result_.omega,
    spd_fitter_.best_result_.phi,
    spd_fitter_.best_result_.C);

  // 限幅
  if (ekf_.x[6] > 6.0) ekf_.x[6] = 6.0;
  if (ekf_.x[6] < -6.0) ekf_.x[6] = -6.0;

  // 方向迟滞（你已经用并验证不反）
  const double kDirLo = 0.05;
  const double kDirHi = 0.15;

  const double spd_now = ekf_.x[6];
  const double abs_spd = std::abs(spd_now);
  const int dir_from_spd = (spd_now > 0.0) ? -1 : 1;

  if (!rotation_dir_locked_) {
    if (abs_spd > kDirHi) {
      rotation_dir_ = dir_from_spd;
      rotation_dir_locked_ = true;
    }
  } else {
    if (abs_spd > kDirHi) {
      rotation_dir_ = dir_from_spd;
    }
  }

  lasttime_ = nowtime;
  unsolvable_ = false;

  last_measurement_ = p;
  has_last_measurement_ = true;

#if AUTO_BUFF_DEBUG
  if (debug_frame_ % AUTO_BUFF_DEBUG_STRIDE == 0) {
    tools::logger()->debug(
      "[AutoBuff][DBG] after_update t={:.3f} track_age={} ekf_phase(u)={:.3f} ekf_phase(w)={:.3f} "
      "dir={} locked={} spd={:.3f} FIT(A,w,fi,C)=({:.3f},{:.3f},{:.3f},{:.3f})",
      nowtime, track_age_, ekf_.x[5], tools::limit_rad(ekf_.x[5]),
      rotation_dir_, rotation_dir_locked_, ekf_.x[6],
      spd_fitter_.best_result_.A,
      spd_fitter_.best_result_.omega,
      spd_fitter_.best_result_.phi,
      spd_fitter_.best_result_.C);
  }
#endif
}

Eigen::MatrixXd BigTarget::h_jacobian() const
{
  Eigen::MatrixXd H0 = Eigen::MatrixXd::Zero(5, 10);
  H0(0, 0) = 1;
  H0(1, 2) = 1;
  H0(2, 3) = 1;
  H0(3, 4) = 1;
  H0(4, 5) = 1;

  Eigen::VectorXd R_ypd{{ekf_.x[0], ekf_.x[2], ekf_.x[3]}};
  Eigen::MatrixXd H_ypd2xyz = tools::ypd2xyz_jacobian(R_ypd);

  Eigen::MatrixXd H1 = Eigen::MatrixXd::Zero(5, 5);
  H1.block(0, 0, 3, 3) = H_ypd2xyz;
  H1(3, 3) = 1;
  H1(4, 4) = 1;

  double yaw = tools::limit_rad(ekf_.x[4]);
  double roll = tools::limit_rad(ekf_.x[5]);
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cr = std::cos(roll), sr = std::sin(roll);
  const double r = kBladeRadius;

  Eigen::MatrixXd H2(3, 5);
  H2.setZero();
  H2(0, 0) = 1;
  H2(1, 1) = 1;
  H2(2, 2) = 1;

  // d/dyaw
  H2(0, 3) = r * cr * cy;
  H2(1, 3) = r * cr * sy;
  H2(2, 3) = 0.0;

  // d/droll
  H2(0, 4) = -r * sr * sy;
  H2(1, 4) =  r * sr * cy;
  H2(2, 4) = -r * cr;

  Eigen::Vector3d B_xyz = point_buff2world(blade_point_in_buff());
  Eigen::MatrixXd H3 = tools::xyz2ypd_jacobian(B_xyz);

  return H3 * H2 * H1 * H0;
}

}  // namespace auto_buff
