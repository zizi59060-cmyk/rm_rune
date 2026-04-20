#ifndef AUTO_BUFF__TARGET_HPP
#define AUTO_BUFF__TARGET_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <optional>
#include <chrono>

#include "buff_type.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{

class Voter
{
public:
  Voter() = default;

  void vote(double last_phase, double now_phase)
  {
    double d = tools::limit_rad(now_phase - last_phase);
    if (d > 0.01) {
      clockwise_++;
    } else if (d < -0.01) {
      clockwise_--;
    }
    clockwise_ = std::max(-50, std::min(50, clockwise_));
  }

  int clockwise() const
  {
    return (clockwise_ >= 0) ? 1 : -1;
  }

private:
  int clockwise_ = 0;
};

class Target
{
public:
  Target();
  virtual ~Target() = default;

  virtual void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) = 0;

  virtual void predict(double dt) = 0;
  virtual Eigen::Vector3d predict_position(double dt) const = 0;

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & point_in_buff) const;

  bool is_unsolve() const;
  Eigen::VectorXd ekf_x() const;

protected:
  void init_common(double nowtime, const PowerRune & p);
  void update_common(double nowtime, const PowerRune & p, double omega_alpha);

  double unwrap_to_near(double reference, double wrapped) const;

  double clamp_dt(double dt) const;
  double clamp_phase_delta(double dphase) const;
  double clamp_omega(double omega) const;
  double clamp_predict_dt(double dt) const;

protected:
  bool unsolvable_ = true;
  bool first_in_ = true;
  int lost_cn_ = 0;

  double lasttime_ = 0.0;
  bool has_start_timestamp_ = false;
  std::chrono::steady_clock::time_point start_timestamp_;

  // tracked state
  Eigen::Vector3d center_world_ = Eigen::Vector3d::Zero();
  double center_yaw_ = 0.0;
  double center_pitch_ = 0.0;
  double center_distance_ = 0.0;

  double phase_ = 0.0;   // continuous phase
  double omega_ = 0.0;   // rad/s

  Voter voter_;

  // 兼容 auto_buff_sim.cpp 调试输出
  Eigen::VectorXd debug_x_;

  static constexpr double kBladeRadius_ = 0.7;

  // ===== 稳定化参数 =====
  static constexpr double kMinUpdateDt_ = 0.01;      // 10 ms
  static constexpr double kMaxPredictDt_ = 0.05;     // 50 ms
  static constexpr double kMaxPhaseStep_ = 0.35;     // rad/frame
  static constexpr double kMaxOmegaSmall_ = 3.5;     // rad/s
  static constexpr double kMaxOmegaBig_ = 8.0;       // rad/s

  // ===== 关键：模拟器旋转方向与当前 phase 观测方向相反 =====
  static constexpr double kOmegaSign_ = -1.0;
};

class SmallTarget : public Target
{
public:
  SmallTarget();

  void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;
  Eigen::Vector3d predict_position(double dt) const override;
};

class BigTarget : public Target
{
public:
  BigTarget();

  void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) override;

  void predict(double dt) override;
  Eigen::Vector3d predict_position(double dt) const override;
};

}  // namespace auto_buff

#endif