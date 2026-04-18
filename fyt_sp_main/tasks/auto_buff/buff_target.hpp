/* tasks/auto_buff/buff_target.hpp */
#ifndef AUTO_BUFF__TARGET_HPP
#define AUTO_BUFF__TARGET_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>
#include <chrono>

#include "buff_detector.hpp"
#include "buff_type.hpp"
#include "tools/extended_kalman_filter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/ransac_sine_fitter.hpp"

namespace auto_buff
{

class Voter
{
public:
  Voter();
  void vote(const double angle_last, const double angle_now);
  int clockwise() const;
  int votes() const { return clockwise_; }

private:
  int clockwise_;
};

class Target
{
public:
  Target();
  virtual void get_target(
    const std::optional<PowerRune> & p,
    std::chrono::steady_clock::time_point & timestamp) = 0;

  virtual void predict(double dt) = 0;
  virtual Eigen::Vector3d predict_position(double dt) const = 0;

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d & point_in_buff) const;

  bool is_unsolve() const;
  Eigen::VectorXd ekf_x() const;

  double spd = 0;

protected:
  virtual void init(double nowtime, const PowerRune & p) = 0;
  virtual void update(double nowtime, const PowerRune & p) = 0;

  // --- timebase ---
  void reset_timebase(std::chrono::steady_clock::time_point & timestamp)
  {
    start_timestamp_ = timestamp;
    has_start_timestamp_ = true;
  }

  bool compute_nowtime(
    std::chrono::steady_clock::time_point & timestamp,
    double & nowtime_out)
  {
    if (!has_start_timestamp_) {
      reset_timebase(timestamp);
      nowtime_out = 0.0;
      return false;
    }
    nowtime_out = tools::delta_time(timestamp, start_timestamp_);
    if (nowtime_out < -1e-3) {
      reset_timebase(timestamp);
      nowtime_out = 0.0;
      return true;
    }
    return false;
  }

  // --- EKF data ---
  Eigen::VectorXd x0_;
  Eigen::MatrixXd P0_;
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  tools::ExtendedKalmanFilter ekf_;
  double lasttime_ = 0.0;

  Voter voter;

  // 方向（+1/-1）
  int rotation_dir_ = 1;
  bool rotation_dir_locked_ = false;

  bool first_in_;
  bool unsolvable_;

  int lost_cn_ = 0;

  std::chrono::steady_clock::time_point start_timestamp_;
  bool has_start_timestamp_ = false;
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

private:
  void init(double nowtime, const PowerRune & p) override;
  void update(double nowtime, const PowerRune & p) override;
  Eigen::MatrixXd h_jacobian() const;

  const double SMALL_W = CV_PI / 3;
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

private:
  void init(double nowtime, const PowerRune & p) override;
  void update(double nowtime, const PowerRune & p) override;
  Eigen::MatrixXd h_jacobian() const;

  // --- fitter ---
  tools::RansacSineFitter spd_fitter_;
  double fit_spd_;

  // --- last PnP measurement for predict_position ---
  PowerRune last_measurement_;
  bool has_last_measurement_ = false;

  // --- debug / edge ---
  int debug_frame_ = 0;
  bool last_has_target_ = false;

  // --- NEW: 用于解决“第一片更远”的相位/对齐稳定 ---
  int track_age_ = 0;          // 进入跟踪后的帧计数
  int last_align_i_ = 0;       // 上一帧使用的等价类偏移
  double last_align_err_ = 1e9;

  double ypr_phase_unwrapped_ = 0.0;   // 观测相位的连续版本（仅用于对齐）
  bool has_ypr_phase_unwrapped_ = false;
};

}  // namespace auto_buff
#endif
