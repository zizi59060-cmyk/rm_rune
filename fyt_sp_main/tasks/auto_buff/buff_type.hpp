#ifndef BUFF__TYPE_HPP
#define BUFF__TYPE_HPP

#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

#include "tools/math_tools.hpp"

namespace auto_buff
{

enum FanBlade_type { _target, _unlight, _light };

class FanBlade
{
public:
  cv::Point2f center;
  std::vector<cv::Point2f> points;   // [0] = r center, [1..4] = ordered corners
  double angle = 0.0;
  double width = 0.0;
  double height = 0.0;
  FanBlade_type type = _unlight;

  FanBlade() = default;

  FanBlade(
    const std::vector<cv::Point2f> & kpt,
    cv::Point2f keypoints_center,
    FanBlade_type t);

  explicit FanBlade(FanBlade_type t);
};

class PowerRune
{
public:
  cv::Point2f r_center;
  std::vector<FanBlade> fanblades;
  int light_num = 0;

  // solver outputs
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();   // [yaw, pitch, phase]
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();

  Eigen::Vector3d blade_xyz_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d blade_ypd_in_world = Eigen::Vector3d::Zero();

  Eigen::Matrix3d R_buff2world = Eigen::Matrix3d::Identity();

  PowerRune(
    std::vector<FanBlade> & ts,
    const cv::Point2f r_center_in,
    std::optional<PowerRune> last_powerrune);

  PowerRune() = default;

  FanBlade & target() { return fanblades[0]; }
  const FanBlade & target() const { return fanblades[0]; }

  bool is_unsolve() const { return unsolvable_; }
  void set_unsolve(bool state) { unsolvable_ = state; }

private:
  bool unsolvable_ = false;
  double atan_angle(cv::Point2f point) const;
};

}  // namespace auto_buff

#endif