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
  std::vector<cv::Point2f> points;
  double angle, width, height;
  FanBlade_type type;

  FanBlade() = default;

  FanBlade(const std::vector<cv::Point2f> & kpt,
           cv::Point2f keypoints_center,
           FanBlade_type t);

  explicit FanBlade(FanBlade_type t);
};

class PowerRune
{
public:
  cv::Point2f r_center;
  std::vector<FanBlade> fanblades;

  int light_num;

  Eigen::Vector3d xyz_in_world;
  Eigen::Vector3d ypr_in_world;
  Eigen::Vector3d ypd_in_world;

  Eigen::Vector3d blade_xyz_in_world;
  Eigen::Vector3d blade_ypd_in_world;

  Eigen::Matrix3d R_buff2world;

  PowerRune(std::vector<FanBlade> & ts,
            const cv::Point2f r_center,
            std::optional<PowerRune> last_powerrune);

  PowerRune() = default;

  FanBlade & target() { return fanblades[0]; }

  bool is_unsolve() const { return unsolvable_; }
  void set_unsolve(bool state) { unsolvable_ = state; }

private:
  bool unsolvable_ = false;
  double atan_angle(cv::Point2f point) const;
};

}  // namespace auto_buff

#endif
