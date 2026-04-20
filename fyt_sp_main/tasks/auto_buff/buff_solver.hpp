#ifndef AUTO_BUFF__SOLVER_HPP
#define AUTO_BUFF__SOLVER_HPP

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <optional>

#include "buff_type.hpp"
#include "tools/math_tools.hpp"

namespace auto_buff
{

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;
  void set_R_gimbal2world(const Eigen::Quaterniond & q);
  void set_camera2gimbal(
    const Eigen::Matrix3d & R_camera2gimbal,
    const Eigen::Vector3d & t_camera2gimbal);

  void solve(std::optional<PowerRune> & ps) const;

  cv::Point2f point_buff2pixel(cv::Point3f x);

  cv::Point2f reproject_world_point(const Eigen::Vector3d & xyz_in_world) const
  {
    Eigen::Vector3d xyz_in_gimbal = R_gimbal2world_.transpose() * xyz_in_world;
    Eigen::Vector3d xyz_in_camera =
      R_camera2gimbal_.transpose() * (xyz_in_gimbal - t_camera2gimbal_);

    std::vector<cv::Point3f> pts_3d;
    pts_3d.emplace_back(xyz_in_camera.x(), xyz_in_camera.y(), xyz_in_camera.z());

    std::vector<cv::Point2f> pts_2d;
    cv::projectPoints(
      pts_3d, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0),
      camera_matrix_, distort_coeffs_, pts_2d);

    if (pts_2d.empty()) {
      return {-1, -1};
    }
    return pts_2d[0];
  }

private:
  Eigen::Vector3d blade_point_from_phase(
    double center_yaw,
    const Eigen::Vector3d & center_world,
    double phase) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;

  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  mutable cv::Vec3d rvec_, tvec_;

  // [0] rune center
  // [1] 左上
  // [2] 右上
  // [3] 右下
  // [4] 左下
  const std::vector<cv::Point3f> OBJECT_POINTS = {
    cv::Point3f(0.0f,  0.00f,  0.00f),
    cv::Point3f(0.0f, -0.85f,  0.16f),
    cv::Point3f(0.0f, -0.54f,  0.18f),
    cv::Point3f(0.0f, -0.54f, -0.18f),
    cv::Point3f(0.0f, -0.85f, -0.16f)
  };

  const double kBladeRadius_ = 0.7;
};

}  // namespace auto_buff

#endif