#include "buff_solver.hpp"
#include "tools/logger.hpp"

namespace auto_buff
{

Solver::Solver(const std::string & config_path)
: R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();

  R_gimbal2imubody_ =
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_ =
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_ =
    Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());

  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);

  rvec_ = cv::Vec3d(0, 0, 0);
  tvec_ = cv::Vec3d(0, 0, 0);
}

Eigen::Matrix3d Solver::R_gimbal2world() const
{
  return R_gimbal2world_;
}

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ =
    R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}

void Solver::set_camera2gimbal(
  const Eigen::Matrix3d & R_camera2gimbal,
  const Eigen::Vector3d & t_camera2gimbal)
{
  R_camera2gimbal_ = R_camera2gimbal;
  t_camera2gimbal_ = t_camera2gimbal;
}

Eigen::Vector3d Solver::blade_point_from_phase(
  double center_yaw,
  const Eigen::Vector3d & center_world,
  double phase) const
{
  const Eigen::Vector3d u(-std::sin(center_yaw), std::cos(center_yaw), 0.0);
  const Eigen::Vector3d v(0.0, 0.0, 1.0);

  Eigen::Vector3d offset =
    kBladeRadius_ * (std::cos(phase) * u + std::sin(phase) * v);

  return center_world + offset;
}

void Solver::solve(std::optional<PowerRune> & ps) const
{
  if (!ps.has_value()) {
    return;
  }

  PowerRune & p = ps.value();

  if (p.target().points.size() < 5) {
    p.set_unsolve(true);
    return;
  }

  std::vector<cv::Point2f> image_points_pnp = p.target().points;

  bool success = cv::solvePnP(
    OBJECT_POINTS,
    image_points_pnp,
    camera_matrix_,
    distort_coeffs_,
    rvec_,
    tvec_,
    false,
    cv::SOLVEPNP_ITERATIVE);

  if (!success) {
    p.set_unsolve(true);
    return;
  }

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);

  Eigen::Vector3d xyz_in_camera = t_buff2camera;
  Eigen::Vector3d xyz_in_gimbal =
    R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  Eigen::Vector3d center_world =
    R_gimbal2world_ * xyz_in_gimbal;

  p.xyz_in_world = center_world;
  p.ypd_in_world = tools::xyz2ypd(center_world);

  const double center_yaw = p.ypd_in_world[0];
  const double center_pitch = p.ypd_in_world[1];

  cv::Point2f img_r = p.r_center;
  cv::Point2f img_b = p.target().center;
  cv::Point2f img_v = img_b - img_r;

  // 这里改成与当前模拟器实际旋向一致的定义
  // 原来是 atan2(-img_v.y, img_v.x)，会导致 obs/pred 与真实扇叶方向相反
  double phase = std::atan2(img_v.y, img_v.x);
  phase = tools::limit_rad(phase);

  Eigen::Vector3d blade_world =
    blade_point_from_phase(center_yaw, center_world, phase);

  p.blade_xyz_in_world = blade_world;
  p.blade_ypd_in_world = tools::xyz2ypd(blade_world);

  p.ypr_in_world = Eigen::Vector3d(center_yaw, center_pitch, phase);

  Eigen::Matrix3d R =
    tools::rotation_matrix(Eigen::Vector3d(center_yaw, 0.0, phase));
  p.R_buff2world = R;

  p.set_unsolve(false);
}

cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  std::vector<cv::Point3d> world_points = {x};
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(
    world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  if (image_points.empty()) {
    return {0, 0};
  }
  return image_points.back();
}

}  // namespace auto_buff