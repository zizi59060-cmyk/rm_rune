/* tasks/auto_buff/buff_solver.cpp */
#include "buff_solver.hpp"
#include "tools/logger.hpp"

namespace auto_buff
{
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);
  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();

  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_  = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_  = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  auto camera_matrix_data  = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());

  cv::eigen2cv(camera_matrix, camera_matrix_);
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
  rvec_ = cv::Vec3d(0, 0, 0);
  tvec_ = cv::Vec3d(0, 0, 0);
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}
void Solver::set_camera2gimbal(const Eigen::Matrix3d & R_camera2gimbal, const Eigen::Vector3d & t_camera2gimbal)
{
  R_camera2gimbal_ = R_camera2gimbal;
  t_camera2gimbal_ = t_camera2gimbal;
}
void Solver::solve(std::optional<PowerRune> & ps) const
{
  if (!ps.has_value()) return;
  PowerRune & p = ps.value();

  if (p.target().points.size() < 5) {
    ps.value().set_unsolve(true);
    return;
  }

  // 使用 5 点 PnP (Center + 4 corners)
  std::vector<cv::Point2f> image_points_pnp = p.target().points;
  bool success = cv::solvePnP(
    OBJECT_POINTS, image_points_pnp, camera_matrix_, distort_coeffs_, rvec_, tvec_, false,
    cv::SOLVEPNP_ITERATIVE);

  if (!success) {
    ps.value().set_unsolve(true);
    return;
  }

  Eigen::Vector3d t_buff2camera;
  cv::cv2eigen(tvec_, t_buff2camera);
  cv::Mat rmat;
  cv::Rodrigues(rvec_, rmat);
  Eigen::Matrix3d R_buff2camera;
  cv::cv2eigen(rmat, R_buff2camera);

  // 计算符臂末端位置 (半径 0.7m, 沿 -Y 轴)
  Eigen::Vector3d blade_xyz_in_buff{{0, -0.7, 0}};

  Eigen::Vector3d xyz_in_camera       = t_buff2camera;
  Eigen::Vector3d blade_xyz_in_camera = R_buff2camera * blade_xyz_in_buff + t_buff2camera;

  Eigen::Matrix3d R_buff2gimbal       = R_camera2gimbal_ * R_buff2camera;
  Eigen::Vector3d xyz_in_gimbal       = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  Eigen::Vector3d blade_xyz_in_gimbal = R_camera2gimbal_ * blade_xyz_in_camera + t_camera2gimbal_;

  p.xyz_in_world = R_gimbal2world_ * xyz_in_gimbal;
  p.ypd_in_world = tools::xyz2ypd(p.xyz_in_world);

  p.blade_xyz_in_world = R_gimbal2world_ * blade_xyz_in_gimbal;
  p.blade_ypd_in_world = tools::xyz2ypd(p.blade_xyz_in_world);

  // 这里根据 PnP 结果计算 R_buff2world，并保存
  Eigen::Matrix3d R_buff2world = R_gimbal2world_ * R_buff2gimbal;
  p.R_buff2world = R_buff2world;

  // ----------------------------
  // 1) yaw / pitch：从 R_buff2world 提取（ZYX: yaw-pitch-roll）
  // 2) phase：保持你原来的 arm 向量法（EKF 用 ypr[2] 当相位）
  // ----------------------------
  Eigen::Vector3d ypr_from_R = tools::eulers(R_buff2world, 2, 1, 0);

  // 将 Arm 向量转回 Buff 系，计算相位 angle
  Eigen::Vector3d arm_vector = p.blade_xyz_in_world - p.xyz_in_world;
  Eigen::Vector3d arm_in_buff = R_buff2world.transpose() * arm_vector;

  // 在 YZ 平面上，角度由 -Y 和 Z 决定（atan2 输出已在 [-pi, pi]）
  double angle = std::atan2(arm_in_buff.z(), -arm_in_buff.y());

  p.ypr_in_world = Eigen::Vector3d(
    ypr_from_R[0],   // yaw
    ypr_from_R[1],   // pitch
    angle            // phase (stored in ypr[2])
  );
}

cv::Point2f Solver::point_buff2pixel(cv::Point3f x)
{
  std::vector<cv::Point3d> world_points = {x};
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(world_points, rvec_, tvec_, camera_matrix_, distort_coeffs_, image_points);
  if (image_points.empty()) return {0, 0};
  return image_points.back();
}

}  // namespace auto_buff
