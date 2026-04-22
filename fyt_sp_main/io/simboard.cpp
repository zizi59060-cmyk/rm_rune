#include "io/simboard.hpp"

#include "tools/yaml.hpp"

#include <algorithm>
#include <sstream>

namespace io
{
namespace
{
Eigen::Quaterniond to_eigen_q(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond qq(q.w, q.x, q.y, q.z);
  return qq.normalized();
}

Eigen::Vector3d to_eigen_t(const geometry_msgs::msg::Vector3 & t)
{
  return Eigen::Vector3d(t.x, t.y, t.z);
}

Mode parse_mode(const YAML::Node & yaml)
{
  if (!yaml["sim_mode"]) {
    return Mode::big_buff;
  }

  const auto value = yaml["sim_mode"].as<std::string>();
  if (value == "idle") return Mode::idle;
  if (value == "auto_aim") return Mode::auto_aim;
  if (value == "small_buff") return Mode::small_buff;
  if (value == "big_buff") return Mode::big_buff;
  if (value == "outpost") return Mode::outpost;
  return Mode::big_buff;
}
}  // namespace

SimBoard::SimBoard(const std::string & config_path)
: bullet_speed(25.0), mode(Mode::big_buff), shoot_mode(ShootMode::left_shoot), ft_angle(0.0)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    initialized_here_ = true;
  }

  load_config(config_path);

  node_ = std::make_shared<rclcpp::Node>("fyt_sim_board");
  cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(cmd_topic_, 10);
  tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
    tf_topic_, 100, std::bind(&SimBoard::tf_callback, this, std::placeholders::_1));

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });
}

SimBoard::~SimBoard()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (node_) {
    executor_.remove_node(node_);
  }

  if (initialized_here_ && rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void SimBoard::load_config(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);

  if (yaml["sim_bullet_speed"]) {
    bullet_speed = yaml["sim_bullet_speed"].as<double>();
  }
  if (yaml["sim_cmd_topic"]) {
    cmd_topic_ = yaml["sim_cmd_topic"].as<std::string>();
  }
  if (yaml["sim_tf_topic"]) {
    tf_topic_ = yaml["sim_tf_topic"].as<std::string>();
  }
  mode = parse_mode(yaml);
}

void SimBoard::tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);

  for (const auto & tf : msg->transforms) {
    const auto & parent = tf.header.frame_id;
    const auto & child = tf.child_frame_id;

    if (child == "gimbal_link") {
      const auto t = stamp_sync_.to_steady(tf.header.stamp);
      imu_queue_.push_back({
        to_eigen_q(tf.transform.rotation),
        to_eigen_t(tf.transform.translation),
        t
      });
      while (imu_queue_.size() > 512) {
        imu_queue_.pop_front();
      }
      cv_.notify_all();
      continue;
    }

    if (parent == "gimbal_link" && child == "camera_link") {
      R_gimbal2camera_link_ = to_eigen_q(tf.transform.rotation).toRotationMatrix();
      t_gimbal2camera_link_ = to_eigen_t(tf.transform.translation);
      has_gimbal2camera_link_ = true;
      continue;
    }

    if (parent == "camera_link" && child == "camera_optical_frame") {
      R_camera_link2optical_ = to_eigen_q(tf.transform.rotation).toRotationMatrix();
      t_camera_link2optical_ = to_eigen_t(tf.transform.translation);
      has_camera_link2optical_ = true;
      continue;
    }
  }

  if (has_gimbal2camera_link_ && has_camera_link2optical_) {
    const Eigen::Matrix3d R_gimbal2optical =
      R_gimbal2camera_link_ * R_camera_link2optical_;
    const Eigen::Vector3d t_gimbal2optical =
      t_gimbal2camera_link_ + R_gimbal2camera_link_ * t_camera_link2optical_;

    R_camera2gimbal_ = R_gimbal2optical.transpose();
    t_camera2gimbal_ = -R_camera2gimbal_ * t_gimbal2optical;
    has_camera2gimbal_ = true;
  }
}

Eigen::Quaterniond SimBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  std::unique_lock<std::mutex> lock(mtx_);
  cv_.wait(lock, [this]() { return !imu_queue_.empty(); });

  if (imu_queue_.size() == 1 || timestamp <= imu_queue_.front().timestamp) {
    return imu_queue_.front().q;
  }

  if (timestamp >= imu_queue_.back().timestamp) {
    return imu_queue_.back().q;
  }

  for (size_t i = 1; i < imu_queue_.size(); ++i) {
    if (imu_queue_[i].timestamp < timestamp) {
      continue;
    }

    const auto & a = imu_queue_[i - 1];
    const auto & b = imu_queue_[i];

    const std::chrono::duration<double> ab = b.timestamp - a.timestamp;
    const std::chrono::duration<double> ac = timestamp - a.timestamp;

    const double k = (ab.count() <= 1e-9) ? 0.0 : (ac.count() / ab.count());
    return a.q.slerp(std::clamp(k, 0.0, 1.0), b.q).normalized();
  }

  return imu_queue_.back().q;
}

Eigen::Vector3d SimBoard::gimbal_position_at(std::chrono::steady_clock::time_point timestamp)
{
  std::unique_lock<std::mutex> lock(mtx_);
  cv_.wait(lock, [this]() { return !imu_queue_.empty(); });

  if (imu_queue_.size() == 1 || timestamp <= imu_queue_.front().timestamp) {
    return imu_queue_.front().t;
  }

  if (timestamp >= imu_queue_.back().timestamp) {
    return imu_queue_.back().t;
  }

  for (size_t i = 1; i < imu_queue_.size(); ++i) {
    if (imu_queue_[i].timestamp < timestamp) {
      continue;
    }

    const auto & a = imu_queue_[i - 1];
    const auto & b = imu_queue_[i];

    const std::chrono::duration<double> ab = b.timestamp - a.timestamp;
    const std::chrono::duration<double> ac = timestamp - a.timestamp;

    const double k = (ab.count() <= 1e-9) ? 0.0 : (ac.count() / ab.count());
    return (1.0 - k) * a.t + k * b.t;
  }

  return imu_queue_.back().t;
}

bool SimBoard::camera2gimbal(Eigen::Matrix3d & R_camera2gimbal, Eigen::Vector3d & t_camera2gimbal) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!has_camera2gimbal_) {
    return false;
  }
  R_camera2gimbal = R_camera2gimbal_;
  t_camera2gimbal = t_camera2gimbal_;
  return true;
}

void SimBoard::send(Command command) const
{
  std_msgs::msg::String msg;
  std::ostringstream oss;
  oss << (command.control ? 1 : 0) << ","
      << (command.shoot ? 1 : 0) << ","
      << command.yaw << ","
      << command.pitch;
  msg.data = oss.str();
  cmd_pub_->publish(msg);
}
}  // namespace io