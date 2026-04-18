#ifndef IO__SIMBOARD_HPP
#define IO__SIMBOARD_HPP

#include <Eigen/Geometry>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "io/command.hpp"
#include "io/ros2/time_sync.hpp"

namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};

enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};

class SimBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;

  explicit SimBoard(const std::string & config_path);
  ~SimBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  bool camera2gimbal(Eigen::Matrix3d & R_camera2gimbal, Eigen::Vector3d & t_camera2gimbal) const;
  void send(Command command) const;

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void load_config(const std::string & config_path);
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

  bool initialized_here_{false};
  std::string cmd_topic_{"/fyt_cmd"};
  std::string tf_topic_{"/tf"};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;

  mutable std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<IMUData> imu_queue_;
  RosStampSync stamp_sync_;

  bool has_gimbal2camera_link_{false};
  bool has_camera_link2optical_{false};
  bool has_camera2gimbal_{false};

  Eigen::Matrix3d R_gimbal2camera_link_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_gimbal2camera_link_{Eigen::Vector3d::Zero()};

  Eigen::Matrix3d R_camera_link2optical_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_camera_link2optical_{Eigen::Vector3d::Zero()};

  Eigen::Matrix3d R_camera2gimbal_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t_camera2gimbal_{Eigen::Vector3d::Zero()};
};
}  // namespace io

#endif  // IO__SIMBOARD_HPP