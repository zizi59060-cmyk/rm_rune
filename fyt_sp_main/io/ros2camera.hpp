#ifndef IO__ROS2CAMERA_HPP
#define IO__ROS2CAMERA_HPP

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include "io/ros2/time_sync.hpp"

namespace io
{
class ROS2Camera
{
public:
  explicit ROS2Camera(const std::string & config_path);
  ~ROS2Camera();

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  void load_config(const std::string & config_path);
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  bool initialized_here_{false};
  std::string image_topic_{"/image_raw"};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  std::mutex mtx_;
  std::condition_variable cv_;
  cv::Mat latest_img_;
  std::chrono::steady_clock::time_point latest_timestamp_{};
  size_t frame_seq_{0};
  size_t last_read_seq_{0};

  RosStampSync stamp_sync_;
};
}  // namespace io

#endif  // IO__ROS2CAMERA_HPP