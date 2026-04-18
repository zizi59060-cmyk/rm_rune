#include "io/ros2camera.hpp"

#include "tools/yaml.hpp"

#include <opencv2/imgproc.hpp>

namespace io
{
ROS2Camera::ROS2Camera(const std::string & config_path)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
    initialized_here_ = true;
  }

  load_config(config_path);

  node_ = std::make_shared<rclcpp::Node>("fyt_ros2_camera");
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, 10, std::bind(&ROS2Camera::image_callback, this, std::placeholders::_1));

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });
}

ROS2Camera::~ROS2Camera()
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

void ROS2Camera::load_config(const std::string & config_path)
{
  const auto yaml = tools::load(config_path);
  if (yaml["sim_image_topic"]) {
    image_topic_ = yaml["sim_image_topic"].as<std::string>();
  }
}

void ROS2Camera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat frame;

  if (msg->encoding == "rgb8") {
    cv::Mat rgb(
      static_cast<int>(msg->height),
      static_cast<int>(msg->width),
      CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()),
      static_cast<size_t>(msg->step));
    cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
  } else if (msg->encoding == "bgr8") {
    cv::Mat bgr(
      static_cast<int>(msg->height),
      static_cast<int>(msg->width),
      CV_8UC3,
      const_cast<unsigned char *>(msg->data.data()),
      static_cast<size_t>(msg->step));
    frame = bgr.clone();
  } else if (msg->encoding == "mono8") {
    cv::Mat mono(
      static_cast<int>(msg->height),
      static_cast<int>(msg->width),
      CV_8UC1,
      const_cast<unsigned char *>(msg->data.data()),
      static_cast<size_t>(msg->step));
    cv::cvtColor(mono, frame, cv::COLOR_GRAY2BGR);
  } else {
    return;
  }

  const auto t = stamp_sync_.to_steady(msg->header.stamp);

  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_img_ = frame;
    latest_timestamp_ = t;
    ++frame_seq_;
  }
  cv_.notify_all();
}

void ROS2Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  std::unique_lock<std::mutex> lock(mtx_);
  cv_.wait(lock, [this]() { return frame_seq_ > last_read_seq_; });

  img = latest_img_.clone();
  timestamp = latest_timestamp_;
  last_read_seq_ = frame_seq_;
}
}  // namespace io