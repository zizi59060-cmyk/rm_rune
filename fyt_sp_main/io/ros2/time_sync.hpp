#ifndef IO__ROS2__TIME_SYNC_HPP
#define IO__ROS2__TIME_SYNC_HPP

#include <builtin_interfaces/msg/time.hpp>

#include <chrono>
#include <cstdint>
#include <mutex>

namespace io
{
class RosStampSync
{
public:
  std::chrono::steady_clock::time_point to_steady(const builtin_interfaces::msg::Time & stamp)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const int64_t ros_ns =
      static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);

    if (!initialized_) {
      initialized_ = true;
      ros_base_ns_ = ros_ns;
      steady_base_ = std::chrono::steady_clock::now();
    }

    return steady_base_ + std::chrono::nanoseconds(ros_ns - ros_base_ns_);
  }

private:
  std::mutex mtx_;
  bool initialized_{false};
  int64_t ros_base_ns_{0};
  std::chrono::steady_clock::time_point steady_base_{};
};
}  // namespace io

#endif  // IO__ROS2__TIME_SYNC_HPP