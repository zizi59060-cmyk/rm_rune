#include <fmt/core.h>
#include <chrono>
#include <memory>
#include <optional>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include "io/ros2camera.hpp"
#include "io/simboard.hpp"

#include "tasks/auto_buff/buff_type.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/yolox_buff.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sim_buff.yaml | yaml配置文件路径}";

namespace
{

cv::Scalar color_from_label(int label)
{
  // label == 0 -> unlight
  // label == 1 -> light
  if (label == 1) return cv::Scalar(0, 0, 255);      // red
  return cv::Scalar(0, 255, 255);                    // yellow
}

void draw_cross(cv::Mat & img, const cv::Point2f & p, const cv::Scalar & color, int size = 6, int thickness = 2)
{
  cv::line(img, cv::Point2f(p.x - size, p.y), cv::Point2f(p.x + size, p.y), color, thickness);
  cv::line(img, cv::Point2f(p.x, p.y - size), cv::Point2f(p.x, p.y + size), color, thickness);
}

void draw_circle_text(
  cv::Mat & img,
  const cv::Point2f & p,
  const cv::Scalar & color,
  const std::string & text,
  int radius = 4)
{
  cv::circle(img, p, radius, color, -1);
  cv::putText(
    img,
    text,
    cv::Point(static_cast<int>(p.x + 6), static_cast<int>(p.y - 6)),
    cv::FONT_HERSHEY_SIMPLEX,
    0.5,
    color,
    1,
    cv::LINE_AA);
}

cv::Point2f mean4(
  const cv::Point2f & a,
  const cv::Point2f & b,
  const cv::Point2f & c,
  const cv::Point2f & d)
{
  return cv::Point2f(
    0.25f * (a.x + b.x + c.x + d.x),
    0.25f * (a.y + b.y + c.y + d.y));
}

// 把 YOLOX 输出的 5 点统一成：
// [0] r_center
// [1] 左上
// [2] 右上
// [3] 右下
// [4] 左下
std::vector<cv::Point2f> normalize_kpt_order(const std::vector<cv::Point2f> & raw_kpt)
{
  std::vector<cv::Point2f> out;
  if (raw_kpt.size() < 5) {
    return out;
  }

  const cv::Point2f r_center = raw_kpt[0];
  std::vector<cv::Point2f> corners(raw_kpt.begin() + 1, raw_kpt.begin() + 5);
  const cv::Point2f c = mean4(corners[0], corners[1], corners[2], corners[3]);

  // 先按极角排序
  std::sort(
    corners.begin(), corners.end(),
    [&](const cv::Point2f & a, const cv::Point2f & b) {
      double aa = std::atan2(a.y - c.y, a.x - c.x);
      double bb = std::atan2(b.y - c.y, b.x - c.x);
      return aa < bb;
    });

  // 以最像左上角的点为起点
  int start_idx = 0;
  float best = corners[0].x + corners[0].y;
  for (int i = 1; i < 4; ++i) {
    float s = corners[i].x + corners[i].y;
    if (s < best) {
      best = s;
      start_idx = i;
    }
  }

  std::vector<cv::Point2f> ordered(4);
  for (int i = 0; i < 4; ++i) {
    ordered[i] = corners[(start_idx + i) % 4];
  }

  // 检查顺逆时针，如果方向反了则交换 2/4
  auto cross =
    (ordered[1].x - ordered[0].x) * (ordered[2].y - ordered[0].y) -
    (ordered[1].y - ordered[0].y) * (ordered[2].x - ordered[0].x);

  if (cross < 0) {
    std::swap(ordered[1], ordered[3]);
  }

  out.reserve(5);
  out.push_back(r_center);
  out.insert(out.end(), ordered.begin(), ordered.end());
  return out;
}

cv::Point2f compute_r_center(const std::vector<auto_buff::FanBlade> & fanblades)
{
  if (fanblades.empty()) {
    return {0, 0};
  }

  cv::Point2f r_center(0, 0);
  int cnt = 0;
  for (const auto & fb : fanblades) {
    if (!fb.points.empty()) {
      r_center += fb.points[0];
      ++cnt;
    }
  }
  if (cnt == 0) {
    return {0, 0};
  }
  r_center *= (1.0f / static_cast<float>(cnt));
  return r_center;
}

void draw_raw_candidate(
  cv::Mat & vis,
  const std::vector<cv::Point2f> & raw_kpt,
  int label,
  int idx)
{
  if (raw_kpt.size() < 5) {
    return;
  }

  const cv::Scalar col = color_from_label(label);

  // 原始 r center
  draw_circle_text(vis, raw_kpt[0], cv::Scalar(255, 255, 255), fmt::format("raw_r{}", idx), 5);

  // 原始四角点
  for (int i = 1; i <= 4; ++i) {
    draw_circle_text(vis, raw_kpt[i], col, fmt::format("raw{}-{}", idx, i), 4);
  }

  // 原始 polygon
  for (int i = 1; i <= 4; ++i) {
    cv::line(vis, raw_kpt[i], raw_kpt[(i % 4) + 1], col, 1);
  }
}

void draw_normalized_candidate(
  cv::Mat & vis,
  const std::vector<cv::Point2f> & kpt,
  int label,
  int idx,
  bool selected)
{
  if (kpt.size() < 5) {
    return;
  }

  cv::Scalar col = selected ? cv::Scalar(0, 255, 0) : color_from_label(label);

  // 归一化 r center
  draw_circle_text(vis, kpt[0], cv::Scalar(255, 200, 200), fmt::format("r{}", idx), 5);

  // 归一化 corners
  for (int i = 1; i <= 4; ++i) {
    draw_circle_text(vis, kpt[i], col, fmt::format("{}-{}", idx, i), 4);
  }

  for (int i = 1; i <= 4; ++i) {
    cv::line(vis, kpt[i], kpt[(i % 4) + 1], col, selected ? 3 : 2);
  }

  cv::Point2f armor_center(0, 0);
  for (int i = 1; i <= 4; ++i) {
    armor_center += kpt[i];
  }
  armor_center *= 0.25f;
  draw_cross(vis, armor_center, selected ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 255), 8, 2);

  cv::putText(
    vis,
    selected ? fmt::format("TARGET {}", idx) : fmt::format("cand {}", idx),
    cv::Point(static_cast<int>(armor_center.x + 10), static_cast<int>(armor_center.y + 15)),
    cv::FONT_HERSHEY_SIMPLEX,
    0.6,
    selected ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 255, 255),
    2,
    cv::LINE_AA);
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
      cli.printMessage();
      rclcpp::shutdown();
      return 0;
    }

    const auto config_path = cli.get<std::string>(0);

    tools::Exiter exiter;

    std::cout << "[dbg_view] config_path = " << config_path << std::endl;

    io::SimBoard simboard(config_path);
    io::ROS2Camera camera(config_path);

    auto_buff::YoloXBuff yolo(config_path);
    auto_buff::Solver buff_solver(config_path);

    cv::Mat img;
    std::optional<auto_buff::PowerRune> last_powerrune;

    while (!exiter.exit() && rclcpp::ok()) {
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) {
        cv::waitKey(1);
        continue;
      }

      // 更新世界系姿态 / 相机外参
      Eigen::Quaterniond q = simboard.imu_at(timestamp - std::chrono::milliseconds(1));
      buff_solver.set_R_gimbal2world(q);

      Eigen::Matrix3d R_camera2gimbal;
      Eigen::Vector3d t_camera2gimbal;
      if (simboard.camera2gimbal(R_camera2gimbal, t_camera2gimbal)) {
        buff_solver.set_camera2gimbal(R_camera2gimbal, t_camera2gimbal);
      }

      cv::Mat vis = img.clone();

      // 1) 直接看模型原始输出
      std::vector<auto_buff::YoloXBuff::Object> results = yolo.get_multicandidateboxes(img);

      std::vector<auto_buff::FanBlade> fanblades;
      std::vector<std::vector<cv::Point2f>> normalized_kpts;
      std::vector<int> labels;

      fanblades.reserve(results.size());
      normalized_kpts.reserve(results.size());
      labels.reserve(results.size());

      for (size_t i = 0; i < results.size(); ++i) {
        const auto & obj = results[i];

        if (obj.kpt.size() < 5) {
          continue;
        }

        draw_raw_candidate(vis, obj.kpt, obj.label, static_cast<int>(i));

        auto kpt = normalize_kpt_order(obj.kpt);
        if (kpt.size() < 5) {
          continue;
        }

        cv::Point2f armor_center(0, 0);
        for (int j = 1; j <= 4; ++j) {
          armor_center += kpt[j];
        }
        armor_center *= 0.25f;

        auto_buff::FanBlade_type fb_type =
          (obj.label == 1) ? auto_buff::_light : auto_buff::_unlight;

        fanblades.emplace_back(auto_buff::FanBlade(kpt, armor_center, fb_type));
        normalized_kpts.emplace_back(kpt);
        labels.emplace_back(obj.label);
      }

      cv::Point2f r_center = compute_r_center(fanblades);
      draw_cross(vis, r_center, cv::Scalar(255, 255, 255), 12, 2);
      cv::putText(
        vis,
        "global_r_center",
        cv::Point(static_cast<int>(r_center.x + 12), static_cast<int>(r_center.y - 12)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA);

      // 2) 画所有归一化候选
      for (size_t i = 0; i < normalized_kpts.size(); ++i) {
        draw_normalized_candidate(vis, normalized_kpts[i], labels[i], static_cast<int>(i), false);
      }

      // 3) 构造 PowerRune，走和正式链路一致的 target 选择
      std::optional<auto_buff::PowerRune> rune_opt;
      if (!fanblades.empty()) {
        auto_buff::PowerRune rune(fanblades, r_center, last_powerrune);
        rune_opt = rune;
      }

      bool solved = false;
      if (rune_opt.has_value()) {
        buff_solver.solve(rune_opt);
        if (!rune_opt->is_unsolve()) {
          solved = true;
          last_powerrune = rune_opt;
        }
      }

      // 4) 高亮真正被选中的 target，并画 solver 结果
      if (rune_opt.has_value() && !rune_opt->fanblades.empty()) {
        const auto & tgt = rune_opt->target();
        if (tgt.points.size() >= 5) {
          draw_normalized_candidate(vis, tgt.points, 0, 999, true);
        }
      }

      // 5) 如果 solve 成功，画重投影点和数值
      if (solved) {
        const auto & p = rune_opt.value();

        cv::Point2f reproj_center = buff_solver.reproject_world_point(p.xyz_in_world);
        cv::Point2f reproj_blade = buff_solver.reproject_world_point(p.blade_xyz_in_world);

        draw_cross(vis, reproj_center, cv::Scalar(255, 0, 0), 14, 2);
        draw_cross(vis, reproj_blade, cv::Scalar(0, 255, 0), 14, 2);

        cv::putText(
          vis,
          "reproj_center",
          cv::Point(static_cast<int>(reproj_center.x + 12), static_cast<int>(reproj_center.y - 12)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.6,
          cv::Scalar(255, 0, 0),
          2,
          cv::LINE_AA);

        cv::putText(
          vis,
          "reproj_blade",
          cv::Point(static_cast<int>(reproj_blade.x + 12), static_cast<int>(reproj_blade.y - 12)),
          cv::FONT_HERSHEY_SIMPLEX,
          0.6,
          cv::Scalar(0, 255, 0),
          2,
          cv::LINE_AA);

        tools::draw_text(
          vis,
          fmt::format("R_yaw:{:.2f} R_pitch:{:.2f} R_dis:{:.2f}",
                      p.ypd_in_world[0] * 57.3,
                      p.ypd_in_world[1] * 57.3,
                      p.ypd_in_world[2]),
          {20, 30}, {255, 255, 255});

        tools::draw_text(
          vis,
          fmt::format("buff_yaw:{:.2f} buff_pitch:{:.2f} buff_phase:{:.2f}",
                      p.ypr_in_world[0] * 57.3,
                      p.ypr_in_world[1] * 57.3,
                      p.ypr_in_world[2] * 57.3),
          {20, 60}, {255, 255, 0});

        tools::draw_text(
          vis,
          fmt::format("blade_yaw:{:.2f} blade_pitch:{:.2f} blade_dis:{:.2f}",
                      p.blade_ypd_in_world[0] * 57.3,
                      p.blade_ypd_in_world[1] * 57.3,
                      p.blade_ypd_in_world[2]),
          {20, 90}, {0, 255, 0});

        tools::draw_text(
          vis,
          fmt::format("raw_candidates:{} selected_target:yes", results.size()),
          {20, 120}, {200, 200, 255});
      } else {
        tools::draw_text(
          vis,
          fmt::format("raw_candidates:{} selected_target:no/unsolved", results.size()),
          {20, 30}, {0, 0, 255});
      }

      tools::draw_text(
        vis,
        "DEBUG VIEW ONLY - NO COMMAND SENT TO SIM",
        {20, 150}, {0, 255, 255});

      cv::Mat show;
      cv::resize(vis, show, {}, 0.5, 0.5, cv::INTER_AREA);
      cv::imshow("auto_buff_debug_view", show);

      int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q') {
        break;
      }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
  } catch (const YAML::BadConversion & e) {
    std::cerr << "[auto_buff_debug_view] YAML::BadConversion: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (const YAML::Exception & e) {
    std::cerr << "[auto_buff_debug_view] YAML::Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "[auto_buff_debug_view] exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    std::cerr << "[auto_buff_debug_view] unknown exception" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}