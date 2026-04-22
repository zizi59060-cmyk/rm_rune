#include <fmt/core.h>
#include <chrono>
#include <memory>
#include <optional>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include "io/ros2camera.hpp"
#include "io/simboard.hpp"

#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sim_buff.yaml | yaml配置文件路径}";

namespace
{

static double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void draw_cross(
  cv::Mat & img,
  const cv::Point2f & p,
  const cv::Scalar & color,
  int size = 8,
  int thickness = 2)
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

void draw_target_points(cv::Mat & img, const std::vector<cv::Point2f> & pts)
{
  if (pts.size() < 5) {
    return;
  }

  draw_circle_text(img, pts[0], cv::Scalar(255, 255, 255), "r_center", 5);

  for (int i = 1; i <= 4; ++i) {
    draw_circle_text(img, pts[i], cv::Scalar(0, 255, 255), fmt::format("p{}", i), 4);
  }

  for (int i = 1; i <= 4; ++i) {
    cv::line(img, pts[i], pts[(i % 4) + 1], cv::Scalar(0, 255, 0), 2);
  }

  cv::Point2f c(0, 0);
  for (int i = 1; i <= 4; ++i) {
    c += pts[i];
  }
  c *= 0.25f;

  draw_cross(img, c, cv::Scalar(255, 0, 255), 10, 2);
  cv::putText(
    img,
    "target_center",
    cv::Point(static_cast<int>(c.x + 10), static_cast<int>(c.y + 12)),
    cv::FONT_HERSHEY_SIMPLEX,
    0.6,
    cv::Scalar(255, 0, 255),
    2,
    cv::LINE_AA);
}

void draw_labeled_cross(
  cv::Mat & img,
  const cv::Point2f & p,
  const cv::Scalar & color,
  const std::string & label,
  int size = 12,
  int thickness = 2)
{
  if (p.x < 0 || p.y < 0) {
    return;
  }

  draw_cross(img, p, color, size, thickness);
  cv::putText(
    img,
    label,
    cv::Point(static_cast<int>(p.x + 10), static_cast<int>(p.y - 8)),
    cv::FONT_HERSHEY_SIMPLEX,
    0.6,
    color,
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
    std::cout << "[debug_view] config_path = " << config_path << std::endl;

    auto yaml = YAML::LoadFile(config_path);
    const std::string sim_buff_mode =
      yaml["sim_buff_mode"] ? yaml["sim_buff_mode"].as<std::string>() : "big";
    const bool use_big_buff = (sim_buff_mode != "small");

    constexpr double kFutureDt1 = 0.03;
    constexpr double kFutureDt2 = 0.06;

    tools::Plotter plotter;
    tools::Exiter exiter;

    std::cout << "[debug_view] create SimBoard" << std::endl;
    io::SimBoard simboard(config_path);
    std::cout << "[debug_view] SimBoard ok" << std::endl;

    std::cout << "[debug_view] create ROS2Camera" << std::endl;
    io::ROS2Camera camera(config_path);
    std::cout << "[debug_view] ROS2Camera ok" << std::endl;

    std::cout << "[debug_view] create Buff_Detector" << std::endl;
    auto_buff::Buff_Detector buff_detector(config_path);
    std::cout << "[debug_view] Buff_Detector ok" << std::endl;

    std::cout << "[debug_view] create Solver" << std::endl;
    auto_buff::Solver buff_solver(config_path);
    std::cout << "[debug_view] Solver ok" << std::endl;

    std::cout << "[debug_view] create Target" << std::endl;
    std::unique_ptr<auto_buff::Target> buff_target;
    if (use_big_buff) {
      buff_target = std::make_unique<auto_buff::BigTarget>();
    } else {
      buff_target = std::make_unique<auto_buff::SmallTarget>();
    }
    std::cout << "[debug_view] Target ok" << std::endl;

    cv::Mat img;
    auto t_begin = std::chrono::steady_clock::now();

    while (!exiter.exit() && rclcpp::ok()) {
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) {
        cv::waitKey(1);
        continue;
      }

      Eigen::Quaterniond q = simboard.imu_at(timestamp - std::chrono::milliseconds(1));
      buff_solver.set_R_gimbal2world(q);

      Eigen::Matrix3d R_camera2gimbal;
      Eigen::Vector3d t_camera2gimbal;
      if (simboard.camera2gimbal(R_camera2gimbal, t_camera2gimbal)) {
        buff_solver.set_camera2gimbal(R_camera2gimbal, t_camera2gimbal);
      }

      cv::Mat vis = img.clone();

      Eigen::Vector3d ypr = tools::eulers(buff_solver.R_gimbal2world(), 2, 1, 0);
      double gimbal_yaw = ypr[0];
      double gimbal_pitch = ypr[1];
      double gimbal_yaw_wrap = wrap_pi(gimbal_yaw);
      double gimbal_pitch_wrap = wrap_pi(gimbal_pitch);

      std::optional<auto_buff::PowerRune> rune_opt = buff_detector.detect(vis);

      bool solved = false;
      bool target_valid = false;

      if (rune_opt.has_value()) {
        auto & rune = rune_opt.value();

        buff_solver.solve(rune_opt);

        if (!rune.is_unsolve()) {
          solved = true;
          buff_target->get_target(rune_opt, timestamp);
        } else {
          std::optional<auto_buff::PowerRune> empty;
          buff_target->get_target(empty, timestamp);
        }

        if (!buff_target->is_unsolve()) {
          target_valid = true;
        }
      } else {
        std::optional<auto_buff::PowerRune> empty;
        buff_target->get_target(empty, timestamp);
      }

      double blade_yaw = 0.0;
      double blade_pitch = 0.0;
      double blade_dis = 0.0;
      bool blade_valid = false;
      cv::Point2f reproj_center(-1, -1);
      cv::Point2f reproj_blade(-1, -1);

      if (solved && rune_opt.has_value()) {
        const auto & p = rune_opt.value();

        blade_yaw = p.blade_ypd_in_world[0];
        blade_pitch = p.blade_ypd_in_world[1];
        blade_dis = p.blade_ypd_in_world[2];
        blade_valid = true;

        reproj_center = buff_solver.reproject_world_point(p.xyz_in_world);
        reproj_blade = buff_solver.reproject_world_point(p.blade_xyz_in_world);
      }

      double pred_blade_yaw = 0.0;
      double pred_blade_pitch = 0.0;
      double pred_blade_dis = 0.0;
      bool pred_blade_valid = false;
      cv::Point2f reproj_pred_blade(-1, -1);

      double fut1_blade_yaw = 0.0;
      double fut1_blade_pitch = 0.0;
      double fut1_blade_dis = 0.0;
      bool fut1_blade_valid = false;
      cv::Point2f reproj_fut1_blade(-1, -1);

      double fut2_blade_yaw = 0.0;
      double fut2_blade_pitch = 0.0;
      double fut2_blade_dis = 0.0;
      bool fut2_blade_valid = false;
      cv::Point2f reproj_fut2_blade(-1, -1);

      if (!buff_target->is_unsolve()) {
        {
          Eigen::Vector3d pred_blade_world = buff_target->predict_position(0.0);
          Eigen::Vector3d pred_blade_ypd = tools::xyz2ypd(pred_blade_world);

          pred_blade_yaw = pred_blade_ypd[0];
          pred_blade_pitch = pred_blade_ypd[1];
          pred_blade_dis = pred_blade_ypd[2];
          pred_blade_valid = true;

          reproj_pred_blade = buff_solver.reproject_world_point(pred_blade_world);
        }

        {
          Eigen::Vector3d fut1_blade_world = buff_target->predict_position(kFutureDt1);
          Eigen::Vector3d fut1_blade_ypd = tools::xyz2ypd(fut1_blade_world);

          fut1_blade_yaw = fut1_blade_ypd[0];
          fut1_blade_pitch = fut1_blade_ypd[1];
          fut1_blade_dis = fut1_blade_ypd[2];
          fut1_blade_valid = true;

          reproj_fut1_blade = buff_solver.reproject_world_point(fut1_blade_world);
        }

        {
          Eigen::Vector3d fut2_blade_world = buff_target->predict_position(kFutureDt2);
          Eigen::Vector3d fut2_blade_ypd = tools::xyz2ypd(fut2_blade_world);

          fut2_blade_yaw = fut2_blade_ypd[0];
          fut2_blade_pitch = fut2_blade_ypd[1];
          fut2_blade_dis = fut2_blade_ypd[2];
          fut2_blade_valid = true;

          reproj_fut2_blade = buff_solver.reproject_world_point(fut2_blade_world);
        }
      }

      double target_phase_raw = 0.0;
      double target_phase_wrap = 0.0;
      double target_omega = 0.0;

      if (!buff_target->is_unsolve()) {
        Eigen::VectorXd x = buff_target->ekf_x();
        if (x.size() > 5) {
          target_phase_raw = x[5];
          target_phase_wrap = wrap_pi(x[5]);
        }
        if (x.size() > 6) {
          target_omega = x[6];
        }
      }

      if (rune_opt.has_value() && !rune_opt->fanblades.empty()) {
        const auto & tgt = rune_opt->target();
        draw_target_points(vis, tgt.points);
      }

      draw_labeled_cross(vis, reproj_center, cv::Scalar(255, 0, 0), "reproj_center", 14, 2);
      draw_labeled_cross(vis, reproj_blade, cv::Scalar(0, 255, 0), "obs_blade", 14, 2);
      draw_labeled_cross(vis, reproj_pred_blade, cv::Scalar(255, 255, 0), "pred_t0", 14, 2);
      draw_labeled_cross(vis, reproj_fut1_blade, cv::Scalar(255, 0, 255), "pred_t30ms", 14, 2);
      draw_labeled_cross(vis, reproj_fut2_blade, cv::Scalar(0, 255, 255), "pred_t60ms", 14, 2);

      if (reproj_pred_blade.x >= 0 && reproj_fut1_blade.x >= 0) {
        cv::line(vis, reproj_pred_blade, reproj_fut1_blade, cv::Scalar(255, 0, 255), 2);
      }
      if (reproj_fut1_blade.x >= 0 && reproj_fut2_blade.x >= 0) {
        cv::line(vis, reproj_fut1_blade, reproj_fut2_blade, cv::Scalar(0, 255, 255), 2);
      }

      {
        nlohmann::json data;
        double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t_begin).count();

        data["t"] = t;
        data["mode"] = use_big_buff ? "buff_debug_big" : "buff_debug_small";

        data["gimbal_yaw"] = gimbal_yaw * 57.3;
        data["gimbal_pitch"] = gimbal_pitch * 57.3;
        data["gimbal_yaw_wrap"] = gimbal_yaw_wrap * 57.3;
        data["gimbal_pitch_wrap"] = gimbal_pitch_wrap * 57.3;

        data["detected"] = rune_opt.has_value() ? 1 : 0;
        data["solved"] = solved ? 1 : 0;
        data["target_valid"] = target_valid ? 1 : 0;

        data["bullet_speed"] = simboard.bullet_speed;

        data["blade_valid"] = blade_valid ? 1 : 0;
        data["blade_yaw"] = blade_valid ? (blade_yaw * 57.3) : 0.0;
        data["blade_pitch"] = blade_valid ? (blade_pitch * 57.3) : 0.0;
        data["blade_dis"] = blade_valid ? blade_dis : 0.0;

        data["pred_blade_valid"] = pred_blade_valid ? 1 : 0;
        data["pred_blade_yaw"] = pred_blade_valid ? (pred_blade_yaw * 57.3) : 0.0;
        data["pred_blade_pitch"] = pred_blade_valid ? (pred_blade_pitch * 57.3) : 0.0;
        data["pred_blade_dis"] = pred_blade_valid ? pred_blade_dis : 0.0;

        data["future_30ms_valid"] = fut1_blade_valid ? 1 : 0;
        data["future_30ms_yaw"] = fut1_blade_valid ? (fut1_blade_yaw * 57.3) : 0.0;
        data["future_30ms_pitch"] = fut1_blade_valid ? (fut1_blade_pitch * 57.3) : 0.0;
        data["future_30ms_dis"] = fut1_blade_valid ? fut1_blade_dis : 0.0;

        data["future_60ms_valid"] = fut2_blade_valid ? 1 : 0;
        data["future_60ms_yaw"] = fut2_blade_valid ? (fut2_blade_yaw * 57.3) : 0.0;
        data["future_60ms_pitch"] = fut2_blade_valid ? (fut2_blade_pitch * 57.3) : 0.0;
        data["future_60ms_dis"] = fut2_blade_valid ? fut2_blade_dis : 0.0;

        data["target_phase_raw"] = target_phase_raw * 57.3;
        data["target_phase_wrap"] = target_phase_wrap * 57.3;
        data["target_omega"] = target_omega * 57.3;

        if (rune_opt.has_value()) {
          const auto & p = rune_opt.value();

          data["buff_R_yaw"] = p.ypd_in_world[0] * 57.3;
          data["buff_R_pitch"] = p.ypd_in_world[1] * 57.3;
          data["buff_R_dis"] = p.ypd_in_world[2];

          data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
          data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
          data["buff_phase"] = p.ypr_in_world[2] * 57.3;
          data["buff_phase_wrap"] = wrap_pi(p.ypr_in_world[2]) * 57.3;
        } else {
          data["buff_phase"] = 0.0;
          data["buff_phase_wrap"] = 0.0;
        }

        plotter.plot(data);
      }

      tools::draw_text(
        vis,
        fmt::format("MODE: {}", use_big_buff ? "DEBUG_BIG" : "DEBUG_SMALL"),
        {10, 30}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("det:{} solve:{} target:{}",
                    rune_opt.has_value() ? 1 : 0,
                    solved ? 1 : 0,
                    target_valid ? 1 : 0),
        {10, 60}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("GY:{:.2f} GP:{:.2f}",
                    gimbal_yaw_wrap * 57.3,
                    gimbal_pitch_wrap * 57.3),
        {10, 90}, {255, 255, 255});

      if (blade_valid) {
        tools::draw_text(
          vis,
          fmt::format("OBS blade_y:{:.2f} blade_p:{:.2f} d:{:.2f}",
                      blade_yaw * 57.3,
                      blade_pitch * 57.3,
                      blade_dis),
          {10, 120}, {0, 255, 0});
      }

      if (pred_blade_valid) {
        tools::draw_text(
          vis,
          fmt::format("PRED0 y:{:.2f} p:{:.2f} d:{:.2f}",
                      pred_blade_yaw * 57.3,
                      pred_blade_pitch * 57.3,
                      pred_blade_dis),
          {10, 150}, {255, 255, 0});
      }

      if (fut1_blade_valid) {
        tools::draw_text(
          vis,
          fmt::format("PRED30 y:{:.2f} p:{:.2f} d:{:.2f}",
                      fut1_blade_yaw * 57.3,
                      fut1_blade_pitch * 57.3,
                      fut1_blade_dis),
          {10, 180}, {255, 0, 255});
      }

      if (fut2_blade_valid) {
        tools::draw_text(
          vis,
          fmt::format("PRED60 y:{:.2f} p:{:.2f} d:{:.2f}",
                      fut2_blade_yaw * 57.3,
                      fut2_blade_pitch * 57.3,
                      fut2_blade_dis),
          {10, 210}, {0, 255, 255});
      }

      tools::draw_text(
        vis,
        fmt::format("phase_raw:{:.2f} phase_wrap:{:.2f} omega:{:.2f}",
                    target_phase_raw * 57.3,
                    target_phase_wrap * 57.3,
                    target_omega * 57.3),
        {10, 240}, {255, 150, 255});

      if (rune_opt.has_value()) {
        const auto & p = rune_opt.value();
        tools::draw_text(
          vis,
          fmt::format("buffR_y:{:.2f} buff_y:{:.2f} phase:{:.2f}",
                      p.ypd_in_world[0] * 57.3,
                      p.ypr_in_world[0] * 57.3,
                      p.ypr_in_world[2] * 57.3),
          {10, 270}, {255, 150, 100});
      }

      tools::draw_text(
        vis,
        "NO GIMBAL CONTROL / VIEW ONLY",
        {10, 300}, {0, 200, 255});

      cv::Mat vis_show;
      cv::resize(vis, vis_show, {}, 0.5, 0.5, cv::INTER_AREA);
      cv::imshow("auto_buff_debug_view", vis_show);

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