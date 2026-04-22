#include <fmt/core.h>
#include <chrono>
#include <memory>
#include <optional>
#include <iostream>
#include <cmath>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include "io/ros2camera.hpp"
#include "io/simboard.hpp"

#include "tasks/auto_buff/buff_aimer.hpp"
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

static double clamp(double x, double lo, double hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// 关键修正：内部状态保持“连续角”，不要在这里 wrap
static double step_towards_angle_continuous(double current, double target, double max_step)
{
  double err = wrap_pi(target - current);
  double step = clamp(err, -max_step, max_step);
  return current + step;
}

static double step_towards_linear(double current, double target, double max_step)
{
  const double err = target - current;
  const double step = clamp(err, -max_step, max_step);
  return current + step;
}

void draw_cross(
  cv::Mat & img,
  const cv::Point2f & p,
  const cv::Scalar & color,
  int size = 8,
  int thickness = 2)
{
  if (p.x < 0 || p.y < 0) {
    return;
  }
  cv::line(img, cv::Point2f(p.x - size, p.y), cv::Point2f(p.x + size, p.y), color, thickness);
  cv::line(img, cv::Point2f(p.x, p.y - size), cv::Point2f(p.x, p.y + size), color, thickness);
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
    std::cout << "[sim_abs_smooth_cont] config_path = " << config_path << std::endl;

    auto yaml = YAML::LoadFile(config_path);
    const std::string sim_buff_mode =
      yaml["sim_buff_mode"] ? yaml["sim_buff_mode"].as<std::string>() : "big";
    const bool use_big_buff = (sim_buff_mode != "small");

    constexpr double kFutureDt = 0.03;
    constexpr int kValidWarmupFrames = 5;

    constexpr double kMaxAbsYawStep = 4.0 / 57.3;
    constexpr double kMaxAbsPitchStep = 2.0 / 57.3;

    constexpr double kPitchClampMin = -0.785;
    constexpr double kPitchClampMax = 0.785;
    constexpr bool kEnableShoot = false;

    tools::Plotter plotter;
    tools::Exiter exiter;

    std::cout << "[sim_abs_smooth_cont] create SimBoard" << std::endl;
    io::SimBoard simboard(config_path);
    std::cout << "[sim_abs_smooth_cont] SimBoard ok" << std::endl;

    std::cout << "[sim_abs_smooth_cont] create ROS2Camera" << std::endl;
    io::ROS2Camera camera(config_path);
    std::cout << "[sim_abs_smooth_cont] ROS2Camera ok" << std::endl;

    std::cout << "[sim_abs_smooth_cont] create Buff_Detector" << std::endl;
    auto_buff::Buff_Detector buff_detector(config_path);
    std::cout << "[sim_abs_smooth_cont] Buff_Detector ok" << std::endl;

    std::cout << "[sim_abs_smooth_cont] create Solver" << std::endl;
    auto_buff::Solver buff_solver(config_path);
    std::cout << "[sim_abs_smooth_cont] Solver ok" << std::endl;

    std::cout << "[sim_abs_smooth_cont] create Target" << std::endl;
    std::unique_ptr<auto_buff::Target> buff_target;
    if (use_big_buff) {
      buff_target = std::make_unique<auto_buff::BigTarget>();
    } else {
      buff_target = std::make_unique<auto_buff::SmallTarget>();
    }
    std::cout << "[sim_abs_smooth_cont] Target ok" << std::endl;

    std::cout << "[sim_abs_smooth_cont] create Aimer" << std::endl;
    auto_buff::Aimer buff_aimer(config_path);
    std::cout << "[sim_abs_smooth_cont] Aimer ok" << std::endl;

    cv::Mat img;
    auto t_begin = std::chrono::steady_clock::now();

    int valid_streak = 0;
    bool abs_cmd_initialized = false;
    double smoothed_abs_yaw = 0.0;    // 连续角
    double smoothed_abs_pitch = 0.0;

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

      if (!abs_cmd_initialized) {
        smoothed_abs_yaw = gimbal_yaw;
        smoothed_abs_pitch = clamp(gimbal_pitch, kPitchClampMin, kPitchClampMax);
        abs_cmd_initialized = true;
      }

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

      if (target_valid) {
        valid_streak++;
      } else {
        valid_streak = 0;
      }

      io::Command aimer_cmd{};
      aimer_cmd.control = false;
      aimer_cmd.shoot = false;
      aimer_cmd.yaw = 0.0;
      aimer_cmd.pitch = 0.0;
      if (target_valid) {
        auto ts_for_aim = timestamp;
        aimer_cmd = buff_aimer.aim(*buff_target, ts_for_aim, simboard.bullet_speed, true);
      }

      double dbg_calc_cmd_yaw = buff_aimer.dbg_calc_cmd_yaw();
      double dbg_calc_cmd_pitch = buff_aimer.dbg_calc_cmd_pitch();
      bool dbg_switch_fanblade = buff_aimer.dbg_switch_fanblade();
      int dbg_mistake_count = buff_aimer.dbg_mistake_count();

      cv::Point2f reproj_obs_blade(-1, -1);
      cv::Point2f reproj_pred_t0(-1, -1);
      cv::Point2f reproj_pred_t30(-1, -1);

      bool obs_blade_valid = false;
      bool pred_t0_valid = false;
      bool pred_t30_valid = false;

      double obs_blade_world_yaw = 0.0;
      double pred_t0_world_yaw = 0.0;
      double pred_t30_world_yaw = 0.0;
      double pred_t0_world_pitch = 0.0;
      double pred_t30_world_pitch = 0.0;
      double pred_t30_world_dis = 0.0;

      double target_phase_wrap = 0.0;
      double target_omega = 0.0;

      if (solved && rune_opt.has_value()) {
        const auto & p = rune_opt.value();
        obs_blade_valid = true;
        obs_blade_world_yaw = p.blade_ypd_in_world[0];
        reproj_obs_blade = buff_solver.reproject_world_point(p.blade_xyz_in_world);
      }

      if (!buff_target->is_unsolve()) {
        {
          Eigen::Vector3d pred_world = buff_target->predict_position(0.0);
          Eigen::Vector3d pred_ypd = tools::xyz2ypd(pred_world);
          pred_t0_valid = true;
          pred_t0_world_yaw = pred_ypd[0];
          pred_t0_world_pitch = pred_ypd[1];
          reproj_pred_t0 = buff_solver.reproject_world_point(pred_world);
        }

        {
          Eigen::Vector3d pred_world = buff_target->predict_position(kFutureDt);
          Eigen::Vector3d pred_ypd = tools::xyz2ypd(pred_world);
          pred_t30_valid = true;
          pred_t30_world_yaw = pred_ypd[0];
          pred_t30_world_pitch = pred_ypd[1];
          pred_t30_world_dis = pred_ypd[2];
          reproj_pred_t30 = buff_solver.reproject_world_point(pred_world);
        }

        Eigen::VectorXd x = buff_target->ekf_x();
        if (x.size() > 5) {
          target_phase_wrap = wrap_pi(x[5]);
        }
        if (x.size() > 6) {
          target_omega = x[6];
        }
      }

      const double image_center_x = static_cast<double>(img.cols) * 0.5;
      const double image_center_y = static_cast<double>(img.rows) * 0.5;

      double pred_t30_px = -1.0;
      double pred_t30_py = -1.0;
      double pixel_err_x = 0.0;
      double pixel_err_y = 0.0;

      if (pred_t30_valid && reproj_pred_t30.x >= 0.0 && reproj_pred_t30.y >= 0.0) {
        pred_t30_px = reproj_pred_t30.x;
        pred_t30_py = reproj_pred_t30.y;
        pixel_err_x = pred_t30_px - image_center_x;
        pixel_err_y = pred_t30_py - image_center_y;
      }

      bool warmup_ok = (valid_streak >= kValidWarmupFrames);

      double desired_abs_yaw = smoothed_abs_yaw;
      double desired_abs_pitch = smoothed_abs_pitch;

      double sim_send_yaw = 0.0;
      double sim_send_pitch = 0.0;
      bool sent_this_frame = false;
      bool released_this_frame = false;

      io::Command send_cmd{};
      send_cmd.control = false;
      send_cmd.shoot = false;
      send_cmd.yaw = 0.0;
      send_cmd.pitch = 0.0;

      if (target_valid && pred_t30_valid && warmup_ok) {
        desired_abs_yaw = pred_t30_world_yaw;
        desired_abs_pitch = clamp(pred_t30_world_pitch, kPitchClampMin, kPitchClampMax);

        smoothed_abs_yaw =
          step_towards_angle_continuous(smoothed_abs_yaw, desired_abs_yaw, kMaxAbsYawStep);
        smoothed_abs_pitch =
          step_towards_linear(smoothed_abs_pitch, desired_abs_pitch, kMaxAbsPitchStep);
        smoothed_abs_pitch = clamp(smoothed_abs_pitch, kPitchClampMin, kPitchClampMax);

        sim_send_yaw = smoothed_abs_yaw;
        sim_send_pitch = smoothed_abs_pitch;

        send_cmd.control = true;
        send_cmd.shoot = kEnableShoot;
        send_cmd.yaw = sim_send_yaw;
        send_cmd.pitch = sim_send_pitch;
        sent_this_frame = true;
      } else {
        send_cmd.control = false;
        send_cmd.shoot = false;
        send_cmd.yaw = 0.0;
        send_cmd.pitch = 0.0;
        released_this_frame = true;

        smoothed_abs_yaw = gimbal_yaw;  // 连续角状态贴回当前
        smoothed_abs_pitch = clamp(gimbal_pitch, kPitchClampMin, kPitchClampMax);
      }

      simboard.send(send_cmd);

      draw_labeled_cross(
        vis,
        cv::Point2f(static_cast<float>(image_center_x), static_cast<float>(image_center_y)),
        cv::Scalar(255, 255, 255),
        "img_center",
        14,
        2);
      draw_labeled_cross(vis, reproj_obs_blade, cv::Scalar(0, 255, 0), "obs_blade", 14, 2);
      draw_labeled_cross(vis, reproj_pred_t0, cv::Scalar(255, 255, 0), "pred_t0", 14, 2);
      draw_labeled_cross(vis, reproj_pred_t30, cv::Scalar(255, 0, 255), "pred_t30", 14, 2);

      if (reproj_pred_t0.x >= 0 && reproj_pred_t30.x >= 0) {
        cv::line(vis, reproj_pred_t0, reproj_pred_t30, cv::Scalar(255, 0, 255), 2);
      }

      if (reproj_pred_t30.x >= 0 && reproj_pred_t30.y >= 0) {
        cv::line(
          vis,
          cv::Point(static_cast<int>(image_center_x), static_cast<int>(image_center_y)),
          cv::Point(static_cast<int>(reproj_pred_t30.x), static_cast<int>(reproj_pred_t30.y)),
          cv::Scalar(0, 255, 255),
          2);
      }

      tools::draw_text(
        vis,
        fmt::format("MODE: {}", use_big_buff ? "ABS_WORLD_CONT_BIG" : "ABS_WORLD_CONT_SMALL"),
        {10, 30}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("det:{} solve:{} target:{} streak:{}",
                    rune_opt.has_value() ? 1 : 0,
                    solved ? 1 : 0,
                    target_valid ? 1 : 0,
                    valid_streak),
        {10, 60}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("GY:{:.2f} GP:{:.2f}",
                    gimbal_yaw_wrap * 57.3,
                    gimbal_pitch_wrap * 57.3),
        {10, 90}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("OBS_Y:{:.2f} P0_Y:{:.2f} P30_Y:{:.2f}",
                    obs_blade_world_yaw * 57.3,
                    pred_t0_world_yaw * 57.3,
                    pred_t30_world_yaw * 57.3),
        {10, 120}, {255, 255, 0});

      tools::draw_text(
        vis,
        fmt::format("pred_t30_px:{:.1f} center_x:{:.1f}",
                    pred_t30_px, image_center_x),
        {10, 150}, {255, 100, 255});

      tools::draw_text(
        vis,
        fmt::format("pixel_err_x:{:+.1f} pixel_err_y:{:+.1f}",
                    pixel_err_x, pixel_err_y),
        {10, 180}, {255, 220, 120});

      tools::draw_text(
        vis,
        fmt::format("DES_Y_WRAP:{:.2f} DES_P:{:.2f}",
                    wrap_pi(desired_abs_yaw) * 57.3,
                    desired_abs_pitch * 57.3),
        {10, 210}, {255, 100, 255});

      tools::draw_text(
        vis,
        fmt::format("SM_Y_RAW:{:.2f} SM_Y_WRAP:{:.2f}",
                    smoothed_abs_yaw * 57.3,
                    wrap_pi(smoothed_abs_yaw) * 57.3),
        {10, 240}, {0, 200, 255});

      tools::draw_text(
        vis,
        fmt::format("SIM_SEND_Y:{:.2f} SIM_SEND_P:{:.2f}",
                    sim_send_yaw * 57.3,
                    sim_send_pitch * 57.3),
        {10, 270}, {0, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("ctrl:{} warmup_ok:{} shoot:{}",
                    static_cast<int>(send_cmd.control),
                    static_cast<int>(warmup_ok),
                    static_cast<int>(send_cmd.shoot)),
        {10, 300}, {255, 150, 255});

      tools::draw_text(
        vis,
        fmt::format("phase_wrap:{:.2f} omega:{:.2f}",
                    target_phase_wrap * 57.3,
                    target_omega * 57.3),
        {10, 330}, {200, 255, 100});

      tools::draw_text(
        vis,
        "ABSOLUTE ANGLE CONTINUOUS SMOOTH SEND / RAW+WRAP DEBUG",
        {10, 360}, {255, 255, 255});

      {
        nlohmann::json data;
        double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t_begin).count();

        data["t"] = t;
        data["mode"] = use_big_buff ? "abs_world_cont_big" : "abs_world_cont_small";

        data["detected"] = rune_opt.has_value() ? 1 : 0;
        data["solved"] = solved ? 1 : 0;
        data["target_valid"] = target_valid ? 1 : 0;
        data["valid_streak"] = valid_streak;
        data["warmup_ok"] = warmup_ok ? 1 : 0;

        data["gimbal_yaw"] = gimbal_yaw * 57.3;
        data["gimbal_pitch"] = gimbal_pitch * 57.3;
        data["gimbal_yaw_wrap"] = gimbal_yaw_wrap * 57.3;
        data["gimbal_pitch_wrap"] = gimbal_pitch_wrap * 57.3;

        data["obs_blade_valid"] = obs_blade_valid ? 1 : 0;
        data["obs_blade_world_yaw"] = obs_blade_valid ? (obs_blade_world_yaw * 57.3) : 0.0;

        data["pred_t0_valid"] = pred_t0_valid ? 1 : 0;
        data["pred_t0_world_yaw"] = pred_t0_valid ? (pred_t0_world_yaw * 57.3) : 0.0;
        data["pred_t0_world_pitch"] = pred_t0_valid ? (pred_t0_world_pitch * 57.3) : 0.0;

        data["future_30ms_valid"] = pred_t30_valid ? 1 : 0;
        data["future_30ms_world_yaw"] = pred_t30_valid ? (pred_t30_world_yaw * 57.3) : 0.0;
        data["future_30ms_world_pitch"] = pred_t30_valid ? (pred_t30_world_pitch * 57.3) : 0.0;
        data["future_30ms_world_dis"] = pred_t30_valid ? pred_t30_world_dis : 0.0;

        data["image_center_x"] = image_center_x;
        data["image_center_y"] = image_center_y;
        data["pred_t30_px"] = pred_t30_px;
        data["pred_t30_py"] = pred_t30_py;
        data["pixel_err_x"] = pixel_err_x;
        data["pixel_err_y"] = pixel_err_y;

        data["desired_abs_yaw_wrap"] = wrap_pi(desired_abs_yaw) * 57.3;
        data["desired_abs_pitch"] = desired_abs_pitch * 57.3;

        data["smoothed_abs_yaw_raw"] = smoothed_abs_yaw * 57.3;
        data["smoothed_abs_yaw_wrap"] = wrap_pi(smoothed_abs_yaw) * 57.3;
        data["smoothed_abs_pitch"] = smoothed_abs_pitch * 57.3;

        data["sim_send_yaw"] = sim_send_yaw * 57.3;
        data["sim_send_pitch"] = sim_send_pitch * 57.3;
        data["cmd_control"] = send_cmd.control ? 1 : 0;
        data["cmd_shoot"] = send_cmd.shoot ? 1 : 0;
        data["sent_this_frame"] = sent_this_frame ? 1 : 0;
        data["released_this_frame"] = released_this_frame ? 1 : 0;

        data["target_phase_wrap"] = target_phase_wrap * 57.3;
        data["target_omega"] = target_omega * 57.3;

        data["aimer_control"] = aimer_cmd.control ? 1 : 0;
        data["aimer_cmd_yaw"] = aimer_cmd.yaw * 57.3;
        data["aimer_cmd_pitch"] = aimer_cmd.pitch * 57.3;
        data["dbg_calc_cmd_yaw"] = dbg_calc_cmd_yaw * 57.3;
        data["dbg_calc_cmd_pitch"] = dbg_calc_cmd_pitch * 57.3;
        data["dbg_switch_fanblade"] = dbg_switch_fanblade ? 1 : 0;
        data["dbg_mistake_count"] = dbg_mistake_count;

        data["bullet_speed"] = simboard.bullet_speed;

        plotter.plot(data);
      }

      cv::Mat vis_show;
      cv::resize(vis, vis_show, {}, 0.5, 0.5, cv::INTER_AREA);
      cv::imshow("buff_sim_abs_world_cont", vis_show);

      int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q') {
        break;
      }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
  } catch (const YAML::BadConversion & e) {
    std::cerr << "[auto_buff_sim_abs_world_cont] YAML::BadConversion: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (const YAML::Exception & e) {
    std::cerr << "[auto_buff_sim_abs_world_cont] YAML::Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception & e) {
    std::cerr << "[auto_buff_sim_abs_world_cont] exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    std::cerr << "[auto_buff_sim_abs_world_cont] unknown exception" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}