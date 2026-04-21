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

static double step_towards_angle(double current, double target, double max_step)
{
  double err = wrap_pi(target - current);
  double step = clamp(err, -max_step, max_step);
  return wrap_pi(current + step);
}

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
    std::cout << "[dbg] config_path = " << config_path << std::endl;

    auto yaml = YAML::LoadFile(config_path);
    const std::string sim_buff_mode =
      yaml["sim_buff_mode"] ? yaml["sim_buff_mode"].as<std::string>() : "big";
    const bool use_big_buff = (sim_buff_mode != "small");

    // =========================
    // 验证开关
    // =========================
    const bool use_absolute_yaw_cmd = false;
    const bool use_relative_yaw_cmd = true;
    const bool use_absolute_pitch_cmd = true;

    const double kMaxYawStep = 4.0 / 57.3;
    const double kMaxPitchStep = 2.0 / 57.3;
    const double kRelativeYawCmdLimit = 4.0 / 57.3;

    const bool sim_invert_yaw = false;
    const bool sim_invert_pitch = true;

    tools::Plotter plotter;
    tools::Exiter exiter;

    std::cout << "[dbg] create SimBoard" << std::endl;
    io::SimBoard simboard(config_path);
    std::cout << "[dbg] SimBoard ok" << std::endl;

    std::cout << "[dbg] create ROS2Camera" << std::endl;
    io::ROS2Camera camera(config_path);
    std::cout << "[dbg] ROS2Camera ok" << std::endl;

    std::cout << "[dbg] create Buff_Detector" << std::endl;
    auto_buff::Buff_Detector buff_detector(config_path);
    std::cout << "[dbg] Buff_Detector ok" << std::endl;

    std::cout << "[dbg] create Solver" << std::endl;
    auto_buff::Solver buff_solver(config_path);
    std::cout << "[dbg] Solver ok" << std::endl;

    std::cout << "[dbg] create Target" << std::endl;
    std::unique_ptr<auto_buff::Target> buff_target;
    if (use_big_buff) {
      buff_target = std::make_unique<auto_buff::BigTarget>();
    } else {
      buff_target = std::make_unique<auto_buff::SmallTarget>();
    }
    std::cout << "[dbg] Target ok" << std::endl;

    std::cout << "[dbg] create Aimer" << std::endl;
    auto_buff::Aimer buff_aimer(config_path);
    std::cout << "[dbg] Aimer ok" << std::endl;

    cv::Mat img;

    double last_sent_yaw = 0.0;
    double last_sent_pitch = 0.0;
    bool sent_initialized = false;

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

      if (!sent_initialized) {
        last_sent_yaw = gimbal_yaw;
        last_sent_pitch = -gimbal_pitch;
        sent_initialized = true;
      }

      std::optional<auto_buff::PowerRune> rune_opt = buff_detector.detect(vis);

      io::Command command{};
      command.control = false;
      command.shoot = false;
      command.yaw = 0.0;
      command.pitch = 0.0;

      bool target_valid = false;

      if (rune_opt.has_value()) {
        auto & rune = rune_opt.value();

        buff_solver.solve(rune_opt);

        if (!rune.is_unsolve()) {
          buff_target->get_target(rune_opt, timestamp);
        } else {
          std::optional<auto_buff::PowerRune> empty;
          buff_target->get_target(empty, timestamp);
        }

        auto ts_for_aim = timestamp;
        command = buff_aimer.aim(*buff_target, ts_for_aim, simboard.bullet_speed, true);

        if (!buff_target->is_unsolve()) {
          target_valid = true;
        }
      } else {
        std::optional<auto_buff::PowerRune> empty;
        buff_target->get_target(empty, timestamp);
      }

      double calc_cmd_yaw = buff_aimer.dbg_calc_cmd_yaw();
      double calc_cmd_pitch = buff_aimer.dbg_calc_cmd_pitch();
      double delta_cmd_yaw = buff_aimer.dbg_delta_cmd_yaw();
      double delta_cmd_pitch = buff_aimer.dbg_delta_cmd_pitch();
      bool switch_fanblade = buff_aimer.dbg_switch_fanblade();
      int mistake_count = buff_aimer.dbg_mistake_count();

      double calc_cmd_yaw_wrap = wrap_pi(calc_cmd_yaw);
      double calc_cmd_pitch_wrap = wrap_pi(calc_cmd_pitch);
      double gimbal_yaw_wrap = wrap_pi(gimbal_yaw);
      double gimbal_pitch_wrap = wrap_pi(gimbal_pitch);

      bool sent_this_frame = false;
      bool released_this_frame = false;
      double sim_send_yaw = 0.0;
      double sim_send_pitch = 0.0;
      double yaw_err = 0.0;
      double yaw_step = 0.0;

      io::Command send_cmd{};
      send_cmd.control = false;
      send_cmd.shoot = false;
      send_cmd.yaw = 0.0;
      send_cmd.pitch = 0.0;

      if (target_valid && command.control) {
        last_sent_yaw = step_towards_angle(last_sent_yaw, calc_cmd_yaw, kMaxYawStep);
        last_sent_pitch = step_towards_angle(last_sent_pitch, calc_cmd_pitch, kMaxPitchStep);

        yaw_err = wrap_pi(last_sent_yaw - gimbal_yaw);
        yaw_step = clamp(yaw_err, -kRelativeYawCmdLimit, kRelativeYawCmdLimit);

        if (use_absolute_yaw_cmd) {
          sim_send_yaw = last_sent_yaw;
        } else if (use_relative_yaw_cmd) {
          sim_send_yaw = yaw_step;
        } else {
          sim_send_yaw = 0.0;
        }

        if (use_absolute_pitch_cmd) {
          sim_send_pitch = last_sent_pitch;
        } else {
          sim_send_pitch = wrap_pi(last_sent_pitch - (-gimbal_pitch));
        }

        if (sim_invert_yaw) {
          sim_send_yaw = wrap_pi(-sim_send_yaw);
        }
        if (sim_invert_pitch) {
          sim_send_pitch = wrap_pi(-sim_send_pitch);
        }

        send_cmd.control = true;
        send_cmd.shoot = command.shoot;
        send_cmd.yaw = sim_send_yaw;
        send_cmd.pitch = sim_send_pitch;

        sent_this_frame = true;
      } else {
        send_cmd.control = false;
        send_cmd.shoot = false;
        send_cmd.yaw = 0.0;
        send_cmd.pitch = 0.0;
        released_this_frame = true;
      }

      simboard.send(send_cmd);

      double sent_yaw_wrap = wrap_pi(last_sent_yaw);
      double sent_pitch_wrap = wrap_pi(last_sent_pitch);

      double err_yaw_wrap = wrap_pi(last_sent_yaw - gimbal_yaw);
      double err_pitch_wrap = wrap_pi(last_sent_pitch - gimbal_pitch);

      double target_phase = 0.0;
      double target_omega = 0.0;
      if (!buff_target->is_unsolve()) {
        Eigen::VectorXd x = buff_target->ekf_x();
        if (x.size() > 5) {
          target_phase = x[5];
        }
        if (x.size() > 6) {
          target_omega = x[6];
        }
      }

      // ===== 新增：预测扇叶点调试量 =====
      double pred_blade_yaw = 0.0;
      double pred_blade_pitch = 0.0;
      double pred_blade_dis = 0.0;
      bool pred_blade_valid = false;

      if (!buff_target->is_unsolve()) {
        Eigen::Vector3d pred_blade_world = buff_target->predict_position(0.0);
        Eigen::Vector3d pred_blade_ypd = tools::xyz2ypd(pred_blade_world);
        pred_blade_yaw = pred_blade_ypd[0];
        pred_blade_pitch = pred_blade_ypd[1];
        pred_blade_dis = pred_blade_ypd[2];
        pred_blade_valid = true;
      }

      {
        nlohmann::json data;
        double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t_begin).count();

        data["t"] = t;
        data["mode"] = use_big_buff ? "buff_sim_big" : "buff_sim_small";

        data["gimbal_yaw"] = gimbal_yaw * 57.3;
        data["gimbal_pitch"] = gimbal_pitch * 57.3;
        data["gimbal_yaw_wrap"] = gimbal_yaw_wrap * 57.3;
        data["gimbal_pitch_wrap"] = gimbal_pitch_wrap * 57.3;

        data["calc_cmd_yaw"] = calc_cmd_yaw * 57.3;
        data["calc_cmd_pitch"] = calc_cmd_pitch * 57.3;
        data["calc_cmd_yaw_wrap"] = calc_cmd_yaw_wrap * 57.3;
        data["calc_cmd_pitch_wrap"] = calc_cmd_pitch_wrap * 57.3;

        data["sent_yaw"] = last_sent_yaw * 57.3;
        data["sent_pitch"] = last_sent_pitch * 57.3;
        data["sent_yaw_wrap"] = sent_yaw_wrap * 57.3;
        data["sent_pitch_wrap"] = sent_pitch_wrap * 57.3;

        data["sim_send_yaw"] = sim_send_yaw * 57.3;
        data["sim_send_pitch"] = sim_send_pitch * 57.3;
        data["sim_send_yaw_wrap"] = wrap_pi(sim_send_yaw) * 57.3;
        data["sim_send_pitch_wrap"] = wrap_pi(sim_send_pitch) * 57.3;

        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["cmd_yaw_wrap"] = wrap_pi(command.yaw) * 57.3;
        data["cmd_pitch_wrap"] = wrap_pi(command.pitch) * 57.3;

        data["aimer_control"] = command.control ? 1 : 0;
        data["yaw_err"] = yaw_err * 57.3;
        data["yaw_step"] = yaw_step * 57.3;

        data["delta_cmd_yaw"] = delta_cmd_yaw * 57.3;
        data["delta_cmd_pitch"] = delta_cmd_pitch * 57.3;
        data["switch_fanblade"] = switch_fanblade ? 1 : 0;
        data["mistake_count"] = mistake_count;

        data["target_phase"] = target_phase * 57.3;
        data["target_omega"] = target_omega * 57.3;

        data["err_yaw_wrap"] = target_valid ? (err_yaw_wrap * 57.3) : 0.0;
        data["err_pitch_wrap"] = target_valid ? (err_pitch_wrap * 57.3) : 0.0;

        data["shoot"] = send_cmd.shoot ? 1 : 0;
        data["control"] = send_cmd.control ? 1 : 0;
        data["target_valid"] = target_valid ? 1 : 0;
        data["sent_this_frame"] = sent_this_frame ? 1 : 0;
        data["released_this_frame"] = released_this_frame ? 1 : 0;
        data["bullet_speed"] = simboard.bullet_speed;

        data["use_absolute_yaw_cmd"] = use_absolute_yaw_cmd ? 1 : 0;
        data["use_relative_yaw_cmd"] = use_relative_yaw_cmd ? 1 : 0;
        data["sim_invert_yaw"] = sim_invert_yaw ? 1 : 0;
        data["sim_invert_pitch"] = sim_invert_pitch ? 1 : 0;

        data["pred_blade_valid"] = pred_blade_valid ? 1 : 0;
        data["pred_blade_yaw"] = pred_blade_valid ? (pred_blade_yaw * 57.3) : 0.0;
        data["pred_blade_pitch"] = pred_blade_valid ? (pred_blade_pitch * 57.3) : 0.0;
        data["pred_blade_dis"] = pred_blade_valid ? pred_blade_dis : 0.0;

        if (rune_opt.has_value()) {
          const auto & p = rune_opt.value();
          data["detected"] = 1;

          data["buff_R_yaw"] = p.ypd_in_world[0] * 57.3;
          data["buff_R_pitch"] = p.ypd_in_world[1] * 57.3;
          data["buff_R_dis"] = p.ypd_in_world[2];

          data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
          data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
          data["buff_phase"] = p.ypr_in_world[2] * 57.3;

          data["blade_yaw"] = p.blade_ypd_in_world[0] * 57.3;
          data["blade_pitch"] = p.blade_ypd_in_world[1] * 57.3;
          data["blade_dis"] = p.blade_ypd_in_world[2];
        } else {
          data["detected"] = 0;
        }

        plotter.plot(data);
      }

      tools::draw_text(
        vis,
        fmt::format("MODE: {}",
                    target_valid ? (use_big_buff ? "AUTO_BIG" : "AUTO_SMALL") : "SIM_DEFAULT"),
        {10, 30}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("GY:{:.2f} GP:{:.2f}",
                    gimbal_yaw_wrap * 57.3,
                    gimbal_pitch_wrap * 57.3),
        {10, 60}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("CALC_Y:{:.2f} CALC_P:{:.2f}",
                    calc_cmd_yaw_wrap * 57.3,
                    calc_cmd_pitch_wrap * 57.3),
        {10, 90}, {255, 100, 255});

      tools::draw_text(
        vis,
        fmt::format("SENT_Y:{:.2f} SENT_P:{:.2f}",
                    sent_yaw_wrap * 57.3,
                    sent_pitch_wrap * 57.3),
        {10, 120}, {50, 255, 50});

      tools::draw_text(
        vis,
        fmt::format("SIM_Y:{:.2f} SIM_P:{:.2f}",
                    wrap_pi(sim_send_yaw) * 57.3,
                    wrap_pi(sim_send_pitch) * 57.3),
        {10, 150}, {0, 200, 255});

      tools::draw_text(
        vis,
        fmt::format("YAW_ERR:{:+.2f} YAW_STEP:{:+.2f}",
                    yaw_err * 57.3,
                    yaw_step * 57.3),
        {10, 180}, {255, 220, 120});

      tools::draw_text(
        vis,
        fmt::format("ERR_Y:{:+.2f} ERR_P:{:+.2f}",
                    target_valid ? (err_yaw_wrap * 57.3) : 0.0,
                    target_valid ? (err_pitch_wrap * 57.3) : 0.0),
        {10, 210}, {0, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("ABS_YAW:{} REL_YAW:{} invY:{} invP:{}",
                    static_cast<int>(use_absolute_yaw_cmd),
                    static_cast<int>(use_relative_yaw_cmd),
                    static_cast<int>(sim_invert_yaw),
                    static_cast<int>(sim_invert_pitch)),
        {10, 240}, {255, 255, 0});

      tools::draw_text(
        vis,
        fmt::format("switch:{} mistake:{} valid:{} aimer_ctrl:{} send_ctrl:{}",
                    static_cast<int>(switch_fanblade),
                    mistake_count,
                    static_cast<int>(target_valid),
                    static_cast<int>(command.control),
                    static_cast<int>(send_cmd.control)),
        {10, 270}, {255, 200, 50});

      tools::draw_text(
        vis,
        fmt::format("shoot:{} bullet_speed:{:.2f}",
                    send_cmd.shoot ? 1 : 0,
                    simboard.bullet_speed),
        {10, 300}, {200, 255, 100});

      if (rune_opt.has_value()) {
        const auto & p = rune_opt.value();
        tools::draw_text(
          vis,
          fmt::format("buffR_yaw:{:.2f} buff_yaw:{:.2f} buff_phase:{:.2f}",
                      p.ypd_in_world[0] * 57.3,
                      p.ypr_in_world[0] * 57.3,
                      p.ypr_in_world[2] * 57.3),
          {10, 330}, {255, 150, 100});

        tools::draw_text(
          vis,
          fmt::format("blade_yaw:{:.2f} blade_pitch:{:.2f} blade_dis:{:.2f}",
                      p.blade_ypd_in_world[0] * 57.3,
                      p.blade_ypd_in_world[1] * 57.3,
                      p.blade_ypd_in_world[2]),
          {10, 360}, {120, 220, 255});
      }

      if (pred_blade_valid) {
        tools::draw_text(
          vis,
          fmt::format("pred_blade_yaw:{:.2f} pred_blade_pitch:{:.2f} pred_blade_dis:{:.2f}",
                      pred_blade_yaw * 57.3,
                      pred_blade_pitch * 57.3,
                      pred_blade_dis),
          {10, 390}, {80, 255, 180});
      }

      cv::Mat vis_show;
      cv::resize(vis, vis_show, {}, 0.5, 0.5, cv::INTER_AREA);
      cv::imshow("buff_sim", vis_show);

      int key = cv::waitKey(1);
      if (key == 'q' || key == 'Q') {
        break;
      }
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
  } catch (const YAML::BadConversion &e) {
    std::cerr << "[auto_buff_sim] YAML::BadConversion: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (const YAML::Exception &e) {
    std::cerr << "[auto_buff_sim] YAML::Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception &e) {
    std::cerr << "[auto_buff_sim] exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    std::cerr << "[auto_buff_sim] unknown exception" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}