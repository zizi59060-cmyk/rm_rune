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
    double last_cmd_yaw = 0.0;
    double last_cmd_pitch = 0.0;
    auto t_begin = std::chrono::steady_clock::now();

    while (!exiter.exit() && rclcpp::ok()) {
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) {
        cv::waitKey(1);
        continue;
      }

      // 1) 云台绝对姿态
      Eigen::Quaterniond q = simboard.imu_at(timestamp - std::chrono::milliseconds(1));
      buff_solver.set_R_gimbal2world(q);

      // 2) 相机外参由仿真器 /tf 实时提供
      Eigen::Matrix3d R_camera2gimbal;
      Eigen::Vector3d t_camera2gimbal;
      if (simboard.camera2gimbal(R_camera2gimbal, t_camera2gimbal)) {
        buff_solver.set_camera2gimbal(R_camera2gimbal, t_camera2gimbal);
      }

      cv::Mat vis = img.clone();

      // 3) detect
      std::optional<auto_buff::PowerRune> rune_opt = buff_detector.detect(vis);

      io::Command command{};
      command.control = false;
      command.shoot   = false;
      command.yaw     = 0.0;
      command.pitch   = 0.0;

      if (rune_opt.has_value()) {
        auto & rune = rune_opt.value();

        // 4) PnP/world solve
        buff_solver.solve(rune_opt);

        // 5) target update
        if (!rune.is_unsolve()) {
          buff_target->get_target(rune_opt, timestamp);
        } else {
          std::optional<auto_buff::PowerRune> empty;
          buff_target->get_target(empty, timestamp);
        }

        // 6) aim
        auto ts_for_aim = timestamp;
        command = buff_aimer.aim(*buff_target, ts_for_aim, simboard.bullet_speed, true);
      } else {
        std::optional<auto_buff::PowerRune> empty;
        buff_target->get_target(empty, timestamp);
      }

      // 当前云台姿态（世界系 yaw / pitch）
      Eigen::Vector3d ypr = tools::eulers(buff_solver.R_gimbal2world(), 2, 1, 0);
      double gimbal_yaw = ypr[0];
      double gimbal_pitch = ypr[1];

      if (command.control) {
        last_cmd_yaw = command.yaw;
        last_cmd_pitch = command.pitch;
      }

      // wrap 后角度
      double cmd_yaw_wrap = wrap_pi(command.yaw);
      double cmd_pitch_wrap = wrap_pi(command.pitch);
      double gimbal_yaw_wrap = wrap_pi(gimbal_yaw);
      double gimbal_pitch_wrap = wrap_pi(gimbal_pitch);

      double err_yaw_wrap = wrap_pi(command.yaw - gimbal_yaw);
      double err_pitch_wrap = wrap_pi(command.pitch - gimbal_pitch);

      // 7) send to simulator
      if (command.control) {
      simboard.send(command);
  }

      // 8) PlotJuggler 输出
      {
        nlohmann::json data;
        double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t_begin).count();

        data["t"] = t;
        data["mode"] = use_big_buff ? "buff_sim_big" : "buff_sim_small";

        // 原始角度
        data["gimbal_yaw"] = gimbal_yaw * 57.3;
        data["gimbal_pitch"] = gimbal_pitch * 57.3;
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;

        // wrap 后角度
        data["gimbal_yaw_wrap"] = gimbal_yaw_wrap * 57.3;
        data["gimbal_pitch_wrap"] = gimbal_pitch_wrap * 57.3;
        data["cmd_yaw_wrap"] = cmd_yaw_wrap * 57.3;
        data["cmd_pitch_wrap"] = cmd_pitch_wrap * 57.3;

        // last cmd
        data["last_cmd_yaw"] = last_cmd_yaw * 57.3;
        data["last_cmd_pitch"] = last_cmd_pitch * 57.3;

        // wrap 后误差
        data["err_yaw_wrap"] = err_yaw_wrap * 57.3;
        data["err_pitch_wrap"] = err_pitch_wrap * 57.3;

        data["shoot"] = command.shoot ? 1 : 0;
        data["control"] = command.control ? 1 : 0;
        data["bullet_speed"] = simboard.bullet_speed;

        if (rune_opt.has_value()) {
          const auto & p = rune_opt.value();
          data["detected"] = 1;
          data["buff_R_yaw"] = p.ypd_in_world[0];
          data["buff_R_pitch"] = p.ypd_in_world[1];
          data["buff_R_dis"] = p.ypd_in_world[2];
          data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
          data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
          data["buff_roll"] = p.ypr_in_world[2] * 57.3;
        } else {
          data["detected"] = 0;
        }

        plotter.plot(data);
      }

      // 9) 画面显示
      tools::draw_text(
        vis,
        fmt::format("MODE: {}",
                    use_big_buff ? "BUFF_SIM_BIG" : "BUFF_SIM_SMALL"),
        {10, 30}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("RAW gimbal_yaw:{:.2f}  gimbal_pitch:{:.2f}",
                    gimbal_yaw * 57.3,
                    gimbal_pitch * 57.3),
        {10, 60}, {255, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("RAW cmd_yaw:{:.2f}  cmd_pitch:{:.2f}",
                    command.yaw * 57.3,
                    command.pitch * 57.3),
        {10, 90}, {154, 50, 205});

      tools::draw_text(
        vis,
        fmt::format("WRAP gimbal_yaw:{:.2f}  cmd_yaw:{:.2f}",
                    gimbal_yaw_wrap * 57.3,
                    cmd_yaw_wrap * 57.3),
        {10, 120}, {50, 255, 50});

      tools::draw_text(
        vis,
        fmt::format("WRAP err_yaw:{:+.2f}  err_pitch:{:+.2f}",
                    err_yaw_wrap * 57.3,
                    err_pitch_wrap * 57.3),
        {10, 150}, {0, 255, 255});

      tools::draw_text(
        vis,
        fmt::format("ctrl:{} shoot:{} bullet_speed:{:.2f}",
                    (int)command.control,
                    (int)command.shoot,
                    simboard.bullet_speed),
        {10, 180}, {255, 200, 50});

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