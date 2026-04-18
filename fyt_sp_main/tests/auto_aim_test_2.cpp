#include <fmt/core.h>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

// 角度 wrap 到 [-pi, pi]
static inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml | yaml配置文件路径}"
  "{bullet-speed   | 20                  | 子弹速度(m/s)，<=10则用yaml/默认值 }"
  "{buff-detect24  | true                | 使用 detect_24 (true/false) }"
  "{buff-future-dt | 0                   | 未来预测时间(秒)，仅用于画点 }"
  "{draw-fans       | true               | 绘制每片扇叶中心/角点/编号 }"
  // 可选：是否强制允许开火（传给 aimer.aim 的最后一个参数）
  "{force-fire      | true               | aimer最后一个参数(true/false) }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);

  const double bullet_speed_in = cli.get<double>("bullet-speed");
  const bool buff_detect24 = cli.get<bool>("buff-detect24");
  const double buff_future_dt = cli.get<double>("buff-future-dt");
  const bool draw_fans = cli.get<bool>("draw-fans");
  const bool force_fire = cli.get<bool>("force-fire");

  tools::Plotter plotter;
  tools::Exiter exiter;

  // ===== 实机：云台 + 相机 =====
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  // ===== 模块初始化 =====
  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::BigTarget buff_target;   // 你如果要小符就换 SmallTarget
  // auto_buff::SmallTarget buff_target;
  auto_buff::Aimer buff_aimer(config_path);

  cv::Mat img;

  // FPS（运行时）
  double fps = 0.0;
  auto last_frame_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  constexpr double fps_update_interval = 1.0;

  // 用于画“云台误差”
  double last_cmd_yaw = 0.0;
  double last_cmd_pitch = 0.0;

  while (!exiter.exit()) {
    // 1) 实机取图 + 时间戳（注意：timestamp 必须是非 const）
    std::chrono::steady_clock::time_point timestamp;
    camera.read(img, timestamp);
    if (img.empty()) break;

    // 2) 读云台姿态（四元数）+ 云台状态
    Eigen::Quaterniond q = gimbal.q(timestamp);
    auto gs = gimbal.state();

    // 3) 更新 solver 外参
    buff_solver.set_R_gimbal2world(q);

    cv::Mat vis = img.clone();

    // 4) detect
    std::optional<auto_buff::PowerRune> rune_opt;
    if (buff_detect24) rune_opt = buff_detector.detect_24(vis);
    else               rune_opt = buff_detector.detect(vis);

    // 5) 默认下发命令（如果没识别到就不控）
    io::Command command{};
    command.control = false;
    command.shoot   = false;
    command.yaw     = 0.0;
    command.pitch   = 0.0;

    // 子弹速度：你给的参数优先；<=10 则用一个合理默认值（也可以改成读 yaml）
    double bullet_speed = bullet_speed_in;
    if (bullet_speed <= 10.0) bullet_speed = 24.0;

    if (rune_opt.has_value()) {
      auto & rune = rune_opt.value();

      // 5.1) PnP/世界系解算（会写回 rune 内部的 xyz/ypd/ypr 等）
      buff_solver.solve(rune_opt);

      // 5.2) EKF 更新（timestamp 传引用，内部可能重置 timebase）
      if (!rune.is_unsolve()) {
        buff_target.get_target(rune_opt, timestamp);
      } else {
        std::optional<auto_buff::PowerRune> empty;
        buff_target.get_target(empty, timestamp);
      }

      // 5.3) 计算目标角（cmd）
      auto ts_for_aim = timestamp; // aim 如果也要 time_point&，用副本更安全
      command = buff_aimer.aim(buff_target, ts_for_aim, bullet_speed, force_fire);

      // ======== 画每片扇叶（中心 + 角点 + 编号 + type）========
      if (draw_fans) {
        for (int i = 0; i < (int)rune.fanblades.size(); i++) {
          auto & fb = rune.fanblades[i];
          cv::Scalar color;
          if (fb.type == auto_buff::_target)      color = cv::Scalar(0, 255, 255); // 黄
          else if (fb.type == auto_buff::_light)  color = cv::Scalar(0, 0, 255);   // 红
          else                                    color = cv::Scalar(0, 255, 0);   // 绿

          cv::circle(vis, fb.center, 10, color, 2);
          for (auto & pt : fb.points) cv::circle(vis, pt, 4, color, -1);

          const char * tstr =
            (fb.type == auto_buff::_target) ? "T" :
            (fb.type == auto_buff::_light)  ? "L" : "N";

          cv::putText(
            vis,
            cv::format("#%d[%s]", i, tstr),
            cv::Point((int)fb.center.x + 12, (int)fb.center.y - 12),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2
          );
        }
        cv::circle(vis, rune.r_center, 8, cv::Scalar(255, 0, 0), 2);
      }

      // ======== 观测点/预测点可视化（全部用 reproject_world_point）========
      if (!buff_target.is_unsolve() && !rune.is_unsolve()) {
        Eigen::Vector3d blade_world = rune.blade_xyz_in_world;
        Eigen::Vector3d pred_world_0 = buff_target.predict_position(0.0);
        Eigen::Vector3d pred_world_future = buff_target.predict_position(buff_future_dt);

        cv::Point2f blade_px = buff_solver.reproject_world_point(blade_world);
        cv::Point2f pred_px_0 = buff_solver.reproject_world_point(pred_world_0);
        cv::Point2f pred_px_future = buff_solver.reproject_world_point(pred_world_future);

        // 红色：观测到的符臂末端（PnP 推出来的）
        if (blade_px.x >= 0 && blade_px.y >= 0)
          cv::circle(vis, blade_px, 10, cv::Scalar(0, 0, 255), 2);

        // 黄色：预测当前位置
        if (pred_px_0.x >= 0 && pred_px_0.y >= 0)
          cv::circle(vis, pred_px_0, 10, cv::Scalar(0, 255, 255), 2);

        // 白色：未来预测点（实心）
        if (pred_px_future.x >= 0 && pred_px_future.y >= 0)
          cv::circle(vis, pred_px_future, 6, cv::Scalar(255, 255, 255), -1);

        if (blade_px.x >= 0 && pred_px_future.x >= 0)
          cv::line(vis, blade_px, pred_px_future, cv::Scalar(255, 255, 255), 1);
      }

      tools::draw_text(
        vis,
        fmt::format("MODE: BUFF HW | gimbal yaw:{:.2f} pitch:{:.2f} (deg)",
                    gs.yaw * 57.3, gs.pitch * 57.3),
        {10, 30}, {255, 255, 255});

    } else {
      cv::putText(vis, "SEARCHING...", {50, 100}, cv::FONT_HERSHEY_SIMPLEX, 1.5, {0, 255, 255}, 3);
      tools::draw_text(
        vis,
        fmt::format("MODE: BUFF HW | gimbal yaw:{:.2f} pitch:{:.2f} (deg)",
                    gs.yaw * 57.3, gs.pitch * 57.3),
        {10, 30}, {255, 255, 255});
    }

    // =====================================================
    // 6) 实机下发（Gimbal 通信）
    //    - 普通 aim 没有速度/加速度，先置 0
    // =====================================================
    if (command.control) {
      gimbal.send(
        command.control,
        command.shoot,      // fire
        command.yaw,
        0.0,                // yaw_vel
        0.0,                // yaw_acc
        command.pitch,
        0.0,                // pitch_vel
        0.0                 // pitch_acc
      );
      last_cmd_yaw = command.yaw;
      last_cmd_pitch = command.pitch;
    } else {
      // 不控时也可以选择发一个 control=false 的包（看你们下位机协议是否需要心跳）
      gimbal.send(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    // 画控制信息（setpoint & error）
    const double dyaw = wrap_pi(last_cmd_yaw - gs.yaw);
    const double dpit = wrap_pi(last_cmd_pitch - gs.pitch);

    tools::draw_text(
      vis,
      fmt::format("SETPOINT cmd_yaw:{:.2f} cmd_pitch:{:.2f} (deg) shoot:{} ctrl:{}",
                  last_cmd_yaw * 57.3, last_cmd_pitch * 57.3,
                  (int)command.shoot, (int)command.control),
      {10, 60}, {154, 50, 205});

    tools::draw_text(
      vis,
      fmt::format("ERROR dyaw:{:+.2f} dpitch:{:+.2f} (deg)",
                  dyaw * 57.3, dpit * 57.3),
      {10, 90}, {0, 255, 255});

    // FPS（运行时）
    auto frame_end = std::chrono::steady_clock::now();
    frame_count++;
    double elapsed_time = tools::delta_time(frame_end, last_frame_time);
    if (elapsed_time >= fps_update_interval) {
      fps = frame_count / elapsed_time;
      frame_count = 0;
      last_frame_time = frame_end;
      tools::logger()->info("Current FPS: {:.1f}, Mode: BUFF_HW", fps);
    }

    // Plotter
    nlohmann::json data;
    data["mode"] = "buff_hw";
    data["cmd_yaw_deg"] = last_cmd_yaw * 57.3;
    data["cmd_pitch_deg"] = last_cmd_pitch * 57.3;
    data["gimbal_yaw_deg"] = gs.yaw * 57.3;
    data["gimbal_pitch_deg"] = gs.pitch * 57.3;
    data["err_yaw_deg"] = dyaw * 57.3;
    data["err_pitch_deg"] = dpit * 57.3;
    data["shoot"] = command.shoot;
    data["control"] = command.control;
    data["fps"] = fps;
    plotter.plot(data);

    cv::Mat vis_show;
    cv::resize(vis, vis_show, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::imshow("buff_hw_test", vis_show);

    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
