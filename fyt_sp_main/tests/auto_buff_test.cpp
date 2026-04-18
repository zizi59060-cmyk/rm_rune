#include <fmt/core.h>
#include <chrono>
#include <cmath>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

static inline double wrap_pi(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// yaw(Z) + pitch(Y) 生成四元数（如你们轴定义不同，可在此处调整）
static inline Eigen::Quaterniond quat_from_yaw_pitch(double yaw, double pitch)
{
  Eigen::AngleAxisd yawAA(yaw,   Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitAA(pitch, Eigen::Vector3d::UnitY());
  Eigen::Quaterniond q = yawAA * pitAA;
  q.normalize();
  return q;
}

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml | yaml配置文件路径}"
  "{video          |                     | 指定视频文件路径 }"
  "{bullet-speed   | 20                  | 子弹速度(m/s) }"
  "{buff-detect24  | true                | 使用 detect_24 (true/false) }"
  "{buff-future-dt | 0                   | 未来预测时间(秒)，仅用于画点 }"
  // ======== 离线模拟下位机云台角（度）========
  "{sim-yaw0        | 0                   | 假设下位机当前yaw初值(度) }"
  "{sim-pitch0      | 0                   | 假设下位机当前pitch初值(度) }"
  // ======== 离线模拟电机响应 ========
  "{sim-mode        | 1                   | 0瞬时到达, 1一阶跟随, 2冻结云台(不更新) }"
  "{sim-alpha       | 0.25                | 一阶跟随系数(0~1) }"
  "{draw-fans       | true                | 绘制每片扇叶中心/角点/编号 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const std::string video_path = cli.get<std::string>("video");
  if (video_path.empty()) {
    tools::logger()->error("Offline video test needs --video=/path/to/video.avi");
    return 1;
  }

  const double bullet_speed_in = cli.get<double>("bullet-speed");
  const bool buff_detect24 = cli.get<bool>("buff-detect24");
  const double buff_future_dt = cli.get<double>("buff-future-dt");

  const double sim_yaw0_deg = cli.get<double>("sim-yaw0");
  const double sim_pitch0_deg = cli.get<double>("sim-pitch0");
  const int sim_mode = cli.get<int>("sim-mode");
  const double sim_alpha = cli.get<double>("sim-alpha");
  const bool draw_fans = cli.get<bool>("draw-fans");

  tools::Plotter plotter;
  tools::Exiter exiter;

  // 模块初始化
  auto_buff::Buff_Detector buff_detector(config_path);
  auto_buff::Solver buff_solver(config_path);
  auto_buff::BigTarget buff_target;
  auto_buff::Aimer buff_aimer(config_path);

  // 打开视频
  cv::VideoCapture cap(video_path);
  if (!cap.isOpened()) {
    tools::logger()->error("Failed to open video: {}", video_path);
    return 1;
  }
  tools::logger()->info("Using video: {}", video_path);

  double fps_video = cap.get(cv::CAP_PROP_FPS);
  if (fps_video <= 0) fps_video = 30.0;
  const double dt_video = 1.0 / fps_video;

  cv::Mat img;

  // FPS（运行时）
  double fps = 0.0;
  auto last_frame_time = std::chrono::steady_clock::now();
  int frame_count = 0;
  constexpr double fps_update_interval = 1.0;

  // ===== 离线模拟：下位机当前云台角（rad） =====
  double sim_gimbal_yaw = sim_yaw0_deg / 57.3;
  double sim_gimbal_pitch = sim_pitch0_deg / 57.3;

  // 用帧号生成单调递增 timestamp（离线 EKF 更稳定）
  long long frame_id = 0;
  auto t0 = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    if (!cap.read(img) || img.empty()) break;

    // ✅ 关键修复：timestamp 不能是 const，因为 get_target 需要 time_point&
    auto timestamp =
      t0 + std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::duration<double>(frame_id * dt_video));
    frame_id++;

    // 用“模拟云台角”生成四元数喂给 solver
    Eigen::Quaterniond sim_q = quat_from_yaw_pitch(sim_gimbal_yaw, sim_gimbal_pitch);
    buff_solver.set_R_gimbal2world(sim_q);

    Eigen::Vector3d sim_gimbal_euler_deg(sim_gimbal_yaw * 57.3, sim_gimbal_pitch * 57.3, 0.0);

    cv::Mat vis = img.clone();

    std::optional<auto_buff::PowerRune> rune_opt;
    if (buff_detect24) rune_opt = buff_detector.detect_24(vis);
    else               rune_opt = buff_detector.detect(vis);

    io::Command command{};
    command.control = false;
    command.shoot   = false;
    command.yaw     = 0.0;
    command.pitch   = 0.0;

    double bullet_speed = bullet_speed_in;
    if (bullet_speed < 10) bullet_speed = 24;

    if (rune_opt.has_value()) {
      auto & rune = rune_opt.value();

      // 1) PnP
      buff_solver.solve(rune_opt);

      // 2) EKF 更新（注意这里 timestamp 可能被内部调整，所以必须是非 const）
      if (!rune.is_unsolve()) {
        buff_target.get_target(rune_opt, timestamp);
      } else {
        std::optional<auto_buff::PowerRune> empty;
        buff_target.get_target(empty, timestamp);
      }

      // 3) 计算目标角（cmd）
      auto ts_for_aim = timestamp;
      command = buff_aimer.aim(buff_target, ts_for_aim, bullet_speed, true);

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

      // ======== 观测点/预测点可视化 ========
      if (!buff_target.is_unsolve() && !rune.is_unsolve()) {
        Eigen::Vector3d blade_world = rune.blade_xyz_in_world;
        Eigen::Vector3d pred_world_0 = buff_target.predict_position(0.0);
        Eigen::Vector3d pred_world_future = buff_target.predict_position(buff_future_dt);

        cv::Point2f blade_px = buff_solver.reproject_world_point(blade_world);
        cv::Point2f pred_px_0 = buff_solver.reproject_world_point(pred_world_0);
        cv::Point2f pred_px_future = buff_solver.reproject_world_point(pred_world_future);

        if (blade_px.x >= 0 && blade_px.y >= 0)
          cv::circle(vis, blade_px, 10, cv::Scalar(0, 0, 255), 2);

        if (pred_px_0.x >= 0 && pred_px_0.y >= 0)
          cv::circle(vis, pred_px_0, 10, cv::Scalar(0, 255, 255), 2);

        // 白色未来预测点：实心点
        if (pred_px_future.x >= 0 && pred_px_future.y >= 0)
          cv::circle(vis, pred_px_future, 6, cv::Scalar(255, 255, 255), -1);

        if (blade_px.x >= 0 && pred_px_future.x >= 0)
          cv::line(vis, blade_px, pred_px_future, cv::Scalar(255, 255, 255), 1);
      }

      tools::draw_text(
        vis,
        fmt::format("MODE: BUFF | SIM gimbal yaw:{:.2f} pitch:{:.2f} (deg)",
                    sim_gimbal_euler_deg[0], sim_gimbal_euler_deg[1]),
        {10, 30}, {255, 255, 255});

    } else {
      cv::putText(vis, "SEARCHING...", {50, 100}, cv::FONT_HERSHEY_SIMPLEX, 1.5, {0, 255, 255}, 3);
      tools::draw_text(
        vis,
        fmt::format("MODE: BUFF | SIM gimbal yaw:{:.2f} pitch:{:.2f} (deg)",
                    sim_gimbal_euler_deg[0], sim_gimbal_euler_deg[1]),
        {10, 30}, {255, 255, 255});
    }

    // =====================================================
    // 离线：sim_gimbal_* 当“当前角”，command 当 setpoint
    // =====================================================
    const double target_yaw = command.yaw;
    const double target_pitch = command.pitch;

    const double delta_yaw = wrap_pi(target_yaw - sim_gimbal_yaw);
    const double delta_pitch = wrap_pi(target_pitch - sim_gimbal_pitch);

    if (sim_mode == 0) {
      sim_gimbal_yaw = target_yaw;
      sim_gimbal_pitch = target_pitch;
    } else if (sim_mode == 1) {
      sim_gimbal_yaw = wrap_pi(sim_gimbal_yaw + sim_alpha * delta_yaw);
      sim_gimbal_pitch = wrap_pi(sim_gimbal_pitch + sim_alpha * delta_pitch);
    } else {
      // sim_mode == 2：冻结云台（不更新）
    }

    tools::draw_text(
      vis,
      fmt::format("SETPOINT cmd_yaw:{:.2f} cmd_pitch:{:.2f} (deg) shoot:{} ctrl:{}",
                  target_yaw * 57.3, target_pitch * 57.3,
                  (int)command.shoot, (int)command.control),
      {10, 60}, {154, 50, 205});

    tools::draw_text(
      vis,
      fmt::format("SIM now_yaw:{:.2f} now_pitch:{:.2f} (deg)",
                  sim_gimbal_yaw * 57.3, sim_gimbal_pitch * 57.3),
      {10, 90}, {255, 255, 255});

    tools::draw_text(
      vis,
      fmt::format("SIM delta_yaw:{:+.2f} delta_pitch:{:+.2f} (deg) mode:{} alpha:{:.2f}",
                  delta_yaw * 57.3, delta_pitch * 57.3, sim_mode, sim_alpha),
      {10, 120}, {0, 255, 255});

    // FPS（运行时）
    auto frame_end = std::chrono::steady_clock::now();
    frame_count++;
    double elapsed_time = tools::delta_time(frame_end, last_frame_time);
    if (elapsed_time >= fps_update_interval) {
      fps = frame_count / elapsed_time;
      frame_count = 0;
      last_frame_time = frame_end;
      tools::logger()->info("Current FPS: {:.1f}, Mode: BUFF", fps);
    }

    // Plotter
    nlohmann::json data;
    data["mode"] = "buff_offline_video";
    data["cmd_yaw_deg"] = target_yaw * 57.3;
    data["cmd_pitch_deg"] = target_pitch * 57.3;
    data["sim_now_yaw_deg"] = sim_gimbal_yaw * 57.3;
    data["sim_now_pitch_deg"] = sim_gimbal_pitch * 57.3;
    data["sim_delta_yaw_deg"] = delta_yaw * 57.3;
    data["sim_delta_pitch_deg"] = delta_pitch * 57.3;
    data["shoot"] = command.shoot;
    data["control"] = command.control;
    data["fps"] = fps;
    plotter.plot(data);

    cv::Mat vis_show;
    cv::resize(vis, vis_show, {}, 0.5, 0.5, cv::INTER_AREA);
    cv::imshow("reprojection", vis_show);

    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}
