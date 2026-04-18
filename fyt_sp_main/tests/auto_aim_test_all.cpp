#include <fmt/core.h>

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                   | 输出命令行参数说明 }"
  "{@config-path   | configs/demo.yaml | yaml配置文件路径}";

// 便于在 PlotJuggler 里画状态曲线：tracking/temp_lost/detecting/lost
static inline int tracker_state_to_int(const std::string& s) {
  if (s == "tracking") return 3;
  if (s == "temp_lost") return 2;
  if (s == "detecting") return 1;
  return 0;
}

// 录像文件名：record_YYYYMMDD_HHMMSS.mp4
static inline std::string make_record_name() {
  std::time_t t = std::time(nullptr);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << "record_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".mp4";
  return oss.str();
}

int main(int argc, char* argv[]) {
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const std::string config_path = cli.get<std::string>(0);

  tools::logger()->info("config: {}", config_path);

  tools::Plotter plotter;
  tools::Exiter exiter;

  // -------------------- 模块初始化 --------------------
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);

  // -------------------- FPS统计 --------------------
  int frame_count = 0;
  double fps = 0.0;
  auto last_fps_time = std::chrono::steady_clock::now();

  // 曲线时间轴（运行时间）
  auto t_begin = std::chrono::steady_clock::now();

  // -------------------- Recording --------------------
  bool recording = false;
  cv::VideoWriter writer;
  std::string record_path;

  auto start_recording = [&](const cv::Size& frame_size, double rec_fps) -> bool {
    record_path = make_record_name();

    // 优先 mp4
    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    bool ok = writer.open(record_path, fourcc, rec_fps, frame_size, true);

    // fallback：avi + MJPG
    if (!ok) {
      record_path = record_path.substr(0, record_path.size() - 4) + ".avi";
      fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
      ok = writer.open(record_path, fourcc, rec_fps, frame_size, true);
    }

    recording = ok;
    if (ok) {
      tools::logger()->info("[REC] start: {}", record_path);
    } else {
      tools::logger()->warn("[REC] failed to open VideoWriter");
    }
    return ok;
  };

  auto stop_recording = [&]() {
    if (writer.isOpened()) writer.release();
    if (recording) tools::logger()->info("[REC] stop: {}", record_path);
    recording = false;
  };

  // -------------------- 主循环 --------------------
  while (!exiter.exit()) {
    // 读图
    cv::Mat img;
    std::chrono::steady_clock::time_point img_stamp;
    camera.read(img, img_stamp);
    if (img.empty()) break;

    // 下位机状态（弹速单位：m/s）
    auto gs = gimbal.state();
    double bullet_speed = static_cast<double>(gs.bullet_speed);
    // 简单兜底：防 0 或离谱值把解算搞炸
    if (bullet_speed < 1.0 || bullet_speed > 200.0) bullet_speed = 27.0;

    // 设置姿态（用于解算/重投影）
    Eigen::Quaterniond gimbal_q = gimbal.q(img_stamp);
    solver.set_R_gimbal2world(gimbal_q);

    // 检测 + 跟踪
    auto armors = yolo.detect(img, 0);
    auto targets = tracker.track(armors, img_stamp);

    // 弹道解算：用下位机回传弹速
    auto command = aimer.aim(targets, img_stamp, bullet_speed, false);

    // 串口发送
    gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);

    // FPS 统计（每秒更新一次）
    frame_count++;
    auto now = std::chrono::steady_clock::now();
    double dt_fps = tools::delta_time(now, last_fps_time);
    if (dt_fps >= 1.0) {
      fps = frame_count / dt_fps;
      frame_count = 0;
      last_fps_time = now;
      tools::logger()->info("FPS: {:.1f}, tracker: {}, v: {:.2f} m/s", fps, tracker.state(),
                            bullet_speed);
    }

    // -------------------- 可视化 --------------------
    // 云台欧拉角（deg）
    Eigen::Vector3d euler =
      tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3;  // deg
    const double gimbal_yaw_deg = euler[0];
    const double gimbal_pitch_deg = euler[2];

    std::string tracker_state = tracker.state();
    cv::Scalar state_color;
    if (tracker_state == "tracking")
      state_color = {0, 255, 0};
    else if (tracker_state == "temp_lost")
      state_color = {0, 255, 255};
    else if (tracker_state == "detecting")
      state_color = {255, 255, 0};
    else
      state_color = {0, 0, 255};

    // 在原图上画调试信息
    tools::draw_text(img, fmt::format("State: {}", tracker_state), {10, 30}, state_color);
    tools::draw_text(img,
                     fmt::format("gimbal yaw:{:.2f}, pitch:{:.2f}", gimbal_yaw_deg,
                                 gimbal_pitch_deg),
                     {10, 60}, {255, 255, 255});
    tools::draw_text(
      img,
      fmt::format("cmd yaw:{:.2f}, pitch:{:.2f}, shoot:{}, v:{:.2f} m/s{}",
                  command.yaw * 57.3, command.pitch * 57.3, command.shoot ? 1 : 0, bullet_speed,
                  recording ? "  [REC]" : ""),
      {10, 90}, {154, 50, 205});

    // 重投影：瞄准点（红/蓝）
    if (!targets.empty()) {
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;

      auto& target = targets.front();
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);

      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});
    }

    // 画所有装甲板重投影（绿色）
    if (!targets.empty()) {
      auto target = targets.front();
      for (const Eigen::Vector4d& xyza : target.armor_xyza_list()) {
        auto pts =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        tools::draw_points(img, pts, {0, 255, 0});
      }
    }

    // -------------------- 曲线输出（PlotJuggler） --------------------
    nlohmann::json data;
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t_begin);  // s

    data["armor_num"] = static_cast<int>(armors.size());
    data["track_state"] = tracker_state_to_int(tracker_state);

    data["gimbal_yaw"] = gimbal_yaw_deg;
    data["gimbal_pitch"] = gimbal_pitch_deg;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["cmd_pitch"] = command.pitch * 57.3;
    data["shoot"] = command.shoot ? 1 : 0;
    data["control"] = command.control ? 1 : 0;
    data["bullet_speed"] = bullet_speed;
    data["fps"] = fps;

    if (!targets.empty()) {
      auto target = targets.front();

      // EKF 状态（如果长度满足）
      Eigen::VectorXd x = target.ekf_x();
      if (x.size() >= 11) {
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
      }
      data["last_id"] = target.last_id;

      // EKF 统计量：有就写（不同版本可能字段不全）
      auto& ekf_data = target.ekf().data;
      if (ekf_data.count("residual_yaw")) data["residual_yaw"] = ekf_data.at("residual_yaw");
      if (ekf_data.count("residual_pitch")) data["residual_pitch"] = ekf_data.at("residual_pitch");
      if (ekf_data.count("residual_distance"))
        data["residual_distance"] = ekf_data.at("residual_distance");
      if (ekf_data.count("residual_angle")) data["residual_angle"] = ekf_data.at("residual_angle");
      if (ekf_data.count("nis")) data["nis"] = ekf_data.at("nis");
      if (ekf_data.count("nees")) data["nees"] = ekf_data.at("nees");
      if (ekf_data.count("nis_fail")) data["nis_fail"] = ekf_data.at("nis_fail");
      if (ekf_data.count("nees_fail")) data["nees_fail"] = ekf_data.at("nees_fail");
      if (ekf_data.count("recent_nis_failures"))
        data["recent_nis_failures"] = ekf_data.at("recent_nis_failures");
    }

    plotter.plot(data);

    // -------------------- 显示（你录的是这张） --------------------
    cv::Mat img_show = img;
    cv::resize(img_show, img_show, {}, 0.5, 0.5);
    cv::imshow("auto_aim_test_all", img_show);

    // 写入录像（保持尺寸一致）
    if (recording && writer.isOpened()) {
      writer.write(img_show);
    }

    // 按键：r 开始/停止录制，q 或 ESC 退出
    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
    if (key == 'r') {
      if (!recording) {
        // 录像 fps：优先用当前统计 fps（太小则给默认）
        double rec_fps = (fps > 5.0 ? fps : 60.0);
        start_recording(img_show.size(), rec_fps);
      } else {
        stop_recording();
      }
    }
  }

  stop_recording();
  return 0;
}
