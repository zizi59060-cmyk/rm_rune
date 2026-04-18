#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include "io/camera.hpp"
#include "tools/recorder.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

int main(int argc, char** argv)
{
    const std::string keys =
        "{help h usage ? |             | 打印帮助信息}"
        "{@config        | configs/camera.yaml | 相机配置文件 }";

    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help") || !cli.has("@config")) {
        cli.printMessage();
        return 0;
    }

    std::string config_path = cli.get<std::string>("@config");

    tools::Exiter exiter;
    tools::Recorder recorder;
    io::Camera camera(config_path);

    bool is_recording = false;

    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;

    tools::logger()->info("工业相机录像测试程序启动");
    tools::logger()->info("按 R 开始录制，按 Q 停止录制，ESC 退出");

    while (!exiter.exit()) {

        // 读取工业相机图像
        camera.read(img, timestamp);

        // 正在录制 → 写入帧
        if (is_recording) {
            Eigen::Quaterniond q(1, 0, 0, 0);  // Recorder 需要四元数，这里使用单位四元数即可
            recorder.record(img, q, timestamp);
        }

        // 显示图像
        cv::imshow("Industrial Camera Record", img);

        int key = cv::waitKey(1);

        // R 开始录制
        if (key == 'r' || key == 'R') {
            if (!is_recording) {
                tools::logger()->info("开始录制...");
                is_recording = true;
            }
        }

        // Q 停止录制
        if (key == 'q' || key == 'Q') {
            if (is_recording) {
                tools::logger()->info("停止录制");
                is_recording = false;
            }
        }

        // ESC 退出
        if (key == 27) break;
    }

    tools::logger()->info("程序退出");
    return 0;
}
