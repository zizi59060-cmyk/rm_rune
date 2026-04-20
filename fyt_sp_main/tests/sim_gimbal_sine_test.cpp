#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

using namespace std::chrono_literals;

static double wrap_pi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static std::string make_cmd(bool control, bool shoot, double yaw_rad, double pitch_rad) {
    std::ostringstream oss;
    oss << (control ? "true" : "false") << ","
        << (shoot ? "true" : "false") << ","
        << std::fixed << std::setprecision(6)
        << yaw_rad << ","
        << pitch_rad;
    return oss.str();
}

class SimGimbalSineTest : public rclcpp::Node {
public:
    SimGimbalSineTest()
        : Node("sim_gimbal_sine_test"),
          start_time_(this->now()) {
        this->declare_parameter<double>("yaw_amp_deg", 90.0);
        this->declare_parameter<double>("pitch_amp_deg", 10.0);
        this->declare_parameter<double>("freq_hz", 0.10);
        this->declare_parameter<double>("pitch_phase_deg", 0.0);
        this->declare_parameter<bool>("enable_pitch_sine", true);
        this->declare_parameter<double>("publish_rate_hz", 100.0);

        yaw_amp_deg_ = this->get_parameter("yaw_amp_deg").as_double();
        pitch_amp_deg_ = this->get_parameter("pitch_amp_deg").as_double();
        freq_hz_ = this->get_parameter("freq_hz").as_double();
        pitch_phase_deg_ = this->get_parameter("pitch_phase_deg").as_double();
        enable_pitch_sine_ = this->get_parameter("enable_pitch_sine").as_bool();
        publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

        cmd_pub_ = this->create_publisher<std_msgs::msg::String>("/fyt_cmd", 10);

        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/tf", 50,
            std::bind(&SimGimbalSineTest::tf_callback, this, std::placeholders::_1)
        );

        auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SimGimbalSineTest::on_timer, this)
        );

        RCLCPP_INFO(
            this->get_logger(),
            "sim_gimbal_sine_test started. yaw_amp=%.2f deg, pitch_amp=%.2f deg, freq=%.3f Hz, rate=%.1f Hz",
            yaw_amp_deg_, pitch_amp_deg_, freq_hz_, publish_rate_hz_
        );
    }

private:
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto &tf : msg->transforms) {
            if (tf.child_frame_id != "gimbal_link") {
                continue;
            }

            const auto &qmsg = tf.transform.rotation;
            Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
            Eigen::Matrix3d R = q.normalized().toRotationMatrix();

            // 与项目里现有姿态解法保持一致
            Eigen::Vector3d ypr = tools::eulers(R, 2, 1, 0);

            std::lock_guard<std::mutex> lock(data_mutex_);
            gimbal_yaw_ = ypr[0];
            gimbal_pitch_ = ypr[1];
            has_gimbal_ = true;
        }
    }

    void on_timer() {
        const double t = (this->now() - start_time_).seconds();

        const double yaw_rad =
            (yaw_amp_deg_ * M_PI / 180.0) * std::sin(2.0 * M_PI * freq_hz_ * t);

        double pitch_rad = 0.0;
        if (enable_pitch_sine_) {
            const double phase = pitch_phase_deg_ * M_PI / 180.0;
            pitch_rad =
                (pitch_amp_deg_ * M_PI / 180.0) * std::sin(2.0 * M_PI * freq_hz_ * t + phase);
        }

        std_msgs::msg::String msg;
        msg.data = make_cmd(true, false, yaw_rad, pitch_rad);
        cmd_pub_->publish(msg);

        double gimbal_yaw = 0.0;
        double gimbal_pitch = 0.0;
        bool has_gimbal = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            gimbal_yaw = gimbal_yaw_;
            gimbal_pitch = gimbal_pitch_;
            has_gimbal = has_gimbal_;
        }

        nlohmann::json data;
        data["t"] = t;

        // 你要的四条核心曲线
        data["cmd_yaw"] = yaw_rad * 57.295779513;
        data["cmd_pitch"] = pitch_rad * 57.295779513;

        if (has_gimbal) {
            data["gimbal_yaw"] = gimbal_yaw * 57.295779513;
            data["gimbal_pitch"] = gimbal_pitch * 57.295779513;
        }

        // 额外加一组 wrap 后曲线，方便排查跨 ±180°
        data["cmd_yaw_wrap"] = wrap_pi(yaw_rad) * 57.295779513;
        data["cmd_pitch_wrap"] = wrap_pi(pitch_rad) * 57.295779513;

        if (has_gimbal) {
            data["gimbal_yaw_wrap"] = wrap_pi(gimbal_yaw) * 57.295779513;
            data["gimbal_pitch_wrap"] = wrap_pi(gimbal_pitch) * 57.295779513;
            data["err_yaw_wrap"] = wrap_pi(yaw_rad - gimbal_yaw) * 57.295779513;
            data["err_pitch_wrap"] = wrap_pi(pitch_rad - gimbal_pitch) * 57.295779513;
        }

        plotter_.plot(data);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "cmd_yaw=%.2f cmd_pitch=%.2f | gimbal_yaw=%.2f gimbal_pitch=%.2f",
            yaw_rad * 57.295779513,
            pitch_rad * 57.295779513,
            gimbal_yaw * 57.295779513,
            gimbal_pitch * 57.295779513
        );
    }

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tools::Plotter plotter_;

    rclcpp::Time start_time_;

    std::mutex data_mutex_;
    bool has_gimbal_ = false;
    double gimbal_yaw_ = 0.0;
    double gimbal_pitch_ = 0.0;

    double yaw_amp_deg_ = 90.0;
    double pitch_amp_deg_ = 10.0;
    double freq_hz_ = 0.10;
    double pitch_phase_deg_ = 0.0;
    bool enable_pitch_sine_ = true;
    double publish_rate_hz_ = 100.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimGimbalSineTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}