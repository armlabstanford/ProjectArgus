// IMU-only orientation for the OpenSim ablation.
// Subscribes to <ns>/imu0 and publishes <ns>/mahoney_odomimu.
// On startup (and on ~/calibrate_gyro) the node averages the gyroscope for
// calibration_seconds and subtracts that bias. The filter still runs at the
// IMU rate; the odometry topic is published at publish_hz so it does not
// flood the link the way raw odomimu does.

#include "argus_mahoney/mahony.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

class MahoneyNode : public rclcpp::Node {
public:
  MahoneyNode()
  : rclcpp::Node("mahoney_node") {
    imu_topic_ = declare_parameter<std::string>("imu_topic", "imu0");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "mahoney_odomimu");
    calibration_seconds_ = declare_parameter<double>("calibration_seconds", 10.0);
    publish_hz_ = declare_parameter<double>("publish_hz", 50.0);
    filter_.kp = declare_parameter<double>("kp", 0.5);
    filter_.ki = declare_parameter<double>("ki", 0.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "mahoney_world");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "imu");

    const auto qos = rclcpp::SensorDataQoS().keep_last(20);
    const auto pub_qos = rclcpp::SensorDataQoS().keep_last(1);
    pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, pub_qos);
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { on_imu(std::move(msg)); });
    calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/calibrate_gyro",
      [this](
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        start_calibration();
        response->success = true;
        response->message = "collecting gyroscope samples";
      });

    if (publish_hz_ > 0.0) {
      timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_hz_),
        [this]() { publish_latest(); });
    }

    RCLCPP_INFO(get_logger(),
      "Mahony IMU orientation: %s -> %s, kp=%.3f ki=%.3f, publish %.1f Hz",
      imu_topic_.c_str(), odom_topic_.c_str(), filter_.kp, filter_.ki, publish_hz_);
    if (calibration_seconds_ > 0.0) {
      start_calibration();
    } else {
      calibrated_ = true;
      RCLCPP_INFO(get_logger(), "Gyro calibration skipped. Bias is zero until calibrate_gyro.");
    }
  }

private:
  void start_calibration() {
    calibrating_ = true;
    calibrated_ = false;
    have_stamp_ = false;
    samples_ = 0;
    gyro_sum_[0] = gyro_sum_[1] = gyro_sum_[2] = 0.0;
    gyro_sq_[0] = gyro_sq_[1] = gyro_sq_[2] = 0.0;
    accel_sum_[0] = accel_sum_[1] = accel_sum_[2] = 0.0;
    RCLCPP_INFO(get_logger(),
      "Hold still. Averaging the gyroscope for %.1f s to estimate bias.",
      calibration_seconds_);
  }

  void finish_calibration() {
    const double n = static_cast<double>(samples_);
    bias_[0] = gyro_sum_[0] / n;
    bias_[1] = gyro_sum_[1] / n;
    bias_[2] = gyro_sum_[2] / n;
    const double sx = std::sqrt(std::max(0.0, gyro_sq_[0] / n - bias_[0] * bias_[0]));
    const double sy = std::sqrt(std::max(0.0, gyro_sq_[1] / n - bias_[1] * bias_[1]));
    const double sz = std::sqrt(std::max(0.0, gyro_sq_[2] / n - bias_[2] * bias_[2]));
    filter_.set_tilt_from_accel(accel_sum_[0], accel_sum_[1], accel_sum_[2]);
    filter_.integral[0] = filter_.integral[1] = filter_.integral[2] = 0.0;
    calibrating_ = false;
    calibrated_ = true;
    have_stamp_ = false;
    have_odom_ = false;
    RCLCPP_INFO(get_logger(),
      "Gyro bias rad/s: [%.6f, %.6f, %.6f] from %zu samples (std [%.5f, %.5f, %.5f])",
      bias_[0], bias_[1], bias_[2], samples_, sx, sy, sz);
    const double std_max = std::max(sx, std::max(sy, sz));
    if (std_max > 0.05) {
      RCLCPP_WARN(get_logger(),
        "Gyroscope moved during calibration (std %.3f rad/s). Hold the module still and call calibrate_gyro again.",
        std_max);
    }
    RCLCPP_INFO(get_logger(), "Publishing %s", odom_topic_.c_str());
  }

  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    const auto &w = msg->angular_velocity;
    const auto &a = msg->linear_acceleration;
    if (calibrating_) {
      if (samples_ == 0) {
        calib_start_ = std::chrono::steady_clock::now();
      }
      gyro_sum_[0] += w.x;
      gyro_sum_[1] += w.y;
      gyro_sum_[2] += w.z;
      gyro_sq_[0] += w.x * w.x;
      gyro_sq_[1] += w.y * w.y;
      gyro_sq_[2] += w.z * w.z;
      accel_sum_[0] += a.x;
      accel_sum_[1] += a.y;
      accel_sum_[2] += a.z;
      ++samples_;
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - calib_start_).count();
      if (samples_ >= 20 && elapsed >= calibration_seconds_) {
        finish_calibration();
      }
      return;
    }
    if (!calibrated_) {
      return;
    }

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    if (!have_stamp_) {
      last_stamp_ = stamp;
      have_stamp_ = true;
      return;
    }
    const double dt = stamp - last_stamp_;
    last_stamp_ = stamp;
    filter_.update(w.x - bias_[0], w.y - bias_[1], w.z - bias_[2], a.x, a.y, a.z, dt);

    nav_msgs::msg::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.orientation.w = filter_.q[0];
    odom.pose.pose.orientation.x = filter_.q[1];
    odom.pose.pose.orientation.y = filter_.q[2];
    odom.pose.pose.orientation.z = filter_.q[3];
    // Position is unused. Roll and pitch are observed; yaw is not.
    odom.pose.covariance[0] = 1e6;
    odom.pose.covariance[7] = 1e6;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 0.02;
    odom.pose.covariance[28] = 0.02;
    odom.pose.covariance[35] = 1.0;
    odom.twist.twist.angular.x = w.x - bias_[0];
    odom.twist.twist.angular.y = w.y - bias_[1];
    odom.twist.twist.angular.z = w.z - bias_[2];
    latest_ = odom;
    have_odom_ = true;
    if (publish_hz_ <= 0.0) {
      pub_->publish(odom);
    }
  }

  void publish_latest() {
    if (have_odom_) {
      latest_.header.stamp = now();
      pub_->publish(latest_);
    }
  }

  argus_mahoney::MahonyFilter filter_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  std::string child_frame_id_;
  double calibration_seconds_ = 10.0;
  double publish_hz_ = 50.0;
  double bias_[3] = {0.0, 0.0, 0.0};
  double gyro_sum_[3] = {0.0, 0.0, 0.0};
  double gyro_sq_[3] = {0.0, 0.0, 0.0};
  double accel_sum_[3] = {0.0, 0.0, 0.0};
  std::size_t samples_ = 0;
  bool calibrating_ = false;
  bool calibrated_ = false;
  bool have_stamp_ = false;
  bool have_odom_ = false;
  double last_stamp_ = 0.0;
  std::chrono::steady_clock::time_point calib_start_;
  nav_msgs::msg::Odometry latest_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MahoneyNode>());
  rclcpp::shutdown();
  return 0;
}
