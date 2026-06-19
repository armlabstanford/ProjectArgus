// file: bmi270_fast_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "bmi2_glue.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <thread>

// Linux I2C
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace bmi270 {
  // Interleaved accel+gyro burst: ax,ay,az,gx,gy,gz (12 bytes)
  constexpr uint8_t REG_DATA_START      = 0x0C;
}

class Bmi270FastNode : public rclcpp::Node {
public:
  Bmi270FastNode()
  : rclcpp::Node("bmi270_fast_node"),
    system_clock_(RCL_SYSTEM_TIME),
    stop_(false) {
    // ---- Parameters
    bus_path_  = declare_parameter<std::string>("i2c_bus_path", "/dev/i2c-1");
    addr_      = static_cast<uint8_t>(declare_parameter<int>("i2c_addr", 0x68));
    topic_     = declare_parameter<std::string>("topic", "/imu0");
    frame_id_  = declare_parameter<std::string>("frame_id", "imu");
    odr_hz_    = declare_parameter<double>("odr_hz", 200.0);           // IMU sampling target
    time_offset_ = declare_parameter<double>("time_offset", 0.0);      // add to IMU time (seconds)
    config_blob_path_ = declare_parameter<std::string>("config_bin_path", "");

    // ---- Open I2C device
    fd_ = ::open(bus_path_.c_str(), O_RDWR);
    if (fd_ < 0) throw std::runtime_error("Failed to open " + bus_path_);
    if (ioctl(fd_, I2C_SLAVE, addr_) < 0) throw std::runtime_error("I2C_SLAVE ioctl failed");

    // ---- Initialize BMI270 (via your glue API)
    init_bmi270();

    // ---- Scaling from ranges (match your glue config: ±2g, ±1000 dps)
    constexpr double kG  = 9.80665;
    constexpr double kPi = 3.14159265358979323846;
    acc_scale_ = (2.0 * kG) / 32768.0;                 // ±2g
    gyr_scale_ = (1000.0 * kPi / 180.0) / 32768.0;     // ±1000 dps -> rad/s

    // ---- Publisher (SensorData QoS)
    rclcpp::SensorDataQoS qos;
    qos.keep_last(20);
    qos.best_effort();
    qos.durability_volatile();
    pub_ = create_publisher<sensor_msgs::msg::Imu>(topic_, qos);

    // ---- Pre-fill constant parts
    base_msg_.header.frame_id = frame_id_;
    base_msg_.orientation.w = 1.0;
    base_msg_.orientation.x = 0.0;
    base_msg_.orientation.y = 0.0;
    base_msg_.orientation.z = 0.0;
    base_msg_.orientation_covariance = {
      -1.0, 0.0, 0.0,
       0.0,-1.0, 0.0,
       0.0, 0.0,-1.0
    };
    base_msg_.angular_velocity_covariance = {
      1.7e-6, 0.0, 0.0,
      0.0, 1.7e-6, 0.0,
      0.0, 0.0, 1.7e-6
    };
    base_msg_.linear_acceleration_covariance = {
      1.6e-5, 0.0, 0.0,
      0.0, 1.6e-5, 0.0,
      0.0, 0.0, 1.6e-5
    };

    // ---- Start read thread (publish on acquisition)
    read_thread_ = std::thread([this]{ readLoop(); });

    RCLCPP_INFO(get_logger(),
      "BMI270 fast node on %s addr=0x%02X → %s | ODR=%.0f Hz | frame_id=%s | time_offset=%.6f s",
      bus_path_.c_str(), addr_, topic_.c_str(), odr_hz_, frame_id_.c_str(), time_offset_);
  }

  ~Bmi270FastNode() override {
    stop_ = true;
    if (read_thread_.joinable()) read_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  // ---- BMI270 init (through your helper API)
  void init_bmi270() {
    if (!bmi270_api_init_with_fd(fd_, addr_)) {
      throw std::runtime_error("BMI270 Sensor API init failed (config not loaded).");
    }
    if (!bmi270_api_set_acc_gyr_200hz(fd_, addr_)) {
      throw std::runtime_error("BMI270 set ODR to 200Hz failed");
    }
    usleep(5000);
  }

  // ---- I2C helpers
  void i2cReadBlock(uint8_t reg, uint8_t* out, size_t len) {
    struct i2c_rdwr_ioctl_data rdwr{};
    struct i2c_msg msgs[2]{};

    msgs[0].addr  = addr_;
    msgs[0].flags = 0;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len   = static_cast<__u16>(len);
    msgs[1].buf   = out;

    rdwr.msgs  = msgs;
    rdwr.nmsgs = 2;

    if (ioctl(fd_, I2C_RDWR, &rdwr) < 0)
      throw std::runtime_error("i2c rdwr read block failed");
  }

  // ---- Read loop: acquire + stamp + publish immediately
  void readLoop() {
    using steady = std::chrono::steady_clock;
    const auto period_d = std::chrono::duration<double>(1.0 / std::max(1.0, odr_hz_));
    const auto period   = std::chrono::duration_cast<steady::duration>(period_d);
    auto next_t = steady::now();

    // warm-up
    for (int i = 0; i < 3 && !stop_; ++i) {
      uint8_t tmp[12]; i2cReadBlock(bmi270::REG_DATA_START, tmp, 12);
    }

    while (!stop_) {
      const double t1 = system_clock_.now().seconds();
      uint8_t raw[12];
      i2cReadBlock(bmi270::REG_DATA_START, raw, 12);
      const double t2 = system_clock_.now().seconds();
      const double t_mid = (t1 + t2) * 0.5 + time_offset_;

      // unpack
      const int16_t ax = static_cast<int16_t>((raw[1]  << 8) | raw[0]);
      const int16_t ay = static_cast<int16_t>((raw[3]  << 8) | raw[2]);
      const int16_t az = static_cast<int16_t>((raw[5]  << 8) | raw[4]);
      const int16_t gx = static_cast<int16_t>((raw[7]  << 8) | raw[6]);
      const int16_t gy = static_cast<int16_t>((raw[9]  << 8) | raw[8]);
      const int16_t gz = static_cast<int16_t>((raw[11] << 8) | raw[10]);

      // stamp
      const int64_t sec  = static_cast<int64_t>(std::floor(t_mid));
      const uint32_t nsec = static_cast<uint32_t>((t_mid - static_cast<double>(sec)) * 1e9);
      const rclcpp::Time t_ros(sec, nsec, RCL_SYSTEM_TIME);

      // publish (copy prefilled covariances & frame_id)
      sensor_msgs::msg::Imu msg = base_msg_;
      msg.header.stamp = t_ros;

      msg.linear_acceleration.x = ax * acc_scale_;
      msg.linear_acceleration.y = ay * acc_scale_;
      msg.linear_acceleration.z = az * acc_scale_;
      msg.angular_velocity.x    = gx * gyr_scale_;
      msg.angular_velocity.y    = gy * gyr_scale_;
      msg.angular_velocity.z    = gz * gyr_scale_;

      pub_->publish(msg);

      next_t += period;
      std::this_thread::sleep_until(next_t);
    }
  }

  // Params/state
  std::string bus_path_, topic_, frame_id_, config_blob_path_;
  uint8_t addr_{0x68};
  double odr_hz_{200.0};
  double time_offset_{0.0};

  int fd_{-1};
  double acc_scale_{}, gyr_scale_{};

  rclcpp::Clock system_clock_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  sensor_msgs::msg::Imu base_msg_;

  std::thread read_thread_;
  std::atomic<bool> stop_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<Bmi270FastNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
