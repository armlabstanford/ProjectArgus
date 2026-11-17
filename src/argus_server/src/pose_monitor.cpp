// Live terminal view of the per-module pose streams arriving on this machine.
// For each module, over a sliding window:
//   rate     messages per second
//   max gap  longest time between two arrivals
//   age      time since the last arrival
//   delay    median (receive time - header stamp); only meaningful when the
//            Pi and this machine share a clock (NTP)
//
// ros2 run argus_server pose_monitor
// ros2 run argus_server pose_monitor --ros-args -p modules:="['argus0','argus4']"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <unistd.h>
#include <vector>

using Clock = std::chrono::steady_clock;

namespace {
const char * kGreen = "\033[32m";
const char * kYellow = "\033[33m";
const char * kRed = "\033[31m";
const char * kDim = "\033[2m";
const char * kReset = "\033[0m";
}

struct Channel {
  std::string module;
  std::string topic;
  std::deque<Clock::time_point> arrivals;
  std::deque<double> delays_s;
  Clock::time_point last{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
};

class PoseMonitor : public rclcpp::Node {
public:
  PoseMonitor()
  : rclcpp::Node("pose_monitor"), tty_(isatty(STDOUT_FILENO) != 0) {
    const auto modules = declare_parameter<std::vector<std::string>>(
      "modules", {"argus0", "argus1", "argus2", "argus3", "argus4"});
    suffix_ = declare_parameter<std::string>("topic_suffix", "odomimu_throttled");
    window_s_ = declare_parameter<double>("window_s", 2.0);
    expected_hz_ = declare_parameter<double>("expected_hz", 50.0);

    channels_.reserve(modules.size());
    for (const auto & module : modules) {
      channels_.push_back(Channel{module, "/" + module + "/" + suffix_, {}, {}, {}, nullptr});
    }
    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    for (size_t i = 0; i < channels_.size(); ++i) {
      channels_[i].sub = create_subscription<nav_msgs::msg::Odometry>(
        channels_[i].topic, qos,
        [this, i](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_message(i, *msg); });
    }

    timer_ = create_wall_timer(std::chrono::milliseconds(250), [this]() { draw(); });
    if (tty_) {
      std::printf("\033[2J");
    }
  }

private:
  void on_message(size_t i, const nav_msgs::msg::Odometry & msg) {
    auto & ch = channels_[i];
    const auto now = Clock::now();
    ch.arrivals.push_back(now);
    ch.last = now;
    // NaN when the publisher left the stamp unset.
    double delay = std::nan("");
    if (msg.header.stamp.sec != 0 || msg.header.stamp.nanosec != 0) {
      delay = (get_clock()->now() - rclcpp::Time(msg.header.stamp, RCL_ROS_TIME)).seconds();
    }
    ch.delays_s.push_back(delay);
  }

  void trim(Channel & ch, Clock::time_point now) const {
    const auto window = std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(window_s_));
    while (!ch.arrivals.empty() && now - ch.arrivals.front() > window) {
      ch.arrivals.pop_front();
      ch.delays_s.pop_front();
    }
  }

  void draw() {
    const auto now = Clock::now();
    std::string out;
    char line[256];
    // In a terminal, redraw in place and clear each line's leftover characters.
    const char * eol = tty_ ? "\033[K\n" : "\n";
    if (tty_) {
      out += "\033[H";
    }
    std::snprintf(line, sizeof(line),
      "Pose channels /<module>/%s   window %.1f s   expected %.0f Hz   (Ctrl-C to quit)%s%s",
      suffix_.c_str(), window_s_, expected_hz_, eol, eol);
    out += line;
    std::snprintf(line, sizeof(line), "%-8s %8s %10s %10s %10s   %s%s",
      "module", "rate", "max gap", "age", "delay", "status", eol);
    out += line;

    for (auto & ch : channels_) {
      trim(ch, now);
      const size_t n = ch.arrivals.size();
      const double rate = n / window_s_;
      double max_gap_ms = 0.0;
      for (size_t k = 1; k < n; ++k) {
        max_gap_ms = std::max(max_gap_ms,
          std::chrono::duration<double, std::milli>(ch.arrivals[k] - ch.arrivals[k - 1]).count());
      }
      const bool seen = ch.last != Clock::time_point{};
      const double age_ms = seen
        ? std::chrono::duration<double, std::milli>(now - ch.last).count() : -1.0;
      std::vector<double> d;
      for (double v : ch.delays_s) {
        if (!std::isnan(v)) d.push_back(v);
      }
      double delay_ms = 0.0;
      if (!d.empty()) {
        std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
        delay_ms = d[d.size() / 2] * 1000.0;
      }

      const char * color = kGreen;
      const char * status = "OK";
      if (!seen) {
        color = kDim; status = "waiting";
      } else if (age_ms > 1000.0) {
        color = kRed; status = "STALE";
      } else if (rate < 0.8 * expected_hz_ || max_gap_ms > 250.0) {
        color = kYellow; status = "SLOW";
      }

      char age_txt[32], gap_txt[32], delay_txt[32];
      std::snprintf(age_txt, sizeof(age_txt), seen ? "%.0f ms" : "-", age_ms);
      std::snprintf(gap_txt, sizeof(gap_txt), n > 1 ? "%.0f ms" : "-", max_gap_ms);
      std::snprintf(delay_txt, sizeof(delay_txt), d.empty() ? "-" : "%.0f ms", delay_ms);
      std::snprintf(line, sizeof(line), "%s%-8s %5.1f Hz %10s %10s %10s   %s%s%s",
        tty_ ? color : "", ch.module.c_str(), rate, gap_txt, age_txt, delay_txt, status,
        tty_ ? kReset : "", eol);
      out += line;
    }
    if (!tty_) {
      out += "\n";
    }
    std::fputs(out.c_str(), stdout);
    std::fflush(stdout);
  }

  bool tty_;
  std::string suffix_;
  double window_s_{2.0};
  double expected_hz_{50.0};
  std::vector<Channel> channels_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseMonitor>());
  rclcpp::shutdown();
  return 0;
}
