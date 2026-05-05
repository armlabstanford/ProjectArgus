#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

class FakeOdomNode : public rclcpp::Node
{
public:
  FakeOdomNode()
  : Node("fake_odom_node"), t_(0.0)
  {
    thorax_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/thorax/odomimu", 10);
    humerus_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/humerus_l/odomimu", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),  // 50 Hz
      std::bind(&FakeOdomNode::publish_msgs, this));

    start_time_ = this->get_clock()->now();
  }

private:
  void publish_msgs()
  {
    auto now = this->get_clock()->now();

    // ---- Thorax: identity quaternion ----
    nav_msgs::msg::Odometry thorax_msg;
    thorax_msg.header.stamp = now;
    thorax_msg.header.frame_id = "world";

    thorax_msg.pose.pose.orientation.w = 1.0;
    thorax_msg.pose.pose.orientation.x = 0.0;
    thorax_msg.pose.pose.orientation.y = 0.0;
    thorax_msg.pose.pose.orientation.z = 0.0;

    thorax_pub_->publish(thorax_msg);

    // ---- Humerus: sinusoidal rotation about X ----
    double elapsed = (now - start_time_).seconds();

    double freq = 0.25; // Hz (one full cycle every 4 seconds)
    double angle_max = 2.0 * M_PI / 3.0; // 120 degrees in radians

    // sinusoid from 0 → angle_max
    double angle = 0.5 * angle_max * (1.0 + std::sin(2.0 * M_PI * freq * elapsed));

    // quaternion for rotation about x-axis
    double half = angle / 2.0;
    double qw = std::cos(half);
    double qx = std::sin(half);

    nav_msgs::msg::Odometry humerus_msg;
    humerus_msg.header.stamp = now;
    humerus_msg.header.frame_id = "world";

    humerus_msg.pose.pose.orientation.w = qw;
    humerus_msg.pose.pose.orientation.x = qx;
    humerus_msg.pose.pose.orientation.y = 0.0;
    humerus_msg.pose.pose.orientation.z = 0.0;

    humerus_pub_->publish(humerus_msg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr thorax_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr humerus_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  double t_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeOdomNode>());
  rclcpp::shutdown();
  return 0;
}