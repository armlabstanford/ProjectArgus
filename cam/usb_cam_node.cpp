// file: usb_cam_node.cpp
#include <chrono>
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;

class UsbCamNode : public rclcpp::Node {
public:
  UsbCamNode() : Node("usb_cam_cpp"), system_clock_(RCL_SYSTEM_TIME) {
    // ---- Parameters
    device_       = declare_parameter<std::string>("device", "/dev/video0");
    width_        = declare_parameter<int>("width", 640);
    height_       = declare_parameter<int>("height", 480);
    fourcc_str_   = declare_parameter<std::string>("fourcc", "MJPG"); // MJPG, YUYV, Y800
    open_fps_     = declare_parameter<double>("open_fps", 100.0);     // request at camera
    target_fps_   = declare_parameter<double>("target_fps", 20.0);    // publish rate
    grayscale_    = declare_parameter<bool>("grayscale", true);       // mono8 vs bgr8
    frame_id_     = declare_parameter<std::string>("camera_frame_id", "cam0");

    // Time alignment params
    timeshift_cam_imu_      = declare_parameter<double>("timeshift_cam_imu", 0.0); // added to cam stamp
    exposure_seconds_       = declare_parameter<double>("exposure_seconds", 0.0);  // if known
    use_mid_exposure_stamp_ = declare_parameter<bool>("use_mid_exposure_stamp", false);
    buffers_                = declare_parameter<int>("buffers", 1);                // small buffer to cut latency

    // ---- OpenCV capture
    cap_.open(device_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(get_logger(), "Failed to open camera: %s", device_.c_str());
      throw std::runtime_error("VideoCapture open failed");
    }

    // Request format
    const int fourcc = cv::VideoWriter::fourcc(
      fourcc_str_.size()>0?fourcc_str_[0]:'M',
      fourcc_str_.size()>1?fourcc_str_[1]:'J',
      fourcc_str_.size()>2?fourcc_str_[2]:'P',
      fourcc_str_.size()>3?fourcc_str_[3]:'G'
    );
    cap_.set(cv::CAP_PROP_FOURCC, fourcc);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, open_fps_);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, buffers_);

    // Log actual settings
    const double aw = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    const double ah = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    const double afps = cap_.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(get_logger(),
      "Opened %s as %.0fx%.0f %s, camera ~%.1f fps, publishing at %.1f Hz | buffers=%d | Δt(cam->imu)=%.6f s",
      device_.c_str(), aw, ah, fourcc_str_.c_str(), afps, target_fps_, buffers_, timeshift_cam_imu_);

    // Publisher (sensor-data friendly)
    rclcpp::SensorDataQoS qos;
    qos.keep_last(10);
    qos.best_effort();
    qos.durability_volatile();
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("cam0/image_raw", qos);

    // Timer for throttled publish
    auto period = std::chrono::duration<double>(1.0 / std::max(0.1, target_fps_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UsbCamNode::tick, this)
    );
  }

  ~UsbCamNode() override {
    cap_.release();
  }

private:
  void tick() {
    // Stamp as close to dequeue as possible: midpoint around grab()
    const rclcpp::Time t_before = system_clock_.now();
    if (!cap_.grab()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to grab frame");
      return;
    }
    const rclcpp::Time t_after = system_clock_.now();

    rclcpp::Time t_stamp(
      (t_before.nanoseconds() + t_after.nanoseconds()) / 2, RCL_SYSTEM_TIME);

    cv::Mat frame;
    if (!cap_.retrieve(frame)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to retrieve frame");
      return;
    }

    // Mid-exposure correction (optional) + Kalibr shift (once)
    if (use_mid_exposure_stamp_ && exposure_seconds_ > 0.0) {
      t_stamp = t_stamp + rclcpp::Duration::from_seconds(exposure_seconds_ * 0.5);
    }
    if (timeshift_cam_imu_ != 0.0) {
      t_stamp = t_stamp + rclcpp::Duration::from_seconds(timeshift_cam_imu_);
    }

    // Convert to desired encoding
    sensor_msgs::msg::Image::SharedPtr msg;
    if (grayscale_) {
      if (frame.channels() == 3)       cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
      else if (frame.channels() == 4)  cv::cvtColor(frame, frame, cv::COLOR_BGRA2GRAY);
      msg = cv_bridge::CvImage({}, "mono8", frame).toImageMsg();
    } else {
      if (frame.channels() == 1)       cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
      else if (frame.channels() == 4)  cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
      msg = cv_bridge::CvImage({}, "bgr8", frame).toImageMsg();
    }

    msg->header.stamp = t_stamp;
    msg->header.frame_id = frame_id_;
    image_pub_->publish(*msg);
  }

  // Params
  std::string device_, fourcc_str_, frame_id_;
  int width_{640}, height_{480}, buffers_{1};
  double open_fps_{100.0}, target_fps_{20.0};
  bool grayscale_{true};
  double timeshift_cam_imu_{0.0};
  double exposure_seconds_{0.0};
  bool   use_mid_exposure_stamp_{false};

  // State
  cv::VideoCapture cap_;
  rclcpp::Clock system_clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<UsbCamNode>());
  rclcpp::shutdown();
  return 0;
}
