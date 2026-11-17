// ArgusDebug: print argus_server's /argus_debug timing summary.
// In a terminal the summary redraws in place; otherwise each one is appended.
//
// ros2 run argus_server argus_debug

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <cstdio>
#include <string>
#include <unistd.h>

class ArgusDebug : public rclcpp::Node {
public:
  ArgusDebug()
  : rclcpp::Node("argus_debug"), tty_(isatty(STDOUT_FILENO) != 0) {
    sub_ = create_subscription<std_msgs::msg::String>(
      "/argus_debug", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) { show(msg->data); });
    if (tty_) {
      std::printf("\033[2J\033[HWaiting for /argus_debug from argus_server...\n");
      std::fflush(stdout);
    }
  }

private:
  void show(const std::string & text) {
    if (!tty_) {
      std::printf("%s\n", text.c_str());
      std::fflush(stdout);
      return;
    }
    // Clear to the end of each line so shorter lines leave no residue.
    std::string out = "\033[H";
    size_t start = 0;
    while (start < text.size()) {
      size_t end = text.find('\n', start);
      if (end == std::string::npos) end = text.size();
      out.append(text, start, end - start);
      out += "\033[K\n";
      start = end + 1;
    }
    out += "\033[J";
    std::fputs(out.c_str(), stdout);
    std::fflush(stdout);
  }

  bool tty_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArgusDebug>());
  rclcpp::shutdown();
  return 0;
}
