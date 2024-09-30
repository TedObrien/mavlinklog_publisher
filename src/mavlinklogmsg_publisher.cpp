#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mavlinklog_publisher/msg/mavlink_log_msg.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("mavlinklogmsg_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<mavlinklog_publisher::msg::MavlinkLogMsg>("mavlink_log_msg", 10);
    timer_ = this->create_wall_timer(
      2s, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = mavlinklog_publisher::msg::MavlinkLogMsg();
    message.level = 2;
    message.id = "test-node";
    message.message = "Hello World!";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.message << "'");
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mavlinklog_publisher::msg::MavlinkLogMsg>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}