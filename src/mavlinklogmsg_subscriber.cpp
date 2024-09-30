#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mavlinklog_publisher/msg/mavlink_log_msg.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("mavlinklog_subscriber")
  {
    subscription_ = this->create_subscription<mavlinklog_publisher::msg::MavlinkLogMsg>(
      "mavlink_log_msg", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const mavlinklog_publisher::msg::MavlinkLogMsg & msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.message << "' from " << msg.id);
  }
  rclcpp::Subscription<mavlinklog_publisher::msg::MavlinkLogMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}