#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mavlinklog_publisher/msg/mavlink_log_msg.hpp"
#include "px4_msgs/msg/mavlink_log.hpp"

// Script needs to subscribe to the MavlinkLogMsg and publish to the MavlinkLog message

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("mavlinklog_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<px4_msgs::msg::MavlinkLog>("mavlink_log", 10);
    // timer_ = this->create_wall_timer(
    //   2s, std::bind(&MinimalSubscriber::timer_callback, this));
    subscription_ = this->create_subscription<mavlinklog_publisher::msg::MavlinkLogMsg>(
    "mavlink_log_msg", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    

  }

private:
  void topic_callback(const mavlinklog_publisher::msg::MavlinkLogMsg & msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.message << "' from " << msg.name);

    // Convert message string to struct with ascii codes
    // break up string into struct with each letter.

    // loop through struct and convert to ascii letter

    
    // Publish to mavlink_log 
    auto message = px4_msgs::msg::MavlinkLog();
    message.timestamp = 100000000; // find how to create timestamp
    // message.text = [1,1,1,1,1,1]; // find example hex array
    message.severity = 1;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.message << "'"); //uncomment once message working
    publisher_->publish(message);
    
  }
  
  // Correct type for publisher_ member variable
  rclcpp::Publisher<px4_msgs::msg::MavlinkLog>::SharedPtr publisher_;
  // rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

  void timer_callback()
  {
    auto message = mavlinklog_publisher::msg::MavlinkLogMsg();
    message.level = 5;
    message.name = "test_publisher";
    message.message = "Hello World";
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.message << "'");
    // publisher_->publish(message);
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