#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mavlinklog_publisher/msg/mavlink_log_msg.hpp"
#include "px4_msgs/msg/mavlink_log.hpp"

// Script needs to subscribe to the MavlinkLogMsg and publish to the MavlinkLog message

// add command line arg for starting message
// add command line arg for topic prefix


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("mavlinklog_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<px4_msgs::msg::MavlinkLog>("/fmu/in/mavlink_log", 10);
    // timer_ = this->create_wall_timer(
    //   2s, std::bind(&MinimalSubscriber::timer_callback, this));
    subscription_ = this->create_subscription<mavlinklog_publisher::msg::MavlinkLogMsg>(
    "mavlink_log_msg", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
  }
  

private:
  void topic_callback(const mavlinklog_publisher::msg::MavlinkLogMsg & msg)
  {

    // When message received, Publish to mavlink_log 
    auto message = px4_msgs::msg::MavlinkLog();
    message.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    message.severity = msg.level;

    // Convert revieved string to ascii struct
    std::string received_text = msg.message;

    // Combine id string to front of message string if populated
    if(msg.id.length() > 0){
      
      RCLCPP_INFO_STREAM(this->get_logger(), "Prepending id to string");
      received_text = "[" + msg.id + "] " + received_text;

    }

    // check if string > 127 characters and print truncation warning if so
    if (msg.message.length() > 126){

      RCLCPP_WARN_STREAM(this->get_logger(), "message truncated as greater than 127 characters");

    } 

    // Copy the string to the std::array, truncating if necessary, and ensure null-termination
    std::copy(received_text.begin(),
              received_text.begin() + std::min(received_text.size(), message.text.size() - 1),
              message.text.begin());

    // Ensure the last element is null-terminated
    message.text[std::min(received_text.size(), message.text.size() - 1)] = '\0';

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << received_text << "' with a severity of:" << unsigned(msg.level));

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
    message.id = "test_publisher";
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