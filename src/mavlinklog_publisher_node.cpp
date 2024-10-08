#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mavlinklog_publisher/msg/log_msg.hpp"
#include "px4_msgs/msg/mavlink_log.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class MavlinkLogPublisher : public rclcpp::Node
{
public:
  MavlinkLogPublisher()
  : Node("mavlinklog_publisher"), count_(0)
  {
    this->declare_parameter("uav_name", ""); // If ROS topic namespace is used. Empty by default
    this->declare_parameter("message_on_start", true); // Send startup message. True by default
    ran_ = false;

    px4_publisher_ = this->create_publisher<px4_msgs::msg::MavlinkLog>(
      this->get_parameter("uav_name").as_string() + "/fmu/in/mavlink_log", 10);

    timer_ = this->create_wall_timer(
      5s, std::bind(&MavlinkLogPublisher::TimerCallback, this));

    subscription_ = this->create_subscription<mavlinklog_publisher::msg::LogMsg>(
      this->get_parameter("uav_name").as_string() + "log_msg", 10, std::bind(&MavlinkLogPublisher::LogMsgCallback, this, _1));
    
  }
  

private:

  void LogMsgCallback(const mavlinklog_publisher::msg::LogMsg & received_message)
  {

    // If LogMsg received publish MavlinkLog
    PublishToPX4(received_message);
    
  }

  void TimerCallback()
  {

    // Send start message
    if (!ran_ &&  this->get_parameter("message_on_start").as_bool()){

      auto message = mavlinklog_publisher::msg::LogMsg();
      message.level = 5;
      message.message = "MavlinkLog Publisher started ";
      PublishToPX4(message);
      ran_ = true;

    }
    
  }
  
  void PublishToPX4(mavlinklog_publisher::msg::LogMsg msg){

    auto message = px4_msgs::msg::MavlinkLog();

    std::string received_text = msg.message;

    // prepend id string to front of message string if populated
    if(msg.id.length() > 0){
      
      received_text = "[" + msg.id + "] " + received_text;

    }

    // check if string > 127 characters and print truncation warning if so
    if (msg.message.length() > 126){

      RCLCPP_WARN_STREAM(this->get_logger(), "message truncated as greater than 127 characters");

    } 

    // Convert revieved string to array
    std::copy(received_text.begin(),
              received_text.begin() + std::min(received_text.size(), message.text.size() - 1),
              message.text.begin());

    // Ensure the last element is null-terminated
    message.text[std::min(received_text.size(), message.text.size() - 1)] = '\0';

    // Populate mavlink_log message
    message.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    message.severity = msg.level;

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << received_text << "' with a severity of: " << unsigned(msg.level));

    px4_publisher_->publish(message); 
    
  }

  bool ran_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::MavlinkLog>::SharedPtr px4_publisher_;
  rclcpp::Subscription<mavlinklog_publisher::msg::LogMsg>::SharedPtr subscription_;
  size_t count_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MavlinkLogPublisher>());
  rclcpp::shutdown();
  return 0;
}