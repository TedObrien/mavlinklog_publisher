#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "mavlinklog_publisher/msg/mavlink_log_msg.hpp"
#include "px4_msgs/msg/mavlink_log.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

// TODO
// Best practise on variable,class names ect

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("mavlinklog_publisher"), count_(0)
  {
    this->declare_parameter("uav_name", ""); 
    this->declare_parameter("message_on_start", true);

    ran_ = false;

    px4_publisher_ = this->create_publisher<px4_msgs::msg::MavlinkLog>(
      this->get_parameter("uav_name").as_string() + "/fmu/in/mavlink_log", 10);

    log_publisher_ = this->create_publisher<mavlinklog_publisher::msg::MavlinkLogMsg>(
      this->get_parameter("uav_name").as_string() + "mavlink_log_msg", 10);

    timer_ = this->create_wall_timer(
      5s, std::bind(&MinimalSubscriber::timer_callback, this));

    subscription_ = this->create_subscription<mavlinklog_publisher::msg::MavlinkLogMsg>(
      "mavlink_log_msg", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
  }
  

private:
  void topic_callback(const mavlinklog_publisher::msg::MavlinkLogMsg & msg)
  {

    auto message = px4_msgs::msg::MavlinkLog();

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


    // Populate mavlink_log message
    message.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    message.severity = msg.level;

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << received_text << "' with a severity of:" << unsigned(msg.level));

    px4_publisher_->publish(message); 
    
  }
  
  
  void timer_callback()
  {

    if (!ran_ &&  this->get_parameter("message_on_start").as_bool()){

      auto message = mavlinklog_publisher::msg::MavlinkLogMsg();
      message.level = 5;
      message.message = "MavlinkLog Publisher started ";
      log_publisher_->publish(message);
      ran_ = true;
    }
    
  }
  
  bool ran_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::MavlinkLog>::SharedPtr px4_publisher_;
  rclcpp::Publisher<mavlinklog_publisher::msg::MavlinkLogMsg>::SharedPtr log_publisher_;
  rclcpp::Subscription<mavlinklog_publisher::msg::MavlinkLogMsg>::SharedPtr subscription_;
  size_t count_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}