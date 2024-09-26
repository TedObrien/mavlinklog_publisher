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
    publisher_ = this->create_publisher<px4_msgs::msg::MavlinkLog>("/fmu/in/mavlink_log", 10);
    // timer_ = this->create_wall_timer(
    //   2s, std::bind(&MinimalSubscriber::timer_callback, this));
    subscription_ = this->create_subscription<mavlinklog_publisher::msg::MavlinkLogMsg>(
    "mavlink_log_msg", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
  }
  
   // Function to convert a string into an array of ASCII codes
    // void stringToAsciiArray(const std::string& str, unsigned char asciiArray[127]) {
    //     // Clear the array
    //     std::memset(asciiArray, 0, 127);
        
    //     // Copy ASCII values to the array, but no more than 126 (127th is for null termination)
    //     size_t length = str.size() > 126 ? 126 : str.size();
    //     for (size_t i = 0; i < length; ++i) {
    //         asciiArray[i] = static_cast<unsigned char>(str[i]);  // Store ASCII code
    //     }
        
    //     // Null-terminate at position 126, just to be safe
    //     asciiArray[126] = '\0';
    // }

    // void demo() {
    //     std::string input = "Hello";
    //     unsigned char asciiArray[127];

    //     // Call the function from within the class
    //     stringToAsciiArray(input, asciiArray);

    //     // Print the ASCII values
    //     std::cout << "ASCII codes: ";
    //     for (int i = 0; i < 126 && asciiArray[i] != '\0'; ++i) {
    //         std::cout << static_cast<int>(asciiArray[i]) << " ";
    //     }
    //     std::cout << std::endl;
    // }

private:
  void topic_callback(const mavlinklog_publisher::msg::MavlinkLogMsg & msg)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.message << "' from " << msg.name);

    // demo();

    // unsigned char asciiArray[127];

    // Call the function from within the class
    // stringToAsciiArray(msg.message, asciiArray);

    // Publish to mavlink_log 
    auto message = px4_msgs::msg::MavlinkLog();
    // message.timestamp = 100000000; // find how to create timestamp
    message.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Example: text received from another ROS 2 topic as a std::string
    std::string received_text = msg.message;

    // Copy the string to the std::array, truncating if necessary, and ensure null-termination
    std::copy(received_text.begin(),
              received_text.begin() + std::min(received_text.size(), message.text.size() - 1),
              message.text.begin());

    // Ensure the last element is null-terminated
    message.text[std::min(received_text.size(), message.text.size() - 1)] = '\0';

    // Print the message details
    std::cout << "Text: ";
    for (char c : message.text) {
        std::cout << c;
        if (c == '\0') break;  // Stop at the null-terminator
    }
    std::cout << "\nSeverity: " << static_cast<int>(message.severity) << "\n";



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