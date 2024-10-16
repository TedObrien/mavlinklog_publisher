cmake_minimum_required(VERSION 3.8)
project(mavlinklog_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LogMsg.msg"
)

add_executable(mavlinklog_publisher_node src/mavlinklog_publisher_node.cpp)
ament_target_dependencies(mavlinklog_publisher_node rclcpp px4_msgs)

add_executable(talker src/logmsg_publisher.cpp)
ament_target_dependencies(talker rclcpp )

# Following lines allow usage of custom message from within package
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mavlinklog_publisher_node  "${cpp_typesupport_target}") 
target_link_libraries(talker "${cpp_typesupport_target}") 


install(TARGETS
  talker
  mavlinklog_publisher_node
  DESTINATION lib/${PROJECT_NAME})


target_compile_features(mavlinklog_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(talker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
