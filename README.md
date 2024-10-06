# mavlinklog_publisher


To publish from the command line:
```
ros2 topic pub --once /mavlink_log_msg mavlinklog_publisher/msg/MavlinkLogMsg '{level: 5, message: "Hello World!"}'
```