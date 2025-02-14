# mavlinklog_publisher

A ROS2 node intended for the publishing of human readable messages from a companion computer to QGroundControl via a flight controller running PX4. The node subscribes to the [`LogMsg`](https://github.com/TedObrien/mavlinklog_publisher/blob/main/msg/LogMsg.msg) defined in this package and published a [`MavlinkLog`](https://github.com/PX4/px4_msgs/blob/main/msg/MavlinkLog.msg) message to the flight controller via the [XRCE-DDS bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html). This functionality enables ROS2 nodes on the companion computer to display messages in QGroundControl, which is especially useful when operating in [offboard mode](https://docs.px4.io/main/en/flight_modes/offboard.html).


![Screenshot from 2024-10-17 13-34-20](https://github.com/user-attachments/assets/c8777135-d031-45c0-a41f-0fd3fdcd7339)

## Build/Run

The mavlinklog_publisher package is built and run like any other ROS2 package.
```
source /opt/ros/humble/setup.bash 
cd ~/ros2_ws/
colcon build --packages-select mavlinklog_publisher
source install/local_setup.bash
ros2 run mavlinklog_publisher mavlinklog_publisher_node
```


### Command-Line Arguments

| Argument          | Description                                      | Default Value   | Required? |
| ---------------   | ------------------------------------------------ | --------------- | --------- |
| `uav_name`        | Prepends `uav_name` to ROS topics. Useful if using a topic namespace.              | `None`     | No       |
| `message_on_start`| Prints message to QGC when node is started                          | `True`    | No        |



> **NOTE**
>See more in depth instructions on how to build and run below.

## Testing in SITL

### Pre-requisites
These instructions assume you have already installed and tested the following:

- [PX4 toolchain](https://docs.px4.io/main/en/dev_setup/dev_env.html)
- [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [uxrce_dds bridge](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [ROS2 humble](https://docs.ros.org/en/humble/Installation.html)
- [QGroundControl](https://qgroundcontrol.com/downloads/)

> **NOTE**
>  Exact commands may vary depending on the PX4 and gazebo versions you are using.

1. If you have not done so already, clone and build the `px4_msgs` and `Micro-XRCE-DDS-Agent` packages.

      ```
      cd ~/ros2_ws/src/
      git clone https://github.com/PX4/px4_msgs.git
      git clone -b ros2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
      cd ~/ros2_ws/
      colcon build --packages-select px4_msgs microxrcedds_agent
      ```
      > **NOTE**
      > Ensure you have checked out the branch corresponding to the version of PX4 you are using. You must already have ROS2 Humble installed.

2. Clone and build `mavlink_log_publisher` repo
      ```
      git clone https://github.com/TedObrien/mavlinklog_publisher.git
      cd ~/ros2_ws/
      source install/local_setup.bash 
      colcon build --packages-select mavlinklog_publisher
      ```


3. Update PX4 `dds_topics.yaml` file  to add the `mavlink_log` uorb message to the list of subscriptions

    ```
    nano ~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
    ```
    Add the following 2 lines under `subscriptions:`
    ```
    subscriptions:

      - topic: /fmu/in/mavlink_log
        type: px4_msgs::msg::MavlinkLog

    ```
     >**NOTE**
     > More detailed instruction on updating this yaml file can be found [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html#dds-topics-yaml)


4. Launch px4 sitl gazebo simulation. In this example a [topic namespace](https://docs.px4.io/main/en/middleware/uxrce_dds.html#customizing-the-topic-namespace) is being used.

    ```
    cd ~/PX4-Autopilot/
    PX4_UXRCE_DDS_NS=uav1 make px4_sitl gz_x500 
    ```
    >**NOTE**
    > you will need to `make clean` first if you previously built the target.

5. Start uXRCE-DDS bridge

    ```
    source ~/ros2_ws/install/local_setup.bash
    MicroXRCEAgent udp4 -p 8888
    ```
    Confirm mavlink_log ROS2 has been created with `ros2 topic list`. Look for the output.

    ```
    /uav1/fmu/in/mavlink_log
    ```

6. Open QgroundControl

7. Start mavlinklog_publisher

    `uav_id` parameter needs to be specified as as a topic namespace was used when launching px4_sitl

    ```
    ros2 run mavlinklog_publisher mavlinklog_publisher_node --ros-args -p uav_name:=uav1
    ```
    Confirm the `log_msg` topic has been created with `ros2 topic list`

    ```
    /uav1/log_msg
    ```

    Publish a message from the command line to QGroundControl:
    ```
    ros2 topic pub --once /uav1/log_msg mavlinklog_publisher/msg/LogMsg '{level: 1, message: "Hello World!"}'
    ```
    Confirm the message has been displayed by QGroundControl


![Screenshot from 2024-10-17 13-26-35](https://github.com/user-attachments/assets/4df62a5a-d4e9-45c8-b701-9b48e8c61f33)

## Integrating into other ROS nodes

The script `/src/logmsg_publisher.cpp` is an example of how to publish to the `mavlinklog_publisher_node` from other nodes. 

To run:

```
ros2 run mavlinklog_publisher talker
```

The severity of the message sent to QGC can can be modified. See the [`LogMsg` definition](https://github.com/TedObrien/mavlinklog_publisher/blob/main/msg/LogMsg.msg)