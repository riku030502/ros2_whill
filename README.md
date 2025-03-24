# whill

## About

The "whill" is a ROS2 package for WHILL Model CR2. <br>

<img width=22% title="WHILL Model CR2" src="https://github.com/WHILL/Model_CR_Technical_Support/assets/129816934/387a8aac-3808-4727-895d-9857059ee342">
<img width=24% title="Wheeled Robot Base" src="https://github.com/WHILL/Model_CR_Technical_Support/assets/129816934/6a63ed45-9e0c-40ca-b71c-8dff614af141">

We also have [Model CR Series Technical Support](https://github.com/WHILL/Model_CR_Technical_Support) for current and potential Model CR series users. <br>

**Attention:** <br>
This package on `humble` branch only provides common functions for all models. <br>
If you want to use features that are only available in WHILL Model CR, you will need to use `crystal-devel` branch.


## Requirements

- Host device
  - Linux OS: Ubuntu 22.04
  - ROS2: Humble Hawksbill (humble)
- Target device
  - WHILL Model CR2 (Note that Model C2 does not have any serial ports)


## Getting started

1. Download projects
2. Configure your serial port
3. Build project
4. Launch <br>
  4-(A). With the bringup package <br>
  4-(B). With 'run' command <br>


### 1. Download projects

```sh
cd ~/<your_ros2_ws>/src
git clone https://github.com/whill-labs/ros2_whill_interfaces.git
git clone https://github.com/whill-labs/ros2_whill.git
```


### 2. Configure your serial port

If your serial port is not `/dev/ttyUSB0`, please edit `port_name` in the following file:

```
~/<your_ros2_ws>/src/ros2_whill/whill_bringup/config/params.yaml
```


### 3. Build project

```sh
colcon build --packages-up-to whill
. install/setup.bash
```


### 4. Launch

#### (A) With the bringup package

```sh
ros2 launch whill_bringup whill_launch.py
```


#### (B) With 'run' command

```sh
ros2 run whill_driver whill
```

**Note:** If your serial port is not `/dev/ttyUSB0`, for example `/dev/ttyUSB1`, please run the following command instead of the one above:
```sh
ros2 run whill_driver whill --ros-args -p port_name:=/dev/ttyUSB1
```


## ROS2 APIs

### Publish

| Topic name | Message type | Work |
|:---|:---|:---|
| /whill/states/model_cr2  | [whill_msgs/ModelCr2State](https://github.com/WHILL/ros2_whill_interfaces/blob/humble/whill_msgs/msg/ModelCr2State.msg)  | Notify WHILL states |


### Subscription

| Topic name | Message type | Work |
|:---|:---|:---|
| /whill/controller/joy | [sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html) | Give WHILL a virtual joystick input |
| /whill/controller/cmd_vel | [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) | Give WHILL a direct velocity input |


### Service

| Service name | Message type | Work |
|:---|:---|:---|
| /whill/set_power_srv | [whill_msgs/SetPower](https://github.com/WHILL/ros2_whill_interfaces/blob/humble/whill_msgs/srv/SetPower.srv) | Turn WHILL power on/off |
| /whill/set_speed_profile_srv | [whill_msgs/SetSpeedProfile](https://github.com/WHILL/ros2_whill_interfaces/blob/humble/whill_msgs/srv/SetSpeedProfile.srv) | Change the speed profile of WHILL |


### *Command examples*

##### Power on WHILL

```sh
ros2 service call /whill/set_power_srv whill_msgs/SetPower '{p0: 1}'
```


##### Power off WHILL

```sh
ros2 service call /whill/set_power_srv whill_msgs/SetPower '{p0: 0}'
```


##### Move WHILL forward with virtual joystick inputs

```sh
ros2 topic pub -r 12 /whill/controller/joy sensor_msgs/Joy "{axes:[0, 0.25]}"
```


##### Spin WHILL counterclockwise with direct velocity inputs

```sh
ros2 topic pub -r 12 /whill/controller/cmd_vel geometry_msgs/Twist '{linear: {x: 0}, angular: {z: 0.785}}'
```


##### Change the speed profile of the serial control (default values for s1=4)

```sh
ros2 service call /whill/set_speed_profile_srv whill_msgs/SetSpeedProfile '{s1: 4, fm1: 60, fa1: 32, fd1: 96, rm1: 20, ra1: 24, rd1: 64, tm1: 35, ta1: 56, td1: 72}'
```


## Packages

| Package name | Explanation |
|:---|:---|
| whill | Meta package. Dependencies are described. |
| [whill_bringup](./whill_bringup/README.md) | Launch package. You can start WHILL node with this package. |
| [whill_description](./whill_description/README.md) | Description package. The URDF file is here. |
| [whill_driver](./whill_driver/README.md) | Controller package. WHILL domains are implemented. |
| [whill_examples](./whill_examples/README.md) | Examples. You can try demos with this package. |
| [whill_msgs](https://github.com/WHILL/ros2_whill_interfaces) | Interfaces package. This package exists in a different repository. |


## License

Copyright (c) 2024 WHILL, Inc.

This repository is licensed under the MIT License, see [LICENSE](./LICENSE) for details.
