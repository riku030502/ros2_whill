# whill_bringup

## About

The "whill_bringup" is a ROS2 package for WHILL Model CR2 launcher. <br>


## Getting started

1. Download projects
2. Configure your serial port
3. Build project
4. Launch


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

```sh
ros2 launch whill_bringup whill_launch.py
```
After these steps, you will be able to execute commands.
[Command examples](./../README.md#command-examples)


## License

Copyright (c) 2024 WHILL, Inc.

Released under the [MIT license](https://opensource.org/licenses/mit-license.php)
