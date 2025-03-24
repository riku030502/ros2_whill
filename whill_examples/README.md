# whill_examples

## About

The "whill_examples" is a ROS2 package for WHILL Model CR2 demo examples. <br>


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
~/<your_ros2_ws>/src/ros2_whill/whill_examples/config/params.yaml
```


### 3. Build project
```sh
colcon build --packages-up-to whill
. install/setup.bash
```


### 4. Launch

```sh
ros2 launch whill_examples whill_examples_launch.py
```
After these steps, WHILL will move following:
1. turn power on
2. spin counterclockwise
3. spin clockwise
4. turn power off


## License

Copyright (c) 2024 WHILL, Inc.

Released under the [MIT license](https://opensource.org/licenses/mit-license.php)
