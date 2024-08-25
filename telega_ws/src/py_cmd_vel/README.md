# py_cmd_vel
ROS script for sending Twist data to Arduino serial port.
## Run

```sh
ros2 run py_cmd_vel cmd_vel
```
## Usage
```sh
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```
