# Teeterbot

A simulated two-wheeled inverted pendulum (self-balancing) robot for ROS 2 and Gazebo. Teeterbot is designed as a teaching platform for control systems, particularly balance control and motor control.

## Overview

Teeterbot is a differential-drive robot that is inherently unstable — like a Segway, it will fall over without active control. Two variants are available:

- **Standard teeterbot** — no stability aids, requires a working balance controller
- **Teeterbot with training wheels** — adds passive support wheels so you can develop and test other behaviors (e.g., navigation) before implementing balance control

## Launching the Simulator

### Standard teeterbot (requires balance controller)

```bash
ros2 launch teeterbot_gazebo teeterbot.launch.xml
```

### Teeterbot with training wheels

```bash
ros2 launch teeterbot_gazebo teeterbot_training_wheels.launch.xml
```

### Including in your own launch file

To start the teeterbot simulator as part of your own launch file, use an `<include>` tag (XML launch files):

**Standard teeterbot:**
```xml
<?xml version="1.0"?>
<launch>

    <include file="$(find-pkg-share teeterbot_gazebo)/launch/teeterbot.launch.xml" />

    <!-- your nodes here -->

</launch>
```

**Teeterbot with training wheels:**
```xml
<?xml version="1.0"?>
<launch>

    <include file="$(find-pkg-share teeterbot_gazebo)/launch/teeterbot_training_wheels.launch.xml" />

    <!-- your nodes here -->

</launch>
```

## ROS Topics

All topics are under the `/teeterbot` namespace.

### Subscribed Topics (inputs — send commands here)

| Topic | Type | Description |
|---|---|---|
| `/teeterbot/left_speed_cmd` | `std_msgs/msg/Float64` | Commanded left wheel speed (rad/s) |
| `/teeterbot/right_speed_cmd` | `std_msgs/msg/Float64` | Commanded right wheel speed (rad/s) |
| `/teeterbot/left_torque_cmd` | `std_msgs/msg/Float64` | Commanded left wheel torque (N·m) |
| `/teeterbot/right_torque_cmd` | `std_msgs/msg/Float64` | Commanded right wheel torque (N·m) |

**Speed vs. torque commands:** You can command the wheels in either speed mode or torque mode. If both types of commands are sent, whichever has the more recent timestamp takes effect.

### Published Topics (outputs — read sensor data here)

| Topic | Type | Description |
|---|---|---|
| `/teeterbot/imu` | `sensor_msgs/msg/Imu` | IMU data: linear acceleration, angular velocity, orientation |
| `/teeterbot/left_speed` | `std_msgs/msg/Float64` | Measured left wheel speed (rad/s) |
| `/teeterbot/right_speed` | `std_msgs/msg/Float64` | Measured right wheel speed (rad/s) |
| `/teeterbot/fallen_over` | `std_msgs/msg/Bool` | `true` when the robot has tipped past 60° from vertical |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock (use with `use_sim_time:=true`) |

Wheel speeds and the fallen-over flag are published at **100 Hz**. The IMU publishes at **100 Hz**.

When `/teeterbot/fallen_over` becomes `true`, the motor controllers are reset and stop applying torque. The robot must be reset in Gazebo to continue.

## Package Structure

| Package | Description |
|---|---|
| `teeterbot` | Metapackage |
| `teeterbot_description` | SDF robot model files |
| `teeterbot_gazebo` | Gazebo plugin, launch files, and world files |
