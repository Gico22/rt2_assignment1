# RT2 Assignment 1 — Robot Navigation with ROS2 Actions and TF2

## Overview

This package implements a complete 2D navigation stack for a differential drive robot in Gazebo, using ROS2 actions and TF2 frames. The user sets a target pose `(x, y, θ)` via a terminal interface; the robot navigates to it autonomously.

Both nodes are implemented as **ROS2 components**, i.e. compiled as shared libraries with no `main()` function, and loaded at runtime into a container.

## Package Structure

```
rt2_assignment/
├── src/
│   ├── set_position_client.cpp   # Action client + user interface
│   └── set_position_server.cpp   # Action server + navigation logic
└── CMakeLists.txt
```

## Working Principle

### Frames

The package uses three main TF2 frames:

```
world
├── goal_frame           (static, broadcast by the client when a goal is sent)
└── base_footprint
        └── base_link    (robot pose, broadcast by the server from /odom)
```

The server subscribes to `/odom` and re-broadcasts the robot pose as a `world → base_footprint` transform. This allows TF2 to chain transforms and compute `base_link → goal_frame` (goal position in robot frame) and `world → base_link` (robot pose in world frame).

### Action Client (`set_position_client`)

- Runs a **dedicated input thread** so `std::cin` never blocks the ROS executor
- Prompts the user for `x`, `y` (metres, range −10 to 10) and `θ` (degrees)
- Converts `θ` to radians and broadcasts `goal_frame` as a **static TF2 transform** relative to `world`
- Sends the goal to the action server and displays feedback (remaining distance and heading error) until the goal completes
- Allows cancellation at any time by typing `c`

### Action Server (`set_position_server`)

Implements a **three-phase proportional controller**:

1. **Rotate to face the goal** — looks up `base_link → goal_frame`, computes heading error with `atan2(y, x)`, rotates in place until error < 0.01 rad
2. **Drive to the goal** — same lookup, drives forward with speed proportional to distance while correcting heading; stops when distance < 0.05 m
3. **Rotate to final θ** — looks up `world → base_link`, extracts current yaw with `tf2::getYaw`, rotates until angular error < 0.01 rad

All three phases check for cancellation requests at every iteration.

## Dependencies

- `rclcpp`, `rclcpp_action`, `rclcpp_components`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- `geometry_msgs`, `nav_msgs`
- `rt2_interfaces` (custom action definition)

## Building

Unzip the file and drag `src` inside your workspace folder (`ws_folder` in the example below):

```bash
cd ~/ws_folder
colcon build
source install/setup.bash
```

## Running

**Terminal 1 — Gazebo world**
```bash
ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
```

**Terminal 2 — Action server**
```bash
source ~/ws_folder/install/setup.bash
ros2 run rt2_assignment set_position_server
```

**Terminal 3 — Action client (user interface)**
```bash
source ~/ws_folder/install/setup.bash
ros2 run rt2_assignment set_position_client
```

The client will prompt for a target pose. Enter `x`, `y` and `θ` (degrees), confirm with `y`, and the robot will navigate to the goal. Type `c` to cancel at any time.
