# Checkpoint 16: Mobile Robot Kinematics

![ROS2](https://img.shields.io/badge/ros2-humble-blue) ![Gazebo](https://img.shields.io/badge/gazebo-latest-green)

## Introduction

This repository contains a series of ROS 2 C++ packages demonstrating the kinematic control of the ROSBot XL mobile robot. You will learn how to:

* Publish wheel velocities for a four-wheeled mecanum platform (`wheel_velocities_publisher`).
* Convert wheel speeds into body-frame commands via the kinematic model (`kinematic_model`).
* Follow a figure-eight trajectory by processing odometry feedback (`eight_trajectory`).

## Summary

The goal of checkpoint16 is to design and implement the kinematic model of the ROSBot XL, using Gazebo Sim to verify each stage:

1. **Task 1 - Kinematic Model of the ROSBot XL**

   1. Part 1: Determine holonomy, publish four wheel speeds in a fixed motion sequence on `/wheel_speed`.
   2. Part 2: Subscribe to `/wheel_speed`, compute and publish `geometry_msgs/msg/Twist` on `/cmd_vel`.
2. **Task 2 - Motion in the Absolute Frame**

   * Implement a waypoint follower in `eight_trajectory` that subscribes to `/odom` and publishes to `/wheel_speed`, making the robot execute a figure‑eight path.

## Prerequisites

Make sure you have the following installed on **Ubuntu 22.04**:

1. **ROS 2 Humble** (Quickstart: [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html))
2. **Gazebo Sim** (bundled with ROS 2 Humble or later)
3. **colcon build tool**: `sudo apt install python3-colcon-common-extensions`
4. **rosdep**: `sudo apt install python3-rosdep`
5. **Git** (to clone this repository)

## Setup

1. **Clone** this repo into your workspace:

   ```bash
   cd ~/ros2_ws/src
   git clone <YOUR_GIT_URL>/checkpoint16.git
   ```
2. **Install dependencies**:

   ```bash
   cd ~/ros2_ws
   rosdep update && rosdep install --from-paths src --ignore-src -r -y
   ```
3. **Build** all packages:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Running the Simulation

1. **Launch Gazebo**:

   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch rosbot_xl_gazebo simulation.launch.py
   ```
2. **Verify** the robot responds to `/cmd_vel`:

   ```bash
   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.2}}"
   ```

## Task 1: Kinematic Model of the ROSBot XL

### Part 1: Wheel Velocities Publisher

* **Package:** `wheel_velocities_publisher`
* **Node:** `wheel_velocities_publisher`
* **Topic:** `/wheel_speed` (`std_msgs/msg/Float32MultiArray`)
* **Sequence:**

  1. Move forward (3 s)
  2. Move backward (3 s)
  3. Strafe left (3 s)
  4. Strafe right (3 s)
  5. Turn clockwise (3 s)
  6. Turn counter-clockwise (3 s)
  7. Stop

#### Run

```bash
ros2 run wheel_velocities_publisher wheel_velocities_publisher
ros2 topic echo /wheel_speed
```

### Part 2: Kinematic Model Node

* **Package:** `kinematic_model`
* **Node:** `kinematic_model`
* **Subscribes:** `/wheel_speed`
* **Publishes:** `/cmd_vel` (`geometry_msgs/msg/Twist`)

#### Run

```bash
ros2 launch kinematic_model kinematic_model.launch.py
```

Confirm that the robot executes the six motions in order by observing its movement in Gazebo.

## Task 2: Motion in the Absolute Frame

* **Package:** `eight_trajectory`
* **Node:** `eight_trajectory`
* **Subscribes:** `/odom` (`nav_msgs/msg/Odometry`)
* **Publishes:** `/wheel_speed` (`std_msgs/msg/Float32MultiArray`)
* **Behavior:** Follows a figure-eight trajectory via eight relative waypoints:

  | Waypoint | \[dφ (rad), dx (m), dy (m)] |
  | -------- | --------------------------- |
  | w₁       | \[ 0.0,  1.0, -1.0]         |
  | w₂       | \[ 0.0,  1.0,  1.0]         |
  | w₃       | \[ 0.0,  1.0,  1.0]         |
  | w₄       | \[-1.5708, 1.0, -1.0]       |
  | w₅       | \[-1.5708,-1.0, -1.0]       |
  | w₆       | \[ 0.0, -1.0,  1.0]         |
  | w₇       | \[ 0.0, -1.0,  1.0]         |
  | w₈       | \[ 0.0, -1.0, -1.0]         |

#### Run

```bash
ros2 launch eight_trajectory eight_trajectory.launch.py
```

The robot should pass each waypoint within a small positional and angular tolerance, completing the figure-eight path.

## Contributing

Contributions, bug reports, and feature requests are welcome!
To contribute:

1. Fork this repository
2. Create a feature branch: `git checkout -b feature-name`
3. Commit your changes: `git commit -m "Add feature"`
4. Push to your fork: `git push origin feature-name`
5. Open a Pull Request
