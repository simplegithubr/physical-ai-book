# Quickstart Guide: Digital Twin Simulation (Gazebo & Unity)

## Overview
This guide provides a minimal example to get started with Gazebo-Unity integration for robotics simulation. Follow these steps to create your first synchronized physics and visualization environment.

## Prerequisites
- ROS 2 installed and sourced
- Gazebo (tested with Garden or Fortress)
- Unity Hub with Unity 2022.3 LTS
- Python 3.8+

## Setup Steps

### 1. Environment Setup
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash  # or your ROS 2 distro

# Create workspace
mkdir -p ~/gazebo_unity_ws/src
cd ~/gazebo_unity_ws
```

### 2. Install Required Packages
```bash
# Install Gazebo ROS packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

# Install Unity ROS bridge (if available) or equivalent
# This would typically be a Unity package or external bridge
```

### 3. Create Basic Gazebo World
Create a simple world file with physics properties:
```xml
<!-- ~/.gazebo/worlds/basic_physics.world -->
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_physics">
    <!-- Physics properties -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Simple box for testing -->
    <model name="test_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### 4. Launch Basic Simulation
```bash
# Launch Gazebo with your world
gz sim -r basic_physics.world
```

### 5. Unity Integration Setup
1. Open Unity Hub and create a new 3D project
2. Import the ROS-TCP-Connector or equivalent package
3. Create a simple scene that mirrors the Gazebo environment
4. Set up a basic cube at position (0, 0, 0.5) to match the Gazebo box

### 6. Connect Systems
The connection between Gazebo and Unity typically involves:
- A ROS bridge node that publishes Gazebo state
- A Unity subscriber that updates visualization
- Synchronization mechanisms to maintain consistency

## Reproducible Example
This minimal example demonstrates:
- Basic physics environment setup
- Simple object with collision and visual properties
- Framework for connecting Gazebo and Unity

## Next Steps
1. Add sensors to your robot model (LiDAR, Depth, IMU)
2. Implement ROS bridge communication
3. Create Unity visualization that responds to Gazebo state
4. Test synchronization between both systems

## Verification
- Objects should fall due to gravity in Gazebo
- Physics parameters should match real-world values
- Environment should maintain stable simulation

## Troubleshooting
- If simulation is unstable, reduce max_step_size
- If visualization lags, optimize Unity scene complexity
- Check ROS network configuration for bridge communication

## Technical Accuracy Note
All examples in this module will be verified against official Gazebo, Unity, and ROS 2 documentation to ensure technical accuracy and reproducibility.