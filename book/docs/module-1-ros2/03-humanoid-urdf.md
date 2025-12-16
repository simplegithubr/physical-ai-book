---
sidebar_position: 4
title: "Humanoid Structure & URDF"
---

# Humanoid Structure & URDF: Describing Robots in Code

This chapter covers URDF (Unified Robot Description Format), the XML-based language used to describe robot models in ROS 2. You'll learn how to create humanoid robot models and connect them to the ROS 2 control system.

## Introduction to URDF

**URDF (Unified Robot Description Format)** is an XML-based format that describes robot models in ROS. A URDF file contains information about:
- Physical structure (links and joints)
- Visual and collision properties
- Inertial properties
- Sensor locations

### Why URDF is Important for Humanoids

Humanoid robots require precise modeling of their complex structure:
- Multiple degrees of freedom in arms, legs, and torso
- Proper joint constraints for realistic movement
- Accurate inertial properties for physics simulation
- Proper sensor placement for perception systems

## Basic URDF Structure

A URDF file has a basic structure with `<robot>` as the root element containing `<link>` and `<joint>` elements:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Joints connect links -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## URDF Elements Explained

### Links

Links represent rigid bodies in the robot. Each link can have multiple properties:

```xml
<link name="example_link">
  <!-- Visual properties (what you see) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Choose one geometry type -->
      <box size="0.1 0.1 0.1"/>
      <!-- <cylinder radius="0.1" length="0.2"/> -->
      <!-- <sphere radius="0.1"/> -->
      <!-- <mesh filename="package://robot_description/meshes/link.stl"/> -->
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <!-- Collision properties (what the physics engine uses) -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties (physics simulation) -->
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

### Joints

Joints connect links and define how they can move relative to each other:

```xml
<!-- Revolute joint (rotational) -->
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/> <!-- Rotation axis -->
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>

<!-- Continuous joint (unlimited rotation) -->
<joint name="continuous_joint" type="continuous">
  <parent link="base_link"/>
  <child link="rotor"/>
  <axis xyz="0 0 1"/>
</joint>

<!-- Prismatic joint (linear motion) -->
<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.1" effort="10.0" velocity="0.5"/>
</joint>

<!-- Fixed joint (no motion) -->
<joint name="fixed_joint" type="fixed">
  <parent link="base_link"/>
  <child link="sensor_mount"/>
</joint>
```

## Building a Minimal Humanoid URDF

Let's create a simple humanoid model with torso, head, arms, and legs:

```xml
<?xml version="1.0"?>
<robot name="minimal_humanoid">
  <!-- Torso (base of the robot) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 1.0"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/> <!-- Pitch (nodding) -->
    <limit lower="-0.5" upper="0.5" effort="5.0" velocity="1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.4 0.4 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.1 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/> <!-- Movement in front/back -->
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/> <!-- Rotation around arm -->
    <limit lower="-1.57" upper="1.57" effort="8.0" velocity="1.0"/>
  </joint>

  <!-- Right Arm (mirror of left arm) -->
  <link name="right_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.4 0.4 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.2 -0.1 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="8.0" velocity="1.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0.05 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/> <!-- Forward/back movement -->
    <limit lower="-0.5" upper="1.0" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/> <!-- Knee bending -->
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Right Leg (mirror of left leg) -->
  <link name="right_upper_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="3.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 -0.05 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="1.0" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <origin xyz="0 0 -0.2" rpy="1.57 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1.57" effort="20.0" velocity="1.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Xacro: URDF's Macro System

For complex robots, Xacro (XML Macros) simplifies URDF by allowing variables, macros, and mathematical expressions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_humanoid">

  <!-- Properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_length" value="1.0" />
  <xacro:property name="torso_radius" value="0.15" />
  <xacro:property name="arm_length" value="0.3" />
  <xacro:property name="leg_length" value="0.4" />

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="prefix side">
    <link name="${prefix}_upper_arm">
      <visual>
        <origin xyz="0 0 -${arm_length/2}" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="0.05" length="${arm_length}"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${prefix}_upper_arm"/>
      <origin xyz="0.2 ${side * 0.1} 0.6" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
    </joint>

    <link name="${prefix}_lower_arm">
      <visual>
        <origin xyz="0 0 -${arm_length/2}" rpy="${M_PI/2} 0 0"/>
        <geometry>
          <cylinder radius="0.04" length="${arm_length}"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_elbow_joint" type="revolute">
      <parent link="${prefix}_upper_arm"/>
      <child link="${prefix}_lower_arm"/>
      <origin xyz="0 0 -${arm_length}" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-1.57" upper="1.57" effort="8.0" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 ${torso_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${torso_radius}" length="${torso_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 ${torso_length/2}" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.6" iyz="0.0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Use the arm macro for both sides -->
  <xacro:arm prefix="left" side="1"/>
  <xacro:arm prefix="right" side="-1"/>

</robot>
```

## Connecting URDF to ROS 2

### Robot State Publisher

The `robot_state_publisher` node reads your URDF and publishes the forward kinematics to the `/tf` topic:

```bash
# Launch robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat /path/to/your/robot.urdf)'
```

Or in a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Load URDF file
    urdf_file = os.path.join(
        get_package_share_directory('your_robot_description'),
        'urdf',
        'humanoid.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc
            }],
        )
    ])
```

### Joint State Publisher

For interactive visualization, use the joint state publisher:

```xml
<launch>
  <node name="robot_state_publisher" pkg="robot_state_publisher" exec="robot_state_publisher">
    <param name="robot_description" value="$(find-pkg-share your_package)/urdf/humanoid.urdf"/>
  </node>

  <node name="joint_state_publisher" pkg="joint_state_publisher" exec="joint_state_publisher">
    <param name="use_gui" value="true"/>
  </node>
</launch>
```

## URDF Best Practices

### 1. Proper Inertial Properties

Accurate inertial properties are crucial for physics simulation:

```xml
<!-- Calculate inertial properties for common shapes -->
<!-- For a cylinder: ixx = izz = m*(3*r² + h²)/12, iyy = m*r²/2 -->
<inertial>
  <mass value="1.0"/>
  <inertia ixx="0.002916667" ixy="0.0" ixz="0.0"
           iyy="0.0025" iyz="0.0" izz="0.002916667"/>
</inertial>
```

### 2. Logical Link Hierarchy

Structure your URDF with a logical kinematic tree:

```xml
<!-- Good: Clear parent-child relationships -->
torso
├── head
├── left_upper_arm
│   └── left_lower_arm
├── right_upper_arm
│   └── right_lower_arm
├── left_upper_leg
│   └── left_lower_leg
│       └── left_foot
└── right_upper_leg
    └── right_lower_leg
        └── right_foot
```

### 3. Appropriate Joint Limits

Set realistic joint limits based on your physical robot:

```xml
<!-- Human-like joint limits -->
<joint name="elbow_joint" type="revolute">
  <limit lower="0" upper="2.356" effort="10.0" velocity="1.0"/>
  <!-- Elbow can bend 135 degrees (0 to 3π/4 radians) -->
</joint>
```

## Validating URDF Models

### Using check_urdf

ROS provides tools to validate your URDF:

```bash
# Install urdfdom tools
sudo apt-get install ros-humble-urdfdom-tools

# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Get detailed information about the model
urdf_to_graphiz /path/to/your/robot.urdf
```

### Visualizing in RViz

```bash
# Launch RViz with robot model
ros2 launch urdf_tutorial display.launch.py model:=/path/to/your/robot.urdf
```

## Integration with Control Systems

### Joint State Message

The robot state publisher works with joint state messages to show actual robot poses:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Define joint names
        self.joint_names = [
            'neck_joint', 'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.time = 0.0

    def publish_joint_states(self):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Example: make joints oscillate for demonstration
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] = 0.5 * math.sin(self.time + i * 0.5)

        self.joint_pub.publish(msg)
        self.time += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Forward and Inverse Kinematics

URDF models work with kinematics packages like MoveIt for advanced motion planning:

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # Example: Use MoveIt's services for kinematics
        # Forward kinematics service
        self.fk_client = self.create_client(
            GetPositionFK, 'compute_fk')

        # Inverse kinematics service
        self.ik_client = self.create_client(
            GetPositionIK, 'compute_ik')

        self.get_logger().info('Kinematics node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Troubleshooting Common Issues

### 1. URDF Validation Errors

```bash
# If you get validation errors, check:
# - All links have proper inertial properties
# - Joint parents and children exist
# - Joint limits are properly specified
# - No circular dependencies in the kinematic tree
```

### 2. Visualization Problems

- Ensure joint names in your URDF match the joint state messages
- Check that the robot_state_publisher is running
- Verify TF frames are being published correctly

### 3. Physics Simulation Issues

- Verify inertial properties are realistic
- Check that collision geometries are properly defined
- Ensure joint limits are appropriate

## Summary

URDF is fundamental to describing robot models in ROS 2. Key takeaways include:

1. **Structure**: Links and joints form the kinematic tree
2. **Properties**: Visual, collision, and inertial properties define each link
3. **Macros**: Xacro simplifies complex robot descriptions
4. **Integration**: Robot state publisher connects URDF to ROS 2 TF system
5. **Validation**: Tools exist to check and visualize your URDF models

With a properly defined URDF model, you can now integrate your robot description with ROS 2 control systems, simulation environments, and motion planning packages like MoveIt.