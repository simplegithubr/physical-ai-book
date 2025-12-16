---
sidebar_position: 5
title: "Quickstart Guide: Digital Twin Setup"
---

# Quickstart Guide: Digital Twin Setup

This guide will help you set up your first Gazebo-Unity digital twin system. Follow these steps to create a basic integrated environment for physics simulation and high-fidelity visualization.

## Prerequisites

Before starting, ensure you have the following software installed:

### System Requirements
- **Operating System**: Ubuntu 20.04/22.04 (for Gazebo) or Windows 10/11 (for Unity)
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Dedicated graphics card with at least 2GB VRAM
- **Storage**: 10GB available space

### Software Dependencies

#### For Gazebo Environment:
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install gazebo11 libgazebo11-dev
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt install ros-noetic-rosbridge-suite
```

#### For Unity Environment:
- Unity Hub 3.0+
- Unity Editor 2021.3 LTS or later
- Visual Studio 2019 or Rider

## Step 1: Setting Up Gazebo Simulation

### 1.1 Create a ROS Workspace

```bash
# Create and navigate to workspace
mkdir -p ~/digital_twin_ws/src
cd ~/digital_twin_ws

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Initialize the workspace
catkin_make
source devel/setup.bash
```

### 1.2 Create a Simple Robot Model

Create a basic robot model in `~/digital_twin_ws/src/digital_twin_robot`:

```bash
cd ~/digital_twin_ws/src
catkin_create_pkg digital_twin_robot rospy std_msgs sensor_msgs geometry_msgs
```

Create the robot's URDF file at `~/digital_twin_ws/src/digital_twin_robot/urdf/simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- LiDAR Sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 0.8"/>
      </material>
    </visual>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- IMU Sensor -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 0.8"/>
      </material>
    </visual>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo Plugins -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="lidar_link">
    <material>Gazebo/Red</material>
    <sensor name="lidar_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <topicName>/scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <topicName>/imu/data</topicName>
        <bodyName>imu_link</bodyName>
        <frameName>imu_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

### 1.3 Create a World File

Create `~/digital_twin_ws/src/digital_twin_robot/worlds/simple_world.world`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Add some obstacles -->
    <model name="box1">
      <pose>2 0 0.5 0 0 0</pose>
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
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### 1.4 Launch the Simulation

Create a launch file at `~/digital_twin_ws/src/digital_twin_robot/launch/simple_robot.launch`:

```xml
<launch>
  <!-- Start Gazebo with world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find digital_twin_robot)/worlds/simple_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Load robot description -->
  <param name="robot_description" command="$(find xacro)/xacro $(find digital_twin_robot)/urdf/simple_robot.urdf" />

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model"
        args="-param robot_description -urdf -model simple_robot -x 0 -y 0 -z 0.5" />

  <!-- Start ROS Bridge -->
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch">
    <arg name="port" value="9090"/>
  </include>
</launch>
```

Launch the simulation:

```bash
cd ~/digital_twin_ws
source devel/setup.bash
roslaunch digital_twin_robot simple_robot.launch
```

## Step 2: Setting Up Unity Visualization

### 2.1 Create a New Unity Project

1. Open Unity Hub
2. Click "New Project"
3. Select "3D (Built-in Render Pipeline)" or "3D (URP)"
4. Name the project "DigitalTwinVisualization"
5. Create the project

### 2.2 Import Required Packages

In Unity, go to Window > Package Manager and install:

- **ProBuilder** (for quick environment creation)
- **ProGrids** (for precise placement)
- **TextMeshPro** (for UI text)

### 2.3 Create Basic Scene Structure

Create the following folder structure in Assets:

```
Assets/
├── Scripts/
├── Materials/
├── Models/
├── Scenes/
└── Plugins/
```

### 2.4 Implement Gazebo Connection Script

Create `Assets/Scripts/GazeboConnection.cs`:

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Text;
using Newtonsoft.Json;

public class GazeboConnection : MonoBehaviour
{
    [Header("Connection Settings")]
    public string gazeboHost = "127.0.0.1";
    public int gazeboPort = 5000;
    public float updateRate = 100f; // Hz

    [Header("Robot References")]
    public Transform robotRoot;
    public Transform lidarPoint;
    public Transform imuPoint;

    private TcpClient tcpClient;
    private NetworkStream stream;
    private bool isConnected = false;

    void Start()
    {
        StartCoroutine(ConnectToGazebo());
    }

    IEnumerator ConnectToGazebo()
    {
        yield return new WaitForSeconds(1f); // Wait a bit before connecting

        try
        {
            tcpClient = new TcpClient(gazeboHost, gazeboPort);
            stream = tcpClient.GetStream();
            isConnected = true;
            Debug.Log("Connected to Gazebo simulation");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to Gazebo: {e.Message}");
        }
    }

    void Update()
    {
        if (isConnected && tcpClient.Connected)
        {
            // Process incoming data in a separate coroutine
            StartCoroutine(ReceiveAndProcessData());
        }
    }

    IEnumerator ReceiveAndProcessData()
    {
        if (stream.DataAvailable)
        {
            byte[] buffer = new byte[4096];
            int bytesRead = stream.Read(buffer, 0, buffer.Length);

            if (bytesRead > 0)
            {
                string data = Encoding.UTF8.GetString(buffer, 0, bytesRead);
                ProcessGazeboData(data);
            }
        }

        yield return null;
    }

    void ProcessGazeboData(string jsonData)
    {
        try
        {
            GazeboState state = JsonConvert.DeserializeObject<GazeboState>(jsonData);

            // Update robot position
            if (robotRoot != null)
            {
                robotRoot.position = new Vector3(
                    (float)state.robot_state.position.x,
                    (float)state.robot_state.position.z, // Swap Y and Z for Unity
                    (float)state.robot_state.position.y
                );

                // Convert Gazebo quaternion to Unity quaternion
                robotRoot.rotation = new Quaternion(
                    (float)state.robot_state.orientation.x,
                    (float)state.robot_state.orientation.z,
                    (float)state.robot_state.orientation.y,
                    (float)state.robot_state.orientation.w
                );
            }

            // Update sensor positions if needed
            if (lidarPoint != null)
            {
                lidarPoint.position = robotRoot.position + robotRoot.forward * 0.15f + robotRoot.up * 0.1f;
            }
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error processing Gazebo data: {e.Message}");
        }
    }

    [System.Serializable]
    public class GazeboState
    {
        public double timestamp;
        public RobotState robot_state;
        public SensorData sensor_data;
    }

    [System.Serializable]
    public class RobotState
    {
        public Position position;
        public Orientation orientation;
    }

    [System.Serializable]
    public class Position
    {
        public double x, y, z;
    }

    [System.Serializable]
    public class Orientation
    {
        public double x, y, z, w; // Quaternion
    }

    [System.Serializable]
    public class SensorData
    {
        public LidarData lidar;
        public ImuData imu;
    }

    [System.Serializable]
    public class LidarData
    {
        public List<double> ranges = new List<double>();
        public double angle_min;
        public double angle_max;
    }

    [System.Serializable]
    public class ImuData
    {
        public Acceleration acceleration;
        public AngularVelocity angular_velocity;
    }

    [System.Serializable]
    public class Acceleration
    {
        public double x, y, z;
    }

    [System.Serializable]
    public class AngularVelocity
    {
        public double x, y, z;
    }

    void OnDestroy()
    {
        if (tcpClient != null)
        {
            tcpClient.Close();
        }
    }
}
```

### 2.5 Create Visualization Environment

1. Create a new scene: File > New Scene
2. Add a plane for the ground
3. Create a simple robot model:
   - Create a cube for the base
   - Create a cylinder for the LiDAR
   - Create a small cube for the IMU
   - Parent them to a root GameObject

4. Attach the GazeboConnection script to the root GameObject
5. Assign the robot components in the Inspector

### 2.6 Set Up Camera and Lighting

1. Position the Main Camera to view the robot
2. Add directional light to simulate environment lighting
3. Set up a basic material for the robot

## Step 3: Establishing Communication

### 3.1 Create a Simple Communication Bridge

Instead of using ROS Bridge for this quickstart, create a custom TCP server in Python that connects to Gazebo:

Create `~/digital_twin_ws/src/digital_twin_robot/scripts/gazebo_to_unity_bridge.py`:

```python
#!/usr/bin/env python3

import rospy
import socket
import json
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from tf.transformations import euler_from_quaternion

class GazeboUnityBridge:
    def __init__(self):
        rospy.init_node('gazebo_unity_bridge')

        # Communication setup
        self.host = 'localhost'
        self.port = 5000
        self.clients = []
        self.socket_lock = threading.Lock()

        # Robot state
        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'timestamp': rospy.get_time()
        }

        self.sensor_data = {
            'lidar': {'ranges': [10.0] * 360, 'angle_min': -3.14, 'angle_max': 3.14},
            'imu': {'acceleration': {'x': 0.0, 'y': 0.0, 'z': 9.81},
                   'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0}}
        }

        # Subscribe to Gazebo topics
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Start TCP server
        self.start_server()

        # Start broadcast thread
        self.running = True
        broadcast_thread = threading.Thread(target=self.broadcast_state)
        broadcast_thread.daemon = True
        broadcast_thread.start()

        rospy.loginfo("Gazebo-Unity Bridge initialized")

    def odom_callback(self, msg):
        """Process odometry messages from Gazebo"""
        self.robot_state['position']['x'] = msg.pose.pose.position.x
        self.robot_state['position']['y'] = msg.pose.pose.position.y
        self.robot_state['position']['z'] = msg.pose.pose.position.z

        self.robot_state['orientation']['x'] = msg.pose.pose.orientation.x
        self.robot_state['orientation']['y'] = msg.pose.pose.orientation.y
        self.robot_state['orientation']['z'] = msg.pose.pose.orientation.z
        self.robot_state['orientation']['w'] = msg.pose.pose.orientation.w

        self.robot_state['timestamp'] = rospy.get_time()

    def lidar_callback(self, msg):
        """Process LiDAR messages from Gazebo"""
        self.sensor_data['lidar']['ranges'] = [r if r < msg.range_max else msg.range_max for r in msg.ranges]
        self.sensor_data['lidar']['angle_min'] = msg.angle_min
        self.sensor_data['lidar']['angle_max'] = msg.angle_max

    def imu_callback(self, msg):
        """Process IMU messages from Gazebo"""
        self.sensor_data['imu']['acceleration']['x'] = msg.linear_acceleration.x
        self.sensor_data['imu']['acceleration']['y'] = msg.linear_acceleration.y
        self.sensor_data['imu']['acceleration']['z'] = msg.linear_acceleration.z

        self.sensor_data['imu']['angular_velocity']['x'] = msg.angular_velocity.x
        self.sensor_data['imu']['angular_velocity']['y'] = msg.angular_velocity.y
        self.sensor_data['imu']['angular_velocity']['z'] = msg.angular_velocity.z

    def start_server(self):
        """Start TCP server to communicate with Unity"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)

        server_thread = threading.Thread(target=self.accept_connections)
        server_thread.daemon = True
        server_thread.start()

        rospy.loginfo(f"TCP Server listening on {self.host}:{self.port}")

    def accept_connections(self):
        """Accept client connections"""
        while not rospy.is_shutdown():
            try:
                client_socket, address = self.server_socket.accept()
                rospy.loginfo(f"Unity client connected: {address}")

                with self.socket_lock:
                    self.clients.append(client_socket)

                # Send initial state to new client
                self.send_initial_state(client_socket)
            except socket.error as e:
                if not rospy.is_shutdown():
                    rospy.logerr(f"Error accepting connections: {e}")

    def send_initial_state(self, client_socket):
        """Send initial robot state to new client"""
        initial_state = {
            'timestamp': rospy.get_time(),
            'robot_state': self.robot_state,
            'sensor_data': self.sensor_data
        }

        try:
            message_json = json.dumps(initial_state)
            client_socket.send(message_json.encode('utf-8'))
        except Exception as e:
            rospy.logerr(f"Error sending initial state: {e}")
            self.remove_client(client_socket)

    def broadcast_state(self):
        """Broadcast robot state to all connected clients"""
        rate = rospy.Rate(50)  # 50 Hz update rate

        while not rospy.is_shutdown() and self.running:
            message = {
                'timestamp': rospy.get_time(),
                'robot_state': self.robot_state,
                'sensor_data': self.sensor_data
            }

            message_json = json.dumps(message)
            message_bytes = message_json.encode('utf-8')

            disconnected_clients = []
            with self.socket_lock:
                for client in self.clients:
                    try:
                        client.send(message_bytes)
                    except socket.error as e:
                        rospy.logwarn(f"Removing disconnected client: {e}")
                        disconnected_clients.append(client)

                for client in disconnected_clients:
                    if client in self.clients:
                        self.clients.remove(client)
                        client.close()

            rate.sleep()

    def remove_client(self, client_socket):
        """Remove a disconnected client"""
        with self.socket_lock:
            if client_socket in self.clients:
                self.clients.remove(client_socket)
                client_socket.close()

if __name__ == '__main__':
    try:
        bridge = GazeboUnityBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gazebo-Unity Bridge shutting down")
    except KeyboardInterrupt:
        rospy.loginfo("Gazebo-Unity Bridge interrupted")
```

### 3.2 Launch the Bridge

Make the script executable and run it:

```bash
cd ~/digital_twin_ws
chmod +x src/digital_twin_robot/scripts/gazebo_to_unity_bridge.py
source devel/setup.bash
rosrun digital_twin_robot gazebo_to_unity_bridge.py
```

## Step 4: Running the Integrated System

### 4.1 Start the Components in Order

1. **Start Gazebo simulation**:
   ```bash
   cd ~/digital_twin_ws
   source devel/setup.bash
   roslaunch digital_twin_robot simple_robot.launch
   ```

2. **Start the bridge**:
   ```bash
   cd ~/digital_twin_ws
   source devel/setup.bash
   rosrun digital_twin_robot gazebo_to_unity_bridge.py
   ```

3. **Run Unity visualization**:
   - Open the Unity project
   - Hit Play in the Unity Editor

### 4.2 Verify Connection

- Check that the robot model moves in Unity when it moves in Gazebo
- Verify that sensor data is being transmitted
- Confirm that the Unity visualization updates in real-time

## Troubleshooting

### Common Issues and Solutions

1. **Connection Refused**:
   - Verify that the TCP server is running
   - Check firewall settings
   - Ensure Unity and Gazebo are on the same network

2. **Robot Not Moving in Unity**:
   - Check that the robot references are assigned in Unity Inspector
   - Verify coordinate system transformations
   - Ensure both systems are using the same update rate

3. **Performance Issues**:
   - Reduce the update rate in both systems
   - Simplify the Unity model
   - Check network bandwidth usage

4. **Data Synchronization Problems**:
   - Implement interpolation between state updates
   - Check for time drift between systems
   - Add connection health monitoring

## Next Steps

Congratulations! You now have a basic Gazebo-Unity digital twin system running. To enhance your setup:

1. **Advanced Sensors**: Add more complex sensor models (depth cameras, GPS)
2. **Multi-Robot Systems**: Extend to multiple robots in the same environment
3. **Control Interfaces**: Implement control commands from Unity to Gazebo
4. **Cloud Integration**: Deploy the system to cloud infrastructure
5. **Visualization Enhancements**: Add advanced rendering effects and UI elements

Explore the other chapters in this module to dive deeper into specific aspects of digital twin development.