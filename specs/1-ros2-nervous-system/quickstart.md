# Quickstart Guide: Module 1: The Robotic Nervous System (ROS 2)

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic command line knowledge

## Setup Environment
1. Install ROS 2 Humble Hawksbill following official installation guide
2. Source ROS 2 environment: `source /opt/ros/humble/setup.bash`
3. Create a new workspace: `mkdir -p ~/ros2_book_ws/src && cd ~/ros2_book_ws`
4. Build the workspace: `colcon build`

## Verify Installation
Run the following command to verify ROS 2 is working:
```bash
ros2 topic list
```

You should see system topics like `/parameter_events` and `/rosout`.

## First Node Example
Create a simple publisher node to verify your setup:

1. Navigate to your workspace: `cd ~/ros2_book_ws`
2. Source the workspace: `source install/setup.bash`
3. Run a simple publisher: `ros2 run demo_nodes_cpp talker`

In another terminal, run a subscriber to see the communication:
```bash
cd ~/ros2_book_ws
source install/setup.bash
ros2 run demo_nodes_py listener
```

## Next Steps
- Proceed to the ROS 2 Foundations chapter
- Practice creating your own simple nodes
- Explore topics, services, and launch files
- Build the minimal humanoid URDF example

## Troubleshooting
- If commands fail, ensure ROS 2 environment is sourced
- Check that your Python and ROS 2 installations match requirements
- Verify all dependencies are installed via `rosdep`