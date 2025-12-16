module.exports = {
  docs: {
    "Module 1: The Robotic Nervous System (ROS 2)": [
      "module-1-ros2/index",
      "module-1-ros2/01-intro-ros2",
      "module-1-ros2/02-python-integration",
      "module-1-ros2/03-humanoid-urdf"
    ],
    "Module 2: Digital Twin (Gazebo & Unity)": [
      "module2/index",
      {
        type: "category",
        label: "Physics Simulation",
        items: [
          "gazebo-physics-simulation",  // New file created
          "module2/physics/physics-simulation",  // Original file
        ],
      },
      {
        type: "category",
        label: "Sensor Simulation",
        items: [
          "sensor-simulation",  // New file created
          "module2/sensors/sensor-simulation",  // Original file
        ],
      },
      {
        type: "category",
        label: "Unity Visualization",
        items: [
          "unity-human-robot-interaction",  // New file created
          "module2/unity/unity-visualization",  // Original file
        ],
      },
      {
        type: "category",
        label: "Digital Twin Architecture",
        items: [
          "digital-twin-architecture",  // New file created
          "module2/integration/gazebo-unity-integration",  // Original file
        ],
      },
      {
        type: "category",
        label: "Quickstart Guide",
        items: [
          "module2/quickstart/digital-twin-quickstart",
        ],
      },
      {
        type: "category",
        label: "Data Model",
        items: [
          "module2/data-model/data-model",
        ],
      },
      {
        type: "category",
        label: "Validation Plan",
        items: [
          "module2/validation/validation-plan",
        ],
      },
    ],
    "Module 3: The AI-Robot Brain (NVIDIA Isaac™)": [
      "module-3-ai-robot-brain/index",
      "module-3-ai-robot-brain/isaac-sim",
      "module-3-ai-robot-brain/isaac-ros",
      "module-3-ai-robot-brain/nav2"
    ],
    "Module 4: Vision-Language-Action (VLA)": [
      "module-4-vla/index",
      "module-4-vla/vla-overview",
      "module-4-vla/voice-to-action",
      "module-4-vla/cognitive-planning",
      "module-4-vla/capstone-autonomous-humanoid"
    ],
  },
};