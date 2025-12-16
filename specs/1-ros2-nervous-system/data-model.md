# Data Model: Module 1: The Robotic Nervous System (ROS 2)

## Entity: ROS 2 Node
- **Description**: A process that performs computation and communicates with other nodes through topics and services
- **Attributes**:
  - node_name: string (unique identifier for the node)
  - node_namespace: string (optional namespace for organization)
  - publishers: list of Publisher objects
  - subscribers: list of Subscriber objects
  - services: list of Service objects
  - clients: list of Client objects
- **Relationships**:
  - Communicates with other ROS 2 Nodes via Topics and Services
- **Validation rules**:
  - node_name must be unique within namespace
  - node_name follows ROS naming conventions (alphanumeric, underscores, hyphens)
- **State transitions**: N/A (nodes are process-based, not stateful entities)

## Entity: Topic/Service
- **Description**: Communication mechanisms that allow nodes to exchange data and request actions
- **Attributes**:
  - name: string (unique identifier for the topic/service)
  - message_type: string (type definition for messages)
  - direction: enum (publisher/subscriber for topics, server/client for services)
  - qos_profile: QoS settings object
- **Relationships**:
  - Connects ROS 2 Nodes for communication
  - Publishers send to Topics, Subscribers receive from Topics
  - Service Servers provide services, Service Clients request services
- **Validation rules**:
  - name must follow ROS naming conventions
  - message_type must be a valid ROS 2 message type
  - Direction must be appropriate for communication pattern
- **State transitions**: N/A (communication channels are conceptual entities)

## Entity: URDF Model
- **Description**: XML-based description of a robot's physical structure including links, joints, and properties
- **Attributes**:
  - model_name: string (name of the robot model)
  - links: list of Link objects (physical components)
  - joints: list of Joint objects (connections between links)
  - materials: list of Material objects
  - transmissions: list of Transmission objects (actuator interfaces)
- **Relationships**:
  - Links connected via Joints to form robot structure
  - Used by Robot State Publisher to visualize robot
- **Validation rules**:
  - Must form a valid kinematic tree (no loops)
  - Joint connections must reference existing links
  - All required physical properties specified
- **State transitions**: N/A (static robot description)

## Entity: rclpy
- **Description**: Python client library for ROS 2 that enables Python-based node development
- **Attributes**:
  - version: string (rclpy version)
  - node_interface: Node object (the main interface for creating nodes)
  - publisher_interface: Publisher class
  - subscriber_interface: Subscriber class
  - service_interface: Service class
  - client_interface: Client class
- **Relationships**:
  - Provides Python interface to ROS 2 Node entity
  - Enables creation of Topic/Service communication
- **Validation rules**:
  - Compatible with current ROS 2 distribution
  - Follows ROS 2 Python API conventions
- **State transitions**: N/A (library interface)

## Entity: Launch File
- **Description**: Configuration file that starts multiple nodes and sets up the ROS 2 environment
- **Attributes**:
  - file_name: string (name of the launch file)
  - nodes_to_launch: list of Node configurations
  - parameters: dict of parameter settings
  - remappings: dict of topic/service remappings
  - conditions: list of conditional launch rules
- **Relationships**:
  - Defines configuration for multiple ROS 2 Nodes
  - Sets up Topic/Service connections
- **Validation rules**:
  - Must be valid Python or XML launch file format
  - All referenced nodes must exist in the system
  - Parameter types must match expected values
- **State transitions**: N/A (configuration entity)