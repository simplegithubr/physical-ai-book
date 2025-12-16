---
sidebar_position: 6
title: "Digital Twin Data Model"
---

# Digital Twin Data Model

This chapter defines the data structures and schemas used in the Gazebo-Unity digital twin system. Understanding these data models is crucial for developing robust simulation and visualization components.

## Overview

The digital twin data model encompasses all information exchanged between Gazebo (physics simulation) and Unity (visualization), including robot states, sensor readings, environment data, and control commands.

## Core Data Structures

### Robot State Model

The RobotState model represents the complete state of a robot in the simulation:

```json
{
  "robot_id": "string",
  "timestamp": "double",
  "pose": {
    "position": {
      "x": "double",
      "y": "double",
      "z": "double"
    },
    "orientation": {
      "x": "double",
      "y": "double",
      "z": "double",
      "w": "double"
    }
  },
  "twist": {
    "linear": {
      "x": "double",
      "y": "double",
      "z": "double"
    },
    "angular": {
      "x": "double",
      "y": "double",
      "z": "double"
    }
  },
  "joint_states": [
    {
      "name": "string",
      "position": "double",
      "velocity": "double",
      "effort": "double"
    }
  ],
  "attached_objects": [
    {
      "object_id": "string",
      "link_name": "string",
      "relative_pose": "Pose"
    }
  ]
}
```

### Sensor Data Model

The SensorData model standardizes sensor readings across different sensor types:

```json
{
  "sensor_id": "string",
  "sensor_type": "enum: lidar | camera | imu | gps | depth_camera | force_torque",
  "timestamp": "double",
  "frame_id": "string",
  "data": "object" // Type depends on sensor_type
}
```

#### LiDAR Data Structure

```json
{
  "sensor_id": "lidar_front",
  "sensor_type": "lidar",
  "timestamp": 1634567890.123,
  "frame_id": "lidar_link",
  "data": {
    "angle_min": "double",
    "angle_max": "double",
    "angle_increment": "double",
    "time_increment": "double",
    "scan_time": "double",
    "range_min": "double",
    "range_max": "double",
    "ranges": ["double"], // Array of range measurements
    "intensities": ["double"] // Optional intensity measurements
  }
}
```

#### Camera Data Structure

```json
{
  "sensor_id": "camera_front",
  "sensor_type": "camera",
  "timestamp": 1634567890.123,
  "frame_id": "camera_link",
  "data": {
    "width": "int",
    "height": "int",
    "encoding": "string", // "rgb8", "bgr8", "mono8", etc.
    "is_bigendian": "bool",
    "step": "int", // Full row length in bytes
    "data": "base64_encoded_image_bytes",
    "header": {
      "stamp": "timestamp",
      "frame_id": "string"
    },
    "camera_info": {
      "width": "int",
      "height": "int",
      "distortion_model": "string",
      "D": ["double"], // Distortion coefficients
      "K": ["double"], // 3x3 intrinsic matrix (row-major)
      "R": ["double"], // 3x3 rectification matrix
      "P": ["double"]  // 3x4 projection matrix
    }
  }
}
```

#### IMU Data Structure

```json
{
  "sensor_id": "imu_base",
  "sensor_type": "imu",
  "timestamp": 1634567890.123,
  "frame_id": "imu_link",
  "data": {
    "orientation": {
      "x": "double",
      "y": "double",
      "z": "double",
      "w": "double"
    },
    "orientation_covariance": ["double"], // 9-element array
    "angular_velocity": {
      "x": "double",
      "y": "double",
      "z": "double"
    },
    "angular_velocity_covariance": ["double"], // 9-element array
    "linear_acceleration": {
      "x": "double",
      "y": "double",
      "z": "double"
    },
    "linear_acceleration_covariance": ["double"] // 9-element array
  }
}
```

## Environment Data Model

### World State Model

The WorldState model represents the complete state of the simulation environment:

```json
{
  "world_id": "string",
  "timestamp": "double",
  "gravity": {
    "x": "double",
    "y": "double",
    "z": "double"
  },
  "models": [
    {
      "model_id": "string",
      "name": "string",
      "pose": "Pose",
      "twist": "Twist",
      "static": "bool"
    }
  ],
  "lights": [
    {
      "name": "string",
      "type": "enum: directional | point | spot",
      "pose": "Pose",
      "diffuse": {"r": "double", "g": "double", "b": "double", "a": "double"},
      "specular": {"r": "double", "g": "double", "b": "double", "a": "double"}
    }
  ],
  "materials": [
    {
      "name": "string",
      "ambient": {"r": "double", "g": "double", "b": "double", "a": "double"},
      "diffuse": {"r": "double", "g": "double", "b": "double", "a": "double"},
      "specular": {"r": "double", "g": "double", "b": "double", "a": "double"},
      "emissive": {"r": "double", "g": "double", "b": "double", "a": "double"},
      "shininess": "double",
      "transparency": "double"
    }
  ]
}
```

### Collision and Physics Model

```json
{
  "collision_id": "string",
  "model_id": "string",
  "link_name": "string",
  "geometry": {
    "type": "enum: box | sphere | cylinder | mesh | plane",
    "size": ["double"], // For box: [x, y, z], for sphere: [radius], etc.
    "mesh_uri": "string", // For mesh type
    "mesh_scale": {"x": "double", "y": "double", "z": "double"}
  },
  "surface": {
    "friction": {
      "ode": {
        "mu": "double",
        "mu2": "double",
        "fdir1": {"x": "double", "y": "double", "z": "double"}
      },
      "bullet": {
        "friction": "double",
        "friction2": "double"
      }
    },
    "bounce": {
      "restitution_coefficient": "double",
      "threshold": "double"
    },
    "contact": {
      "ode": {
        "kp": "double", // Spring stiffness
        "kd": "double", // Damping coefficient
        "max_vel": "double",
        "min_depth": "double"
      }
    }
  }
}
```

## Control Command Model

### Robot Control Commands

```json
{
  "command_id": "string",
  "robot_id": "string",
  "timestamp": "double",
  "command_type": "enum: velocity | position | effort | trajectory",
  "targets": [
    {
      "joint_name": "string",
      "position": "double", // For position control
      "velocity": "double", // For velocity control
      "effort": "double",   // For effort control
      "k_position": "double", // Position gain (optional)
      "k_velocity": "double"  // Velocity gain (optional)
    }
  ],
  "duration": "double", // Command duration in seconds
  "header": {
    "seq": "int",
    "stamp": "timestamp",
    "frame_id": "string"
  }
}
```

### Navigation Commands

```json
{
  "command_id": "string",
  "robot_id": "string",
  "timestamp": "double",
  "command_type": "navigation",
  "goal": {
    "position": {"x": "double", "y": "double", "z": "double"},
    "orientation": {"x": "double", "y": "double", "z": "double", "w": "double"}
  },
  "tolerance": {
    "position": "double",
    "orientation": "double"
  },
  "path_constraints": {
    "max_velocity": "double",
    "max_acceleration": "double",
    "min_turn_radius": "double"
  }
}
```

## Communication Protocol Models

### Message Envelope

All messages follow a standardized envelope structure:

```json
{
  "message_id": "string",
  "type": "string", // Message type identifier
  "version": "string", // Schema version (e.g., "1.0.0")
  "timestamp": "double", // Unix timestamp with nanosecond precision
  "source": {
    "system": "enum: gazebo | unity | bridge | client",
    "component": "string", // Specific component name
    "instance_id": "string" // Unique instance identifier
  },
  "destination": {
    "system": "enum: gazebo | unity | bridge | client",
    "component": "string",
    "instance_id": "string"
  },
  "payload": "object", // Actual data payload
  "metadata": {
    "priority": "enum: low | normal | high | critical",
    "reliability": "enum: best_effort | reliable",
    "sequence_number": "int",
    "checksum": "string" // Optional data integrity check
  }
}
```

### Error and Status Models

```json
{
  "message_id": "error_response_123",
  "type": "error",
  "timestamp": 1634567890.123,
  "source": {
    "system": "gazebo",
    "component": "physics_engine",
    "instance_id": "gazebo_server_1"
  },
  "destination": {
    "system": "unity",
    "component": "visualization",
    "instance_id": "unity_client_1"
  },
  "payload": {
    "error_code": "int",
    "error_message": "string",
    "severity": "enum: warning | error | fatal",
    "context": "object" // Additional context-specific data
  },
  "metadata": {
    "priority": "high",
    "reliability": "reliable",
    "sequence_number": 12345
  }
}
```

## Data Serialization Formats

### JSON Schema Definitions

For robust data validation, we define JSON schemas for each data type:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "RobotState",
  "type": "object",
  "required": ["robot_id", "timestamp", "pose"],
  "properties": {
    "robot_id": {
      "type": "string",
      "pattern": "^[a-zA-Z0-9_-]+$"
    },
    "timestamp": {
      "type": "number",
      "minimum": 0
    },
    "pose": {
      "$ref": "#/definitions/Pose"
    },
    "twist": {
      "$ref": "#/definitions/Twist"
    },
    "joint_states": {
      "type": "array",
      "items": {
        "$ref": "#/definitions/JointState"
      }
    }
  },
  "definitions": {
    "Pose": {
      "type": "object",
      "properties": {
        "position": {"$ref": "#/definitions/Vector3"},
        "orientation": {"$ref": "#/definitions/Quaternion"}
      }
    },
    "Vector3": {
      "type": "object",
      "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"}
      }
    },
    "Quaternion": {
      "type": "object",
      "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"},
        "w": {"type": "number"}
      }
    },
    "Twist": {
      "type": "object",
      "properties": {
        "linear": {"$ref": "#/definitions/Vector3"},
        "angular": {"$ref": "#/definitions/Vector3"}
      }
    },
    "JointState": {
      "type": "object",
      "properties": {
        "name": {"type": "string"},
        "position": {"type": "number"},
        "velocity": {"type": "number"},
        "effort": {"type": "number"}
      }
    }
  }
}
```

## Coordinate System Conventions

### Unified Coordinate System

To ensure consistency between Gazebo and Unity:

- **Gazebo**: Right-handed coordinate system (X forward, Y left, Z up)
- **Unity**: Left-handed coordinate system (X right, Y up, Z forward)

### Coordinate Transformation

When transferring data between systems, apply these transformations:

```csharp
// Gazebo to Unity transformation
Vector3 GazeboToUnity(Vector3 gazeboPos)
{
    return new Vector3(
        (float)gazeboPos.Y,    // Y -> X
        (float)gazeboPos.Z,    // Z -> Y
        (float)gazeboPos.X     // X -> Z
    );
}

Quaternion GazeboToUnity(Quaternion gazeboQuat)
{
    return new Quaternion(
        (float)gazeboQuat.Y,   // Y -> X
        (float)gazeboQuat.Z,   // Z -> Y
        (float)gazeboQuat.X,   // X -> Z
        (float)gazeboQuat.W    // W -> W
    );
}
```

## Data Validation and Quality Assurance

### Schema Validation

Implement schema validation for all incoming data:

```python
import jsonschema
from jsonschema import validate

def validate_robot_state(data):
    """Validate robot state against schema"""
    schema = {
        "type": "object",
        "required": ["robot_id", "timestamp", "pose"],
        "properties": {
            "robot_id": {"type": "string"},
            "timestamp": {"type": "number", "minimum": 0},
            "pose": {
                "type": "object",
                "required": ["position", "orientation"],
                "properties": {
                    "position": {
                        "type": "object",
                        "required": ["x", "y", "z"],
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "z": {"type": "number"}
                        }
                    },
                    "orientation": {
                        "type": "object",
                        "required": ["x", "y", "z", "w"],
                        "properties": {
                            "x": {"type": "number", "minimum": -1, "maximum": 1},
                            "y": {"type": "number", "minimum": -1, "maximum": 1},
                            "z": {"type": "number", "minimum": -1, "maximum": 1},
                            "w": {"type": "number", "minimum": -1, "maximum": 1}
                        }
                    }
                }
            }
        }
    }

    try:
        validate(instance=data, schema=schema)
        return True, "Valid"
    except jsonschema.exceptions.ValidationError as e:
        return False, str(e)
```

### Data Integrity Checks

Implement checksums and validation for critical data:

```python
import hashlib
import json

def calculate_checksum(data):
    """Calculate SHA-256 checksum for data integrity"""
    json_str = json.dumps(data, sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(json_str.encode()).hexdigest()

def verify_data_integrity(data, expected_checksum):
    """Verify data integrity using checksum"""
    actual_checksum = calculate_checksum(data)
    return actual_checksum == expected_checksum
```

## Performance Considerations

### Data Compression

For high-frequency data transmission, implement compression:

```python
import numpy as np
import struct
import zlib

def compress_sensor_data(ranges):
    """Compress LiDAR range data using quantization and zlib"""
    # Quantize to 16-bit values
    quantized = np.clip(np.array(ranges) * 1000, 0, 65535).astype(np.uint16)

    # Pack binary data
    binary_data = struct.pack(f'<{len(quantized)}H', *quantized)

    # Compress
    compressed = zlib.compress(binary_data)

    return compressed

def decompress_sensor_data(compressed_data):
    """Decompress LiDAR range data"""
    # Decompress
    binary_data = zlib.decompress(compressed_data)

    # Unpack
    num_ranges = len(binary_data) // 2
    quantized = struct.unpack(f'<{num_ranges}H', binary_data)

    # Convert back to float
    ranges = [q / 1000.0 for q in quantized]

    return ranges
```

### Efficient Data Streaming

For real-time applications, consider binary protocols:

```python
import struct

class EfficientDataStreamer:
    """Efficient binary data streaming for real-time applications"""

    @staticmethod
    def serialize_robot_state_binary(robot_state):
        """Serialize robot state in compact binary format"""
        data = bytearray()

        # Pack timestamp (double - 8 bytes)
        data.extend(struct.pack('d', robot_state['timestamp']))

        # Pack position (3 doubles - 24 bytes)
        pos = robot_state['pose']['position']
        data.extend(struct.pack('ddd', pos['x'], pos['y'], pos['z']))

        # Pack orientation (4 doubles - 32 bytes)
        orient = robot_state['pose']['orientation']
        data.extend(struct.pack('dddd', orient['x'], orient['y'], orient['z'], orient['w']))

        # Pack linear velocity (3 doubles - 24 bytes)
        if 'twist' in robot_state:
            linear = robot_state['twist']['linear']
            data.extend(struct.pack('ddd', linear['x'], linear['y'], linear['z']))

        return bytes(data)

    @staticmethod
    def deserialize_robot_state_binary(binary_data):
        """Deserialize robot state from binary format"""
        offset = 0

        # Unpack timestamp
        timestamp = struct.unpack_from('d', binary_data, offset)[0]
        offset += 8

        # Unpack position
        pos_x, pos_y, pos_z = struct.unpack_from('ddd', binary_data, offset)
        offset += 24

        # Unpack orientation
        orient_x, orient_y, orient_z, orient_w = struct.unpack_from('dddd', binary_data, offset)
        offset += 32

        # Unpack linear velocity
        linear_x, linear_y, linear_z = struct.unpack_from('ddd', binary_data, offset)

        return {
            'timestamp': timestamp,
            'pose': {
                'position': {'x': pos_x, 'y': pos_y, 'z': pos_z},
                'orientation': {'x': orient_x, 'y': orient_y, 'z': orient_z, 'w': orient_w}
            },
            'twist': {
                'linear': {'x': linear_x, 'y': linear_y, 'z': linear_z}
            }
        }
```

## Data Persistence Models

### Log File Format

For simulation logging and replay:

```json
{
  "log_version": "1.0",
  "session_id": "string",
  "start_time": "timestamp",
  "duration": "double",
  "metadata": {
    "gazebo_version": "string",
    "unity_version": "string",
    "robot_model": "string",
    "world_file": "string"
  },
  "events": [
    {
      "timestamp": "double",
      "event_type": "enum: state_update | sensor_data | command | collision | error",
      "data": "object" // Event-specific data
    }
  ]
}
```

## Best Practices

1. **Version Compatibility**: Always include version information in data schemas
2. **Backward Compatibility**: Design schemas to support older versions
3. **Validation**: Implement comprehensive validation at system boundaries
4. **Performance**: Optimize data structures for high-frequency transmission
5. **Security**: Validate all incoming data to prevent injection attacks
6. **Documentation**: Maintain up-to-date schema documentation
7. **Testing**: Include data model validation in automated tests

## Next Steps

With the data model established, we'll now create a comprehensive validation plan to ensure the accuracy and reliability of our digital twin simulations.