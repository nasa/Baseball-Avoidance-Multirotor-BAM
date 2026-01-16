# mat_airsim_pub ROS2 Package

[Back..](../README.md/#publisher-s-functions)

## Overview
The `mat_airsim_pub` package provides a MATLAB/Simulink S-function that publishes aircraft kinematics information to ROS2 nodes. This package creates a bridge between the Simulink simulation and ROS2 nodes.

## Features
- Publishes aircraft pose (position and orientation) from Simulink to ROS2
- Uses standard `geometry_msgs/msg/PoseStamped` message format
- Generates a MATLAB MEX file that can be used as an S-function in Simulink models
- Designed to interface with AirSim or other ROS2-compatible simulation environments

## Dependencies
- ROS2 (tested with Humble/Foxy/Rolling/Jazzy)
- MATLAB/Simulink
- `rclcpp` (ROS2 C++ client library)
- `geometry_msgs` package
- `std_msgs` package

## Installation

### Prerequisites
- ROS2 installed and configured
- MATLAB/Simulink installed (required for MEX file compilation)

### Build from Source
1. Create a ROS2 workspace (if you don't have one):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository:
```bash
git clone <repository_url> mat_airsim_pub
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select mat_airsim_pub
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage
### In Simulink
1. Add the `mat_airsim_pub` S-function block to your Simulink model
2. Connect the following inputs to the block:
   - Input 1: Position vector [x, y, z] (3 elements)
   - Input 2: Quaternion [qx, qy, qz, qw] (4 elements)
3. The block will output a combined vector [x, y, z, qx, qy, qz, qw] (7 elements)

### ROS2 Interface
- **Node Name**: `pose_pub_node`
- **Published Topics**:
  - `/pub_pose` (geometry_msgs/msg/PoseStamped) - Aircraft pose information

## MEX File Generation
The CMake configuration automatically generates a MEX file using MATLAB's MEX compiler. This MEX file will be used as an S-function in Simulink.

## Message Structure
The S-function publishes `geometry_msgs/msg/PoseStamped` messages with the following structure:
```
std_msgs/Header header
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

## Configuration
You can modify the following parameters in the source code:
- Topic name (default: `pub_pose`) 
- Node name (default: `pose_pub_node`)
- Debug flag (default: `0`, set to `1` to enable debug output)

## Compatibility
This package is designed to work with the AirSim simulator by publishing pose information that can be consumed by an AirSim node. The system architecture allows for:
1. Simulink simulation producing vehicle kinematics
2. Publishing the kinematics over ROS2 topics
3. AirSim subscribing to these topics and updating the visual simulation

## Package Structure (key files)
- `src/ros_msg_sim.cpp` - Main S-function code for publishing pose data
- `CMakeLists.txt` - Build configuration for MATLAB MEX file
- `package.xml` - Package metadata and dependencies
- `mat_airsim_pub.mexa64` - mat_airsim_pub pre-packaged binary built on Ubuntu 20.04
- `mat_airsim_pub.mexw64` - mat_airsim_pub pre-packaged binary built on Windows 10
- `install_mat_airsim_pub.sh` - Helper script to install mat_airsim_pub as a package in your local ROS2 directory

[Back..](../README.md/#publisher-s-functions)
