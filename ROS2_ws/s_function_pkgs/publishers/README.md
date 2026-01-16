# Publisher S-Functions

[Back..](../README.md)

This directory contains MATLAB S-functions that publish data from Simulink to ROS2 topics.  The publisher examples provided are of two types: library and non-library.  The library s-functions utilize a static built library such that all ROS2 functionality is within the static library and a Simulink s-function merely calls the associated library functions.  The second type of s-functions utilize ROS2 directly within the Simulink s-functions.  

## S-Functions

### mat_airsim_pub (static library version)

Publishes vehicle state data from Simulink to ROS2 for use by analysis and visualization nodes.

#### Parameters
- Topic name (string): Name of the ROS2 topic to publish to
- Message type (string): Type of message to publish (e.g., 'geometry_msgs/PoseStamped')
- Queue size (int): Publisher queue size

#### Inputs
- Position (3x1): [x, y, z] position in meters
- Orientation (4x1): [qx, qy, qz, qw] quaternion
- Linear velocity (3x1): [vx, vy, vz] in m/s
- Angular velocity (3x1): [wx, wy, wz] in rad/s

#### Outputs
- Status (1x1): Publishing status (1=success, 0=failure)

## Usage

```matlab
% In your Simulink model:
% 1. Add an S-Function block
% 2. Set S-function name to 'mat_airsim_pub'
% 3. Configure parameters
% 4. Connect inputs
```

## Creating a New Publisher

1. Create a new C or MATLAB S-function file
2. Implement the required S-function methods
3. Initialize a ROS2 publisher in the `mdlStart` function
4. Publish data in the `mdlOutputs` function
5. Clean up resources in the `mdlTerminate` function

[Back..](../README.md)