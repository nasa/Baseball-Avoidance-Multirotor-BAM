# BAM S-Functions for ROS 2

[Back..](../README.md#s-function-packages)

This directory contains MATLAB S-functions for integrating BAM/Simulink with ROS 2. These S-functions enable bidirectional communication between Simulink models and ROS 2 nodes (without the use of MATLAB ROS2 toolbox). Users who plan to work on the `ChallengeProblems` will need to make use of publishing and subscribing nodes in conjunction with their own external algorithms to alter the multi-rotor trajectory and/or controller to avoid collisions.

## Directories

- **publishers/**: S-functions that publish data from Simulink to ROS 2
  - **mat_airsim_pub/**: Publisher for AirSim simulation data

- **subscribers/**: S-functions that subscribe to ROS 2 topics and bring data into Simulink
  - Contains subscribers for analysis results and other ROS 2 data

- **test/**: Test models and scripts for S-functions

## Building S-Functions

To build a C/C++ S-function:

```matlab
mex -setup C++
cd s_functions/publishers/mat_airsim_pub
mex mat_airsim_pub.c
```

For MATLAB S-functions, no compilation is needed.

## Usage

1. Add the S-function directory to your MATLAB path:
   ```matlab
   addpath(genpath('path/to/ROS2/s_functions'));
   ```

2. Use the S-function in your Simulink model:
   - Drag an S-Function block from the Simulink library
   - Set the S-function name to the desired function (e.g., `mat_psw_sub`)
   - Configure parameters as needed

## Best Practices

- Handle errors gracefully in S-functions
- Use appropriate sample times for real-time performance
- Document parameters and ports clearly
- Test S-functions thoroughly before integration

[Back..](../README.md#s-function-packages)
