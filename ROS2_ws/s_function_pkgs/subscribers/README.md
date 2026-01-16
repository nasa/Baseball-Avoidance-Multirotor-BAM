# Subscriber S-Functions

[Back..](../README.md)


This directory contains MATLAB S-functions that subscribe to ROS 2 topics and bring data into Simulink.

## S-Functions

### psw_sub

Subscribes to Phase Space Warping analysis results and provides them to Simulink.

#### Parameters
- Topic name (string): Name of the ROS 2 topic to subscribe to (Default: '/phase_space_warping_analyzer')

#### Outputs
- PSW data (7x1): [timestep, tracking_magnitude, tracking_variance, max_warping, confidence, significance, drift_rate]

## Usage

```matlab
% In your Simulink model:
% 1. Add an S-Function block
% 2. Set S-function name to 'psw_sub' or 'psw_sub_mex'
% 3. Configure parameters
% 4. Connect outputs to downstream blocks
```

## Creating a New Subscriber

The following steps are an overview of creating a user-defined subscriber.  For detailed instructions on creating user-defined subscriber and publisher nodes see: [Creating User Nodes](./CONTRIBUTING.MD).

1. Create a new C or MATLAB S-function file
2. Implement the required S-function methods
3. Initialize a ROS 2 subscriber in the `mdlStart` function
4. Process received messages in a callback function
5. Output the latest data in the `mdlOutputs` function
6. Clean up resources in the `mdlTerminate` function

[Back..](../README.md)
