# BAM Interface Definitions

[Back..](../README.md)

This directory contains custom message and service definitions for the BAM ROS2 integration. These interfaces define the data structures used for communication between BAM/Simulink and ROS2 nodes.

## Components

- **[ros_msg_iface](./ros_msg_iface/README.md)**:  
C++ interface library providing standardized interfaces for handling ROS2 messages in the BAM system.  This library is utilized in BAM to interface Simulink s-functions with some of the ROS2 publishing nodes (the library versions of the nodes).  This Simulink s-function using a ROS2 msg interface was required during the linking process when building BAM within the MATLAB command line.  This interface:
  - Abstracts ROS2 message handling for Simulink components
  - Provides seamless conversion between Simulink data structures and ROS2 message types
  - Handles publication of messages to ROS2 topics

[Back..](../README.md)
