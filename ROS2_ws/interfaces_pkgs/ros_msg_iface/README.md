
# ROS2 Message Interface Package (`ros_msg_iface`)

[Back..](../README.md)

The `ros_msg_iface` class is an interface class for use with MATLAB S-Functions and encapsulates ROS2 functionality, specifically for the `PoseStamped` message type.  This interface:

- Abstracts ROS2 message handling for Simulink components
- Provides seamless conversion between Simulink data structures and ROS2 message types
- Handles publication of PoseStamped messages to ROS2 topics

## Usage

### Using in a MATLAB Level 2 S-Function

```cpp
// Example C++ usage of the ros_msg_iface class in a MATLAB Level 2 S-function
#include "ros_msg_iface/ros_msg_iface.hpp"

static void mdlStart(SimStruct *S)
{
  // Create an interface instance
  // Initialize ros_msg_iface
  ros_msg_iface::start();
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
  const real_T *Pos_bii_in = (real_T *) ssGetInputPortRealSignal(S, 0);
  const real_T *Q_i2b_in = (real_T *) ssGetInputPortRealSignal(S, 1);

  real_T *y = (real_T *) ssGetOutputPortRealSignal(S, 0);
  // Publish a message
  ros_msg_iface::outputs(
                         // inputs
                         static_cast<const double*>(Pos_bii_in),
                         static_cast<const double*>(Q_i2b_in),
                         // outputs
                         static_cast<double*>(y)
                        );
}

static void mdlTerminate(SimStruct *S)
{
  // Terminate the interface
  ros_msg_iface::terminate();
}
```

### Using in Another Package

To use this package in your package:

1. Add a dependency in your `package.xml`:
   ```xml
   <depend>ros_msg_iface</depend>
   ```

2. Add a dependency in your `CMakeLists.txt`:
   ```cmake
   find_package(ros_msg_iface REQUIRED)

   ament_target_dependencies(${PROJECT_NAME} ros_msg_iface)
   ```

## Building

To build the interface packages:

```bash
colcon build --packages-select ros_msg_iface
```

## Extending

This library can easily be modified by users to utilize other built in ROS2 message types and/or user defined ROS2 message types.  Users who desire to integrate different ROS2 message types into their own simulation and who require the simulation to be autocode capable (within MATLAB) should modify the ros_msg_iface.cpp source code to include and utilize the message types of their choice.  For example, the `ros_msg_iface.cpp` code currently only includes the `pose_stamped` ROS2 message type:


```cpp
#include <iostream>
#include <sstream>

#include <cmath>

// ros includes
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // DESIRED RO2 MSG TYPE HERE

#include "ros_msg_iface/ros_msg_iface.hpp"
```


Additionally, users can readily modify the publishing node for use with other topics.  Users can simply alter the node class definition with new class name, node name and new topic (see below):

```cpp
namespace ros_msg_iface
{

  class PubPoseNode : public rclcpp::Node // MODIFY CLASS NAME
  {
  public:
    PubPoseNode (const std::string& strNodeName, const std::string& strTopicName) : Node(strNodeName) // MODIFY NODE NAME
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(strTopicName, 10); //TOPIC NAME HERE
      RCLCPP_INFO(this->get_logger(), "Publish Pose Node Station has been started.");
      if (DEBUG_FLAG)
      {
        std::cout << "Constructor for "
                  << strNodeName
                  << " node is executed, topic name: " 
                  << strTopicName 
                  << std::endl; 
      }
    }

    ~PubPoseNode ()
    {
      if (DEBUG_FLAG) { std::cout << "Destructor for node is executed\n"; }
    }
...
```

The use of the `ros_msg_iface` library is required during the MATLAB and Simulink autocode process due to linking issues.  Specifically, unless the static library is built, then MATLAB is unable to find the correct ROS2 libraries during linking. It should be noted that users don't need to use the ros_msg_iface library to integrate ROS2 into their simulation.  Several non-library versions of the ROS2 nodes can be found in the publishers subfolder: `mat_airsim_pub_nonlib` and `mat_airssim_bball_pub_nonlib`.  These ROS2 nodes integrate ROS2 directly into the Simulink s-functions and can be run directly in Simulink (as long as ROS2 was sourced when opening MATLAB).  Users who wish to build the BAM simulation using these non-lib nodes can readily do so using the `Cmake` toolchain found in MATLAB R2024b to generate the source code for BAM, and then using CMAKE and/or `colcon build` to incorporate their non-library ROS2 s-function nodes into the final BAM executable.   


## Best Practices

- Keep message definitions simple and focused
- Use standard message types when possible
- Document each field in the message definition
- Test interfaces with all components before integration

[Back..](../README.md)