# Creating User Nodes

[Back to s-function packages..](../README.md)

[Back to subsciber..](./README.md)

# Contributing to ROS2 Jazzy S-functions for MATLAB Simulink with Robostack

This guide provides comprehensive instructions for developing ROS2 Jazzy subscriber S-functions for MATLAB Simulink using the Robostack environment. It aims to be a self-contained reference that minimizes the need to consult external documentation.

## Table of Contents

1. [Introduction](#introduction)
2. [Prerequisites](#prerequisites)
3. [Robostack Environment Setup](#robostack-environment-setup)
4. [S-function Architecture](#s-function-architecture)
5. [Development Workflow](#development-workflow)
6. [ROS2 Subscriber Implementation](#ros2-subscriber-implementation)
7. [Data Type Handling](#data-type-handling)
8. [Thread Safety](#thread-safety)
9. [Compilation and Integration](#compilation-and-integration)
10. [Testing and Debugging](#testing-and-debugging)
11. [Advanced Techniques](#advanced-techniques)
12. [Troubleshooting](#troubleshooting)

## Introduction

This document provides guidelines for creating ROS2 Jazzy subscriber S-functions that integrate with MATLAB Simulink using the Robostack environment. These S-functions enable Simulink models to receive and process data from ROS2 topics in real-time.

## Prerequisites

### Required Software

- MATLAB R2022a or newer with Simulink
- Conda package manager
- C++17 compatible compiler
- CMake 3.5+

### Knowledge Requirements

- Basic understanding of ROS2 concepts
- Familiarity with C++ programming
- Understanding of Simulink modeling principles

## Robostack Environment Setup

### Installing Robostack

```bash
# Create a new conda environment for ROS2 Jazzy
conda create -n ros2_jazzy python=3.9
conda activate ros2_jazzy

# Add required channels
conda config --env --add channels conda-forge
conda config --env --add channels robostack
conda config --env --add channels robostack-experimental

# Install ROS2 Jazzy
conda install ros-jazzy-desktop

# Verify installation
ros2 --version
```

### Configuring MATLAB to Use Robostack

1. Ensure MATLAB can find the Robostack environment:

```matlab
% Add Robostack binaries to MATLAB path
if ispc
    setenv('PATH', [getenv('CONDA_PREFIX') '\Library\bin;' getenv('PATH')]);
else
    setenv('PATH', [getenv('CONDA_PREFIX') '/bin:' getenv('PATH')]);
end

% Set ROS2 environment variables
setenv('AMENT_PREFIX_PATH', getenv('CONDA_PREFIX'));
setenv('ROS_VERSION', '2');
setenv('ROS_PYTHON_VERSION', '3');
setenv('ROS_DISTRO', 'jazzy');

% Verify ROS2 is accessible from MATLAB
!ros2 --version
```

2. Configure MEX compiler to use compatible C++ compiler:

```matlab
mex -setup C++
```

## S-function Architecture

### Core Components

A ROS2 subscriber S-function consists of these essential components:

1. **ROS2 Context Management**: Initialize and manage ROS2 lifecycle
2. **Node Creation**: Create a ROS2 node for communication
3. **Subscription Handling**: Subscribe to specified topics
4. **Message Processing**: Process incoming messages
5. **Data Buffer Management**: Store and synchronize data
6. **Simulink Interface**: Expose data to Simulink model

### Standard S-function Structure

```cpp
#define S_FUNCTION_NAME  YourSFunctionName
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <rclcpp/rclcpp.hpp>
// Include message type headers

// Global variables for ROS2 communication
static std::shared_ptr<rclcpp::Node> node = nullptr;
static std::shared_ptr<rclcpp::Subscription</* message type */>> subscription;
static std::mutex dataMutex;
static std::vector<double> latestData;
static bool dataReceived = false;

// ROS2 callback function
void messageCallback(const /* message type */::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    // Process message data
    dataReceived = true;
}

// ROS2 initialization function
static bool initializeROS() { /* ... */ }

// ROS2 subscription setup
static bool setupSubscriber(const std::string& topicName) { /* ... */ }

// ROS2 cleanup function
static void cleanupROS() { /* ... */ }

// Required S-function methods
static void mdlInitializeSizes(SimStruct *S) { /* ... */ }
static void mdlInitializeSampleTimes(SimStruct *S) { /* ... */ }
#define MDL_START
static void mdlStart(SimStruct *S) { /* ... */ }
static void mdlOutputs(SimStruct *S, int_T tid) { /* ... */ }
static void mdlTerminate(SimStruct *S) { /* ... */ }

// Required S-function trailer
#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
```

## Development Workflow

### 1. Define Requirements

- Identify the ROS2 topic to subscribe to
- Determine the message type
- Define output port configuration
- Establish timing requirements

### 2. Create S-function File

Create a new C++ file with the S-function template structure.

### 3. Implement ROS2 Functionality

Implement the core ROS2 functionality:
- ROS2 initialization
- Node creation
- Subscription setup
- Message callback

### 4. Implement S-function Methods

Implement the required S-function methods:
- `mdlInitializeSizes`: Define parameters, inputs, and outputs
- `mdlInitializeSampleTimes`: Set timing behavior
- `mdlStart`: Initialize ROS2 components
- `mdlOutputs`: Process ROS2 messages and update outputs
- `mdlTerminate`: Clean up ROS2 resources

### 5. Compile and Test

Compile the S-function and test it in a Simulink model.

## ROS2 Subscriber Implementation

### Initializing ROS2

```cpp
static bool initializeROS()
{
    // Initialize ROS2 if not already initialized
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }
    
    // Create node if it doesn't exist
    if (node == nullptr) {
        node = std::make_shared<rclcpp::Node>("simulink_subscriber_node");
    }
    
    return rclcpp::ok();
}
```

### Creating a Subscription

```cpp
// For std_msgs::msg::Float64MultiArray example
static bool setupSubscriber(const std::string& topicName)
{
    if (!rclcpp::ok() || node == nullptr) {
        return false;
    }
    
    // Set QoS profile
    rclcpp::QoS qos(10);  // Queue size of 10
    
    // Create subscription
    subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        topicName,
        qos,
        [](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(dataMutex);
            latestData.clear();
            latestData.insert(latestData.end(), msg->data.begin(), msg->data.end());
            dataReceived = true;
        }
    );
    
    return subscription != nullptr;
}
```

### Processing Messages

```cpp
// In mdlOutputs function
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Process any pending ROS2 messages
    if (rclcpp::ok()) {
        rclcpp::spin_some(node);
    }
    
    // Get output port signal
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    
    // Copy latest data to output
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        if (dataReceived) {
            // If output port size needs to be updated
            if (ssGetOutputPortWidth(S, 0) != latestData.size()) {
                ssSetOutputPortWidth(S, 0, latestData.size());
            }
            
            // Copy data to output
            for (size_t i = 0; i < latestData.size(); i++) {
                y[i] = latestData[i];
            }
        } else {
            // No data received yet
            y[0] = 0.0;
        }
    }
}
```

## Data Type Handling

### Common ROS2 Message Types and Their Simulink Equivalents

| ROS2 Type | Simulink Type | Conversion Notes |
|-----------|---------------|-----------------|
| bool | boolean_T | Direct mapping |
| int8, uint8 | int8_T, uint8_T | Direct mapping |
| int16, uint16 | int16_T, uint16_T | Direct mapping |
| int32, uint32 | int32_T, uint32_T | Direct mapping |
| int64, uint64 | int64_T, uint64_T | Direct mapping |
| float32 | real32_T | Direct mapping |
| float64 | real_T | Direct mapping |
| string | char_T[] | Requires special handling |
| array | Vector/Matrix | Requires dimension handling |

### Handling Complex Message Types

For complex message types, extract the relevant fields and convert them to appropriate Simulink types:

```cpp
void messageCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Extract position data
    positionData[0] = msg->position.x;
    positionData[1] = msg->position.y;
    positionData[2] = msg->position.z;
    
    // Extract orientation data (quaternion)
    orientationData[0] = msg->orientation.w;
    orientationData[1] = msg->orientation.x;
    orientationData[2] = msg->orientation.y;
    orientationData[3] = msg->orientation.z;
    
    dataReceived = true;
}
```

## Thread Safety

ROS2 callbacks run in separate threads, requiring proper synchronization with Simulink's execution thread:

```cpp
// Global mutex for thread synchronization
static std::mutex dataMutex;

// In callback function
void messageCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Lock mutex before accessing shared data
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Update shared data
    // ...
}

// In mdlOutputs function
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Lock mutex before accessing shared data
    std::lock_guard<std::mutex> lock(dataMutex);
    
    // Access shared data
    // ...
}
```

## Compilation and Integration

### Compiling the S-function

```matlab
% Compile S-function with Robostack ROS2
mex('YourSFunction.cpp', ...
    ['-I' getenv('CONDA_PREFIX') '/include'], ...
    ['-L' getenv('CONDA_PREFIX') '/lib'], ...
    '-lrclcpp', '-lrcutils', '-lrosidl_runtime_c', ...
    '-lrosidl_typesupport_cpp', '-std=c++17');
```

### Integrating with Simulink

1. Add an S-Function block from the Simulink library
2. Set the S-function name to your compiled S-function
3. Configure parameters (e.g., topic name)
4. Connect the output ports to other blocks in your model

## Testing and Debugging

### Testing with ROS2 Publishers

1. Start a ROS2 publisher in a separate terminal:
   ```bash
   conda activate ros2_jazzy
   ros2 topic pub /test_topic std_msgs/msg/Float64MultiArray "data: [1.0, 2.0, 3.0]"
   ```

2. Monitor topic activity:
   ```bash
   ros2 topic echo /test_topic
   ```

### Debugging S-functions

1. Add debug prints to your S-function:
   ```cpp
   #ifdef MATLAB_MEX_FILE
   mexPrintf("Debug message: %s\n", message.c_str());
   mexEvalString("drawnow;"); // Flush print buffer
   #endif
   ```

2. Use MATLAB's debugger with MEX files:
   ```matlab
   mex -g YourSFunction.cpp ... % Compile with debug symbols
   ```

## Advanced Techniques

### Quality of Service (QoS) Configuration

```cpp
// Configure QoS settings
rclcpp::QoS qos(10);
qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
qos.durability(rclcpp::DurabilityPolicy::Volatile);
qos.deadline(std::chrono::milliseconds(100));

// Create subscription with custom QoS
subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    topicName, qos, messageCallback);
```

### Dynamic Message Type Handling

```cpp
// Function to create appropriate subscription based on message type string
static bool createSubscription(const std::string& topicName, const std::string& msgType)
{
    if (msgType == "std_msgs/msg/Float64") {
        subscription = node->create_subscription<std_msgs::msg::Float64>(
            topicName, 10, &float64Callback);
    }
    else if (msgType == "std_msgs/msg/Float64MultiArray") {
        subscription = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            topicName, 10, &float64MultiArrayCallback);
    }
    // Add more message types as needed
    else {
        return false;
    }
    
    return true;
}
```

### Handling Multiple Subscriptions

```cpp
// Global variables for multiple subscriptions
static std::vector<std::shared_ptr<rclcpp::SubscriptionBase>> subscriptions;

// Function to add a new subscription
template<typename MsgType>
static bool addSubscription(const std::string& topicName, 
                           void (*callback)(const typename MsgType::SharedPtr))
{
    auto sub = node->create_subscription<MsgType>(topicName, 10, callback);
    if (sub) {
        subscriptions.push_back(sub);
        return true;
    }
    return false;
}
```

## Troubleshooting

### Common Issues and Solutions

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| MEX compilation fails | Missing ROS2 headers | Verify Robostack paths in compilation command |
| S-function crashes at initialization | ROS2 not initialized properly | Check ROS2 environment variables |
| No data received | Topic name mismatch | Verify topic name with `ros2 topic list` |
| Data type errors | Message type mismatch | Ensure correct message type headers are included |
| Simulink freezes | Blocking ROS2 operations | Use `spin_some()` instead of `spin()` |
| Memory leaks | Resources not properly cleaned up | Implement proper cleanup in `mdlTerminate` |

### Debugging ROS2 Communication

```bash
# List all active topics
ros2 topic list

# Check topic type
ros2 topic type /your_topic

# Monitor message rate
ros2 topic hz /your_topic

# View topic content
ros2 topic echo /your_topic
```

---

By following this guide, you should be able to create robust ROS2 Jazzy subscriber S-functions for MATLAB Simulink using the Robostack environment. For further assistance or to report issues, please contact the project maintainers.

[Back to s-function packages..](../README.md)

[Back to subsciber..](./README.md)