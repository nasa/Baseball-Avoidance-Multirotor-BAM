/**
 * @file psw_sub.cpp
 * @brief ROS2 Subscriber S-function for Phase Space Warping data
 * @author Newton Campbell
 * @date 04/09/2025
 *
 * This S-function subscribes to a ROS2 topic containing Phase Space Warping
 * analysis data and makes it available to a MATLAB Simulink model.
 * Built for Robostack ROS2 Jazzy environment.
 */

 #define S_FUNCTION_NAME psw_sub
 #define S_FUNCTION_LEVEL 2
 
 #include "simstruc.h"
 #include <rclcpp/rclcpp.hpp>
 #include <std_msgs/msg/float32_multi_array.hpp>  // Changed to Float32MultiArray to match publisher
 #include <string>
 #include <vector>
 #include <mutex>
 #include <memory>
 
 // Global variables to maintain state between calls
 static std::shared_ptr<rclcpp::Node> node = nullptr;
 static std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>> subscription;
 static std::vector<float> latestData; // Buffer for the most recent data
 static std::mutex dataMutex;          // Mutex to protect data access
 static bool dataReceived = false;     // Flag to indicate if data has been received
 
 // Hardcoded topic name to match the publisher
 static const std::string TOPIC_NAME = "/phase_space_warping_analyzer";
 
 /**
  * Callback function for ROS2 subscriber
  * Processes incoming messages from the PSW analyzer topic
  */
 void messageCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
 {
     // Lock to prevent data race conditions
     std::lock_guard<std::mutex> lock(dataMutex);
 
     // Update our local copy of the data
     latestData.clear();
     latestData.insert(latestData.end(), msg->data.begin(), msg->data.end());
     dataReceived = true;
 }
 
 /**
  * Initialize ROS2 if not already initialized
  * @return true if initialization successful, false otherwise
  */
 static bool initializeROS()
 {
     if (!rclcpp::ok())
     {
         // Initialize ROS2 context
         rclcpp::init(0, nullptr);
     }
 
     if (node == nullptr)
     {
         // Create a node with a unique name
         node = std::make_shared<rclcpp::Node>("simulink_psw_subscriber_node");
     }
 
     return rclcpp::ok();
 }
 
 /**
  * Set up the ROS2 subscriber to the specified topic
  * @return true if setup successful, false otherwise
  */
 static bool setupSubscriber()
 {
     if (!rclcpp::ok() || node == nullptr)
     {
         return false;
     }
 
     // Create subscription with quality of service settings
     rclcpp::QoS qos(10); // Queue size of 10
     subscription = node->create_subscription<std_msgs::msg::Float32MultiArray>(
         TOPIC_NAME,
         qos,
         messageCallback);
 
     return subscription != nullptr;
 }
 
 /**
  * Clean up ROS2 resources
  */
 static void cleanupROS()
 {
     // Reset subscription
     subscription.reset();
 
     // Reset node
     if (node != nullptr)
     {
         node.reset();
     }
 
     // Shutdown ROS2 if it's running
     if (rclcpp::ok())
     {
         rclcpp::shutdown();
     }
 }
 
 /**
  * Initialize sizes of input/output ports and parameters
  * Required method for Simulink S-functions
  */
 static void mdlInitializeSizes(SimStruct *S)
 {
     // No parameters needed anymore
     ssSetNumSFcnParams(S, 0);
     if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
     {
         return; // Parameter mismatch error
     }
 
     // No continuous or discrete states
     ssSetNumContStates(S, 0);
     ssSetNumDiscStates(S, 0);
 
     // No input ports - we're only subscribing
     ssSetNumInputPorts(S, 0);
 
     // One output port for PSW data
     ssSetNumOutputPorts(S, 1);
 
     // Output port is dynamically sized based on incoming data
     // Initially set to width 7 to match the publisher's data format
     ssSetOutputPortWidth(S, 0, 7);
     ssSetOutputPortDataType(S, 0, SS_DOUBLE);
 
     // Sample times
     ssSetNumSampleTimes(S, 1);
 
     // No work vectors needed
     ssSetNumRWork(S, 0);
     ssSetNumIWork(S, 0);
     ssSetNumPWork(S, 0);
     ssSetNumModes(S, 0);
     ssSetNumNonsampledZCs(S, 0);
 
     // Options
     ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
 }
 
 /**
  * Initialize sample times for the S-function
  * Required method for Simulink S-functions
  */
 static void mdlInitializeSampleTimes(SimStruct *S)
 {
     // Use inherited sample time from the model
     ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
     ssSetOffsetTime(S, 0, 0.0);
 }
 
 /**
  * Initialize the S-function block
  * Called once at the start of model execution
  */
 #define MDL_START
 static void mdlStart(SimStruct *S)
 {
     // Initialize ROS2 and subscriber
     if (!initializeROS())
     {
         ssSetErrorStatus(S, "Failed to initialize ROS2");
         return;
     }
 
     if (!setupSubscriber())
     {
         ssSetErrorStatus(S, "Failed to setup ROS2 subscriber");
         return;
     }
 }
 
 /**
  * Calculate outputs for the S-function
  * Called at each simulation time step
  */
 static void mdlOutputs(SimStruct *S, int_T tid)
 {
     // Process any pending ROS2 messages
     if (rclcpp::ok())
     {
         rclcpp::spin_some(node);
     }
 
     // Get output port signal
     real_T *y = ssGetOutputPortRealSignal(S, 0);
 
     // Copy latest data to output
     {
         std::lock_guard<std::mutex> lock(dataMutex);
         if (dataReceived)
         {
             // If this is the first time we've received data or size has changed,
             // resize the output port
             if (ssGetOutputPortWidth(S, 0) != latestData.size())
             {
                 ssSetOutputPortWidth(S, 0, latestData.size());
             }
 
             // Copy data to output
             for (size_t i = 0; i < latestData.size(); i++)
             {
                 y[i] = latestData[i];
             }
         }
         else
         {
             // No data received yet, output zeros
             for (int i = 0; i < ssGetOutputPortWidth(S, 0); i++)
             {
                 y[i] = 0.0;
             }
         }
     }
 }
 
 /**
  * Perform tasks at termination of simulation
  * Clean up resources
  */
 static void mdlTerminate(SimStruct *S)
 {
     // Clean up ROS2 resources
     cleanupROS();
 }
 
 // Required S-function trailer
 #ifdef MATLAB_MEX_FILE
 #include "simulink.c"
 #else
 #include "cg_sfun.h"
 #endif