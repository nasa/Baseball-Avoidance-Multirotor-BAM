// This s-function is used to publish the desired baseball pose information 
// from simulink for use with airsim.  It makes use of the built in ros2
// message type: 
// ***************************************
// geometry_msgs/msg/PoseStamped
// std_msgs/Header header
//        builtin_interfaces/Time stamp
//                int32 sec
//                uint32 nanosec
//        string frame_id
// Pose pose
//        Point position
//                float64 x
//                float64 y
//                float64 z
//        Quaternion orientation
//                float64 x 0
//                float64 y 0
//                float64 z 0
//                float64 w 1
// **************************************

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME mat_airsim_bball_pub_nonlib

#define NUM_INPUTS 2
#define NUM_OUTPUTS 1
#define DEBUG_FLAG 1

#include "simstruc.h"
#include <iostream>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" // DESIRED RO2 MSG TYPE HERE

class PubBballPoseNode : public rclcpp::Node // MODIFY CLASS NAME
{
public:
  PubBballPoseNode() : Node("pose_pub_bball_node") // MODIFY NODE NAME
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("bball_pub_pose",10); //TOPIC NAME HERE
    	RCLCPP_INFO(this->get_logger(), "Publish Bball Pose Node Station has been started.");
	if (DEBUG_FLAG){std::cout << "Constructor for node is executed\n";}
    }
    virtual ~PubBballPoseNode()
    {
      if (DEBUG_FLAG){std::cout << "Destructor for node is executed\n";}
         // publisher_.reset(); //Uncomment for Linux
    }
 
    void publishPose(double* pos, double* quat)
	{
	  auto msg = geometry_msgs::msg::PoseStamped();
    // Set the position in the message
	msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];
    // Set the quaternion in the message
    msg.pose.orientation.x = quat[0];
    msg.pose.orientation.y = quat[1];
    msg.pose.orientation.z = quat[2];
    msg.pose.orientation.w = quat[3];

	  publisher_->publish(msg);
	}
private:	
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_; //create publisher
};

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }
        
    // Set the number of continuous and discrete states to zero
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumPWork(S,1); // reserve 1 elements in the pointers vector for c++ object

    // If number of actual input ports doesn't match defined inputs, exit..
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

        /* in */
    ssSetInputPortWidth(S, 0, 3);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    //ssSetInputPortRequiredContiguous(S, 0, 1);


    ssSetInputPortWidth(S, 1, 4);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    //ssSetInputPortRequiredContiguous(S, 1, 1);


    // If number of actual output ports doesn't match defined outputs, exit...
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;

    /* out */
    ssSetOutputPortWidth(S, 0, 7);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    ssSetNumSampleTimes(S, 0);
    
    // Set the how s-function save and restore handled (restart)
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    
    /* Set this S-function as runtime thread-safe for multicore execution */
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);

    // Set s-function options (see Matlab documentation for various options)
    ssSetOptions(S,SS_OPTION_CALL_TERMINATE_ON_EXIT);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specify that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
  ssSetModelReferenceSampleTimeDefaultInheritance(S);
}


#undef MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
  /* Function: mdlInitializeConditions ========================================
   * Abstract:
   *    In this function, you should initialize the continuous and discrete
   *    states for your S-function block.  The initial states are placed
   *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
   *    You can also perform any other initialization activities that your
   *    S-function may require. Note, this routine will be called at the
   *    start of simulation and if it is present in an enabled subsystem
   *    configured to reset states, it will be call when the enabled subsystem
   *    restarts execution to reset the states.
   */
  static void mdlInitializeConditions(SimStruct *S)
  {
  }
#endif /* MDL_INITIALIZE_CONDITIONS */



#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START) 
  /* Function: mdlStart =======================================================
   * Abstract:
   *    This function is called once at start of model execution. If you
   *    have states that should be initialized once, this is the place
   *    to do it.
   */
  static void mdlStart(SimStruct *S)
  {
    if (!rclcpp::ok()){
      rclcpp::init(0,nullptr); // Initialize ROS2
    }
    if (DEBUG_FLAG){std::cout << "Bball Node is being created..\n";}
    ssGetPWork(S)[0] = (void *)  new PubBballPoseNode; // creates the new object
   
    if (DEBUG_FLAG){std::cout << "Exiting mdlStart...\n";}
  }
#endif /*  MDL_START */

static void mdlOutputs(SimStruct *S, int_T tid)
{
    PubBballPoseNode *node = (PubBballPoseNode *) ssGetPWork(S)[0];

    double pos [3];
    double quat[4];

    InputRealPtrsType u = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType u2 = ssGetInputPortRealSignalPtrs(S,1);
    real_T *y = ssGetOutputPortRealSignal(S,0);

    // Copy over the values
    pos[0] = *u[0];
    pos[1] = *u[1];
    pos[2] = *u[2];

    quat[0] = *u2[0];
    quat[1] = *u2[1];
    quat[2] = *u2[2];
    quat[3] = *u2[3];

    // Output the values...
    y[0] = *u[0]; // set the first output 
    y[1] = *u[1]; // set the first output
    y[2] = *u[2]; // set the first output
    y[3] = *u2[0]; // set the first output
    y[4] = *u2[1]; // set the first output
    y[5] = *u2[2]; // set the first output
    y[6] = *u2[3]; // set the first output
   if (DEBUG_FLAG){
    std::cout << "pos = [" << y[0] << ", " << y[1] << ", " << y[2] << "]" << std::endl;
    std::cout << "quat = [" << y[3] << ", " << y[4] << ", " << y[5] << ", " << y[6] << "]" << std::endl;
   }

   node->publishPose(pos, quat);
}

static void mdlTerminate(SimStruct *S){
    if (DEBUG_FLAG){std::cout << "Starting model terminate..\n";}
      
    if (DEBUG_FLAG){std::cout << "Deleting Bball Node..\n";}

    // Assign the node in this function so it can be deleted
    PubBballPoseNode *node = static_cast<PubBballPoseNode *>(ssGetPWorkValue(S, 0));

    // NOTE: on linux, uncomment the line below and it correctly destroys node
    // so node doesn't appear using > ros2 node list
    // However in Windows, uncommenting delete node causes crash (likely double delete or similar)
    
    // delete node;  // Uncomment in linux
    //ssSetPWorkValue(S, 0, nullptr);

    // Shutdown rclcpp
    if (DEBUG_FLAG){std::cout << "Shutting Down rclcpp.." << std::endl;}
    if (rclcpp::ok()){
        rclcpp::shutdown(); // Terminate ROS2
    }
  
    if (DEBUG_FLAG){std::cout << "Exiting model terminate.." << std::endl;}
}


#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif