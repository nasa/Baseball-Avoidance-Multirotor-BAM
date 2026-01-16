// This s-function is used to publish the desired pose information 
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
#define S_FUNCTION_NAME mat_airsim_pub

#define NUM_INPUTS 2
/* Input Port  0 */
#define IN_PORT_0_NAME        Pos_bii_in
#define INPUT_0_WIDTH         3
#define INPUT_DIMS_0_COL      1
#define INPUT_0_DTYPE         real_T
#define INPUT_0_COMPLEX       COMPLEX_NO
#define IN_0_DIMS             2-D
#define INPUT_0_FEEDTHROUGH   1
/* Input Port  1 */
#define IN_PORT_1_NAME        Q_i2b_in
#define INPUT_1_WIDTH         4
#define INPUT_DIMS_1_COL      1
#define INPUT_1_DTYPE         real_T
#define INPUT_1_COMPLEX       COMPLEX_NO
#define IN_1_DIMS             2-D
#define INPUT_1_FEEDTHROUGH   1
#define NUM_OUTPUTS 1
/* Output Port  0 */
#define OUT_PORT_0_NAME       y
#define OUTPUT_0_WIDTH        7
#define OUTPUT_DIMS_0_COL     1
#define OUTPUT_0_DTYPE        real_T
#define OUTPUT_0_COMPLEX      COMPLEX_NO
#define OUT_0_FRAME_BASED     FRAME_NO
#define OUT_0_DIMS            2-D

#include "simstruc.h"
#include <iostream>
#include <sstream>

#include "ros_msg_iface/ros_msg_iface.hpp"



static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    }
        
    // Set the number of continuous and discrete states to zero
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    // If number of actual input ports doesn't match defined inputs, exit..
    if (!ssSetNumInputPorts(S, NUM_INPUTS)) return;

    /* Input Port 0 */
    ssSetInputPortWidth(S, 0, INPUT_0_WIDTH);
    ssSetInputPortDataType(S, 0, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 0, INPUT_0_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 0, INPUT_0_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 0, 1); /*direct input signal access*/

    /* Input Port 1 */
    ssSetInputPortWidth(S, 1, INPUT_1_WIDTH);
    ssSetInputPortDataType(S, 1, SS_DOUBLE);
    ssSetInputPortComplexSignal(S, 1, INPUT_1_COMPLEX);
    ssSetInputPortDirectFeedThrough(S, 1, INPUT_1_FEEDTHROUGH);
    ssSetInputPortRequiredContiguous(S, 1, 1); /*direct input signal access*/

    // If number of actual output ports doesn't match defined outputs, exit...
    if (!ssSetNumOutputPorts(S, NUM_OUTPUTS)) return;

    /* Output Port 0 */
    ssSetOutputPortWidth(S, 0, OUTPUT_0_WIDTH);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);
    ssSetOutputPortComplexSignal(S, 0, OUTPUT_0_COMPLEX);

    ssSetNumPWork(S,1); // reserve 1 element in the pointers vector for c++ object

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
    ros_msg_iface::start("pose_pub_node","pub_pose");
  }
#endif /*  MDL_START */

static void mdlOutputs(SimStruct *S, int_T tid)
{
  const real_T *Pos_bii_in = (real_T *) ssGetInputPortRealSignal(S, 0);
  const real_T *Q_i2b_in = (real_T *) ssGetInputPortRealSignal(S, 1);

  real_T *y = (real_T *) ssGetOutputPortRealSignal(S, 0);
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
  ros_msg_iface::terminate();
}


#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
