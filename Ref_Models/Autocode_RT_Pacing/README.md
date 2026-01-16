## Autocode Real-Time Pacing

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)

The Autocode_RT_Pacing reference model contains a variant subsystem with two variants: None and RT_Pacing.  The none option is the default selection: 

**userStruct.variants.rt_pacing = RT_NONE**
 where the `PaceEnum.m` enumeration has the options: RT_NONE and RT_PACE. 

As its' name implies, RT_NONE selection does not slow down Simulink execution and therefore the entire BAM simulation executes at the fastest permissible speed based on user hardware. Conversely, selecting RT_PACE slows down Simulink execution to the pace specified by the associated mask parameter.  

The real-time pacing variant uses a Simulink MEX block with a system timer that slows down Simulink execution to the specified real-world pace. This mex and the associated source code can be obtained via the Mathworks code sharing at:
[Simulink Block for Real Time Execution](https://www.mathworks.com/matlabcentral/fileexchange/30953-simulink-block-for-real-time-execution?s_tid=prof_contriblnk)

 
[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)