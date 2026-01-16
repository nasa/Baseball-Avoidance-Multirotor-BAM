## Reference Models and Variants Subsystems in BAM
[Back..](../README.md#api-documentation)

The BAM architecture is designed to be easily modified by the user.  To that end, the various subsystems of BAM are implemented as Simulink Reference Models.  This separates the top-level BAM simulation from the various subsystems implemented. The BAM repo comes with the following Reference Models at the top level of the simulation: aero-propulsive model, vehicle equations of motion, own-ship (and baseball) reference inputs (trajectories), motor models, BAM controller, sensors, autocode real-time pacing, simulation scopes (plots), ROS2 publishing and subscribing (own-ship and baseball) and SimOut (data logging).

Additionally, the BAM simulation makes extensive use of Simulink Variant subsystems. Users make use of the user structure: userStruct.variants to select the desired subsystem variants.  See [userStruct](../setup/README.md#structure-details) for more details on userStruct and userStruct.variants. 

Users can navigate the BAM Reference Models folder structure here:

**Reference Models**
1. [Autocode Real-Time Pacing](./Autocode_RT_Pacing/README.md): This subsystem should be used only when it is desired to perform simulation real-time pacing in **executable** code.  For normal simulation pacing, use the build in Simulink Run/Simulation pacing button  
2. [Baseball](./Baseball/README.md): Baseball scripts and simulation for baseball trajectory generation (e.g., randomized initial conditons, baseball drag and spin models etc.)
3. [Controller](./Controller/README.md): High performance quadrotor geometric controller
4. [Environment](./Environment/README.md): Atmospheric model (1976), wind, and Dryden turbulence models
5. [Plots](./Plots/README.md): Visualization capabilities native in MATLAB and Simulink
6. [Sensors](./Sensors/README.md): Sensor models 

[Back..](../README.md#api-documentation)
