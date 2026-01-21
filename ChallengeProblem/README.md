# **Collision Avoidance Challenge Problem**

[Back..](../README.md#api-documentation)

## **The BAM Simulation**
The intent behind the release of the Baseball Avoidance Multirotor (BAM) simulation was to provide the aerospace autonomy research community 
a simulation framework which would enable researchers to focus on an unsolved autonomy problem of autonomous collision avoidance (ACA).  It is hoped that by releasing an open-source complete simulation, autonomy researchers can compare and collaborate using a common simulation framework.  Specifically, this software release assists in addressing many of the research barriers to comparison and collaboration such as: limited staffs and resources, access to relevant sensor models, and proprietary data or vehicles.  The BAM simulation software release provides:

1. A multi-rotor (quad) simulation framework with extensive aerospace bus structures and aerospace blocks
2. A robust quadrotor geometric controller 
3. Motor dynamics models
4. Environmental conditions (i.e., winds and turbulence)
5. Ease with autocoding and batch running of simulation (including real time executables)
6. Easy integration of external algorithms (e.g., Python and c++) into Simulink using Robot Operating System 2 (ROS2)
7. High fidelity graphics and sensor models using CodexLabsLLC [Colosseum](https://github.com/CodexLabsLLC/Colosseum) and Unreal Engine
    * High fidelity sensors (e.g., camera, lidar) with Colosseum
    * Premium environmental modeling with Unreal Engine 5
    * APIs for easy use of sensors  

A simple quadrotor vehicle was specifically chosen for this simulation as these vehicles are ubiquitous and have well known, very capable existing control algorithms.  It is better to first demonstrate real-time ACA on simple vehicles as opposed to more complex flight vehicles (e.g., electrified propulsion transition vehicles) which are more challenging to control.  Additionally, this work will facilitate incorporating real-time ACA algorithms on multi-rotor hardware.


## **Autonomous Collision Avoidance**

Autonomous collision avoidance can be subdivided into the sequential problems of: perception, prediction, and planning.  Each of these areas is an entire research field onto itself and as such, there is an expectation that one research group might not be on the cutting edge in research across all three areas (which is a strong argument for collaboration!)  Accordingly, the BAM simulation is structured in such a  way that researchers can tackle each sub-problem separately or in combination with one or more of the others.   

The BAM simulation was designed to provide very high fidelity visualization and sensor modeling capability.  Specifically, the use of CodexLabsLLC Colosseum coupled with Unreal Engine V5+ allows for photorealistic rendering with the most up-to-date weather representations (e.g., fog, smoke, visibility etc.)  The use of Colosseum allows users to utilize APIs to easily access various sensors (e.g., camera, Lidar) and easily specify sensor properties via json files etc.  The BAM simulation is released with an Unreal Engine environment executable which allows users to (out of the box) obtain high fidelity sensor data with realistic weather to challenge their perception algorithms.  Moreover, it is easy for users to add sensor noise and or pointing bias/noise.  

Once an object is perceived, ACA requires generating a prediction of the objects trajectory in order to forecast its' position in the near future.  The assumption of trajectory linearity (e.g., continuation on last known velocity in the near term) is not sufficient for ACA.  For this reason, a series of baseball trajectories are provided with BAM.  These trajectories are not merely parabolic (gravity only).  They include velocity opposed drag modeling and baseball spin effects.  Additionally, these trajectories are generated with a host of random parameters: baseball drag coefficient, spin rate and axis, gravity, initial velocities etc.  These trajectories should be rich enough to challenge trajectory prediction algorithms, while providing easily interrogatable known trajectories (Bernstein polynomials) that facilitate algorithm performance analysis.  Moreover, the scripts used to generate these trajectories are provided to enable users to modify them and even make much larger data sets (necessary for training machine learning models).  

The final subtask in ACA is planning. Planning is the process given an uncertain own-ship intended trajectory and an uncertain, predicted object trajectory, a planning algorithm is needed to perform real-time (dynamically feasible) own-ship trajectory replanning to avoid a collision scenario.  The BAM simulation comes with 3000 own-ship trajectories and baseball trajectory near-collision scenarios.  Researchers can increase the level of difficulty by adding in enviromental effects (wind and turbulence) as well as using the SimPAR (MATLAB parameter object) features of BAM to vary own-ship parameters (e.g., inertial properties, mass, rotor performance).  In this way, BAM can readily accommodate very challenging real-time collision avoidance planning (and control) algorithms.  

## **Challenge Problem**
Now with easy access to an open-source simulation framework, and associated data sets (as well as generation scripts), we challenge the autonomous collision avoidance community to demonstrate their best algorithms for solving ACA in real-time.  It is our hope that this Gitlab repo will serve as the foundation of an active research community who will publish their results for others to baseline against. Moreover, we hope this simulation tool fosters collaboration among researchers on aspects of ACA beyond their current capabilities.  Our goal is to advance the state-of-the-art for ACA with both dynamic feasibility and safety guarantees under uncertainty.  Researchers who solve all three subproblems in real time will have definitely **hit for the cycle!**

## **Trajectory Generation and Data Sets**

The `ChallengeProblem` folder contains the following:

- Own-ship trajectory generation script: `Create_OwnShip_Traj.m`
- Own-ship trajectories and meta-data: `ref_trajectory_data.mat`
- Baseball trajectory generation script: `Create_Baseball_Traj.m`
- Baseball trajectories and meta-data: `bball_trajectory_data.mat`
- Plotting script to visual own-ship & baseball trajectories: `Plot_Chal_Prob_DSets.m`
- Stand-alone baseball dynamics simulation: `Dynamics.slx`
- Folder `Chal_Prob_Videos`: folder with example video files generated using FFMPEG for a trajectory using BAM, Colosseum, and Unreal Engine to obtain camera image data.   

[Back..](../README.md#api-documentation)
