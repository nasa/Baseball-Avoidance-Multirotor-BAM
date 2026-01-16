[Back to main](../README.md#api-documentation)

# BAM Example Files

This folder contains a collection of example scenario scripts for the Baseball Avoidance Multirotor (BAM) simulation. The files: `example_template.m`, `example_customTrajectory.m`, `example_circle.m`, and `example_landing.m` are intended to familiarize users with the process of setting up and running the BAM simulation but also to demonstrate methods for using user-generated trajectory configurations. The script `example_modifyIC.m` demonstrates how users can perturb the multirotor initial conditions after the setup process but prior to running BAM.  The `example_collision_avoidance.m` script shows users how to run select collision scenarios from the `ChallengeProblem` folder.  Specifically, this file shows users how to publish own-ship and baseball pose information via ROS2 for one of the near-collision challenge problem scenarios. All of these example scripts are BAM focused and don't cover the use of Colosseum and Unreal Engine.

**NOTE:** Example files are intended to be run from the root directory to preserve paths and dependencies. Please move the files to the root directory and run the file in that location with the MATLAB 'Current Folder' set to the root directory. 


Additionally, this folder contains a few example `.bat` and `.bash` scripts that show how a user could run the BAM simulation and the associated ROS2 publishing from a system script.  


## example_template.m

Minimal template for creating new simulation scenarios.
- Basic structure with path checks and workspace cleanup
- Documentation sections
- User parameter placeholders
- Setup call and execution code
- Visualization commands



## example_customTrajectory.m

Demonstrates custom waypoint-based trajectory creation.
- Custom XYZ position waypoints with timing
- Piecewise Bezier trajectory generation
- Takeoff, climb and turn profile
- Trajectory visualization


## example_circle.m

Creates a circular flight path trajectory.
- 7-waypoint circular configuration
- Combined horizontal and vertical movement
- Complete 360° path with smooth transitions
- Visualization and performance plotting



## example_landing.m

Shows a controlled landing trajectory.
- Climb profile terminated and immediately lands
- Landing profile visualization

## example_modifyIC.m

Shows how to modify own-ship initial conditions prior to simulation execution
- Uses `setup.m` to initialize default Bernstein polynomial trajectory
- Uses `SimPar` to modify initial position 
- **NOTE:** The geometric controller is robust, but large initial condition variations from the desired starting conditions can result in the controller driving the vehicle unstable

## example_collision_avoidance.m

Advanced collision avoidance scenario setup.
- Multi-object trajectory configuration
- Selectable "own-ship" and "baseball" trajectories, from challenge problem
- Collision scenario simulation
- Optional ROS2 publishing support
- Part of the Collision Avoidance Challenge Problem

[Back to main](../README.md#api-documentation)