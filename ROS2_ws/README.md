# BAM ROS2 Workspace

[Back..](../README.md#api-documentation)

## Overview
This directory contains ROS2 packages for the BAM (Baseball Avoidance MultiRotor) project. These packages integrate with MATLAB/Simulink for simulation and analysis of multi-rotor vehicles avoiding dynamic obstacles.

ROS2 is used within BAM for several different reasons:  

1. ROS2 is utilized to publish ROS2 messages from Simulink (see `s_function_pkgs/publishers`).  Specifically, these publishing nodes publish both own-ship and baseball pose information to the ROS2 topics `pub_pose` and `bball_pub_pose`.
2. The bam_2_airsim is a cpp executable which subscribes to the published POSE information and pipes it to Airsim or Colloseum via their APIs for use with the provided Unreal Engine environment executable.
3. User modifiable example nodes are also provided including: `phase_space_warping_py` and `s_function_pkgs/subscribers`.  The `phase_space_warping_py` node is an example to show users how to implement an external python node to receive data from MATLAB in real-time.  The subscriber node shows users how to pipe information from an external node back into the Simulink simulation.
4. The `drone_plotter_py` node is a visualization tool that plots the simulation output.  The `bam_launcher` is a helper tool to launch multiple nodes, specifically both the `phase_space_warping_py` node and the `drone_plotter_py` node.

Please note that the ROS nodes in the folder are not all required for use with BAM.  For users that only intend to run the BAM Simulink simulation including the Unreal Engine visualization, only three ROS2 nodes are used.  The first two nodes are publishing nodes (in the form of Simulink s-functions) used to publish the own-ship and baseball POSE information (`mat_airsim_pub` and ` mat_airsim_bball_pub` respectively) found in the `s_function_pkgs/publishers` folder.  The final node required for the BAM to Unreal visualization pipeline is the `bam_2_airsim` node/executable found in the `visualization_pkgs/bam_2_airsim` folder. This node subscribes to the POSE publishing nodes and pipes this data thru Colosseum (Airsim) APIs to the Unreal Environment. Additionally, the own-ship and baseball publisher nodes come in two varieties: library and non-library.  The library version of these nodes packages all ROS2 into a stand-alone static library that is called by the respective publishing s-function.  The non-library nodes just incorporate ROS2 directly into the respective s-function.  The static library version of the publishing nodes were required to autocode the BAM simulation (including the ROS2 publishing) *from the MATLAB command line*. The non-library versions can be built into a full BAM executable but this is performed outside of MATLAB.  For more information on building BAM within MATLAB see [AutoCodeModels](../AutoCodeModels/README.md).

However, for users who wish to solve the collision avoidance challenge problem provided with this repository, we have provided additional sample nodes (`phase_space_warping_py` and `s_function_pkgs/subscribers`) so users can see how they can integrate external algorithms via ROS and then within Simulink subscribe to the output from those external algorithms. These types of nodes will be required for users who wish to demonstrate real-time collision avoidance within the BAM and Unreal framework. For example, users who get familiar with these nodes should be able to pipe information from Simulink to external applications and send information back to BAM for execution such that a new Bernstein trajectory is flown which avoids collision, or perhaps the external algorithm bypasses the controller and directly controls the vehicle effectors to avoid collision.

### Directory Structure

```
├── analysis_pkgs/       # Packages for analyzing simulation data
│   └── phase_space_warping_py/  # Python implementation of phase space warping analysis
├── config/              # Configuration and setup scripts
│   ├── unix/            # Unix (Linux, macOS)-specific installation scripts and config
│   └── win64/           # Windows-specific installation scripts and config
├── interfaces_pkgs/     # ROS2 message and service interfaces
│   └── ros_msg_iface/   # C++ interface for ROS2 messaging
├── launch_pkgs/         # Launch packages for coordinating nodes
│   └── bam_launcher/    # Main launcher for BAM system components
├── s_function_pkgs/     # MATLAB/Simulink integration packages
│   ├── publishers/      # S-functions to publish data from MATLAB/Simulink to ROS2
│   ├── subscribers/     # S-functions to subscribe to ROS2 topics within MATLAB/Simulink
│   └── test/            # Testing utilities for S-functions
└── visualization_pkgs/  # Visualization tools and packages
    └── bam_2_airsim/      # Airsim/Unreal Visualization
    └── drone_plotter_py/  # Python-based trajectory visualization
```

### Key Components

#### [Analysis Packages](./analysis_pkgs/README.md)
Example of advanced algorithms for analyzing simulation data:
- **[phase_space_warping_py](./analysis_pkgs/phase_space_warping_py/README.md)**: Example python implementation of phase space warping for trajectory analysis

#### [Interface Packages](./interfaces_pkgs/README.md)
Custom ROS2 message and service definitions:
- **[ros_msg_iface](./interfaces_pkgs/ros_msg_iface/README.md)**: C++ library providing standardized interfaces for BAM communications.  This library was necessary when building the BAM simulation using the MATLAB codegen process.  This library interface was not required when users build the simulation outside the MATLAB process.

#### [Launch Packages](./launch_pkgs/README.md)
Coordinated system startup files:
- **bam_launcher**: Example launch package which starts up `phase_space_warping_py` and `drone_plotter_py` nodes

#### [S-Function Packages](./s_function_pkgs/README.md)
MATLAB/Simulink integration components:

- **[Publishers](./s_function_pkgs/publishers/README.md)**: S-functions that publish data from MATLAB/Simulink to ROS2
  - **[mat_airsim_pub](./s_function_pkgs/publishers/mat_airsim_pub/README.md)**: Interface between MATLAB and AirSim
  - **mat_airsim_bball_pub**: Baseball trajectory publisher
  - **mat_airsim_pub_nonlib**: Interface between MATLAB and AirSim
  - **mat_airsim_bball_pub_nonlib**: Baseball trajectory publisher
- **[Subscribers](./s_function_pkgs/subscribers/README.md)**: S-functions for receiving ROS2 data in MATLAB/Simulink
  - **psw_sub**: Phase Space Warping subscriber

#### [Visualization Packages](./visualization_pkgs/README.md)
Data visualization tools:
- **[bam_2_airsim](./visualization_pkgs/bam_2_airsim/README.md)**: Ros Node server (and test node publisher) for transmitting BAM data to an Unreal Airsim environment.
- **[drone_plotter_py](./visualization_pkgs/drone_plotter_py/README.md)**: Python-based visualization for drone trajectories

#### [Creating User Nodes](./s_function_pkgs/subscribers/CONTRIBUTING.MD) 
This link provides detailed instructions for users who wish to create and use their own ROS2 nodes within BAM. 

---

## Prerequisites

- ROS2 distribution (Humble or newer recommended)
- MATLAB/Simulink (R2022b or newer recommended)
- colcon build tools
- Python 3.8+ (for Python-based packages)

---

## Running BAM with ROS2

This repo includes pre-built `mex` binaries for publishers and subscribers in the `bin` folder. These allow the MATLAB/simulink simulation can be run on its own without building the external ROS2 packages (`phase_space_warping_py`, `drone_plotter_py`, `bam_to_airsim`). This section describes how to run MATLAB/Simulink with*out* the external nodes.

1. Initialize a ROS2 workspace and activate the environment. See, for example, [Robostack + pixi](https://robostack.github.io/GettingStarted.html). The `ROS2_ws` directory can be used for this. On Windows, the batch script `config\win64\load_ros2_dev.bat` can be used.

   Prior to running BAM with ROS2, we suggest verifying your ROS2 environment successfully runs the [Turtlesim](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html) ROS2 example package.

2. In the ROS2 environment, start MATLAB from the command line.
   #### Windows
   You can use the helper script:
   ```
   call config\win64\load_matlab_ros_dev.bat
   ```
   Or from a developer's command prompt:
   ```
   C:/<path/to/matlab>/bin/matlab
   ```
   #### Linux
   ```bash
   /usr/local/MATLAB/R2024b/bin/matlab
   ```
   #### macOS
   ```zsh
   /Applications/MATLAB_R2024b.app/bin/matlab
   ```

3. Verify that MATLAB has access to ROS2 by entering
   ```matlab
   !ros2
   ```
   in the MATLAB command window.

4. In the [`userStruct`](../setup/README.md) for your BAM simulation (see e.g., [`example_collision_avoidance.m`](../Examples/example_collision_avoidance.m)), set
   ```matlab
   userStruct.variants.pubType = 2;
   userStruct.variants.pubTypeBball = 2;
   ```

5. Open a second command window and activate ROS2.

6. Run the BAM MATLAB/Simulink simulation. While the simulation is running, in
   the second command window, enter
   ```shell
   ros2 topic echo /pub_pose
   ```
   You should see printouts of the multirotor's position and attitude quaternion (`pub_pose` topic messages output by BAM). For example,
   ```shell
   ---
   header:
   stamp:
      sec: 0
      nanosec: 0
   frame_id: ''
   pose:
   position:
      x: 57.56432813568191
      y: -29.893639731060922
      z: -49.67726236167068
   orientation:
      x: 0.8783494938256347
      y: -0.004965431178318116
      z: 0.013104739037961673
      w: -0.47781353790437125
   ```

### Troubleshooting

In step 6, if instead of position and attitude messages, you get an error message like
```
WARNING: topic [pub_pose] does not appear to be published yet
Could not determine the type for the passed topic
```
try the following troubleshooting steps:

1. Verify that MATLAB has access to ROS2 as in step 3 above.
2. Verify that `userStruct` has set `userStruct.variants.pubType = 2;` to publish `pub_pose` messages.
3. In the Simulink block diagram, verify that the `ROS2` block has the `ROS2 Publisher` variant subsystem enabled. Within this subsystem, you should be able to see scopes displaying multirotor position and attitude.
4. Instead of running the simulation with the play button, use the _step forward_ command to advance one timestep, then try step 6 again. If you don't see the above error, you should be able to run the simulation as normal and see `pub_pose` messages.

If you get a MATLAB error that says that S-functions are not available, refer to
the [Installation](#installation) section below.

---

## Installation

If you want to run the example external packages 
(`phase_space_warping_py`, `drone_plotter_py`, `bam_to_airsim`), or if the
provided `mex` binaries do not run on your system, you will need to build them.
The `config` directory provides scripts for this purpose.

This section presents an overview of how to use the provided installation
scripts. For detailed information, see
[`config/win64/README.md`](config/win64/README.md) (Windows) or
[`config/unix/README.md`](config/unix/README.md) (Linux, macOS).

### [Windows](config/win64/README.md)

The `config/unix` directory contains build tools for Windows. These include
- `install_bam_ros_packages.bat`: Main installation script.
- `load_ros2_dev.bat`: Script to set up ROS2 development environment
- `load_matlab_ros_dev.bat`: Script to configure MATLAB for ROS2 development
- `ros_workspace_cleaner.bat`: Tool to clean and rebuild ROS2 workspaces

To use `install_bam_ros_packages.bat`, first initialize a ROS2 workspace (the
`ROS2_ws` directory can be used for this), and ensure that `colcon` and `rosdep`
are installed and available. Then run
```
call config\win64\load_ros2_dev.bat
call config\win64\install_bam_ros_packages.bat [-w|--workspace <ros_workspace_path>]
```

### [Unix (Linux, macOS)](config/unix/README.md)

The `config/unix` directory contains build tools for Unix (Linux, macOS)
systems. These include
- `install_bam_ros_packages.bash`: Main installation script.
- `ros_workspace_cleaner.bash`: Tool to clean and rebuild ROS2 workspaces.
- `environment.yml`: Example Conda environment specification

To use `install_bam_ros_packages.bash`, first initialize a ROS2 workspace (the
`ROS2_ws` directory can be used for this), and ensure that `colcon` and `rosdep`
are installed and available. Then run
```shell
source config/unix/install_bam_ros_packages.bash [options]
```
Options:
* `-w`, `--workspace <PATH>`  Specify ROS2 workspace path directly
* `-a`, `--airsim`            Install the `bam_2_airsim` package (optional, requires Airsim installation)
* `-e`, `--editable`          Call `colcon build` with `--symlink-install` (optional)
* `-s`, `--sudo`              Initialize `rosdep` using `sudo`, which may be required on some systems (optional)
* `-h`, `--help`              Display help message

If no workspace is specified, the script will attempt to find an existing ROS2
workspace in common locations. If no workspace is found, it will create one at
`config/unix/ros_ws`.

To check that packages were successfully installed, run:
```shell
ros2 pkg list | grep drone
```
You should see `drone_plotter_py` in the terminal.

---

## Running BAM ROS2 nodes

After [installing](#installation) the BAM ROS2 nodes, execute steps [1-5 as
described above](#running-bam-with-ros2). Then follow these steps.

6. In the second command window, make sure the installed packages are available
   to ROS2.

   #### Windows
   You can use the helper script:
   ```
   call config\win64\load_ros2_dev.bat
   ```
   #### Linux, macOS
   ```shell
   cd <path/to/ros/workspace>/install
   source setup.bash
   ```

7. Launch the BAM ROS2 nodes:
   ```shell
   ros2 launch bam_launcher bam_launch.py
   ```
   The Python nodes that subscribe to and publish back to BAM should now be launched.

8. Run the BAM MATLAB/Simulink simulation. You should see the multirotor's
   trajectory plotted in a pop-up window, and printouts from the phase space
   warping analyzer in the second command window.

### Troubleshooting

If issues arise during testing, check the following:

1. Ensure ROS2 environment is properly loaded in both terminals
2. Verify that all BAM ROS2 packages are correctly installed
3. Check that the correct paths are being used for scripts and commands
4. Examine command outputs for error messages

## Contributing

When contributing to this project:
- Follow the existing directory structure
- Create appropriate unit tests for new functionality
- Update relevant `README.md` files to document changes
- Ensure cross-platform compatibility (Windows/Linux/macOS)

[Back..](../README.md#api-documentation)
