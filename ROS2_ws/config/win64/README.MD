# Windows ROS 2 Workspace

This directory contains scripts and utilities for setting up and managing the ROS 2 workspace for BAM (Baseball Avoidance Multi-rotor) on Windows systems. It provides a structured environment for building and running BAM ROS 2 components on Windows.

## Directory Contents

- `install_ros2_jazzy_robostack.bat`: Script to install ROS2 Jazzy using Robostack
- `load_matlab_ros_dev.bat`: Script to load the ROS2 Jazzy environment and prepare MATLAB
- `install_bam_ros_packages.bat`: Script to install BAM ROS2 packages into a workspace
- `ros_workspace_cleaner.bat`: Utility to clean and optionally rebuild ROS workspaces

## Windows Installation

### Installing ROS2 Jazzy with Pixi/[Robostack](https://robostack.github.io/GettingStarted.html)

1. Open Command Prompt as a regular user (not administrator)
2. Navigate to this directory
3. Run the installation script:
```
install_ros2_jazzy_robostack.bat
```
4. Follow the prompts to complete the installation

The script will:
- Install Pixi if not already installed
- Create a [RoboStack](https://robostack.github.io/GettingStarted.html) project in your home directory (`%USERPROFILE%\robostack`)
- Configure and install ROS2 Jazzy
- Create a convenient activation shortcut

### Using the ROS2 Jazzy Environment with Robostack 

To activate the ROS2 Jazzy environment and prepare MATLAB:

```
cd %USERPROFILE%\workspace\bam_baseball_avoidance_multirotor
ROS2\win64\load_matlab_ros_dev.bat
```

This will load the Robostack Pixi environment for ROS Jazzy in a new window and provide instructions for running MATLAB with the necessary ROS variables.

```
cd %USERPROFILE%\robostack pixi update
```


## Installing BAM ROS2 Packages

To install BAM ROS2 packages into your workspace:

```
ROS2\win64\install_bam_ros_packages.bat
```

This script will:
- Detect or create a ROS2 workspace
- Copy or link BAM packages into the workspace
- Build the packages using colcon
- Source the workspace setup file

## Cleaning the Workspace

To clean your ROS2 workspace:

```
ROS2\win64\ros_workspace_cleaner.bat
```

This utility allows you to:
- Automatically detect ROS workspaces
- Remove build, install, and log directories
- Optionally clean source packages
- Rebuild the workspace after cleaning

## Usage

After setting up the workspace, you can run the BAM ROS2 components from MATLAB Simulink. The `run_bam.bat` script in the root directory will handle most of the setup for you.

## Development

For development on Windows:

1. Make changes to the package source files in the main directories
2. Use `ros_workspace_cleaner.bat` to clean the workspace if needed
3. Rebuild the packages using `install_bam_ros_packages.bat`
4. Test your changes
5. Commit your changes to version control

## Notes

- Windows support is primarily for the MATLAB/Simulink integration
- Analysis and visualization nodes are typically run on Ubuntu
- Ensure proper network configuration for cross-platform communication