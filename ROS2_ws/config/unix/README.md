# Unix ROS2 build tools

[Back..](../../README.md)

This directory contains scripts and utilities for building and managing ROS2
packages for Unix systems (Linux, macOS). This readme also describes how to
manually install the packages using `colcon build`.

_**Note:** `mex` binaries for publishers and subscribers are included in the
`ROS2_ws/bin` folder. The simulation can be run on its own without building the
other ROS2 packages (for example, `bam_to_airsim`). The installation scripts and
process are used to build these external packages for your system, or rebuild
the binaries in case they do not run._

## Bash scripts

### `install_bam_ros_packages`

This script installs BAM ROS2 packages into a ROS2 workspace. It creates
symbolic links from the BAM repository to the ROS2 workspace, builds the
packages using `colcon`, and sources the workspace setup file.

To use this script, first initialize a ROS2 workspace (the `ROS2_ws` directory
can be used for this), and ensure that `colcon` and `rosdep` are installed and
available. Then run
```shell
source install_bam_ros_packages.bash [options]
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

_**Note:** If you create new ROS2 packages, you may need to manually update this
script to point to them._

### `ros_workspace_cleaner`

This script provides a user-friendly way to clean ROS2 workspaces by removing
`build`, `install`, `log`, and `src` directories. It can be useful for cleanly
testing ROS2 nodes against the simulator. It includes automatic workspace
detection in common locations with comprehensive naming patterns, and safeguards
to prevent accidental data loss. Also offers automatic rebuilding of the
workspace after cleaning by running `install_bam_ros_packages.bash`.

To use this script, run
```shell
bash ros_workspace_cleaner.bash [options]
```
Options:
* `-w`, `--workspace <PATH>`  Specify ROS2 workspace path directly
* `-a`, `--airsim`            When rebuilding, install the `bam_2_airsim` package (optional, requires Airsim installation)
* `-e`, `--editable`          When rebuilding, call `colcon build` with `--symlink-install` (optional)
* `-s`, `--sudo`              When rebuilding, initialize `rosdep` using `sudo`, which may be required on some systems (optional)
* `-h`, `--help`              Display help message

## Manual installation

Here we describe the basic steps to manually build the BAM ROS2 packages.

1. Initialize a ROS2 workspace and activate the environment. See, for example,
[Robostack + pixi](https://robostack.github.io/GettingStarted.html). The
`ROS2_ws` directory can be used for this. If needed, an example conda
environment is provided in [`environment.yml`](./environment.yml).

2. Inside your ROS2 workspace, create symbolic links to the package sources:
    ```shell
    mkdir src
    BAM_ROS_DIR=<path/to/bam/ROS2_ws>
    ln -sf $BAM_ROS_DIR/analysis_pkgs/phase_space_warping_py src/
    ln -sf $BAM_ROS_DIR/interfaces_pkgs/ros_msg_iface src/
    ln -sf $BAM_ROS_DIR/launch_pkgs/bam_launcher src/
    ln -sf $BAM_ROS_DIR/s_function_pkgs/subscribers/ src/
    ln -sf $BAM_ROS_DIR/s_function_pkgs/publishers/ src/
    ln -sf $BAM_ROS_DIR/visualization_pkgs/drone_plotter_py src/
    ```
    If Airsim is installed, you can also run
    ```shell
    ln -sf $BAM_ROS_DIR/visualization_pkgs/bam_2_airsim src/
    ```
   
3. Install dependencies:
    ```shell
    rosdep init  # May require sudo
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

4. Build the packages:
    ```shell
    colcon build
    ```

5. Run the setup script:
    ```shell
    cd install
    source setup.bash
    cd ..
    ```
    Depending on how you've installed ROS2, you may get an error, but this does not
    necessarily mean that the setup failed. To check that packages were successfully
    installed, run:
    ```shell
    ros2 pkg list | grep drone
    ```
    You should see `drone_plotter_py` in the terminal.

## Usage

After setting up the workspace, you can run the BAM ROS2 components:
```shell
# Run the Phase Space Warping analyzer
ros2 run phase_space_warping_py psw_node

# Run the drone plotter
ros2 run drone_plotter_py drone_plotter

# Or launch both of these at the same time
ros2 launch bam_launcher bam_launch.py
```
Instructions for running the MATLAB/Simulink simulation with ROS2 publishing
enabled are provided in the [ROS2_ws/README](../../README.md).

[Back..](../../README.md)
