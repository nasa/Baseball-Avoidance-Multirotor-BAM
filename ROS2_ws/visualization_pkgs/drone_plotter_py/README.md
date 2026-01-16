Drone Plotter Visualization Package – README

[Back..](../README.md#bam-visualization-tools)

[![ROS2](https://img.shields.io/badge/ROS2-Rolling-green)](https://docs.ros.org/en/rolling/)

Overview
--------
This package provides a ROS2 node for visualizing a drone’s trajectory and orientation in three dimensions. The node subscribes to the `/pub_pose` topic (which should publish messages of type geometry_msgs/PoseStamped) and uses rclpy for ROS2 integration and matplotlib for live 3D plotting. The computed orientation is based on a quaternion measurement (assuming the drone’s forward direction is along its body x-axis) and is rendered as an arrow on the plot.

This package is part of a larger repository that is in the process of being open-sourced. All source code in this repository is vetted for open-source release in accordance with NASA's software release process. Sensitive internal information is excluded.

Directory Structure
-------------------
The package is organized as follows:

```
drone_plotter_py/
├── drone_plotter_py
│   ├── drone_plotter.py       # Main ROS2 node source code
│   └── __init__.py
├── package.xml                # Package metadata and dependency declarations
├── resource
│   └── drone_plotter_py       # Package resource file (e.g., for icons, if applicable)
├── setup.cfg               	# Configuration for packaging and linting
├── setup.py                   # Python setup script for installation
└── test
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

Key functionality in the source code (drone_plotter.py):
• Initializes a ROS2 node named “drone_plotter.”
• Subscribes to the `/pub_pose` topic, receiving PoseStamped messages.
• Maintains an array of received positions to plot the drone’s trajectory.
• Computes and updates a forward orientation vector based on the most recent quaternion.
• Utilizes a separate thread to run matplotlib’s interactive plotting loop so that ROS2 spinning remains uninterrupted.

Installation and Building
-------------------------
Prerequisites:
• ROS2 (e.g., ROS Rolling or another supported distribution) must be installed and configured.
• Ensure that your Python environment includes the necessary dependencies: rclpy, geometry_msgs, matplotlib, numpy, and any other listed in package.xml and setup.cfg.
• A ROS2 workspace with a typical structure (with top-level src, build, devel, and optionally install directories).

Steps to build:
1. Clone the larger repository’s open-source branch (or the specific subdirectory for drone_plotter_py) into your ROS2 workspace’s src folder. For example:
   ```
   git clone <repository-URL>
   cd ~/ros_ws/src
   cp -R $LOCAL_REPO_DIRECTORY/ROS2/ubuntu/ros2_ws/src/drone_plotter_py .
   ```
2. Build the workspace using colcon:
   ```
   cd ~/ros_ws
   colcon build --symlink-install
   ```
3. Source the resulting setup file:
   ```
   source install/setup.bash
   ```

Usage
-----
After building and sourcing your workspace, launch the drone plotter node with:
```
ros2 run drone_plotter_py drone_plotter
```
The node opens an interactive matplotlib window titled “Drone Visualization” that displays the drone’s trajectory along with its orientation (green arrow) and current position (highlighted in red). Terminate the node with Ctrl+C.

Development and Contributing
-----------------------------
Contributions and enhancements are welcome. Please ensure that any changes adhere to the coding standards and guidelines outlined in the repository documentation. Automated tests are provided in the test/ directory to maintain code quality (including tests for PEP 257, flake8 compliance, etc.).

Feedback and Support
--------------------
For questions, bug reports, or enhancement requests, please refer to the repository’s issue tracker on GitLab.

Disclaimer
----------
This open-source package has been prepared for public release as part of the larger NASA repository. No ITAR, EAR, or PII data is included within this repository.

This README serves as an introduction and guide for both open-source users and NASA professionals working with the drone_plotter_py package.

Feedback on this open-source release and documentation is welcome via the repository’s issue tracker or the provided internal channels.

[Back..](../README.md#bam-visualization-tools)
