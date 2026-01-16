# BAM Launch Files

[Back..](../README.md#launch-packages)

This directory contains launch files to demonstrate the use of multiple ROS2 nodes with the BAM simulation system. These launch files simplify the process of starting multiple ROS 2 nodes with the appropriate configuration.

## Packages

- **bam_launcher/**: Launch package with configuration files for BAM

## Usage

To launch the BAM system:

```bash
ros2 launch bam_launcher bam_launch.py
```

This command launches the following components:
- Drone Plotter node
- Phase Space Warping node

## Directory Structure

```
ROS2/launch/
├── bam_launcher
│   ├── bam_launcher
│   │   └── __init__.py
│   ├── launch
│   │   └── bam_launch.py
│   ├── package.xml
│   ├── resource
│   │   └── bam_launcher
│   ├── setup.cfg
│   ├── setup.py
│   └── test
│       ├── test_copyright.py
│       ├── test_flake8.py
│       └── test_pep257.py
└── README.MD
```

## Creating New Launch Files

1. Create a new `.launch.py` file in the `bam_launcher/launch/` directory
2. Define the launch configuration using the ROS 2 launch API
3. Add appropriate parameters and node configurations
4. Test the launch file with the target components

## Additional Launch Configurations

For specialized configurations, you can create additional launch files:

- For analysis components only
- For visualization components only
- For specific simulation scenarios

These can be added to the `bam_launcher/launch/` directory following the same pattern as `bam_launch.py`.

[Back..](../README.md#launch-packages)