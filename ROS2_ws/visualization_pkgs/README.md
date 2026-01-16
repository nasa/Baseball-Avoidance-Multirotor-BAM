# BAM Visualization Tools

[Back..](../README.md#visualization-packages)

This directory contains visualization tools for BAM simulation data. These tools provide visual feedback on the simulation state and analysis results.

## Packages

- [BAM to Airsim](./bam_2_airsim/Readme.md): Node package acts as a real-time server between BAM and an Airsim equipped Unreal Environment application
- [Drone Plotter](./drone_plotter_py/README.md): 3D visualization for drone trajectories

## Adding a New Visualization Tool

1. Create a new package using the ROS 2 package template:
   ```bash
   ros2 pkg create --build-type ament_python my_visualizer_py
   ```

2. Implement your visualization as a ROS 2 node
3. Add appropriate dependencies to `package.xml`
4. Update the launch files to include your new visualization

## Best Practices

- Subscribe to the appropriate topics from BAM and analysis nodes
- Use efficient visualization techniques to minimize CPU/GPU usage
- Provide interactive controls when appropriate
- Include proper error handling and logging
- Document your visualization's parameters and controls

[Back..](../README.md#visualization-packages)
