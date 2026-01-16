# BAM Analysis Algorithms

[Back..](../README.md#analysis-packages)


This directory contains analysis algorithms for processing BAM simulation data. These algorithms extract insights, detect anomalies, and provide feedback to the simulation.  This node is an example of how users can receive data (messages) from BAM for use with their own algorithm (Python in this instance).

## Packages

- [**phase_space_warping_py**](./phase_space_warping_py/README.md): Implementation of Phase Space Warping for anomaly detection in eVTOL dynamics
 
## Adding a New Analysis Algorithm

1. Create a new package using the ROS 2 package template:
   ```bash
   ros2 pkg create --build-type ament_python my_algorithm_py
   ```

2. Implement your algorithm as a ROS 2 node
3. Add appropriate dependencies to `package.xml`
4. Update the launch files to include your new algorithm

## Best Practices

- Subscribe to the appropriate topics from BAM
- Process data efficiently to minimize latency
- Publish results in a standardized format
- Include proper error handling and logging
- Document your algorithm's parameters and outputs

[Back..](../README.md#analysis-packages)
