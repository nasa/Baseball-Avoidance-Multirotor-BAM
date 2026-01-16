# Phase Space Warping (PSW) Analysis for eVTOL Dynamics

[Back..](../README.md)


[![ROS2](https://img.shields.io/badge/ROS2-Rolling-green)](https://docs.ros.org/en/rolling/)

## Overview

The Phase Space Warping (PSW) analyzer is an advanced anomaly detection module for the BAM eVTOL simulation framework. It implements a rigorous mathematical approach to detect parameter drift, system degradation, and off-nominal behavior in flight dynamics by analyzing the evolution of the system's state in phase space.

This implementation is based on the theoretical framework established by Chelidze & Cusumano (2006) in their seminal paper "Phase space warping: nonlinear time-series analysis for slowly drifting systems." The approach has been adapted and optimized for real-time analysis of eVTOL flight dynamics within the BAM simulation environment.

## Technical Approach

### Phase Space Warping Theory

Phase Space Warping detects parameter drift by:

1. **Embedding the system state** in a higher-dimensional phase space using delay coordinates (Takens embedding)
2. **Establishing a reference model** of nominal system dynamics from initial flight data
3. **Calculating a tracking function** that measures deviations from expected behavior
4. **Computing statistical metrics** to quantify the significance of detected anomalies

Unlike traditional anomaly detection methods that rely on predefined thresholds or supervised learning, PSW is particularly effective for detecting gradual drift in complex nonlinear systems where the underlying dynamics are not fully known a priori.

### Implementation Details

The analyzer operates in the following sequence:

1. **Data Collection**: Buffers position and orientation data from the eVTOL simulation
2. **Phase Space Reconstruction**: Creates a delay-coordinate embedding of the system state
3. **Reference Model Construction**: Establishes a statistical model of nominal dynamics
4. **Tracking Function Calculation**: Computes deviations from expected behavior using local linear models
5. **Statistical Analysis**: Calculates metrics to quantify the significance of detected anomalies
6. **Results Publication**: Provides real-time feedback to the simulation environment

## Usage

### Prerequisites

- (Tested on) ROS 2 Rolling, Humble, Jazzy, Kilted
- Python 3.8+
- NumPy, SciPy, scikit-learn

### Manual Installation

This is in place of using the `install_bam_ros_packages` scripts:

```bash
# Clone the BAM repository (if not already done)
git clone <BAM_REPOSITORY>
cp -R ROS2_ws/analysis_pkgs/phase_space_warping_py <ROS_WORKSPACE>/src/

# Build the package
cd <ROS_WORKSPACE>
colcon build --packages-select phase_space_warping_py
source install/setup.bash
```

### Running the Node

```bash
# Run with default parameters
ros2 run phase_space_warping_py psw_node

# Run with custom parameters
ros2 run phase_space_warping_py psw_node --ros-args -p buffer_size:=150 -p analysis_interval_steps:=10
```

### Enabling Debug Output

```bash
# Set debug level for detailed analysis information
ros2 run phase_space_warping_py psw_node --ros-args --log-level phase_space_warping_analyzer:=debug
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `buffer_size` | 100 | Number of state vectors to buffer for analysis |
| `analysis_interval_steps` | 15 | Number of simulation timesteps between analyses |
| `embedding_dimension` | 4 | Dimension of the delay-coordinate embedding |
| `time_delay` | 4 | Time delay for the embedding |
| `reference_percentage` | 0.25 | Fraction of initial data used as reference |
| `k_neighbors` | 12 | Number of nearest neighbors for local linear models |
| `svd_threshold` | 0.95 | Variance threshold for dimensionality reduction |
| `confidence_threshold` | 0.75 | Threshold for confidence in analysis results |

## Output Metrics

The analyzer publishes a `Float32MultiArray` message to the `/phase_space_warping_analyzer` topic with the following elements:

| Index | Metric | Description | Typical Range |
|-------|--------|-------------|---------------|
| 0 | Timestep | Current simulation timestep | [0, ∞) |
| 1 | Tracking Magnitude | Average deviation from expected dynamics | [0.01, 10.0] |
| 2 | Tracking Variance | Variability of deviations | [0.001, 5.0] |
| 3 | Max Warping | Maximum deviation observed | [0.1, 20.0] |
| 4 | Confidence | Reliability of the analysis | [0, 1] |
| 5 | Significance | Statistical significance of deviations | [0.5, 10.0] |
| 6 | Drift Rate | Rate of change in deviation magnitude | [-0.1, 0.1] |

## Integration with BAM and Simulink

### Subscribing to PSW Results in Simulink

To receive PSW analysis results in Simulink:

1. Create a ROS Subscriber block in Simulink
2. Configure it to subscribe to the `/phase_space_warping_analyzer` topic
3. Set the message type to `std_msgs/Float32MultiArray`
4. Extract the data array for downstream processing

### Interpretation of Results

- **Tracking Magnitude**: Primary indicator of system health; higher values indicate greater deviation from nominal behavior
- **Confidence**: Indicates reliability of the analysis; values below 0.5 suggest the system is operating in regions not well-represented in the reference data
- **Significance**: Statistical measure of how unusual the current behavior is; values above 3.0 typically indicate anomalous behavior
- **Drift Rate**: Indicates whether the system's condition is improving (negative) or deteriorating (positive)

## Mathematical Foundation

The core PSW methodology involves:

1. **Takens Embedding**: If $\mathbf{x}(t)$ is the system state at time $t$, the delay-coordinate embedding is:
   $\mathbf{y}(t) = [\mathbf{x}(t), \mathbf{x}(t-\tau), \mathbf{x}(t-2\tau), ..., \mathbf{x}(t-(d-1)\tau)]$

2. **Local Linear Models**: For each point $\mathbf{y}_i$ in the embedded space, we predict its successor using:
   $\mathbf{y}_{i+1} = \mathbf{A}(\mathbf{y}_i)\mathbf{y}_i + \mathbf{b}(\mathbf{y}_i)$

3. **Tracking Function**: The difference between actual and predicted successors:
   $\mathbf{e}_R(\phi, \mathbf{y}_i) = \mathbf{y}_{i+1} - [\mathbf{A}(\mathbf{y}_i)\mathbf{y}_i + \mathbf{b}(\mathbf{y}_i)]$

4. **Tracking Metric**: The average norm of the tracking function:
   $\gamma(\phi) = \frac{1}{N} \sum_{i=1}^{N} \|\mathbf{e}_R(\phi, \mathbf{y}_i)\|$

## Performance Considerations

- **Computational Complexity**: O(N·k·d²) where N is the number of points, k is the number of neighbors, and d is the embedding dimension
- **Memory Usage**: Approximately (buffer_size × state_dimension × 8 bytes) for the main data buffer
- **Latency**: Typically < 50ms per analysis on modern hardware

## References

1. Chelidze, D., & Cusumano, J. P. (2006). Phase space warping: nonlinear time‐series analysis for slowly drifting systems. *Philosophical Transactions of the Royal Society A*, 364(1846), 2495-2513.

2. Takens, F. (1981). Detecting strange attractors in turbulence. In *Dynamical Systems and Turbulence*, Lecture Notes in Mathematics, vol. 898, pp. 366-381.

3. Sauer, T., Yorke, J. A., & Casdagli, M. (1991). Embedology. *Journal of Statistical Physics*, 65(3-4), 579-616.

## Authors

- **Newton Campbell** - *Initial implementation* - [newton.h.campbell@nasa.gov]

## Acknowledgments

- Chelidze & Cusumano for the original PSW methodology
- The BAM simulation framework development team

---

*For questions or support, please contact the BAM development team.*

[Back..](../README.md)
