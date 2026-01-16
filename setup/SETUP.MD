# userStruct

[Back to setup..](./README.md#structure-details)

## Baseball Avoidance Multirotor (BAM) UserStruct Setup Guide

This document provides a comprehensive guide to configuring the BAM simulation by adjusting the `userStruct` variable. The BAM simulation allows for detailed configuration of multirotor dynamics, environmental conditions, control systems, and visualization options.

## Configuration Structure Overview

The BAM simulation is configured through a hierarchical structure called `userStruct`. This setup guide documents all available parameters within this structure that can be modified to customize your simulation.

## Variant Configuration Options

These options control which implementation variants are used for different parts of the simulation.

### Publisher Variants

| Parameter | Description | Options |
|-----------|-------------|---------|
| `userStruct.variants.pubType` | Specifies whether or not to turn on the vehicle transceiver, which will publish vehicle data to external sources (like ROS nodes) and receive data that can be used downstream. | `PubEnum.NONE (1)`: No data publishing<br>`PubEnum.ROS2 (2)`: Publish and receive data using ROS2 messages |

### Reference Input Variants

| Parameter | Description | Options |
|-----------|-------------|---------|
| `userStruct.variants.refInputType` | Specifies the reference trajectory input method | `RefInputEnum.BEZIER (1)`: Uses Bezier curves for trajectory definition<br>`RefInputEnum.TIMESERIES (2)`: Uses time series data for trajectory definition |

### Equations of Motion Variants

| Parameter | Description | Options |
|-----------|-------------|---------|
| `userStruct.variants.eomType` | Specifies the equations of motion model to use | `EOMEnum.STARS`: Uses the STARS equations of motion model<br>`EOMEnum.Simple`: Uses simplified equations of motion |

### Atmosphere Model Variants

| Parameter | Description | Options |
|-----------|-------------|---------|
| `userStruct.variants.atmosType` | Specifies the atmospheric model to use | `AtmosphereEnum.US_STD_ATMOS_76`: Uses the U.S. Standard Atmosphere 1976 model |

### Data Output Variants

| Parameter | Description | Options |
|-----------|-------------|---------|
| `userStruct.variants.dataOutType` | Specifies the data output format | `DataOutEnum.NORMAL`: Standard data output format<br>`DataOutEnum.CODEGEN`: Code generation compatible data output format |

### Turbulence Variants

| Parameter | Description | Options |
|-----------|-------------|---------|
| `userStruct.variants.turbulenceType` | Controls turbulence modeling in the simulation | `TurbulenceEnum.None (1)`: No turbulence effects<br>`TurbulenceEnum.Enabled (2)`: Enables turbulence modeling |

## Simulation Parameters

### Model Parameters

| Parameter | Description | Units | Usage |
|-----------|-------------|-------|-------|
| `userStruct.model_params.sim_rate` | The simulation time step rate | Seconds | Controls the temporal resolution of the simulation |

### Types Configuration

| Parameter | Description | Options | Default |
|-----------|-------------|---------|---------|
| `userStruct.types.sensors.useNoise` | Flag to enable/disable sensor noise in the simulation | 0 (disabled), 1 (enabled) | 0 |

## Simulation Defaults

### Trim Configuration (Initial Conditions)

| Parameter | Description | Units | Usage |
|-----------|-------------|-------|-------|
| `userStruct.simulation_defaults.trim.vel_bIi` | Initial velocity vector in body-to-inertial frame | meters/second | Sets the initial velocity state for the multirotor |
| `userStruct.simulation_defaults.trim.acc_i` | Initial acceleration vector in inertial frame | meters/second² | Sets the initial acceleration state for the multirotor |
| `userStruct.simulation_defaults.trim.psi` | Initial yaw angle | radians | Sets the initial heading direction of the multirotor |
| `userStruct.simulation_defaults.trim.pos_i` | Initial position vector in inertial frame | meters | Sets the starting position of the multirotor |
| `userStruct.simulation_defaults.trim.psi_d` | Initial yaw rate | radians/second | Sets the initial yaw rotational velocity |

### Environment Configuration - Turbulence

| Parameter | Description | Values/Units | Usage |
|-----------|-------------|-------------|-------|
| `userStruct.simulation_defaults.Environment.Turbulence.RandomSeedLong` | Random seed for longitudinal turbulence generation | Integer (e.g., 3366+521) | Ensures repeatable turbulence patterns for longitudinal axis |
| `userStruct.simulation_defaults.Environment.Turbulence.RandomSeedLat` | Random seed for lateral turbulence generation | Integer (e.g., 23+521) | Ensures repeatable turbulence patterns for lateral axis |
| `userStruct.simulation_defaults.Environment.Turbulence.RandomSeedVert` | Random seed for vertical turbulence generation | Integer (e.g., 1369+521) | Ensures repeatable turbulence patterns for vertical axis |
| `userStruct.simulation_defaults.Environment.Turbulence.intensity` | Turbulence intensity level | 1: None<br>2: Light<br>3: Moderate<br>4: Severe | Controls the magnitude of atmospheric turbulence effects |
| `userStruct.simulation_defaults.Environment.Turbulence.WindAt5kft` | Wind speed at 5,000 feet altitude | knots | Sets the reference wind speed for the simulation |
| `userStruct.simulation_defaults.Environment.Turbulence.WindDirectionAt5kft` | Wind direction at 5,000 feet altitude | degrees from North | Sets the reference wind direction for the simulation |

### Container Parameters

These parameters serve as containers for more specific configuration parameters:

| Parameter | Description | Usage |
|-----------|-------------|-------|
| `userStruct.simulation_defaults.Environment.Atmosphere` | Container for atmospheric model parameters | Configure atmospheric conditions for the simulation |
| `userStruct.simulation_defaults.Environment.Wind` | Container for wind model parameters | Configure wind conditions for the simulation |
| `userStruct.simulation_defaults.Vehicle` | Container for vehicle configuration parameters | Configure physical properties of the multirotor vehicle |
| `userStruct.simulation_defaults.FCS` | Container for flight control system parameters | Configure controller gains, limits, and behavior |
| `userStruct.simulation_defaults.RefInputs` | Container for reference trajectory parameters | Configure desired path parameters based on selected reference input type |

## Output Configuration

| Parameter | Description | Options | Default |
|-----------|-------------|---------|---------|
| `userStruct.outputFunc` | Specifies the output function to use for simulation data collection | 'userOutFull': Full simulation output<br>'userOutFullwithTraffic': Full simulation output with traffic data<br>Custom function name (user-defined) | 'userOutFull' |

## Information Structure

| Parameter | Description | Default | Usage |
|-----------|-------------|---------|-------|
| `userStruct.info.paths.model_name` | Name of the Simulink model | Current model name | Used for reference and documentation |
| `userStruct.info.date` | Timestamp of when the simulation was configured | Current date and time | Used for reference and documentation |

## Usage Notes

1. Configure your simulation by modifying the `userStruct` variable before running the setup script.
2. The setup script will process your configuration and initialize the simulation environment accordingly.
3. For advanced usage, you can modify the relevant setup functions directly.

## Example Configuration

```matlab
% Basic configuration example
userStruct = struct();

% Set model parameters
userStruct.model_params.sim_rate = 0.01; % 100 Hz simulation

% Configure variants
userStruct.variants.pubType = PubEnum.ROS2; % Use ROS2 for publishing
userStruct.variants.refInputType = RefInputEnum.BEZIER; % Use Bezier curves for trajectory
userStruct.variants.eomType = EOMEnum.Simple; % Use simplified equations of motion
userStruct.variants.atmosType = AtmosphereEnum.US_STD_ATMOS_76; % Use standard atmosphere
userStruct.variants.dataOutType = DataOutEnum.NORMAL; % Use normal data output format
userStruct.variants.turbulenceType = TurbulenceEnum.None; % No turbulence modeling

% Configure sensor settings
userStruct.types.sensors.useNoise = 0; % Disable sensor noise

% Set initial conditions
userStruct.simulation_defaults.trim.vel_bIi = [0; 0; 0]; % Zero initial velocity
userStruct.simulation_defaults.trim.acc_i = [0; 0; 0]; % Zero initial acceleration
userStruct.simulation_defaults.trim.psi = 0; % Zero initial yaw
userStruct.simulation_defaults.trim.pos_i = [0; 0; 100]; % Start at 100m altitude
userStruct.simulation_defaults.trim.psi_d = 0; % No initial yaw rate

% Configure environment
userStruct.simulation_defaults.Environment.Turbulence.intensity = 1; % No turbulence
userStruct.simulation_defaults.Environment.Turbulence.WindAt5kft = 0; % No wind
userStruct.simulation_defaults.Environment.Turbulence.WindDirectionAt5kft = 0;
userStruct.simulation_defaults.Environment.Turbulence.RandomSeedLong = 3366+521;
userStruct.simulation_defaults.Environment.Turbulence.RandomSeedLat = 23+521;
userStruct.simulation_defaults.Environment.Turbulence.RandomSeedVert = 1369+521;

% Run the setup script
setup;
```

[Back to setup..](./README.md#structure-details)