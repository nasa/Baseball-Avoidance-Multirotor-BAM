# S-Function Test Models

This directory contains test models and scripts for verifying the functionality of BAM S-functions.

## Example Test Model Setup:

### test_publisher.slx

Simulink model for testing publisher S-functions. Generates test signals and publishes them to ROS 2.

### test_subscriber.slx

Simulink model for testing subscriber S-functions. Subscribes to ROS 2 topics and displays the received data.

## Test Scripts ### test_mat_airsim_pub.m

Script for testing the AirSim publisher S-function.

### test_mat_phasespacwarpinganalyzer_sub.m

Script for testing the PSW subscriber S-function.

## Running Tests

1. Start ROS 2:
   ```bash
   # In a terminal
   ros2 daemon start
   ```

2. Run the test script in MATLAB:
   ```matlab
   cd s_functions/test
   test_mat_phasespacwarpinganalyzer_sub
   ```

3. Verify the results in the MATLAB console and Simulink scopes

## Adding New Tests

1. Create a new Simulink model or MATLAB script
2. Configure the model to use the S-function being tested
3. Add appropriate test signals and visualization
4. Document the expected behavior and test procedure
