## External Sensors

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)

External sensors in BAM are provided via Airsim and Colosseum.  Some of these systems are turned on by default unless otherwise specified; namely the Barometer, Magnetometer, GPS, and IMU.  However, to enable some other features, such as LIDAR point clouds, they will need to be enabled and configured in an Airsim settings JSON file.

Note: The BAM model has a simulink reference subsystem "Sensors" which is currently just a sensor passthrough.  This subsystem is provided to allow users to conveniently modify vehicle onboard sensor data as they see fit (e.g., apply sensor noise).

### Available Sensor Platforms

The following sensor modules are available for activation on a quadrotor vehicle (in Colosseum and Airsim):
- Camera
- Barometer
- Inertial Measurement Unit (IMU)
- Magnetometer
- GPS
- Distance Sensor (critical for enabling depth related camera views)
- LIDAR

Each of these sensors have a series of parameters that can be configured by the user in the "settings.json" file used with Airsim. 

Similarly, each sensor has a simple API interface call in C++ / python to access the data provided.  For example, to attain barometric data, in python a user would call
```python
data = client.getBarometerData()
```
to get the timestamped pressure, altitude, and QNF values for the default quadrotor vehicle.

More detailed information (including the lists of parameters for configuring each individual sensor) can be found in the Airsim / Colosseum documentation online at:
- [Microsoft Airsim Sensors Documentation](https://microsoft.github.io/AirSim/sensors/)
- [CodexLabsLLC Colosseum Sensors Documentation](https://codexlabsllc.github.io/Colosseum/sensors/)

Both sites are essentially the same, but they have both been provided for convenience of researchers and developers.

### Example of Sensor Enabling

The following is an excerpt from an airsim JSON file that enables various features of Airsim.  This example focuses on showing the configuration of a few standard sensors.

```json
{
  ...
  "DefaultSensors": {
    "Barometer": {
      "SensorType": 1,
      "Enabled": true,
      "PressureFactorSigma": 0.001825,
      "PressureFactorTau": 3600,
      "UncorrelatedNoiseSigma": 2.7,
      "UpdateLatency": 0,
      "UpdateFrequency": 50,
      "StartupDelay": 0
    },
    "Imu": {
      "SensorType": 2,
      "Enabled": true,
      "AngularRandomWalk": 0.3,
      "GyroBiasStabilityTau": 500,
      "GyroBiasStability": 4.6,
      "VelocityRandomWalk": 0.24,
      "AccelBiasStabilityTau": 800,
      "AccelBiasStability": 36
    },
    "Gps": {
      "SensorType": 3,
      "Enabled": true,
      "EphTimeConstant": 0.9,
      "EpvTimeConstant": 0.9,
      "EphInitial": 25,
      "EpvInitial": 25,
      "EphFinal": 0.1,
      "EpvFinal": 0.1,
      "EphMin3d": 3,
      "EphMin2d": 4,
      "UpdateLatency": 0.2,
      "UpdateFrequency": 50,
      "StartupDelay": 1
    },
    "Magnetometer": {
      "SensorType": 4,
      "Enabled": true,
      "NoiseSigma": 0.005,
      "ScaleFactor": 1,
      "NoiseBias": 0,
      "UpdateLatency": 0,
      "UpdateFrequency": 50,
      "StartupDelay": 0
    },
    "Distance": {
      "SensorType": 5,
      "Enabled": true,
      "MinDistance": 0.2,
      "MaxDistance": 50.0,
      "X": 0,
      "Y": 0,
      "Z": -1,
      "Yaw": 0,
      "Pitch": 0,
      "Roll": 0,
      "DrawDebugPoints": false
    }
  },
  ...
```

Further examples of JSON files with specific platform sensors enabled can be found under the documentation for BamEcho under [\<root>/BamEcho/AirsimJsonFiles/](../../BamEcho/AirsimJsonExamples/)

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)
