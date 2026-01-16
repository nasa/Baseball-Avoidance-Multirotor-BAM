# Bam2Airsim ROS Node Package 

[Back..](../README.md#bam-visualization-tools)

**Version:** 1.0
---

## Overview
This node package acts as a real-time server between BAM and an Airsim equipped Unreal Environment application.
Once started, the package will start up two subscriber nodes (depending on settings): one for a multirotor aircraft and one for a baseball.
After they have received published messages, the subscriber classes will translate the data and then transmit it via Airsim's client interface.  
The change should then be visible in the Unreal Environment unless something has gone wrong during configuration.  The node logs should issue an error message if this is the case.

## Nodes
### 1. Bam2Airsim

The main node application that acts as a server between BAM and an Airsim Unreal Environment.  Once started, the application will initiate subscribers to the topics tied to BAM's multirotor and baseball pose message outputs.  Upon receiving the messages, the node will relay the information to Airsim, which will update the visualization of the the data. The update rate depends completely on the publisher transmission rate.

### 2. TestPosePub

This supplemental application provides stub data across the pose topics for Bam2Airsim to test with (or validate data connection).  The multi-rotor and baseball actors will be given circular flights around the spawn point in the Unreal Environment.

## Operation
Both applications are run like any standard ROS node application
```console
>> ros2 run bam_2_airsim_pkg Bam2Airsim <UserArgs> --ros-args <RosArgs>
```

Both applications can make use of two command line arguments:
- **-nobb** - Disables the baseball part of the node.  Useful for rotor only testing.
- **-do** - Enable debugging output, which will log the position updates into the node specific log files. (located under the *workspace/bam_2_airsim_pkg/log/*)

As an example, the following code will run Bam2Airsim with the baseball inactive.
```console
>> ros2 run bam_2_airsim_pkg Bam2Airsim -nobb
```

As another example, the following code will run TestPosePub with pose data debug logging active.
```python
>> ros2 run bam_2_airsim_pkg TestPosePub -do
```

## Requirements
- ROS2 Environment
- (For Unreal 4) Airsim Source Code & supporting libraries (Airsim - https://github.com/microsoft/AirSim)
- (For Unreal 5+) Colosseum Upgraded Airsim Code & libraries (Colosseum - https://github.com/CodexLabsLLC/Colosseum)
- Unreal Environment Application w/ Airsim Plugin Installed
    * One Airsim multi-rotor aircraft named "Drone"
    * Baseball asset "BamBaseball" installed within the Unreal environment (see BamEcho documentation)
- BAM Simulator or some other Bam compliant data publisher.
- ROS Topics Expected by the Subscriber:
    * Multirotor - "/pub_pose"
    * Baseball - "/bball_pub_pose"

## Important Developer Note
This software was developed under Unreal 5+ and Colosseum's fork of Airsim.  There could be issues if a user were to build and operate with Airsim core and Unreal 4.  There could be other issues depending on which version of Unreal 5 is used.  For example, it was found that Colosseums version of "simSpawnObject" in the c++ API was broken within the development environment, because Unreal Engine's "WorldSimApi->spawnObject()" required a slightly different function argument list than Airsim / Colosseum had bound. Alternatively, it may have been missed when Colosseum updated Airsim to Unreal 5.   If you need assistance with this or have questions, the lead developer can be contacted at daniel.r.hill@nasa.gov

## Version History
- **.8** - Initial Beta. Establishes the two basic subscribers and validity checks on Airsim / Unreal.
- **1.0** - First full release.  Handles both baseball and multirotor data transmissions to Airsim / Unreal; validated in testing.  Does environment / configuration checks to ensure Unreal is ready for BAM data.

[Back..](../README.md#bam-visualization-tools)
