# BamEcho: Airsim JSON Examples

## Overview

The files contained in this folder demonstrate some sample configurations for Airsim as it is loaded up in an Unreal Environment.  The user will need to either leverage one of these examples or create their own that will need to be readily available for Airsim when it is run.  

The JSON file can be made available to the Unreal Environment in a few ways:

1. Place the file under '<%UserProfile%>/Documents/Airsim/'.   Airsim will naturally look here when starting up under Windows.  For the default location on other platforms, see the Airsim Documentation on Settings: https://microsoft.github.io/AirSim/settings/#where-are-settings-stored

2. Place the file in the same directory as the binary for the Airsim Unreal Environment.

3. Place the file in the same directory where the Airsim Unreal Environment will be started up.  Specifically, if a command or script will launch the application in a different working directory than where it actually resides; put the file there.

4. Pass the path to a specific JSON file in as a command line argument for the Unreal Environment application.  For example: 'AirSimEnvironment.exe -settings="<%PathToSettings%>\settings.json"

## Notes

The Airsim settings file placed in directories as "default" locations for Airsim to find will need to be named *settings.json*.  Files name otherwise, like the examples here, will not be found otherwise.

The Airsim documentation goes into more depth on the topic of settings that can be configured, but for Echo's purposes it might be useful to focus on the following

- *CameraDefaults / CameraSettings* - The subgroupings in this section will set the image resolution that Echo will receive when capturing.  Note also that the settings still use the index value for the camera versus the name.  (See Airsim Doc for indexing cross referencing, https://microsoft.github.io/AirSim/camera_views/)

- *DefaultSensors* - This lets the user activate whatever instruments are available.  This can be critical depending on the CameraViews that Echo is trying to capture.  For example, to get any of the depth camera views to work, the "*Distance*" sensor needs to be enabled.  See the Airsim documentation for more information.

- *PhysicsEngineName* - This needs to be set to "*ExternalPhysicsEngine*" to prevent Echo's trajectory transmission from fighting with Unreal's internal physics, which can produce some dizzying animations.

- *Vehicles* - This sets the instantiation information for the Airsim environment's actors.  Echo needs at least one multirotor vehicle named "Drone" to work.