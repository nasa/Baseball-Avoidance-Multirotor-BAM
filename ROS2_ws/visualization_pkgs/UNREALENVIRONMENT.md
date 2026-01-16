# Unreal Environment Supplement
[Back..](../../README.md)

## Airsim / Colosseum

Airsim works as a plugin within Unreal Applications, and is provided as source or dated binaries.  It is recommended to pull the source from the website and build directly.  Information on this can be found on the website.

Airsim has been archived by Microsoft, so to use it with newer versions of Unreal (5+), Colosseum must be used (a fork development of airsim).
- For Unreal 4: Airsim can be found here: https://microsoft.github.io/AirSim/
- For Unreal 5: Colosseum can be found here: https://github.com/CodexLabsLLC/Colosseum

## Creating An Unreal Environment

As a precursor to the following process, it is assumed that the user is familiar with Unreal Engine, Unreal Editor, or going to take time to learn about these items from online resources.  This process will not fill in the knowledge gap on the basic operation of Unreal Application development.

1. (If applicable) Download and Install Unreal Engine.
2. (If applicable) Download and Build the appropriate package for the corresponding Unreal Engine.
3. Follow the Airsim / Colosseum directions to build the Unreal Plugin package.
4. Open Unreal Editor, Start a new project (user's preference, but I've typically built using the Simulation template.)
5. Using FAB (formerly Quixel Bridge / Unreal Marketplace), find a desired environment package to use and install it to the project.  (or create / use any other environment as you see fit.).
6. In the folder structure for the new project, under  /Plugins/, drop in the Airsim Plugin folder built in step 3.  This binary package should be located under \<root>/Unreal/Plugins/Airsim.  (Note: it will be labeled `Airsim` even in Colosseum.)
7. Change the "GameMode Override" under "WorldSettings" to "AirsimGameMode".
8. Add the asset package `BamBaseball` to the level; use the directions to do so found here.
9. Build the package for the desired platform by going to "Platforms->\<platform>->Package Project" (using Windows as an example).  This step can take a while if the world hasn't been built or packaged before.

[Back..](../../README.md)