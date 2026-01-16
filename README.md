# Baseball Avoidance Multirotor (BAM) Simulation

Welcome to the BAM simulation GitHub repository! This open-source project aims
to foster collaboration and comparison between researchers working on
autonomous collision avoidance (i.e., perception, prediction, and planning) for operations of urban air 
mobility (UAM) vehicles. This simulation challenges researchers to demonstrate the ability for a quad-rotor to autonomously avoid baseballs on collision courses (in real time). BAM is MATLAB & Simulink based, but provides easy interfaces with high fidelity graphics (Unreal Engine) and sensors (e.g., CodeLabsLLC Colosseum, and Microsoft Airsim) while also easily accommodating external algorithms (e.g., Python and C++) via the Robot Operating System II (ROS2).

<center>
<img src=".graphics/Exam_4.gif" width="400" height="250" />
</center>

**Version:** 1.1

**MATLAB & Simulink Version:** 2024b

**Compatibility: Windows, Linux, Mac**

## Table of Contents

1. [Overview and Key Features](#overview-and-key-features)
2. [Documentation](#api-documentation)
3. [Getting Started](#getting-started)
4. [Compatibility Notes](#compatability-notes)
5. [License](#license-information)
6. [Contact Information](#contact-information)
7. [Citation](#citation)

## Overview and Key Features

BAM integrates various modeling aspects including environmental effects, tunable parameters, control systems, and data visualization, with an emphasis on high-complexity simulation for realistic UAM scenarios. It supports 
MATLAB/Simulink and Robot Operating System 2 (ROS2) integration, offering
comprehensive tools and datasets for an "Autonomous Collision Avoidance" challenge
problem.  BAM can be run as a stand-alone Simulink simulation or in combination with ROS2, Colosseum, and Unreal Engine to take advantage of the sensors, visualization, and perception tools.  A "Users Guide" (see README.html at the repository top level) is provided which consists of a series of linked README files.

### Key Components

1. **Simulation Architecture**: Built using Simulink, provides comprehensive bus
   structures for a six-degree-of-freedom, rigid body, non-linear aerospace simulation of a quad-rotor.  The simulation is fully autocode capable, including ROS2 communications with external algorithms.  BAM also includes a variety of aerospace library blocks and scripts for computing many aerospace quantities.

2. **ROS2 Integration**: Uses MATLAB s-functions to facilitate communication 
   between MATLAB/Simulink simulations and ROS2 nodes (NO MATLAB ROS2 TOOLBOX required!)

3. **Data Visualization**: Includes high-fidelity visualization integration with CodexLabLLC Colosseum / Microsoft Airsim 
   and Unreal Engine, as well as scripts and Simulink subsystems for plotting vehicle state
   information.

4. **Baseball Aerodynamic & Ballistic Modeling**: Offline trajectory generation data sets and scripts for generation of near collision baseball trajectories which include, gravitational modeling, drag effects and spin aerodynamics.

5. **Environmental and Sensor Models**: Incorporates wind and turbulence (Dryden based)
   effects within the Simulink model.  Additionally, Unreal Engine v5+ enables very high fidelity atmospheric modeling (e.g., visibility, fog) for sensor degradation modeling.

6. **Control Systems**: Includes baseline geometric controller with an inner/outer loop control
   structure derived from recent research, enabling sophisticated guidance
   systems.  Easy for users to drop in replacement control algorithms.

7. **Trajectory Injection**: Supports Bernstein
   polynomial reference trajectories (both own-ship and baseball).  Additionally, has the capability to utilize user-specified time series trajectories.

8. **Colosseum and Airsim**: These simulation frameworks were used in this repo to take advantage of their API capability to interface with Unreal Engine and provide users with various sensors (e.g., Lidar and cameras).  The BAM Simulink simulation provides the dynamic (physics) simulation and the Colosseum/Airsim Unreal Plugin was used in "External Physics" mode.  A standalone executable (and source code) is provided with this repo to receive the BAM vehicle POSE information via ROS2 messages and communicate via API to the Unreal Engine Environment.  

8. **Unreal Engine Executable Environment**: An Unreal Engine executable (including Colosseum plug-in) is provided based on the Unreal Engine asset "English College Level 4 Sample" [Fab.com](https://www.fab.com/channels/unreal-engine).  This executable enables users to obtain sensor data (e.g., lidar, camera) in a photo-realistic environment.  Instructions and scripts provided which enable users to drop in Colosseum (or Airsim) plug-in into their own user provided Unreal Engine environment and readily connect BAM to this environment.   

### Suggested Tools & Plug-ins

- **CodexLabsLLC Colosseum (Microsoft Airsim Plugin)**: Users who wish to rebuild the provided Unreal Engine environment executable or to utilize their own Unreal environment should obtain these open-source repositories. 
- **Unreal Engine Environment**: While not required, users who wish to build their own environment ecosystem will need to obtain a copy of the Unreal Engine editor etc..  Additionally, for users for which the provided Unreal environment executable doesn't work, they will need to rebuild it by providing Unreal Engine, Colosseum plug-in, and the Unreal Engine asset "English College" noted above.
- **FFMPG**: Some example videos created using FFMPEG (open-source) from the Unreal executable are included in this repository. While optional, users may find it useful to use this or a similar tool to generate videos from camera sensor images obtained using the Colosseum plug-in.
- **ROS2**: While not required for execution of the BAM simulation, users who want to utilize the Unreal Engine & Colosseum environment will need to utilized ROS2 (see Compatibility for suggested versions).
- **Robostack**: In lieu of a direct installation of ROS2, we also utilized Robostack (conda pixi package) for ROS2 (see compatibility for suggested versions).
- **Python**: While python is not utilized with the BAM simulation, or to enable the ROS2/Colosseum/Unreal Engine pipeline, it is used extensively for a variety of ROS2 nodes.  See the associated README files [ROS2_ws/Analysis Packages]( ./ROS2_ws/analysis_pkgs/README.md) or the [Env .yml Linux](./ROS2_ws/config/linux/environment.yml) and [pixi Win](./ROS2_ws/config/win64/pixi.toml) for details on relevant Python packages.

---

## API Documentation

A top level README file is provided which consists of a compilation of the associated README files in this repo (see links below).  The following documentation listed below aligns with the folder structure in BAM.

1. [AeroProp](AeroProp/README.md): Multirotor Aero-Propulsive Model which includes thrust, torque, basic aerodynamics, and vehicle mass/inertia properties
2. [AutoCodeModels](AutoCodeModels/README.md) Scripts and utilities to autocode Simulink models with/without ROS2
3. [BamEcho](BamEcho/README.md) Postprocess Unreal Visualization
4. [Bez_Functions](Bez_Functions/README.md) Bernstein Polynomial toolbox
5. [ChallengeProblem](ChallengeProblem/README.md) Autonomous Collision Avoidance Challenge Problem
6. [Examples](Examples/README.md) Contains example scripts/template for initiating the BAM simulation with a variety of trajectories. Demonstrates how to utilize userStruct structure, and vary SimPar parameters (e.g., initial conditions). **NOTE**: Before running any example files, first run `setup.m` in the main BAM directory. This ensures all paths and dependencies work correctly when executing the files
7. **lib** Folder contains a variety of aerospace related MATLAB scripts and Simulink libraries (e.g., quaternions, vehicle dynamics, Geodesics, axis transformations)
8. [Ref_Models](Ref_Models/README.md)
   1. [Autocode Real-Time Pacing](Ref_Models/Autocode_RT_Pacing/README.md) This subsystem should be used only when it is desired to perform simulation real-time pacing in **executable** code.  For normal simulation pacing, use the built in Simulink Run/Simulation pacing button  
   2. [Baseball](Ref_Models/Baseball/README.md) Baseball scripts and Simulink simulation for baseball trajectory generation (e.g., randomized initial conditions , baseball drag and spin models etc..)
   3. [Controller](Ref_Models/Controller/README.md) High performance quadrotor geometric controller
   4. [Environment](Ref_Models/Environment/README.md) Atmospheric model (1976), wind, and Dryden turbulence models
   5. [Plots](Ref_Models/Plots/README.md) Visualization capabilities native in MATLAB and Simulink
   6. [Sensors](Ref_Models/Sensors/README.md) Sensor models 
9. [ROS2_ws](ROS2_ws/README.md) ROS2 nodes and installation scripts
10. [setup](setup/README.md) Simulation setup scripts 
11. [Trim](Trim/README.md) MATLAB m-files used to create and implement a vehicle trim schedule
12. **Util** Folder contains a variety of utility scripts.  One script recursively builds bus objects from a given MATLAB structure.  Another script recursively downgrades all .slx files to a previous version of MATLAB.  A .bat script is provided that recursively produces linked HTML documents from all markdown files in the repo.  This script was used to create the top-level README.html document which serves as the user guide for this repository  
13. **VisualEnv** This folder contains the executable of the Unreal Engine environment for use with BAM.  Users will find the compressed version of the environment executable for the Windows operating system.  With the instructions provided, users can generate a Linux version as needed.  These executables are not necessary to run the Simulink BAM simulation but there are required for those users who wish to work with the Airsim/Colosseum and Unreal Engine visualization pipeline.  This environment is released as a binaries (not source code) as the FAB marketplace free asset "English College" was used which is allowed under the FAB marketplace licensing.  **NOTE**: These binaries are large files ~1.2gb and therefore are not included in the git repo but instead are released as part of a BAM release version on GitHub.  Users will need to download the Windows executable separately.

---

## Getting Started

### MATLAB and Simulink (only)
To get started with the BAM simulation (no ROS2), ensure you have MATLAB/Simulink installed, then follow these steps:

1. Clone this repository, move example file, and open MATLAB:
   ```bash
   git clone https://github.com/nasa/Baseball-Avoidance-Multirotor-BAM.git
   cd bam_baseball_avoidance_multirotor
   mv ./Examples/example_template.m .
   matlab
    ```
 
2. Run example script in MATLAB command window:
   ```matlab
   example_template;
   ```
3. Users can interact with the simulation (e.g., add or remove publisher and subscriber nodes via the [setup](./setup/README.md) process).  See [examples](./Examples/README.md) for details on modifying own-ship trajectory or executing BAM `Challenge Problem` scenarios.

### ROS2

For details on using ROS2 in conjunction with the BAM simulation see the [ROS2_ws](./ROS2_ws/README.md) folder.

### BAM to Unreal Engine Pipeline (Real-time)

To utilize the BAM to Unreal pipeline, it is important to first note that in addition to sourcing ROS2, the user will need to have an Airsim / Colosseum compliant environment application.  BAM comes with a Windows 11 (and Linux) / Colosseum binary named `BAM_EnglishCollege.exe` using Unreal 5.  If users require another solution, they will need to produce their own Unreal environment application, which is described in the [Unreal Environment Supplemental](./ROS2_ws/visualization_pkgs/UNREALENVIRONMENT.md) and in both the Airsim and Colosseum documentation websites.

1. Enable ROS2 (users need to install ROS2 on their machine or utilize `robostack` for a packaged version of ROS2). See Compatibility notes below
   - Example: (Windows) robostack users: 
   ```bash
   >cd <path to robostack installation>
   pixi shell -e jazzy
   ... once jazzy shell opens ...
   (robostack:jazzy) >cd <path to BAM/ros2_ws>
   colcon build # only perform this step if no install folder (i.e., fresh clone of repo)
   call ./install/local_setup.bat # source local ROS2 packages
   ```
   - Example: (Windows) non-robostack users: open a `Developer Command Prompt for VS 2022`. NOTE: ROS2 doesn't work properly on Windows using just a `command Prompt`
   ```bash
   >cd <path to ROS2 installation> # Only required if ROS2 is not already in path
   >call ./setup.bat # Only required if ROS2 is not already in path
   >cd <path to BAM/ros2_ws>
   >colcon build # only perform this step if no install folder (i.e., fresh clone of repo)
   >call ./install/local_setup.bat # source local ROS2 packages
   ```

2. Startup MATLAB / Simulink 
   - Example: (Windows) robostack users: 
   ```bash
   (robostack:jazzy)>matlab # start matlab from robostack command window so MATLAB has ROS2 sourced
   ```
   - Example: (Windows) non-robostack users: 
   ```bash
   >matlab # start matlab from command window which has ROS2 sourced so MATLAB also has ROS2 sourced
   ```
   - Verify ROS2 is sourced by typing `>> ! ros2` in the MATLAB command line
   
3. Move the `settings.json` file:
   - Copy the example `settings.json` file from the folder: `./ROS2_ws/visualization_pkgs/bam_2_airsim/Sample_settings.json`
   - Move and rename the sample file to the folder `<path to user's Windows Documents folder>/Airsim/settings.json` (windows).  Note: Windows users may need to create the Airsim folder: `<path to user's Windows Documents folder>/Airsim`.  Modify the `settings.json` file as desired.  Linux users should place the modified `settings.json` file in the `~/Documents/AirSim` folder.

4. Run the default Unreal Environment Executable (e.g., `EnglishCollege.exe`).  More details on this can be found at: [Unreal Environment Supplemental](./ROS2_ws/visualization_pkgs/UNREALENVIRONMENT.md). (NOTE: Users can provide their own Unreal Environments for use with BAM if desired.)

5. Build and run Bam_2_Airsim ROS2 executable.  See [bam_2_airsim](./ROS2_ws/visualization_pkgs/bam_2_airsim/README.md) on how to build the bam_2_airsim executable and link to Colosseum etc.
   - Run the Bam_2_Airsim node
   ```bash 
   (robostack:jazzy)>ros2 run bam_2_airsim_pkg Bam2Airsim # runs the ROS node for robostack ROS2 (unable to run => colcon build & source local setup)
   ```
   ```bash
   >ros2 run bam_2_airsim_pkg Bam2Airsim # runs the ROS node for non-robostack ROS2 (unable to run => colcon build & source local setup)
   ```
   - NOTE: this Bam_2_Airsim node should be run AFTER the Unreal Environment executable is running as it tries to connect to the Unreal executable via the Airsim/Colosseum plug-in.

6. Run the BAM simulation. Instructions for setting up and running the BAM simulation for a collision scenario can be found in: [example_collision_avoidance.m](./Examples/example_collision_avoidance.m).  If users have run `setup` previously, then they can run `example_collision_avoidance` directly from the MATLAB command line at the BAM top level folder, otherwise users will need to move `./Examples/example_collision_avoidance.m` to the BAM top level to run it.
   - The BAM repo provides binaries for the ROS2 Simulink s-functions. If these built binaries don't work as provided, then users will need to rebuild them see [ROS2_ws](./ROS2_ws/README.md)

### Bam To Unreal Engine Video Generation With Echo (Replay)
The following allows users to take individual runs from BAM and generate video files with specified camera and sensor views using Airsim & Unreal.  Caution: this only does one trajectory at a time, because of the resource load of generating videos.

Much like the last pipeline, this pipeline requires the installation and configuration of a variation of Airsim and an Unreal Environment application ([Unreal Environment Supplemental](./ROS2_ws/visualization_pkgs/UNREALENVIRONMENT.md).  This pipeline also leverages the Python Client that comes with Airsim / Colosseum.

1. Run MATLAB / Simulink

2. Run BAM to generate a data run for visualization

3. Create an Echo compliant data file by processing the Bam data run with Echo's companion tool (see [BamEcho Preparation documentation](./BamEcho/README.md#obtaining-bamecho-scenario-data) for more info.)

4. Launch and run an Airsim compatible Unreal Environment application..

5. Configure and run BamEcho with the data files produced in Step 3. See [BamEcho Documentation](./BamEcho/README.md) for details, options, and examples.

### Autocode

Details on the processes to autocode the BAM simulation can be found in [AutoCodeModels](./AutoCodeModels/README.md#getting-started-guide) and additionally for details on batch processing see [Batch Processing](./AutoCodeModels/examples/BatchProcessing/README.md).

---

## Compatibility Notes

- Operating Systems: Testing of the simulation was only performed on Windows 11, MAC OS 15 (Apple and Intel processors), and some Linux versions.

- MATLAB and Simulink:  The base BAM simulation is provided in a newer version of Simulink (2024b).  This was done solely for taking advantage of the Cmake Toolchain for Simulink code generation available with 2024b.  The basic BAM simulation was tested and found to be fully compatible with multiple prior versions of Simulink.  Users who wish to utilize previous versions of Simulink, see the m-function [Downgrade Simulink](./Util/downgradeModelsTo2024a.m)).

- ROS2: Usage of ROS2 to communicate with external algorithms was shown to be compatible with installations of Humble and Jazzy.  Additionally, we utilized the bundled ROS2 [robostack](https://robostack.github.io) versions Jazzy and higher. The robostack version of Humble did result in some errors.

- [Colosseum](https://github.com/CodexLabsLLC/Colosseum) and [Airsim](https://microsoft.github.io/AirSim/): Airsim is an open source simulation environment which we utilized solely as a plug-in with an Unreal Engine environment. Colosseum is an open-source repo which is the follow on to Airsim after it was archived.

- Unreal Engine: Users are encouraged to utilize Colosseum instead of Airsim, as Colosseum is the repository continuing Airsim development after Airsim was archived.  Specific versions of Unreal Engine required are: CodexLabsLLC Colosseum - Unreal Engine 5.4+, Microsoft Airsim - Unreal Engine 4.25+

---

## License Information

This software is released by NASA as open-source. Users must provide MATLAB and Simulink, but the other software packages users may wish to enable are either open-source and/or no-cost usage. If users utilize the binaries (e.g., Unreal Environment executable) provided with this repository, then no other software is required.  Users who wish to autocode the BAM simulation will also need the appropriate MATLAB toolboxes (e.g., Simulink coder).  See attached [BAM License](./LICENSE.pdf).


---

## Contact Information:

**Contributors**: Michael J. Acheson, Kasey Ackerman, Garrett Asper, Rachel Axten, Barton Bacon, John Bullock, Thomas Britton, Newton Campbell, Stephen Derry, Irene Gregory, Daniel Hill, John "Dana" McMinn, Nicolas Miguel, Tenavi Nakamura-Zimmerer, Andrew Patterson, and Benjamin Simmons.

**Contact**:
- **Name:** Michael J. Acheson
- **Organization:** NASA Langley Research Center (LaRC)
- **Department:** Dynamics Systems and Control Branch (D-316)
- **Email:** [michael.j.acheson@nasa.gov](mailto:michael.j.acheson@nasa.gov)

--- 

## Citation

To cite the Baseball Avoidance Multirotor, please use:

@misc{BAM2025,
  author = {Acheson, M., Ackerman, K., Campbell, N. et al. },
  title = {Baseball Avoidance Multirotor (BAM)},
  year = {2025},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/nasa/Baseball-Avoidance-Multirotor-BAM}},
  commit = {TBD}
}

---

## Required Notices and Disclaimers (See section 3B of open-source LICENSE.pdf file):

Copyright Notice for the NASA Software “Baseball Avoidance Multirotor Simulation” v.1.1 (LAR-20691-1): Copyright 2025 United States Government as represented by the Administrator of the National Aeronautics and Space Administration. All Rights Reserved. 1. The NASA Software “Baseball Avoidance Multirotor Simulation” v.1.1 (LAR-20691-1) may be released/downloaded with “BAM_English College,” which is NASA developed software in executable code only, for Royalty-Free distribution Only as a Non-Engine Product, both as defined in the Unreal® Engine End User License Agreement. Unreal® Engine, Engine Code, Examples, and Starter Content software are not bundled with this NASA software; users of this NASA software may obtain their own license from Epic Games, Inc. The NASA Software “Baseball Avoidance Multirotor Simulation” v.1.1 (LAR-20691-1) may include a rendered linear media product within such executable code, such as English College Level 4 Sample, by user AccuCities, via FAB. Each is subject to the terms and conditions of its licensor. English College Level 4 Sample, by user AccuCities, via FAB. https://www.fab.com/listings/374fb588-5711-41f1-a69b-4e60c95beea5 (Fab Standard License: https://fab.com/s/0d437fa051bd) CC BY 4.0 https://creativecommons.org/licenses/by/4.0/legalcode.en https://accucities.com/ AccuCities® is a trademark or registered trademark of AccuCities Ltd, in the USA and/or elsewhere. Fab End User License Agreement https://www.fab.com/eula Unreal® Engine, Copyright 1998 – 2025, Epic Games, Inc. All rights reserved Unreal® Engine End User License Agreement: https://www.unrealengine.com/en-US/eula/unreal Unreal® and Fab are trademarks or registered trademarks of Epic Games, Inc. in the USA and/or elsewhere. 2. This NASA software may include NASA developed User Files as defined in the Mathworks Program Offering Guide for MATLAB® software products, which may be found in the applicable Documentation for Release. MATLAB® software is not bundled with this NASA software; users of this NASA software may obtain their own license from The Mathworks, Inc., which is subject to the terms and conditions of its licensor (e.g., The MathWorks, Inc. Software License Agreement), as applicable at the time of licensing. Hyperlinks are provided here for information purposes only: https://www.mathworks.com/ MATLAB® is a registered trademark of The MathWorks, Inc. 3. This NASA software may be bundled with the following third-party software which is subject to the terms and conditions of its licensor, as applicable at the time of licensing. Original copies of the third-party software may be available from the licensor. a. Simulation Environment for Multirotor UAVs, Version 1.2.0.0 (127 KB) by Jan Vervoorst, A Modular Simulation Environment for the Improved Dynamic Simulation of Multirotor UAVs Copyright (c) 2016, Jan Vervoorst All rights reserved. Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer. * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution * Neither the name of University of Illinois at Urbana-Champaign nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. Cite As: Jan Vervoorst (2025). Simulation Environment for Multirotor UAVs (https://www.mathworks.com/matlabcentral/fileexchange/59705-simulation-environment-for-multirotor-uavs), MATLAB Central File Exchange. Retrieved August 15, 2025. b. spdlog The MIT License (MIT) Copyright (c) 2016 Gabi Melman. Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. “-- NOTE: Third party dependency used by this software -- This software depends on the fmt lib (also MIT License), Copyright (c) 2012 - present, Victor Zverovich and {fmt} contributors, and users must comply to its license: https://raw.githubusercontent.com/fmtlib/fmt/master/LICENSE” https://github.com/gabime/spdlog 4. The NASA Software “Baseball Avoidance Multirotor (BAM) Simulation” v.1.1 (LAR-20691-1) calls the following third-party software, which is subject to the terms and conditions of its licensor, as applicable at the time of licensing. The third-party software is not bundled or included with this software but may be available from the licensor. Users must supply their own third-party software. License hyperlinks are provided here for information purposes only. ********** pytest MIT: https://github.com/pytest-dev/pytest Copyright (c) 2004 Holger Krekel and others ********** MSR Aerial Informatics and Robotics Platform MSR Aerial Informatics and Robotics Simulator (AirSim) MIT: https://github.com/microsoft/airsim?tab=License-1-ov-file#readme https://microsoft.github.io/AirSim/ Copyright (c) Microsoft Corporation All rights reserved. ********** Colosseum (“a successor of AirSim”) MIT: https://github.com/CodexLabsLLC/Colosseum? tab=License-1-ov-file#readme https://codexlabsllc.github.io/Colosseum/ MSR Aerial Informatics and Robotics Platform MSR Aerial Informatics and Robotics Simulator (AirSim)  Copyright (c) Microsoft Corporation. 2022 All rights reserved. (Colosseum) MSR Aerial Informatics and Robotics Platform MSR Aerial Informatics and Robotics Simulator (AirSim) Copyright (c) Codex Laboratories LLC. 2022 All rights reserved. ********** ROS2 Apache 2.0: https://github.com/ros2 (No copyright notice located) ********** Unreal® Engine https://www.unrealengine.com/en-US/eula/unreal  Copyright 1998 – 2025, Epic Games, Inc. All rights reserved ********** MATLAB®/SIMULINK® brand simulation software products Offered by The MathWorks, Inc. https://www.mathworks.com **** Any trademarks used herein are the property of their respective owner. Trade names and trademarks are used for identification only. Their usage does not constitute an official endorsement, either expressed or implied, by the National Aeronautics and Space Administration.

Disclaimers
No Warranty: THE SUBJECT SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY OF ANY KIND, case EITHER EXPRESSED, IMPLIED, OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL CONFORM TO SPECI-FICATIONS, ANY IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICU-LAR PURPOSE, OR FREEDOM FROM INFRINGEMENT, ANY WARRANTY THAT THE SUBJECT SOFTWARE WILL BE ERROR FREE, OR ANY WARRANTY THAT DOCUMENTATION, IF PRO-VIDED, WILL CONFORM TO THE SUBJECT SOFTWARE. THIS AGREEMENT DOES NOT, IN ANY MANNER, CONSTITUTE AN ENDORSEMENT BY GOVERNMENT AGENCY OR ANY PRI-OR RECIPIENT OF ANY RESULTS, RESULTING DESIGNS, HARDWARE, SOFTWARE PROD-UCTS OR ANY OTHER APPLICATIONS RESULTING FROM USE OF THE SUBJECT SOFT-WARE.  FURTHER, GOVERNMENT AGENCY DISCLAIMS ALL WARRANTIES AND LIABILITIES REGARDING THIRD-PARTY SOFTWARE, IF PRESENT IN THE ORIGINAL SOFTWARE, AND DISTRIBUTES IT "AS IS."

Waiver and Indemnity:  RECIPIENT AGREES TO WAIVE ANY AND ALL CLAIMS AGAINST THE UNITED STATES GOVERNMENT, ITS CONTRACTORS AND SUBCONTRACTORS, AS WELL AS ANY PRIOR RECIPIENT.  IF RECIPIENT'S USE OF THE SUBJECT SOFTWARE RESULTS IN ANY LIABILITIES, DEMANDS, DAMAGES, EXPENSES OR LOSSES ARISING FROM SUCH USE, IN-CLUDING ANY DAMAGES FROM PRODUCTS BASED ON, OR RESULTING FROM, RECIPI-ENT'S USE OF THE SUBJECT SOFTWARE, RECIPIENT SHALL INDEMNIFY AND HOLD HARMLESS THE UNITED STATES GOVERNMENT, ITS CONTRACTORS, AND SUBCONTRAC-TORS, AS WELL AS ANY PRIOR RECIPIENT, TO THE EXTENT PERMITTED BY LAW.  RECIPI-ENT'S SOLE REMEDY FOR ANY SUCH MATTER SHALL BE THE IMMEDIATE, UNILATERAL TERMINATION OF THIS AGREEMENT.
