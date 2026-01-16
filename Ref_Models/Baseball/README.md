## Baseball Reference Model

[Back to BAM..](../../README.md#api-documentation)   **or** [Back to Ref_Models..](../README.md)

The creation of baseball near-collision trajectories was accomplished offline using the m-file `ChallengeProblem/Create_BBall_Traj.m` and the
corresponding baseball dynamics Simulink model `ChallengeProblem/Dynamics.slx`.

The baseball trajectory generation script uses some basic baseball parameters:
```matlab
% *************************************************************************
% Set baseball parameters
bball_mass  = 0.145; % Baseball mass (kg)
bball_diam  = 2.90*SimIn.Units.in; % Baseball diameter (meters)
bball_area  = pi/4*bball_diam^2; % Baseball flat plate area (meters^2)
Atmos_den   = 1.224999155887712; % Density of standard atmos air (kg/m^3)
bball_MagC  = 4.1e-5; % Magnus Force coefficient for baseball 
```

and then utilizes a set of randomized variable parameters:

```matlab
% The following values should be set to correspond with Create_OwnShip_Traj.m values
min_alt         = 10; % Minimum starting/ending altitude in meters 
max_alt         = 50; % Maximum starting altitude in meters
% Max sim execution time (stop block kills sim when alt <= 0)
bball_t_max     = 50; % Maximum simulation time (should align with max_time from Create_OwnShip_Traj.m)

% Set the desired impact azimuth angles
min_az_ang      = -45*pi/180; % min baseball impact azimuth angle (radians)
max_az_ang      = 45*pi/180; % max baseball impact azimuth angle (radians)
cd_min          = 0.2; % Minimum drag coefficient
cd_max          = 0.4; % Maximum drag coefficient
grav_max        = grav_act; % Define maximum gravity
grav_min        = 0.6 * grav_act; % Define minimum gravity
spin_min        = 1600; % RPM
spin_max        = 2600; % RPM
spin_ang_max    = 45*SimIn.Units.deg; % +/- spin axis rotation angle (around x-axis)

% Prescribe collision point offset parameters from a randomly selected own-ship trajectory collision point
max_perp_dist   = 10*defUnits.ft; % maximum perpendicular distance from the own-ship velocity vector at the trajectory “collision” point

num_traj        = 3000; % Specify number of baseball trajectories to create from own_ship trajectories..
% *************************************************************************
```

These aforementioned parameters are sampled from a uniform distribution, and paired with a randomized initial velocity and offset heading from own-ship heading at the time of "collision".  This information is then simulated in the baseball dynamics simulation *Dynamics.slx*:

<center>
<img src=".graphics/Baseball_Dynamics.png" width="450" height="450" />
</center>

The resulting baseball trajectory is shifted and rotated to ensure that it arrives at the "near-collision" point randomly selected for the given own-ship trajectory.  The resulting baseball trajectory is then converted to a Bernstein polynomial and saved in the `bball_trajectory_data.mat` file.

This process was repeated for all 3000 own-ship trajectories that were randomly created previously.  This process results in 3000 "near-collision" scenarios for use with the autonomous collision avoidance challenge problem.

[Back to BAM..](../../README.md#api-documentation)  **or** [Back to Ref_Models..](../README.md)
