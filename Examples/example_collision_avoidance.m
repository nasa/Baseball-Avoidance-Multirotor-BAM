%{
EXAM_COLLISION_AVOIDANCE_RUNME

Description:
    This file demonstrates how to specify one of the own-ship trajectories
    and the corresponding baseball trajectory that will "collide" with ownship.
    The file is intended to show users how to specify a run from the
    ownship trajectory file and put the trajectory in the userStruct.  This
    demonstration is for running in simulink not for using an autocoded
    executable.

Input: None
Output: None
Dependencies: None
Details: Example script for the Collision Avoidance Challenge Problem running an
    example of the trajectories provided with BAM

Versions:
    2025-05-19: APP - moved to examples folder, with path changes
    2025-04-03: MJA - Initial script created. 
%}
%% Clean workspace
% remove workspace variables
clear userStruct
% clear command window
clc;

if ~any(contains({dir('*/').name}, 'setup'))
    error('BAM Error: Move example files into root directory for execution.');
end
if ~any(contains({dir().name}, 'ChallengeProblem'))
    error('BAM Error: Cannot find ChallengeProblem directory.');
end

plot_flag = 0; % plot_flag == 0 => perform setup only (user run in simulink)
               % plot_flag == 1 => perform setup, run sim and plot trajectories

% ************************************************************************
% If ROS2 publishing is to be utilized that must uncomment the userStruct 
% variant line below to utilize the ROS2 variant subsystem:

userStruct.variants.pubType = 1; % 1=>Default, no ROS2 publishing used
userStruct.variants.refInputTypeBball=2; % 2=> use BEZ bball ref trajectory
userStruct.variants.pubTypeBball=1; % 2=> publish bball pose via ROS2 msg

% NOTE: in order to use the ROS2 publishing variant, users must make sure
% that MATLAB has access to ROS2.  In order to do this on windows, use a
% developers cmd prompt (i.e. MS Visual Studio Community 2022). A standard
% command window in Windows won't work. The following instructions are for
% when ROS2 is installed on the machine)
% 1) Source ros in the and source developer command: > call
%   C:/ros2_humble/setup.bat
% 2) Call matlab from this command prompt:
%   C:/..your_path_to_matlab../bin/matlab
% 3) Now in Matlab command line verify that Matlab environment has access to ros2
%   >> ! ros2
%   This should result in ros2 help information, not an error message that
%   "ros2" is not a recognized internal or external command
% 4) Then uncomment the userStruct.variants.pubType = 2; variant line in
%   this script and run this m-file
% 5) To verify that ROS2 publishing messages are going out, then open a new
%   developers command prompt, and source ROS2 as in step one above.  Then
%   in this command window type: 
%   >ros2 topic echo /pub_pose
%   With the matlab simulation running, you should see the ROS2 messages
%   echoed.  Note if you don't use Matlab simulation pacing to real time
%   then some ROS2 messages may get missed.
% ************************************************************************


%% User Selected Parameters - userStruct
% If no custom values are provided, all user-modifiable parameters within 
% userStruct default to their initial settings. The file setup.m fully 
% populates userStruct, allowing you to identify which parameters can be 
% adjusted. Ensure that only information relevant to modifying the 
% simulation is included in userStruct.
%
% userStruct stores pre-setup values that guide simulation configuration. 
% These values are preferable to post-setup modifications, as they enable 
% computations necessary for aircraft setup, like using initial velocity 
% to determine vehicle pitch angle.

model_name = 'BAM'; % Define slx model name to run

% Specify the desired own-ship trajectory and the baseball trajectory
% numbers to execute in simulation and initialize the userStruct with their
% BP waypoints.
own_traj_num = 4; % Select desired own-ship traj number
bball_traj_num = 4; % Select desired bball traj number

% Load the ownship reference trajectory of interest
file_obj = matfile(['ChallengeProblem' filesep 'ref_trajectory_data.mat']); %
wptsX_cell      = file_obj.own_traj(own_traj_num,1);
wptsY_cell      = file_obj.own_traj(own_traj_num,2);
wptsZ_cell      = file_obj.own_traj(own_traj_num,3);
time_wptsX_cell = file_obj.own_traj(own_traj_num,4);
time_wptsY_cell = file_obj.own_traj(own_traj_num,5);
time_wptsZ_cell = file_obj.own_traj(own_traj_num,6);

% Set the own-ship traj information in userStruct
userStruct.simulation_defaults.RefInputs.waypointsX = wptsX_cell{1};
userStruct.simulation_defaults.RefInputs.waypointsY = wptsY_cell{1};
userStruct.simulation_defaults.RefInputs.waypointsZ = wptsZ_cell{1};
userStruct.simulation_defaults.RefInputs.time_wptsX = time_wptsX_cell{1};
userStruct.simulation_defaults.RefInputs.time_wptsY = time_wptsY_cell{1};
userStruct.simulation_defaults.RefInputs.time_wptsZ = time_wptsZ_cell{1}; 

% Set the sim stop time to the last time of the BP
userStruct.model_params.stop_time = time_wptsX_cell{1}(end);

% Load the baseball trajectory of interest
file_obj = matfile(['ChallengeProblem' filesep 'bball_trajectory_data.mat']);
wptsX_cell      = file_obj.bball_traj(bball_traj_num,1);
wptsY_cell      = file_obj.bball_traj(bball_traj_num,2);
wptsZ_cell      = file_obj.bball_traj(bball_traj_num,3);
time_wptsX_cell = file_obj.bball_traj(bball_traj_num,4);
time_wptsY_cell = file_obj.bball_traj(bball_traj_num,5);
time_wptsZ_cell = file_obj.bball_traj(bball_traj_num,6);

% Set the own-ship traj information in userStruct
userStruct.simulation_defaults.RefInputsBball.waypointsX = wptsX_cell{1};
userStruct.simulation_defaults.RefInputsBball.waypointsY = wptsY_cell{1};
userStruct.simulation_defaults.RefInputsBball.waypointsZ = wptsZ_cell{1};
userStruct.simulation_defaults.RefInputsBball.time_wptsX = time_wptsX_cell{1};
userStruct.simulation_defaults.RefInputsBball.time_wptsY = time_wptsY_cell{1};
userStruct.simulation_defaults.RefInputsBball.time_wptsZ = time_wptsZ_cell{1}; 

%% Setup Model

% setup must be run again after userStruct input is specified..
% setup is in the BAM top level folder, so call it from there if currently
% in the ChallengeProblem directory
setup;

%% Exit if setup only
if plot_flag==0
    return;
end

%% Post-Setup Processing 
% During development, it is often convenient to directly change the
% parameters of the simulation after setup. This can lead to problems, and
% conflicts with related or interconnected variables that will need to be
% identified and added to the setup process before inclusion in userStruct.
%
% Also note, this will not modify SimPar which is used for compiled models.

% change initial position
% SimIn.IC.Pos_bii = [0; 0; 10]; 


%% Run Simulation
% load_system(model_name);  % load without graphic interface for running
open_system(model_name);  % load with graphics for monitoring or editing
simout = sim(model_name); % run simulation

%% Post Run Processing
% Process post-run simulation varaibles, plot and save data.
close all

% Create a pw Bernstein polynomial curve for the baseball
bball_pwcurve = genPWCurve( ...
    [wptsX_cell, wptsY_cell, wptsZ_cell], ...
    [time_wptsX_cell, time_wptsY_cell, time_wptsZ_cell]);

out_fpath = sprintf('./ChallengeProblem/Chal_Prob_Plots/Exam_CA_Traj%i/', ...
                    own_traj_num);
if ~exist(out_fpath, 'dir')
    mkdir(out_fpath)
end

plot_trajectories(simout.logsout, bball_pwcurve, save_dir=out_fpath);
plot_multirotor_states(simout.logsout, save_dir=out_fpath);
