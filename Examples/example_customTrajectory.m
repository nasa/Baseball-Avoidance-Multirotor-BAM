%{
example_defaults

Description:
    Example scenario file for defining repeatable simulations.

Notes:    
    This file must be moved to, and run from, the root directory.

Input: TBD
Output: TBD
Dependencies: TBD
Details: TBD

Versions:
    2025-03-18: APP - created.
    2025-04-08: TNZ - add basic plot scripts.
%}

%% Clean workspace
% remove all workspace variables
clearvars;
% clear command window
clc;
% check location and add 
if ~any(contains({dir('*/').name}, 'setup'))
    error('BAM Error: Move example files into root directory for execution.');
end

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


% Create RefInputs Field - userStruct.simulation_defaults.RefInputs
% see defaultBez.m for another example of creating waypoints
%   RefInputs:
%       waypointsX: [2×3 double]
%       waypointsY: [2×3 double]
%       waypointsZ: [2×3 double]
%       time_wptsX: [0 40]
%       time_wptsY: [0 40]
%       time_wptsZ: [0 40]
%
% within row = pos vel acc, rows are waypoints, NOTE: NED frame -z is up...

% clf([1:4])
initial_time = 0;
wptsX = [0 3 0; 
        120 3 0
        240 3 0
        300 0 0];
time_wptsX = [initial_time 40 80 120];
wptsY = [0 0 0; 
        0 0 0;
        0 0 0;
        60 3 0]; 
time_wptsY = [initial_time 40 80 120];
wptsZ = [0 0 0 ; 
    -60 -3 0 
    -120 0 0
    -120 0 0]; 
time_wptsZ = [initial_time 40 80 120];

ri.waypointsX = wptsX;
ri.waypointsY = wptsY;
ri.waypointsZ = wptsZ;
ri.time_wptsX = time_wptsX;
ri.time_wptsY = time_wptsY;
ri.time_wptsZ = time_wptsZ;

% ri = userStruct.simulation_defaults.RefInputs;
waypoints = {ri.waypointsX, ri.waypointsY, ri.waypointsZ};
time_wpts = {ri.time_wptsX, ri.time_wptsY, ri.time_wptsZ};


% Assign to userStruct
userStruct.simulation_defaults.RefInputs = ri;
userStruct.model_params.stop_time = time_wptsX(end);

%% Setup Model
setup;
Plot_PW_Bezier(waypoints,time_wpts)

%% Post-Setup Processing 
% During development, it is often convenient to directly change the
% parameters of the simulation after setup. This can lead to problems, and
% conflicts with related or interconnected variables that will need to be
% identified and added to the setup process before inclusion in userStruct.
%
% Also note, this will not modify SimPar which is used for compiled models.


%% Run Simulation
load_system(model_name);  % load without graphic interface for running
% open_system(model_name);  % load with graphics for monitoring or editing
simout = sim(model_name); % run simulation

%% Post Run Processing
% Process post-run simulation variables, plot and save data.

close all
plot_trajectories(simout.logsout);
plot_multirotor_states(simout.logsout);
