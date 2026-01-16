% ************************ setup.m ******************************
% This script is used to initialize the BAM (baseball avoidance
% multi-rotor) simulation.  This simulation flies a simulated quad-rotor
% (based on the NASA impact vehicle) on a short trajectory segment,
% simulates a baseball in flight whose trajectory is designed to intersect
% with the quadrotor during the trajectory timeline.  Additionally, this
% simulation, provides the ability to output vehicle/baseball pose
% information via ROS2 messages.  This repo also contains an Airsim plug-in
% for use with Unreal Engine version 4.27 for high fidelity visualization.
% Execution of a c++ script can be used to obtain the ROS2 messages and
% provide them to the Airsim plug-in for Unreal Engine. This script should
% be run from the main BAM repository folder.

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Software authors/contributors for this repository include: Ben Simmons,
% Kasey Ackermann, Garret Asper, John Bullock, Michael J. Acheson, Barton
% Bacon, Steve Derry, Tom Britton, Dan Hill, Eugene Heim, Andrew Patterson,
% and ...

% Versions:
% 3.18.2025, APP: TBD
% 1.22.2025, MJA: Initial version of script.  Created for open-source
% release of entire .git repository
% *************************************************************************

%% Setup matlab
sim_directory = cd;
model_name = 'BAM.slx';
setup_folder = 'setup';
FLAG_SAVE = 0; % do you want to save simulink model if we need to close it?


% Setup Path
addpath(genpath(setup_folder))
setupPath;

% Close Hidden Models and start sim engine
%   if you don't start it here, it will start up somewhere...
fprintf('Initializing simulink...')
% close_system(model_name,FLAG_SAVE);
% load_system(model_name); % load system if used in setup
fprintf('done.\n')

%% Setup userStruct
% setup/userStruct
fprintf('Processing user input...')
setupUserStruct; % userStruct structure check
setupInfoStruct; % store setup information
setupVariants; % setup matlab variants
setupTypes; % flags/types that do not rise to level of "variant"
setupDefaults; % simulation specific parameters, e.g., sample rate or trim
setupOutput;
fprintf('done.\n')

%% Setup SimIn
% setup/SimIn
SimIn = struct();
% Setup Units
SimIn = setupUnits(SimIn,userStruct);

% setup sample_rates
SimIn.model_params.sim_rate = userStruct.model_params.sim_rate;

% setup rt_pacing wall clock factor (only used if RT pacing variant selected)
SimIn.model_params.rt_pace = userStruct.model_params.rt_pace;

% Setup Environment
fprintf('Setting up Environment...')
SimIn = setupEnvironment(SimIn,userStruct);
fprintf('done.\n')

% Setup Vehicle
fprintf('Setting up aircraft...')
SimIn = setupVehicle(SimIn,userStruct);
fprintf('done.\n')

% Setup RefInputs
fprintf('Setting up reference inputs...')
SimIn = setupRefInputs(SimIn,userStruct);
fprintf('done.\n')

% Setup BballRefInputs
fprintf('Setting up baseball reference inputs...')
SimIn = setupBballRefInputs(SimIn,userStruct);
fprintf('done.\n')

%% Setup Initial Conditions
fprintf('Setting up initial conditions...')
SimIn = setupIC(SimIn,userStruct);
fprintf('done.\n')

%% Setup Flight Controller
fprintf('Setting up flight control...')
SimIn = setupFCS(SimIn,userStruct);
fprintf('done.\n')

%% Setup Variant Strings
fprintf('Setting up variant strings...')
SimIn = setupVariantSelect(SimIn,userStruct);
setupVariantControl;
fprintf('done.\n')

%% Setup Parameters
fprintf('Setting up parameters...')
[SimIn, SimPar] = setupParams(SimIn,userStruct);
fprintf('done.\n')

% Create buses
fprintf('Setting up bus definitions...')
setupBuses;
fprintf('done.\n')

% Setup Files
fprintf('Setting up files...')
setupFiles(userStruct);
fprintf('done.\n')