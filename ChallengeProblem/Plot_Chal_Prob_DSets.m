% Plot_Chal_Prob_DSets.m script is used to plot user selected portions of the 
% Challenge problem Data Sets (own-ship trajectory, and/or baseball trajectories).
% These data sets are for the publicly releasable NASA 
% multi-rotor vehicle configuration.  These data sets are intended for use 
% with the BAM simulation and in particular to facilitate research on
% autonomous collision avoidance Challenge Problems

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 4.1.2025, MJA: Initial version of script.  Created for open-source
% release with BAM version 1.0
% *************************************************************************

% Add required paths to run this file stand-alone (not required if
% simSetup.m was run)
if ~exist('userStruct','var')
    addpath('../ChallengeProblem');
    addpath('../Bez_Functions');
    addpath('../lib/utilities'); % needed for setUnits
end

% Initialize the default units structure
defUnits = setUnits('m','kg');

% Challenge problem data sets
OwnTraj_fname       = './ref_trajectory_data.mat'; % Filename of own-ship trajectory data file
BBallTraj_fname     = './bball_trajectory_data.mat'; % Filename of baseball obstacle data file

% Select the desired data runs to plot from each data file.
% (Non-zero/non-empty data runs will be plotted.  Row vectors of desired
% runs are acceptable)

Desired_OwnTraj     = 50; % Designate which own-ship trajectories to plot
Desired_BBallTraj   = 50; % Designate which baseball trajectories to plot

% Specify the trajectory times of interest to plot
t_start_des = []; % Empty set implies beginning of trajectory, must be >= 0
t_end_des   = []; % Empty set implies end of trajectory, must be > t_start_des
t_int       = 0.01; % Select the plotting time step

% Create the figure
DS_fig = figure; % Create handle to Data Sets figure
hold on;
grid on;
cur_ax = gca;
% *************************************************************************
% Plot the desired own-trajectory(ies)
OwnTraj_fobj = matfile(OwnTraj_fname);
leg_cell     = cell(length(Desired_OwnTraj),1); 
leg_cnt      = 0;
for loop = Desired_OwnTraj
    % Load in the data for just the desired own-ship trajectory
    cur_traj_cell = OwnTraj_fobj.own_traj(loop,:);
    % Create the pw Bernstein polynomial curve
    CurTraj_pwcurve = genPWCurve({cur_traj_cell{1},cur_traj_cell{2},cur_traj_cell{3}},...
        {cur_traj_cell{4}, cur_traj_cell{5}, cur_traj_cell{6}});
    % Plot the trajectory

    if isempty(t_start_des) 
        t_start = 0;
    else
        t_start = t_start_des;
    end
    if isempty(t_end_des) || t_end_des > cur_traj_cell{4}(end)
        t_end = cur_traj_cell{4}(end);
    else
        t_end = t_end_des;
    end
    time    = linspace(t_start, t_end, (t_end-t_start)/t_int); % Create linearly spaced time vector
    pos     = evalPWCurve(CurTraj_pwcurve, time,0);
    plot3(cur_ax, pos(:,1), pos(:,2), -pos(:,3));
    hold all;

    leg_cnt              = leg_cnt + 1;
    leg_cell{leg_cnt}    = sprintf('Own-ship Traj #%i',loop);
end
% ************************************************************************


% *************************************************************************
% Plot the desired baseball-trajectory(ies)
BballTraj_fobj = matfile(BBallTraj_fname);
% leg_cell     = cell(length(Desired_BBallTraj),1); 
% leg_cnt      = 0;
for loop = Desired_BBallTraj
    % Load in the data for just the desired own-ship trajectory
    cur_traj_cell = BballTraj_fobj.bball_traj(loop,:);
    % Create the pw Bernstein polynomial curve
    CurTraj_pwcurve = genPWCurve({cur_traj_cell{1},cur_traj_cell{2},cur_traj_cell{3}},...
        {cur_traj_cell{4}, cur_traj_cell{5}, cur_traj_cell{6}});
    % Plot the trajectory

    if isempty(t_start_des) 
        t_start = cur_traj_cell{4}(1);
    else
        t_start = t_start_des;
    end
    if isempty(t_end_des) || t_end_des > cur_traj_cell{4}(end)
        t_end = cur_traj_cell{4}(end);
    else
        t_end = t_end_des;
    end
    time    = linspace(t_start, t_end, (t_end-t_start)/t_int); % Create linearly spaced time vector
    pos     = evalPWCurve(CurTraj_pwcurve, time,0);
    plot3(cur_ax, pos(:,1), pos(:,2), -pos(:,3));
    hold all;

    leg_cnt              = leg_cnt + 1;
    leg_cell{leg_cnt}    = sprintf('Base Ball Traj #%i',loop);
end
% ************************************************************************

% Plots axis labels and legend
xlabel('North (ft)');
ylabel('East (ft)');
zlabel('-Down (ft)');
grid on;
legend(leg_cell);
disp('')
