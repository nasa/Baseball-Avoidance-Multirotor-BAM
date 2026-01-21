% ************************ Create_OwnShip_Traj.m ******************************
% This script generates ownship piecewise Bernstein polynomial trajectories
% for use with the BAM simulation.  Thes trajectories are created with
% random variation of several factors including: initial starting point,
% velocity profile, altitude profile, initial heading/terminal heading, and
% trajectory type: straight line, constant turn rate, and parabolic
% curves (not yet implemented).  The trajectories all include a random constant 
% rate climb or decent within a designated altitude block.  The straight 
% trajectories include a random speed variation (i.e., constant acceleration)
% within a specified speed range.  The randomly created constant turn rate 
% trajectories are performed at a constant speed. The trajectories are stored 
% as a series of two BP waypoints (initial and final).  The BP waypoints 
% contain 3 derivatives (pos, vel and accel) only. In all, 3000 trajectories
% were created and stored in the file designated by the variable out_fname 
% ('ref_trajectory_data.mat'). In the BAM simulation setup process, users can select
% one of these profiles to run. This script is provided so users can
% recreate the data but MORE IMPORTANTLY modify this script to generate
% trajectories that suit their needs.
%
% *************************************************************************
% Inputs:   None
% Outputs:  Bernstein polynomial waypoints (X,Y,Z and time_X,time_Y,time_Z)

% *************************************************************************
% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 3.26.2025, Michael Acheson, Initial version of script.  Created for generation
% of own-ship trajectories as part of open-source release of entire BAM
% .git repository
% *************************************************************************
% *************************************************************************

% Add required paths to run this file stand-alone (not required if setup.m was run)
if ~exist('userStruct','var')
    addpath('../ChallengeProblem');
    addpath('../Bez_Functions');
    addpath('../lib/utilities'); % needed for setUnits
end

% Specify the seed/generator for use with random number generator
rng('default'); % Use the matlab default: seed number =0,  generator = Mersenne Twister

% Initialize the default units structure
defUnits = setUnits('m','kg');

% *************************************************************************

% Designate specific scenario/performance limits
% ** THE FOLLOWING LIMITS ARE NOT YET USED IN GENERATING TRAJECTORIES **
% Max_turn_rate       = 3*pi/180; % Units in rad/sec
% Max_phi             = 30*pi/180; % Max angle-of-bank, units in rad
% Phi_time            = 3.5; % Time to roll from wings level to necessary aob for the turn
% Min_Head_2Sided     = 20*pi/180; % Below use two sided roll, above use 2 one-sided rolls
% grav                = defUnits.g0; % m/sec^2 Earths gravity constant
% pre_flag            = 1; % Flag to utilize early turn computations

% User defined/derived trajectory planning variables
num_traj        = 3000; % Specify the number of random trajectories to create
out_path        = './'; % Desired absolute or relative path for output .mat file
out_fname       = 'ref_trajectory_data.mat';
min_speed       = 0.01; % Minimum starting speed in m/sec, (must be >0)
max_speed       = 3.5; % Maximum starting speed in m/sec
min_alt         = 10; % Minimum starting/ending altitude in meters
max_alt         = 50; % Maximum starting altitude in meters
min_time        = 30; % Minimum trajectory time in sec
max_time        = 50; % Maximum trajectory time in sec
min_turn_rate   = 0.1*pi/180; % Minimum turn rate for turning traj (must be >=0), rad/sec
max_turn_rate   = 3*pi/180; % Maximum turn rate for turning traj (must be >=0), rad/sec

% Note trajectory generated in the NED (north, east, down) frame
% Users can specify multiple choices of x&y starting positions
init_pos_arr       = [0, 0; 100, 100]; % Initial positions array [x1,y1;x2,y2] in m
num_init_pos        = size(init_pos_arr,1);
% Prescribes range of allowable initial headings (rad) for each initial starting position
init_head_arr      = [0, 2*pi; 3/4*pi, 3*pi/2]; 

% *************************************************************************

% ******************** Randomly generate trajectories *********************
% Generate random trajectory types. Random (uniform) integer 0,1,2 
% (0 => straight traj, 1 => const turn rate, 2 => elliptical)
traj_type      = randi([0,1],[num_traj,1]); % Elliptical not yet implemented 

% **************** Create random initial positions ************************
init_alt            = (rand([num_traj,1])*(max_alt-min_alt) + min_alt); % Initial altitude
init_pos_type       = randi([1, num_init_pos],[num_traj,1]); % 
init_pos            = [init_pos_arr(init_pos_type,:) init_alt];

%  ************* Create random initial velocities and headings ************
                %init_speed      = rand([num_traj,1])*0; %  Initial fwd velocity
init_speed      = (rand([num_traj,1])*(max_speed-min_speed) + min_speed); %  Initial fwd velocity
init_head       = rand([num_traj,1]).*(init_head_arr(init_pos_type,2)-init_head_arr(init_pos_type,1)) + init_head_arr(init_pos_type,1);
init_vel_xy     = [cos(init_head).*init_speed sin(init_head).*init_speed]; % Convert heading and speeds to velocities

% ****************** Compute final speed **********************************
% Note the type of trajectory determines final speed
% Constant turn => constant speed, straight traj => random final speed
fin_speed = init_speed; % This is only true for constaint rate turns (init_traj_type == 1)

% Compute random fin_speed and only assign to init_traj_type == 0 (straight traj)
        %acc_dec_type  = 2*ones(num_traj,1); %randi([1, 2],[num_traj,1]); % 1== decelerate, 2 = accelerate
acc_dec_type  = randi([1, 2],[num_traj,1]); % 1== decelerate, 2 = accelerate
ran_fin_speed = (acc_dec_type==1).*(init_speed-rand([num_traj,1]).*(init_speed-min_speed)) + ...
    (acc_dec_type==2).*(rand([num_traj,1]).*(max_speed-init_speed)+init_speed);
fin_speed(traj_type == 0) = ran_fin_speed(traj_type == 0);
% **************** Compute Turn *******************************************
% Compute a random turn  (only applies to turning trajectories)
turn_rate       = (rand([num_traj,1]).*(max_turn_rate-min_turn_rate)+min_turn_rate); 
turn_set        = [-1; 1]; % set of permissible turn directions, -1 => left turn, 1 => right turn
turn_dir_ind    =  randi([1, 2],[num_traj,1]); 
turn_dir        = turn_set(turn_dir_ind); % -1 => left turn, 1 => right turn

% ************ Compute a random total trajectory time *********************
traj_time = rand([num_traj,1]).*(max_time-min_time)+min_time;

% ************ Compute average speed **************************************
avg_speed = (fin_speed+init_speed)/2; % speed is constant for turning traj

% ************ Compute altitude change and vertical velocity **************
% Create random (constant speed) altitude change: Note NED frame => negative is climbing
% and then compute constant vertical velocities
del_alt_set     = [-1; 1]; % set of permissible alt changes, -1 => climb, 1 => descend
del_alt_ind     =  randi([1, 2],[num_traj,1]); 
del_alt_coef    = del_alt_set(del_alt_ind); % -1 => climb, 1 => descend 
fin_alt         = (del_alt_coef==1).*(init_alt-rand([num_traj,1]).*(init_alt-min_alt)) + ...
    (del_alt_coef==-1).*(rand([num_traj,1]).*(max_alt-init_alt)+init_alt);
vert_vel_init   = (fin_alt-init_alt)./traj_time; % NED frame => -alt delta == climb
vert_vel_fin    = vert_vel_init; % All altitude changes are done at constant vert vel

% **************** Compute final x & y velocities based on traj type ************
% Now compute the final velocities based on traj type
fin_lin_head    = init_head; % For linear traj, final head == initial heading
fin_lin_vel_xy  = [cos(fin_lin_head).*fin_speed, sin(fin_lin_head).*fin_speed];
turn_ang        = turn_rate.*traj_time.*turn_dir;
fin_head        = init_head + turn_ang;
fin_turn_vel_xy  = [cos(fin_head).*fin_speed, sin(fin_head).*fin_speed];
fin_turn_vel_xy2 = [cos(turn_ang.*turn_dir).*fin_lin_vel_xy(:,1)-sin(turn_ang.*turn_dir).*fin_lin_vel_xy(:,2), ...
    sin(turn_ang.*turn_dir).*fin_lin_vel_xy(:,1)+cos(turn_ang.*turn_dir).*fin_lin_vel_xy(:,2)];

% *************** Compute final positions based on traj type **************
% Compute fin_lin_pos (straight trajectories only) as init_pos + avg_speed*time
fin_lin_pos = [init_pos(:,1:2) + [cos(init_head).*avg_speed.*traj_time, sin(init_head).*avg_speed.*traj_time], fin_alt];

% Compute fin_turn_pos (turning trajectories)
% Determine center of circle and rotate by the heading change along the circle.
circ_Rad        = init_speed./turn_rate; % Note speed and turn rat >=0
rad_vec         = circ_Rad.*[cos(init_head) sin(init_head)]; % Create radius vector in direction of initial heading
% Rotate the radius by 90 deg (depending on direction of turn..) but cos(pi/2) =0
% rot_rad_vec     = [cos(pi/2.*turn_dir).*rad_vec(:,1)-sin(pi/2.*turn_dir).*rad_vec(:,2), ...
%     sin(pi/2.*turn_dir).*rad_vec(:,1)+cos(pi/2.*turn_dir).*rad_vec(:,2)];
rot_rad_vec     = [-sin(pi/2.*turn_dir).*rad_vec(:,2), sin(pi/2.*turn_dir).*rad_vec(:,1)];
circ_cen_xy     = init_pos(:,1:2)+rot_rad_vec; % Add the rotated directional Radius vector to init_pos
circ_cen_xy_rel = rot_rad_vec; % Add the rotated directional Radius vector to init_pos

% Since aircraft heading change is same as angle which prescribes arc
% change along the circle (not true for ellipse), then just rotate vec from
% circle center to init_pos by heading change and add the circle center position
% to arrive at final position
vec_cen_2_init = -circ_cen_xy_rel;

rot_cen_2_init = [cos(turn_ang).*vec_cen_2_init(:,1)-sin(turn_ang).*vec_cen_2_init(:,2), ...
    sin(turn_ang).*vec_cen_2_init(:,1)+cos(turn_ang).*vec_cen_2_init(:,2)];
fin_turn_pos = [circ_cen_xy + rot_cen_2_init, fin_alt];

% ************** Compute initial and final accelerations ******************
mean_lin_acc        = (fin_speed-init_speed)./traj_time; % Compute avg scalar accel
init_accel_xy_lin   = [cos(fin_lin_head).*mean_lin_acc, sin(fin_lin_head).*mean_lin_acc]; % Rotate scalar accel in x & y axes based on straight traj heading
fin_accel_xy_lin    = init_accel_xy_lin;

mean_turn_acc   = avg_speed.^2./circ_Rad; % Note turn is at constant speed
init_accel_xy_turn   = [cos(init_head).*mean_turn_acc, sin(init_head).*mean_turn_acc];
% Correct rotation computation below (but since cos(pi/2) == 0)
% init_accel_xy_turn = [cos(pi/2).*turn_dir.*init_accel_xy_turn(:,1)-sin(pi/2).*turn_dir.*init_accel_xy_turn(:,2), ...
%     sin(pi/2).*turn_dir.*init_accel_xy_turn(:,1)+cos(pi/2).*turn_dir.*init_accel_xy_turn(:,2)];
init_accel_xy_turn = [-sin(pi/2.*turn_dir).*init_accel_xy_turn(:,2), ...
      sin(pi/2.*turn_dir).*init_accel_xy_turn(:,1)];

fin_accel_xy_turn   = [cos(fin_head).*mean_turn_acc, sin(fin_head).*mean_turn_acc];
% Correct rotation below (but since cos(pi/2) == 0)
% fin_accel_xy_turn = [cos(pi/2).*turn_dir.*fin_accel_xy_turn(:,1)-sin(pi/2).*turn_dir.*fin_accel_xy_turn(:,2), ...
%     sin(pi/2).*turn_dir.*fin_accel_xy_turn(:,1)+cos(pi/2).*turn_dir.*fin_accel_xy_turn(:,2)];
fin_accel_xy_turn = [-sin(pi/2.*turn_dir).*fin_accel_xy_turn(:,2), ...
    sin(pi/2.*turn_dir).*fin_accel_xy_turn(:,1)];

% ************* Concatenate final pos, vel and accels *********************
% Concatenate fin_pos
fin_pos = fin_lin_pos.*(traj_type==0) + fin_turn_pos.*(traj_type==1);

% Concatenate final velocities
fin_vel_xy = fin_lin_vel_xy.*(traj_type==0) + fin_turn_vel_xy.*(traj_type==1);

% Concatenate final accelerations
init_accel_xy = (traj_type==0).*init_accel_xy_lin + (traj_type==1).*init_accel_xy_turn;
%init_accel_xy = (traj_type==0).*init_accel_xy_lin;
fin_accel_xy  = (traj_type==0).*fin_accel_xy_lin + (traj_type==1).*fin_accel_xy_turn;
%fin_accel_xy  = (traj_type==0).*fin_accel_xy_lin;
% ************************************************************************
% Set positions, velocities and accels into init and final BP waypoints
% *************************************************************************

% Set Initial wptsX (pos, vel and accel)
clear wptsX wptsY wptsZ time_wptsX time_wptsY time_wptsZ
wptsX(1,1,:) = init_pos(:,1);
wptsX(1,2,:) = init_vel_xy(:,1); 
wptsX(1,3,:) = init_accel_xy(:,1);

% Set Final wptsX (pos, vel and accel)
wptsX(2,1,:) = fin_pos(:,1);
wptsX(2,2,:) = fin_vel_xy(:,1);
wptsX(2,3,:) = fin_accel_xy(:,1);

% Set Initial wptsY (pos, vel and accel)
wptsY(1,1,:) = init_pos(:,2);
wptsY(1,2,:) = init_vel_xy(:,2);
wptsY(1,3,:) = init_accel_xy(:,2);

% Set Final wptsY (pos, vel and accel)
wptsY(2,1,:) = fin_pos(:,2);
wptsY(2,2,:) = fin_vel_xy(:,2);
wptsY(2,3,:) = fin_accel_xy(:,2);

% Set Initial wptsZ (pos, vel and accel)
wptsZ(1,1,:) = init_pos(:,3);
wptsZ(1,2,:) = vert_vel_init;
wptsZ(1,3,:) = zeros(num_traj,1); % Constant rate-of-climb => no accel

% Set Final wptsZ (pos, vel and accel)
wptsZ(2,1,:) = fin_pos(:,3);
wptsZ(2,2,:) = vert_vel_fin;
wptsZ(2,3,:) = zeros(num_traj,1); % Constant rate-of-climb => no accel

% Set time_wpts variables
time_wptsX = [zeros(num_traj,1) traj_time];
time_wptsY = [zeros(num_traj,1) traj_time];
time_wptsZ = [zeros(num_traj,1) traj_time];

% ******* Store ownship trajectories in own_traj cell array ***************
own_traj_arrays = {};
own_traj_arrays{1} = wptsX; own_traj_arrays{2} = wptsY; 
own_traj_arrays{3} = -wptsZ; % Need to flip signs on wptsZ to get to NED frame
own_traj_arrays{4} = time_wptsX; own_traj_arrays{5} = time_wptsY; own_traj_arrays{6} = time_wptsZ;

% ****** store each individual ownship traj as its' own cell *************
own_traj = {};
for loop = 1:num_traj
    own_traj{loop, 1} = wptsX(:,:,loop); own_traj{loop,2} = wptsY(:,:,loop); own_traj{loop,3} = -wptsZ(:,:,loop);
    own_traj{loop,4} = time_wptsX(loop,:); own_traj{loop,5} = time_wptsY(loop,:); own_traj{loop,6} = time_wptsZ(loop,:);
end

% ************ Save own-ship trajectories in .mat file ********************
fullname = fullfile(out_path, out_fname);
save(fullname,'own_traj', 'own_traj_arrays','-v7.3');
fprintf(1,'Successfully created own-ship trajectory file (with %i trajectories):\n%s\n', num_traj, fullname);