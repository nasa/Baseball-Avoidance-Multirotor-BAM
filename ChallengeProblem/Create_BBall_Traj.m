% Create_BBall_Traj.m creates a baseball trajectory that intersects the 
% given own-ship trajectory. This is accomplished by sampling "middle" (time) 
% of the given own-ship trajectory for the range of altitudes in the
% own-ship trajectory. A required "vertical" velocity range is computed which
% is then randomly sampled in this range.  Next a forward velocity is
% randomly sampled (adhering to a maximum total velocity). Additionally, 
% a random baseball spin is generated which will give a horizontal magnus force
% component. Baseball drag coefficient and gravity are randomly
% chosen. This trajectory of the baseball is then simulated using the 
% the Dynamics.slx simulation. To compute the "collision" point, first the 
% specified own-ship % trajectory is interrogate at the randomly chosen impact time.  
% This sampling % provides the position and velocity of own-ship at the specified 
% time. The collision point is then randomly offset in a random orientation 
% (360 degrees) and a random radius in the circle perpendicular to the own-ship 
% velocity. Finally once the baseball trajectory is simulated it is rotated to be 
% directly opposite the own-ship trajectory and then randomly perturbe to have
% and incoming angle (e.g., +/- 45 deg).  

% Written by: Michael J. Acheson, michael.j.acheson@nasa.gov
% NASA Langley Research Center (LaRC), 
% Dynamics Systems and Control Branch (D-316)

% Versions:
% 4/8/2025, MJA: Initial version of script.  Randomly creates a baseball
% trajectory for a given height range for a baseball own-ship trajectory, 
% within the desired impact time window
% *************************************************************************

% *************************************************************************
% Add required paths to run this file stand-alone (not required if setup.m was run)
if ~exist('userStruct','var')
    addpath('../ChallengeProblem');
    addpath('../Bez_Functions');
    addpath('../lib/utilities'); % needed for setUnits
    addpath('../Ref_Models/Baseball'); % Needed to have access to Dyanamics.slx
end

% Specify the seed/generator for use with random number generator
rng('default'); % Use the matlab default: seed number =0,  generator = Mersenne Twister

% Initialize the default units structure
defUnits    = setUnits('m','kg');
grav_act    = defUnits.g0; % get Earths gravity constant

% *************************************************************************
% Set baseball parameters
bball_mass  = 0.145; % Baseball mass (kg)
bball_diam  = 2.90*defUnits.in; % Baseball diameter (meters)
bball_area  = pi/4*bball_diam^2; % Baseball flat plate area meters^2)
Atmos_den   = 1.224999155887712; % Density of standard atmos air (kg/m^3)
bball_MagC  = 4.1e-5; % Magnus Force coefficient for baseball 

% Set user defined collision trajectory parameters
out_path        = './'; % Desired absolute or relative path for output .mat file
out_fname       = 'bball_trajectory_data.mat'; % Output file for baseball trajectories
net_time        = 10; % No earlier than time (after start of own-ship traj) for collision to occur (seconds)
nlt_time        = 10; % No later than time (prior to end of own-ship traj) for collision to occur (seconds)
%                    min_bball_time  = 10; % Minimum time baseball is in flight prior to impact

% The following values should corresponding with Create_OwnShip_Traj.m values
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
spin_ang_max    = 45*defUnits.deg; % +/- spin axis rotation angle (around x-axis)

% Prescibe collision point offset parameters from randomly selected own-ship trajectory collision point
max_perp_dist   = 10*defUnits.ft; % maximum perp distance from unit own-ship vel vector at traj "col" point

num_traj        = 3000; % Specify number of baseball trajectories to create from own_ship trajectories..
% *************************************************************************

% *************************************************************************
% Allocate array space
bball_wptsX      = NaN(3,3,num_traj);
bball_wptsY      = NaN(3,3,num_traj);
bball_wptsZ      = NaN(3,3,num_traj);
bball_time_wptsX = NaN(num_traj, 3);
bball_time_wptsY = NaN(num_traj, 3);
bball_time_wptsZ = NaN(num_traj, 3);
bball_meta       = NaN(17,num_traj);
col_meta         = NaN(14,num_traj);

for own_traj_num = 1:num_traj % Own-ship trajectory number
    % Randomize gravity to use to alter the resulting ballistic trajectory for each run
    grav = rand(1)*(grav_max-grav_min) + grav_min; % Set gravity term 

    % Randomly choose a coefficient of drag for the baseball.
    cd =rand(1)*(cd_max-cd_min)+cd_min; % Uniform random baseball coefficient of drag (non-dimensional)

    % Randomly choose baseball spin vector 
    spin_mag = rand(1)*(spin_max - spin_min) + spin_min; % Spin rate in rpm
    spin_sign = rand(1); spin_sign = 1*(spin_sign>=0.5)-1*(spin_sign<0.5);
    spin_vec = [0;0;spin_sign*spin_mag]; % Baseball spin axis vector (RPM)
    spin_ang = (rand(1)*(2*spin_ang_max)-spin_ang_max)*defUnits.deg; % Spin axis rotation angle in rad
    spin_vec = [1 0 0; 0 cos(spin_ang) -sin(spin_ang);0 sin(spin_ang) cos(spin_ang)]*spin_vec;

    % Load in the own-ship trajectory of interest
    file_obj        = matfile('./ref_trajectory_data.mat'); %
    wptsX_cell      = file_obj.own_traj(own_traj_num,1);
    wptsY_cell      = file_obj.own_traj(own_traj_num,2);
    wptsZ_cell      = file_obj.own_traj(own_traj_num,3);
    time_wptsX_cell = file_obj.own_traj(own_traj_num,4);
    time_wptsY_cell = file_obj.own_traj(own_traj_num,5);
    time_wptsZ_cell = file_obj.own_traj(own_traj_num,6);
    
    % create BP piecewise polynomial
    CurTraj_pwcurve = genPWCurve({wptsX_cell{1},wptsY_cell{1},wptsZ_cell{1}},...
            {time_wptsX_cell{1}, time_wptsY_cell{1}, time_wptsZ_cell{1}});
    
    % Interrogate the trajectory
    traj_time       = time_wptsX_cell{1}(end); % Get the total own-ship trajectory time
    col_time_win    = [net_time traj_time-nlt_time]; % Time window for collision to occur
    traj_col_time        = rand(1)*(col_time_win(2)-col_time_win(1))+col_time_win(1);
    
    % Determine own_traj position at collision time
    pos         = evalPWCurve(CurTraj_pwcurve, traj_col_time,0); % own-ship position at collision time
    vel         = evalPWCurve(CurTraj_pwcurve, traj_col_time,1); % own-ship velocity at collision time
    vel_hat     = vel/vecnorm(vel); % Unit vector of own-ship velocity

    % *********************************************************************
    % Randomly create offset radius and direction for placement of
    % collision point relative to "collision" point on own-ship traj
    obj_perp_dist   = rand(1).*(max_perp_dist); % Distance (ft) of object ball center from own-trajectory at obj_time
    obj_orient      = rand(1)*2*pi; % Orientation of object ball center with respect to xy velocity vector at object time

    % Create vector perpendicular to own_ship velocity with z component 
    % directly above the trajectory "collision" point
    temp_vec            = [0;0;-1];     
    temp_vec_perp       = temp_vec-(temp_vec'*vel_hat')*vel_hat';
    temp_vec_perp_hat   = temp_vec_perp/vecnorm(temp_vec_perp);

    % Create the quaternion to rotate around the velocity vector hat
    qr = [cos(obj_orient/2);sin(obj_orient/2)*vel_hat'];

    % Rotate the perp unit vector around the unit velocity vector
    temp_perp_hat_rot = Qtrans(qr, temp_vec_perp_hat);

    % Determine the position of the collision center
    col_cent_pos = pos + temp_perp_hat_rot'*obj_perp_dist;
    alt_col = abs(col_cent_pos(3)); % Modify collision altitude from rand selected own-ship to offset collision pt

    % Compute the minimum vert vels to reach various alts (ballistic only, i.e., no drag) 
    % (i.e. Discriminant = v_^2-4*(g/2)*(h_f-h_0) = 0)
    bball_h0         = 0; % Initialize all bball starts at ground (can vary this...)
    min_vvel_min_alt = sqrt(2*grav*(min_alt-bball_h0)); % Compute min vert vel to "just" get to min altitude
    min_vvel_col_alt = sqrt(2*grav*(alt_col-bball_h0)); % Compute min vert vel to "just" get to collision alt
    min_vvel_max_alt = sqrt(2*grav*(1.1*(max_alt-bball_h0))); % Compute min vert vel to "just" get to 1.1*max altitude
    
    % Randomly select a vert vel in [min_vvel_col_alt, min_vvel_max_alt]
    vvel_init    = rand(1)*(min_vvel_max_alt-min_vvel_col_alt) + min_vvel_col_alt;
    
    % ***************** Analytic Solution not used ************************
    % Compute the analytic collision time (quadratic formula on the vertical position 
    % equation with constant acceleration of gravity: P_f=P_i+V_i*t-a/2*t^2 and no vert drag, nor spin)
    % t_2_col1    = (vvel_init - sqrt(vvel_init^2-4*grav/2*(abs(alt_col)-bball_h0)))/grav;
    % t_2_col2    = (vvel_init + sqrt(vvel_init^2-4*grav/2*(abs(alt_col)-bball_h0)))/grav; % time from launch till ball gets to collision height 
    % t_2_col_set = [t_2_col1, t_2_col2]; % [First time ball is at col_alt, second time ball is at col_alt]
    % *********************************************************************

    % Randomly choose first or second time to impact height
    time_set_ind    = randi([1, 2],1); 
    % t_2_col         = t_2_col_set(time_set_ind); % 1 => first time at alt, 2 => second time at alt
    
    % ***************** Analytic Solution not used ************************
    % % Determine analytic Total Ball TOF (till hits ground) using only gravity
    % ball_tof_act    = (vvel_init + vvel_init)/grav; % This is the time from launch till bball hits ground (i.e. total time of ball's flight)
    % vvel_at_col     = vvel_init-grav*t_2_col; % This is vertical velocity at collision time (pointing up if first time, down if second time)
    % *********************************************************************
    
    % Compute a random forward velocity
    forward_vel_range   = [min_vvel_min_alt, min_vvel_max_alt]; % Allowable forward velocity ranges, this should set the impact angle from ~0deg to ~30deg
    init_forward_vel    = rand(1)*(forward_vel_range(2)-forward_vel_range(1))+forward_vel_range(1); % This is the bball starting intial forward velocity

    % *********************************************************************
    % Simulate the ball dynamics
    mdl ='Dynamics';
    SimInDyn = struct();
    SimInDyn.('Vel_i')          = [init_forward_vel; 0; vvel_init];
    SimInDyn.('Pos_i')          = [0; 0; bball_h0];
    SimInDyn.('bball_mass')     = bball_mass;
    SimInDyn.('MagFor_Coef')    = bball_MagC; % This value was numerically derived to get sufficient movement
    SimInDyn.('Spin')           = spin_vec*2*pi/60; % Rev/min converted to rad/sec
    SimInDyn.('bball_diam')     = bball_diam; % Baseball diameter (meters)
    SimInDyn.('bball_area')     = bball_area; % Baseball flat plate area meters^2)
    SimInDyn.('bball_cd')       = cd; % Baseball drag coefficient (non-dimensional)
    SimInDyn.('Atmos_den')      = Atmos_den; % Density of standard atmos air (kg/m^3)
    SimInDyn.('grav')           = grav;
    % Simulate the baseball trajectory using Dynamics.slx    
    out = sim(mdl,StopTime=num2str(bball_t_max), SimulationMode="accelerator"); 

    % ****** Interrogate dynamics sim output for bball states *************
        
    % remove any data at the end of the sim for which z is less than zero.
    % end_ind = length(out.yout{1}.Values.time);
    % while(out.yout{1}.Values.Data(end_ind,3)<0)
    %     end_ind = end_ind -1;
    % end

    % time_sim = out.yout{1}.Values.time(1:end_ind); % col vector
    % pos_sim = out.yout{1}.Values.Data(1:end_ind,:); % rows => time steps, columns => x,y,z
    % vel_sim = out.yout{2}.Values.Data(1:end_ind,:); % rows => time steps, columns => x,y,z
    % acc_sim = out.yout{3}.Values.Data(1:end_ind,:); % rows => time steps, columns => x,y,z

    time_sim = out.yout{1}.Values.time(1:end); % col vector
    pos_sim = out.yout{1}.Values.Data(1:end,:); % rows => time steps, columns => x,y,z
    vel_sim = out.yout{2}.Values.Data(1:end,:); % rows => time steps, columns => x,y,z
    acc_sim = out.yout{3}.Values.Data(1:end,:); % rows => time steps, columns => x,y,z
    
    % Find the intervals in sim vert height that contain the collision alt (if any)
    len_time = length(time_sim);
    found_ind  = [];
    ht_max_ind = [];
    ht_max     = 0;
    for loop = 1:len_time-1
        if (pos_sim(loop,3) <= alt_col && alt_col <= pos_sim(loop+1,3)) || ...
            (pos_sim(loop,3) >= alt_col && alt_col >= pos_sim(loop+1,3))
            found_ind =[found_ind loop];
        end
        if ht_max <= pos_sim(loop,3)
            ht_max = pos_sim(loop,3);
            ht_max_ind = loop;
        end
    end

    % Determine the collision time or the time of max height reached (if col alt not reached)

    if isempty(found_ind)
        % Need to raise the initial altitude such that a collision occurs..
        if time_set_ind == 1
            alt_delta = alt_col-pos_sim(ht_max_ind,3); % Altitude delta reqd to make collision happen
            sim_t_2_col = time_sim(ht_max_ind);
        else 
            alt_delta = alt_col-pos_sim(ht_max_ind+1,3); % Altitude delta reqd to make collision happen
            sim_t_2_col = time_sim(ht_max_ind+1);
        end
    elseif isscalar(found_ind) % only one interval found
        % Interpolate the data to find the one time that the collision height was reached..
        ind = found_ind;
        %sim_t_2_col = makima(pos_sim(ind:ind+2,3),time_sim(ind:ind+2), alt_col);
        sim_t_2_col = interp1(pos_sim(ind:end,3),time_sim(ind:end), alt_col);
        if ~(sim_t_2_col >= time_sim(ind) && sim_t_2_col <= time_sim(ind+2))
            % Use alternate interpolation if interpolation fails
            sim_t_2_col = pchip(pos_sim(ind:ind+2,3),time_sim(ind:ind+2), alt_col);
        end
        if ~(sim_t_2_col >= time_sim(ind) && sim_t_2_col <= time_sim(ind+2))
            error('Polynomial interpolation failed to find a time at which bball reaches the desired height within the window. Likely issue with interpolation function..');
        end
        alt_delta = alt_col -interp1(time_sim,pos_sim(:,3), sim_t_2_col,'spline'); % Altitude delta reqd to make collision exactly happen
    elseif length(found_ind) >= 2
        % Select the first or second collision height reached based on the random time_set_ind
        ind = found_ind(time_set_ind);
        % if (time_set_ind ==1 && ind ~=1) % || ((ind+1)==len_time)
        if (time_set_ind ==1) && (ind+1>=4)
            if found_ind(2) == found_ind(1)+1
                sim_t_2_col = interp1(pos_sim(1:ind,3),time_sim(1:ind), alt_col,'spline');
            else
                sim_t_2_col = interp1(pos_sim(1:ind+1,3),time_sim(1:ind+1), alt_col,'spline');  
            end
        elseif (time_set_ind ==1) && (ind+1<4)
            %sim_t_2_col = makima(pos_sim(ind-1:ind+1,3),time_sim(ind-1:ind+1), alt_col);
            sim_t_2_col = interp1(pos_sim(1:4,3),time_sim(1:4), alt_col,'spline');
        elseif (time_set_ind ==2 && (len_time-ind+1)>=3) % Have at least three pts
            %sim_t_2_col = makima(pos_sim(ind:ind+2,3),time_sim(ind:ind+2), alt_col);
            if found_ind(1) == found_ind(2)-1
                sim_t_2_col = interp1(pos_sim(ind:end,3),time_sim(ind:end), alt_col,'spline');
                if ~((sim_t_2_col>=time_sim(ind)) && (sim_t_2_col<=time_sim(ind+1)))
                    sim_t_2_col = interp1(pos_sim(ind-1:end,3),time_sim(ind-1:end), alt_col,'spline');
                end
            else
                sim_t_2_col = interp1(pos_sim(ind-1:end,3),time_sim(ind-1:end), alt_col,'spline');
                if ~((sim_t_2_col>=time_sim(ind)) && (sim_t_2_col<=time_sim(ind+1)))
                    sim_t_2_col = interp1(pos_sim(ind:end,3),time_sim(ind:end), alt_col,'spline');
                end
            end
        elseif (time_set_ind ==2 && (len_time-ind+1)<3) % Have at least three pts
            sim_t_2_col = interp1(pos_sim(end-3:end,3),time_sim(end-3:end), alt_col,'spline');
        else
            fprintf(1,'How did this happen?')
            keyboard
        end
        if ~((sim_t_2_col>=time_sim(ind)) && (sim_t_2_col<=time_sim(ind+1)))
            fprintf('Polynomial interpolation failed to find a time at which bball reaches the desired height within the window. Likely issue with makima function..')
            keyboard
        end
        alt_delta = alt_col -interp1(time_sim,pos_sim(:,3), sim_t_2_col,'spline'); % Altitude delta reqd to make collision exactly happen
    end

    % Interpolate sim results for the bball pos, vel and acc at t_col
    bball_pos_col_sim   = interp1(out.yout{1}.Values.Time, out.yout{1}.Values.Data, sim_t_2_col, 'spline');
    % Augment the position with altitude as reqd to make collision occur
    bball_pos_col_sim   = bball_pos_col_sim + [0 0 alt_delta];
    bball_vel_col_sim   = interp1(out.yout{2}.Values.Time, out.yout{2}.Values.Data, sim_t_2_col, 'spline');
    bball_acc_col_sim   = interp1(out.yout{3}.Values.Time, out.yout{3}.Values.Data, sim_t_2_col, 'spline');

    % Get the pos, vel and acc from sim data at t=0
    bball_pos_sim   = out.yout{1}.Values.Data(1,:); % Simulation of relative position change (i.e. Pos_i = 0)
    % Augment the position with altitude as reqd to make collision occur
    bball_pos_sim   = bball_pos_sim + [0 0 alt_delta]; 
    bball_vel_sim   = out.yout{2}.Values.Data(1,:); % Simulation of long velocity with drag model only
    bball_acc_sim   = out.yout{3}.Values.Data(1,:); % Simulation of long deceleration with drag model only

    % Set the bball final time as:
    if (time_sim(end)-sim_t_2_col) < traj_time-traj_col_time
        % Check if time remaining in baseball (after collision) is >
        % time remaining in own traj (after collision)
        bball_t_fin = time_sim(end);
    else 
        % This situation shouldn't occur.. but randomly it is possible..
        % Set the final time to the collision time + the smaller time left
        % in the ownship traj (after col)
        bball_t_fin = sim_t_2_col + traj_time-traj_col_time;
    end

    % Interpolate sim results for the bball pos, vel and acc at t_end
    bball_pos_sim_end   = interp1(out.yout{1}.Values.Time, out.yout{1}.Values.Data, bball_t_fin, 'spline');
    % Augment the position with altitude as reqd to make collision occur
    bball_pos_sim_end   = bball_pos_sim_end + [0 0  alt_delta]; 
    bball_vel_sim_end   = interp1(out.yout{2}.Values.Time, out.yout{2}.Values.Data, bball_t_fin, 'spline');
    bball_acc_sim_end   = interp1(out.yout{3}.Values.Time, out.yout{3}.Values.Data, bball_t_fin, 'spline');

    % Concatenate baseball states at t=0, t= col, t=final
    bball_pos = [bball_pos_sim; bball_pos_col_sim; bball_pos_sim_end];
    bball_vel = [bball_vel_sim; bball_vel_col_sim; bball_vel_sim_end];
    bball_acc = [bball_acc_sim; bball_acc_col_sim; bball_acc_sim_end];
    % *********************************************************************
    
    % Rotate x&y sim results to be directly opposite + desired random az angle off own-ship heading
    rand_az_ang     = rand(1)*(max_az_ang-min_az_ang)+min_az_ang; % random azimuth angle from baseball vel vec at time of impact
    vel_xy_hat      = vel(1:2)/vecnorm(vel(1:2));
    own_traj_head   = atan2(vel_xy_hat(2), vel_xy_hat(1));
    tot_rot_ang     = own_traj_head+pi+rand_az_ang; % This is the desired final heading of the bball at collision...
    rot_mat         = [cos(tot_rot_ang) sin(tot_rot_ang);-sin(tot_rot_ang) cos(tot_rot_ang)]; % Note LH rotation for NED (looking from above)

    bball_pos(:,1:2) = bball_pos(:,1:2)*rot_mat; 
    bball_vel(:,1:2) = bball_vel(:,1:2)*rot_mat;
    bball_acc(:,1:2) = bball_acc(:,1:2)*rot_mat;

    % Shift the sim positions to be coicident with the own_ship collision point
    col_delta         = col_cent_pos(1:2) - bball_pos(2,1:2);
    bball_pos(:,1:2)  = bball_pos(:,1:2) +  col_delta;

    % Multiple z components by -1 for z-axis down
    bball_pos(:,3) = -bball_pos(:,3);
    bball_vel(:,3) = -bball_vel(:,3);
    bball_acc(:,3) = -bball_acc(:,3);

    % ***************** Analytic Solution not used ************************
    % % Compute analytic position and vertical vel of bball at t_final
    % bball_hf = bball_h0 +vvel_init*bball_t_fin-1/2*grav*bball_t_fin^2; % Analytic height of bball at t_final
    % if bball_hf< 0 % set ball height to zero if numerically just below 0
    %     bball_hf = 0;
    % end
    % vvel_t_fin  = vvel_init-grav*bball_t_fin; % This is vertical velocity at collision time (pointing up if first time, down if second time)  
    % *********************************************************************
    
    % Set the bball BP waypoints and time waypoints
    bball_wptsX(:,:,own_traj_num) = [bball_pos(:,1) bball_vel(:,1) bball_acc(:,1)];
    bball_wptsY(:,:,own_traj_num) = [bball_pos(:,2) bball_vel(:,2) bball_acc(:,2)];
    bball_wptsZ(:,:,own_traj_num) = [-bball_pos(:,3) -bball_vel(:,3) -bball_acc(:,3)];

    bball_time_wptsX(own_traj_num,:) = [traj_col_time-sim_t_2_col traj_col_time traj_col_time-sim_t_2_col+bball_t_fin]; 
    bball_time_wptsY(own_traj_num,:) = [traj_col_time-sim_t_2_col traj_col_time traj_col_time-sim_t_2_col+bball_t_fin]; 
    bball_time_wptsZ(own_traj_num,:) = [traj_col_time-sim_t_2_col traj_col_time traj_col_time-sim_t_2_col+bball_t_fin];     
    
    % *********************************************************************
    % Store the metadata results
    % bball_meta: Vel_i; Pos_i; bball_mass; bball_MagC; SimInDyn.Spin,
    %       bball_diam; bball_area; cd; Atmos_den; grav; traj_num
    bball_meta(:,own_traj_num) =  [SimInDyn.Vel_i; SimInDyn.Pos_i; bball_mass; 
        bball_MagC; SimInDyn.Spin; bball_diam; bball_area; cd; Atmos_den; 
        grav; own_traj_num];
    % col_meta: 
        % own-ship pos at col; own-ship vel at col; own-ship col time;
        % offset col pos; radial distance from own-ship to offset col pos;
        % radial angle to offset col pos; baseball sim col time; traj num
    col_meta(:,own_traj_num) = [pos'; vel'; traj_col_time; col_cent_pos'; obj_perp_dist; obj_orient;
        sim_t_2_col; own_traj_num];
    % *********************************************************************

    if mod(own_traj_num, 10) == 0
        fprintf('Completed baseball trajectory generation #%i\n', own_traj_num);
    end
end

% ******* Store ownship trajectories in own_traj cell array ***************
bball_traj_arrays = {};
bball_traj_arrays{1} = bball_wptsX; bball_traj_arrays{2} = bball_wptsY; 
bball_traj_arrays{3} = bball_wptsZ; % Need to flip signs on wptsZ to get to NED frame
bball_traj_arrays{4} = bball_time_wptsX; bball_traj_arrays{5} = bball_time_wptsY; bball_traj_arrays{6} = bball_time_wptsZ;

% ****** store each individual baseball traj as its' own cell *************
bball_traj = cell(num_traj,6); % Preallocate cell storage
for loop = 1:num_traj
    bball_traj{loop, 1} = bball_wptsX(:,:,loop); bball_traj{loop,2} = bball_wptsY(:,:,loop); bball_traj{loop,3} = -bball_wptsZ(:,:,loop);
    bball_traj{loop,4} = bball_time_wptsX(loop,:); bball_traj{loop,5} = bball_time_wptsY(loop,:); bball_traj{loop,6} = bball_time_wptsZ(loop,:);
end

% ************ Save own-ship trajectories in .mat file ********************
fullname = fullfile(out_path, out_fname);
save(fullname,'bball_traj', 'bball_traj_arrays','bball_meta','col_meta','-v7.3');
fprintf(1,'Successfully created baseball trajectory file (with %i trajectories):\n%s\n', num_traj, fullname);