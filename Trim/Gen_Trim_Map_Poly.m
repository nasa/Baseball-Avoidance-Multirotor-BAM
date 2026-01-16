function [t_struc] = Gen_Trim_Map_Poly(tmap_fname)

% function [trim_array] = Gen_Trim_Map(c, units, rho)
% The goal of this algorith is to use the trim algorithm is to determine the quad-rotor
% orientation (phi, theta not psi) and thrust settings that yield
% the desired vehicle orientation in a trimmed manner (i.e., x_dot(1..9) = 0)
% and to perform this repeatedly to generate a trim map.
%
% Inputs
%   tmap_fname: path/filename of trim map .mat data file   
% 
% Outputs:
%   t_struc: output structure of trim maps (pitch, roll, prop rpms)
%
% Parameters:
%   c:          structure of multi-rotor prop parameters
%   units:      units structure (i.e., SimIn.Units)
%   rho:        atmosphere density (in units slug/ft3)
%
% Written by Michael J. Acheson, Dynamics and Control Branch (D-316)
% NASA Langley Research Center
%
% History:
% 2/11/2025, MJA Initial Version

% Non-linear (rigid body) states and controls
%   x: states 
%       = [phi,theta,psi,... %(rad)
%          u,v,w,...         %(ft/s)
%          p,q,r,...         %(rad/s)
%          x,y,z]            %(ft)
%   u: control inputs vector (rad/sec)
% *************************************************************************

% Load in multirotor relevant information
SimIn = struct();
addpath('setup');
SimIn.Units = setUnits('m','kg');
SimIn = IMPACT_props2(SimIn);


% Assume we need monotonic increasing indexes for Matlab interp3 command
u_vec = -20:5:30; % Body frame longitudinal speed array in ft/sec
v_vec = -10:5:10; % Body frame longitudinal speed array in ft/sec
w_vec = -15:5:15; % Body frame vertical speed array in ft/sec
u_len = length(u_vec);
v_len = length(v_vec);
w_len = length(w_vec);

% Set up the trim struct
t_struc = struct();
t_struc.u_vec = u_vec;
t_struc.v_vec = v_vec;
t_struc.w_vec = w_vec;
t_struc.phi = NaN(v_len, u_len, w_len);
t_struc.theta = NaN(v_len, u_len, w_len);
t_struc.p1 = NaN(v_len, u_len, w_len);
t_struc.p2 = NaN(v_len, u_len, w_len);
t_struc.p3 = NaN(v_len, u_len, w_len);
t_struc.p4 = NaN(v_len, u_len, w_len);

% Define units necessary for the non-linear EOMs
g_fps2 = SimIn.Units.g0/SimIn.Units.ft;

% Set the optimization lower and upper bounds
lb = [-45*pi/180; -45*pi/180; 0; 0; 0; 0];
ub = [45*pi/180; 45*pi/180; inf; inf; inf; inf];
prop_0 = 7000*ones(4,1); % Initial prop rotor estimate in rpm

% Compute trim conditions
u_cnt = 0;  
for u_loop = u_vec
    u_cnt = u_cnt + 1;
    v_cnt = 0;
    for v_loop = v_vec
        v_cnt = v_cnt + 1;
        w_cnt = 0;
        for w_loop = w_vec
            w_cnt = w_cnt + 1;
            % Initialize x = [pitch roll psi;u v w; omega; pos];
            x = [zeros(3,1);u_loop;v_loop;w_loop;zeros(6,1)];

            % Initialize optimization vars: 
            % [pitch (rad), roll (rad); 4 rotor speeds (rad/s)]
            eu_0 = [x(1:2);prop_0];
            [ eu, fval, exitflag, output, ~, ~, ~ ] = fmincon(@(eu)mycost_poly(x(3:12), eu, SimIn.c, g_fps2),...
                eu_0, [], [], [], [],lb, ub,  @(eu)nlinCon_poly(x(4:9), eu, g_fps2, SimIn.c.mass_slug));
            t_struc.phi(v_cnt, u_cnt,w_cnt) = eu(1);
            t_struc.theta(v_cnt, u_cnt,w_cnt) = eu(2);
            t_struc.p1(v_cnt, u_cnt,w_cnt) = eu(3)*60/2/pi; % Store rad/sec output as rpm
            t_struc.p2(v_cnt, u_cnt,w_cnt) = eu(4)*60/2/pi; % Store rad/sec output as rpm
            t_struc.p3(v_cnt, u_cnt,w_cnt) = eu(5)*60/2/pi; % Store rad/sec output as rpm
            t_struc.p4(v_cnt, u_cnt,w_cnt) = eu(6)*60/2/pi; % Store rad/sec output as rpm
            fprintf(1, sprintf('Current iter (u,v,w): %g %g %g\n',u_vec(u_cnt), v_vec(v_cnt), w_vec(w_cnt)));
            if exitflag ~=1 && exitflag ~=2
                disp('');
            end
        end % End of for w_loop
    end % End of for v_loop
end % End of for u_loop

% Output the results to a file if desired
if nargin == 1
    save(tmap_fname,'-struct', 't_struc', '-v7.3');
end