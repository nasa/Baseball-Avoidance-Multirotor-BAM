function [ineq, eq] = nlinCon_poly(x_in, u_in, g_fps2, mass_slugs)
% This script is the used to enforce the equilibrium of the nonlinear
% dynamic equations of motion.  
%
% ******************* Inputs *************************************
%   x_in:       = [phi, theta, psi,... % (rad)
%               u,v,w,...         % (ft/s) (Inertial frame)
%               p,q,r,...         % (rad/s)
%               x,y,z]            % (ft)
%   u_in        = [phi, theta, controls] % angles (rad), rotors (rpm)
%   g           = Earth's gravity constant (ft/sec^2)
%   mass_slugs  = Structure of units and conversions (e.g., SimIn. Units)
%
% ******************* Outputs *************************************
% ineq  = vector of inequality constraints
% eq    = vector of equality constraints

% Written by Michael J. Acheson, Dynamics Systems and Controls Branch
% (D-316), NASA Langley Research Center
%
% Versions:
% 6/18/2025: MJA, Initial version

% *************************************************************************

% Input velocities are inertial and need to use body frame
% Map controls to variables
phi     = u_in(1);
theta   = u_in(2);
Om1_rps = u_in(3);
Om2_rps = u_in(4);
Om3_rps = u_in(5);
Om4_rps = u_in(6);

% Map states to variables
rot_phi = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
rot_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
v_bIb = rot_theta*rot_phi*x_in(1:3);
u_fps = v_bIb(1);
v_fps = v_bIb(2);
w_fps = v_bIb(3);
p_rps = x_in(4);
q_rps = x_in(5);
r_rps = x_in(6);
dOm1_rps2 = 0; % Want steady state so no rotor accel/decel
dOm2_rps2 = 0; % Want steady state so no rotor accel/decel
dOm3_rps2 = 0; % Want steady state so no rotor accel/decel
dOm4_rps2 = 0; % Want steady state so no rotor accel/decel


AnalDerivFlag = 0; % Don't compute analytic derivatives
[X_lbf,Y_lbf,Z_lbf,L_ftlbf,M_ftlbf,N_ftlbf, ~, ~] = ...
        IMPACT_AeroProp_RSE_V1_10Jun25_180239( ... 
        u_fps, v_fps, w_fps, p_rps, q_rps, r_rps, ... 
        Om1_rps, Om2_rps, Om3_rps, Om4_rps, ... 
        dOm1_rps2, dOm2_rps2, dOm3_rps2, dOm4_rps2, AnalDerivFlag);
% rot_phi = [1 0 0; 0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
% rot_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
g_rot = rot_phi*rot_theta*[0;0;-g_fps2*mass_slugs];

eq = [X_lbf-g_rot(1),Y_lbf-g_rot(2),Z_lbf-g_rot(3),L_ftlbf,M_ftlbf,N_ftlbf]';
ineq = [];
disp('');



