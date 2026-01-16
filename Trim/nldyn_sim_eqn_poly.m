function [ineq, eq] = nldyn_sim_eqn_poly(u, x, c, g_fps2)

% nldyn_sim_eqn_poly - nonlinear aircraft equations of motion
%
% DESCRIPTION: 
%   This function contains the aircraft equations of motion. This function
%   has similar functionality to the SIDPAC code "nldyn_deq.m" but has been
%   rewritten.
%
% INPUTS: 
%   param - parameter estimates vector space holder [not used]
%   u - control inputs vector
%   x - state vector 
%           = [phi,theta,psi,... %(rad)
%          u,v,w,...         %(ft/s)
%          p,q,r,...         %(rad/s)
%          x,y,z]            %(ft)
%   c -  structure of multi-rotor configuration parameters
%   g_fps2 - Earth's gravity constant (ft/sec^2)
%
% OUTPUTS:
%   ineq - None used here
%   eq -  equality constraints <=> xd_dot or time derivative of the state vector
%       = [phi_dot,theta_dot,psi_dot,... %(rad/s)
%          u_dot,v_dot,w_dot,...         %(ft/s^2)
%          p_dot,q_dot,r_dot,...         %(rad/s^2)
%          x_dot,y_dot,z_dot]            %(ft/s)
%
% CALLS:
%   None
%
% WRITTEN BY:
%   Benjamin M. Simmons and Michael J. Acheson
%   NASA Langley Research Center
%

%   The function was originally written by Benjamin M. Simmons for the
%   IMPACT multirotor simulation. The function was modified by Michael J.
%   Acheson for use in the BAM Simulation.
%
% HISTORY:
%   26 JUL 2023 - created and debugged, BMS
%   1/31/2025 - Modified by MJA to accept gravity and rho in function call
%               which ensures common definitions throughout the simulation.
%   6/23/2025 - Modified by MJA to use polynomial aero-propulsive database.
%

xd      = zeros(size(x));% initialize state derivative vector
ineq = [];

%% Define Simulation States

% Euler angles [rad]
phi_rad     = x(1,1);
theta_rad   = x(2,1);
psi_rad     = x(3,1);

% velocity components in the body frame [ft/s]
u_fps_iner   = x(4,1);
v_fps_iner   = x(5,1);
w_fps_iner   = x(6,1);

rot_phi = [1 0 0; 0 cos(phi_rad) sin(phi_rad);0 -sin(phi_rad) cos(phi_rad)];
rot_theta = [cos(theta_rad) 0 -sin(theta_rad); 0 1 0; sin(theta_rad) 0 cos(theta_rad)];
vel_fps_body = rot_phi*rot_theta*[u_fps_iner;v_fps_iner;w_fps_iner];

u_fps = vel_fps_body(1); % Body frame velocity
v_fps = vel_fps_body(2); % Body frame velocity
w_fps = vel_fps_body(3); % Body frame velocity

% angular rates in the body frame [rad/sec]
p_rps   = x(7,1);
q_rps   = x(8,1);
r_rps   = x(9,1);

% inertial position [ft]
xe_ft   = x(10,1);
ye_ft   = x(11,1);
ze_ft   = x(12,1);

% extract mass properties
m=c.mass_slug;% aircraft mass, slug
Ix=c.Ixx_slugft2;% roll moment of inertia, slug.ft^2
Iy=c.Iyy_slugft2;% pitch moment of inertia, slug.ft^2
Iz=c.Izz_slugft2;% yaw moment of inertia, slug.ft^2
Ixz=c.Ixz_slugft2;% product of inertia, slug.ft^2

%% aero and propulsion model

% ******** Compute aero forces and moments using polynomial model *********
Om1_rps = u(1)/60*2*pi; % convert rpm to rps
Om2_rps = u(2)/60*2*pi; % convert rpm to rps
Om3_rps = u(3)/60*2*pi; % convert rpm to rps
Om4_rps = u(4)/60*2*pi; % convert rpm to rps

% Set motor accels to zero (want steady state)
dOm1_rps2 = 0;
dOm2_rps2 = 0; 
dOm3_rps2 = 0;
dOm4_rps2 = 0; 

AnalDerivFlag = 0; % Don't compute analytic derivatives
[X_lbf,Y_lbf,Z_lbf,L_ftlbf,M_ftlbf,N_ftlbf, ~, ~] = ...
        IMPACT_AeroProp_RSE_V1_10Jun25_180239( ... 
        u_fps, v_fps, w_fps, p_rps, q_rps, r_rps, ... 
        Om1_rps, Om2_rps, Om3_rps, Om4_rps, ... 
        dOm1_rps2, dOm2_rps2, dOm3_rps2, dOm4_rps2, AnalDerivFlag);

%% Rigid body aircraft equations of motion

% Rotational Kinematics
phi_dot=p_rps+tan(theta_rad)*(q_rps*sin(phi_rad)+r_rps*cos(phi_rad));
theta_dot=q_rps*cos(phi_rad)-r_rps*sin(phi_rad);
psi_dot=sec(theta_rad)*(q_rps*sin(phi_rad)+r_rps*cos(phi_rad));

% Translational Dynamics
u_dot=-q_rps*w_fps + r_rps*v_fps - g_fps2*sin(theta_rad) + X_lbf/m;
v_dot=-r_rps*u_fps + p_rps*w_fps + g_fps2*cos(theta_rad)*sin(phi_rad) + Y_lbf/m;
w_dot=-p_rps*v_fps + q_rps*u_fps + g_fps2*cos(theta_rad)*cos(phi_rad) + Z_lbf/m;

% Rotational Dynamics (propulsion angular momentum neglected)
%    (Note: Propulsor gyroscopic effects were deemed to be negligible.
%     Propulsor transient torque effects are modeled in the IMPACT
%     polynomial aero-propulsive model.)
p_dot = -(Iz*L_ftlbf + Ixz*N_ftlbf - Ixz^2*q_rps*r_rps - Iz^2*q_rps*r_rps + Ix*Ixz*p_rps*q_rps - Ixz*Iy*p_rps*q_rps + Ixz*Iz*p_rps*q_rps + Iy*Iz*q_rps*r_rps)/(Ixz^2 - Ix*Iz);
q_dot = (M_ftlbf - Ixz*p_rps^2 + Ixz*r_rps^2 - Ix*p_rps*r_rps + Iz*p_rps*r_rps)/Iy;
r_dot = -(Ixz*L_ftlbf + Ix*N_ftlbf + Ix^2*p_rps*q_rps + Ixz^2*p_rps*q_rps - Ix*Iy*p_rps*q_rps - Ix*Ixz*q_rps*r_rps + Ixz*Iy*q_rps*r_rps - Ixz*Iz*q_rps*r_rps)/(Ixz^2 - Ix*Iz);

% Rotation matrix from the body frame to the inertial frame
R_IB=[ cos(psi_rad)*cos(theta_rad), cos(psi_rad)*sin(phi_rad)*sin(theta_rad) - cos(phi_rad)*sin(psi_rad), sin(phi_rad)*sin(psi_rad) + cos(phi_rad)*cos(psi_rad)*sin(theta_rad);
       cos(theta_rad)*sin(psi_rad), cos(phi_rad)*cos(psi_rad) + sin(phi_rad)*sin(psi_rad)*sin(theta_rad), cos(phi_rad)*sin(psi_rad)*sin(theta_rad) - cos(psi_rad)*sin(phi_rad);
               -sin(theta_rad),                              cos(theta_rad)*sin(phi_rad),                              cos(phi_rad)*cos(theta_rad)];

% Translational Kinematics
vt=R_IB*[u_fps;v_fps;w_fps];
x_dot=vt(1);
y_dot=vt(2);
z_dot=vt(3);


%% Define state derivatives

% Euler angles rate [rad/s]
xd(1,1)=phi_dot;
xd(2,1)=theta_dot;
xd(3,1)=psi_dot;

% Rate of change of body-axis velocity [ft/s^2]
xd(4,1)=u_dot;
xd(5,1)=v_dot;
xd(6,1)=w_dot;

% Rate of change of body-axis angular rates [rad/s^2]
xd(7,1)=p_dot;
xd(8,1)=q_dot;
xd(9,1)=r_dot;

% Position state derivatives [ft/s]
xd(10,1)=x_dot;
xd(11,1)=y_dot;
xd(12,1)=z_dot;
xd_sub = xd(1:9);

eq = xd(1:9);
return