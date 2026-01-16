function [xd,accel] = nldyn_sim_eqn(param,u,x,c, g_fps2, rho_slugpft3, aero_p, units)
% nldyn_sim_eqn - nonlinear aircraft equations of motion
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
%   c       - structure of multi-rotor configuration parameters
%   g       - Earth's gravity constant (ft/sec^2)
%   rho     - atmospheric density (slug/ft3)
%   aero_p  - struture of drag force related multirotor properties
%
% OUTPUTS:
%   xd -  time derivative of the state vector
%       = [phi_dot,theta_dot,psi_dot,... %(rad/s)
%          u_dot,v_dot,w_dot,...         %(ft/s^2)
%          p_dot,q_dot,r_dot,...         %(rad/s^2)
%          x_dot,y_dot,z_dot]            %(ft/s)
%   accel - vector of translational/rotational accelerations
%       = [ax,ay,az,...     %(ft/s^2)
%          pdot,qdot,rdot]  %(rad/s^2)
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
%   1/31/2025 - Modified by MJA to incorporate quadrotor drag model.

% define constants
% g_fps2 = 32.174;% gravitational acceleration [ft/s^2]
% rho_slugpft3 = 0.00237717;% SSL air density [slug/ft^3]

xd      = zeros(size(x));% initialize state derivative vector
accel   = [0;0;0;0;0;0];% initialize translational/rotational acceleration vector

%% Define Simulation States

% Euler angles [rad]
phi_rad     = x(1,1);
theta_rad   = x(2,1);
psi_rad     = x(3,1);

% velocity components in the body frame [ft/s]
u_fps   = x(4,1);
v_fps   = x(5,1);
w_fps   = x(6,1);

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
 
% compute propeller forces and moments
[F_t, M_t] = compute_multirotor_FM([u_fps,v_fps,w_fps],[p_rps,q_rps,r_rps],u,rho_slugpft3,c, units.ft);

% Compute quadrotor drag forces (neglect any induces moments due to drag..)
[X_lbf_d,Y_lbf_d,Z_lbf_d] = quad_drag([u_fps;v_fps;w_fps ], aero_p, rho_slugpft3);

% Total propulsive and aerodynamic forces
X_lbf = F_t(1) + X_lbf_d;
Y_lbf = F_t(2) + Y_lbf_d;
Z_lbf = F_t(3) + Z_lbf_d;
L_ftlbf = M_t(1);
M_ftlbf = M_t(2);
N_ftlbf = M_t(3);

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

%% Define acceleration outputs

% Translational acceleration output [ft/s^2]
accel(1)=X_lbf/m;
accel(2)=Y_lbf/m;
accel(3)=Z_lbf/m;

% Angular acceleration output [rad/s^2]
accel(4)=xd(7,1);
accel(5)=xd(8,1);
accel(6)=xd(9,1);

return