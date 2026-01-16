function [ineq, eq] = Aero_Eval(x, u, g_fps2, mass_slugs)

%g_fps2 = -32.174048556430442; 
% Define units necessary for the non-linear EOMs
%g_fps2 = SimIn.Units.g0/SimIn.Units.ft;
%mass_slugs = 0.130821755373787; 

u_fps = x(1);
v_fps = x(2);
w_fps = x(3);
p_rps = x(4);
q_rps = x(5);
r_rps = x(6);
dOm1_rps2 = 0; % Want steady state so no rotor accel/decel
dOm2_rps2 = 0; % Want steady state so no rotor accel/decel
dOm3_rps2 = 0; % Want steady state so no rotor accel/decel
dOm4_rps2 = 0; % Want steady state so no rotor accel/decel
phi = u(1);
theta = u(2);
Om1_rps = u(3);
Om2_rps = u(4);
Om3_rps = u(5);
Om4_rps = u(6);

AnalDerivFlag = 0; % Don't compute analytic derivatives
[X_lbf,Y_lbf,Z_lbf,L_ftlbf,M_ftlbf,N_ftlbf, ~, ~] = ...
        IMPACT_AeroProp_RSE_V1_10Jun25_180239( ... 
        u_fps, v_fps, w_fps, p_rps, q_rps, r_rps, ... 
        Om1_rps, Om2_rps, Om3_rps, Om4_rps, ... 
        dOm1_rps2, dOm2_rps2, dOm3_rps2, dOm4_rps2, AnalDerivFlag);
rot_phi = [1 0 0; 0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
rot_theta = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
g_rot = rot_phi*rot_theta*[0;0;-g_fps2*mass_slugs];

eq = [X_lbf-g_rot(1),Y_lbf-g_rot(2),Z_lbf-g_rot(3),L_ftlbf,M_ftlbf,N_ftlbf]';
ineq = [];
disp('');