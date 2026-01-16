function [cost, eu_in, xd] = mycost(x_in, eu_in, c, g, rho, aero_p, units)
% This script is the cost function for trimming the multirotor.  In
% particular, this script seeks to find the Euler angles (pitch and yaw)
% and the propeller rpms which provide a trim solution.

% Written by Michael J. Acheson, Dynamics Systems and Controls Branch
% (D-316), NASA Langley Research Center

% Inputs:
%   x_in:   = [phi, theta, psi,... %(rad)
%               u,v,w,...         %(ft/s)
%               p,q,r,...         %(rad/s)
%               x,y,z]            %(ft)
%   eu_in   = [phi, theta,controls] % rotors rpm
%   c       =  structure of multi-rotor configuration parameters
%   g       = Earth's gravity constant (ft/sec^2)
%   rho     = atmospheric density (slug/ft3)
%   aero_p  = struture of drag force related multirotor properties
%
% Non-linear (rigid body) states and controls
%   x: states 
%       = [phi,theta,psi,... %(rad)
%          u,v,w,...         %(ft/s)
%          p,q,r,...         %(rad/s)
%          x,y,z]            %(ft)
%   u: control inputs vector (rad/sec)

% Compute the state derivatives (using impact aero computations)
param = [];
% Reconstruct and separate the state and control vectors.
x = [eu_in(1:2);x_in(1:end)]; 
u = eu_in(3:6);
[xd,~] = nldyn_sim_eqn(param,u,x,c, g, rho, aero_p, units); % Note accel in [xd, accel]= nldyn... not used here
% Determine the solution cost
cost = xd(1:9)'*xd(1:9);
end