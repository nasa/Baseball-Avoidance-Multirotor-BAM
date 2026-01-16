function [X_lbf, Y_lbf, Z_lbf] = quad_drag(velT_bib, aero_params, rho)

% function [X_lbf, Y_lbf, Z_lbf] = quad_drag(velT_bib, aero_params, rho)
% This file computes a model of aerodynamic drag. The drag forces are
% computed using the methodology described in the Master's thesis:
% "A Modular Simulation Environment for the Improved Dynamic Simulation
%  of Multirotor Unmanned Aerial Vehicles," written by Jan Willem Vervoorst,
% University of Illinois at Urbana-Champain, 2016
%
% Inputs
%   velT_bib: (TAS) velocity of the multirotor cg wrt inertial frame (non-spinning
%       Earth) expressed in body frame coordinates
%   x: states 
%       = [phi,theta,psi,... %(rad)
%          u,v,w,...         %(ft/s)
%          p,q,r,...         %(rad/s)
%          x,y,z]            %(ft)
%   aero_params: structure of quadrotor surface area and drag parameters
%   rho:     atmosphere density (in units slug/ft3)
%
% Written by Michael J. Acheson, Dynamics and Control Branch (D-316)
% NASA Langley Research Center
%
% History:
% 1/31/2025, MJA Initial Version

% Non-linear (rigid body) states and controls
%   x: states 
%       = [phi,theta,psi,... %(rad)
%          u,v,w,...         %(ft/s)
%          p,q,r,...         %(rad/s)
%          x,y,z]            %(ft)
%   u: control inputs vector (rad/sec)

% Compute "exposed model surface area" using ellipsoidal model
scale_fac = 1/sqrt(sum((velT_bib./aero_params.Surface_params).^2));
MR_Surf_Area = vecnorm(velT_bib*scale_fac);

if all(velT_bib==0)
    Drag = [0;0;0];
else
    % Compute drag force using simplified model: Drag = 1/2*rho*V^2*Area
    Drag = -velT_bib/vecnorm(velT_bib)*rho*(velT_bib'*velT_bib)*aero_params.C_D*MR_Surf_Area/2;
end
% Map drag to function outputs
X_lbf = Drag(1);
Y_lbf = Drag(2);
Z_lbf = Drag(3);

end