function SimIn = IMPACT_props2(SimIn)
% IMPACT_props - constants for the IMPACT aircraft simulation
%
% DESCRIPTION: 
%   This script contains constants used throughout the IMPACT modeling
%   programs and simulation.
% 
% INPUTS:
%   None
%
% OUTPUTS:
%   c - structure containing vehicle constants
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
%   22 JAN 2025 - modified to use units structure provided, MJA
%   26 JUN 2025 - updated parameters based on updated aero-prop model, 
%                 added simple thrust and aero drag model parameters, MJA
%

% IMPACT vehicle propeller numbering convention (top view):
%
%  ->     <-
%   3     1      ^ x
%   <-   ->      |
%     \ /        |
%      X         O-----> y
%     / \
%   <-   ->
%   2     4
%  ->     <-

% inertial properties
c.mass_kg = 1.9593; % Updated 6/26/25, Original value 1.9092;
c.Ixx_kgm2 = 0.029262; % Updated 6/26/25,Original value 0.029693;
c.Iyy_kgm2 = 0.031578 ;% Updated 6/26/25,Original value 0.032027
c.Izz_kgm2 = 0.047887;% Updated 6/26/25,Original value 0.048778
c.Ixz_kgm2 = 0;% Updated 6/26/25,Original value 0

% Convert to slug units for use in trim: nldyn_sim_eqn.m (since aero-prop
% model is in English units)
c.mass_slug = c.mass_kg/SimIn.Units.slug; % aircraft mass in slugs;
c.Ixx_slugft2 = c.Ixx_kgm2/SimIn.Units.slug/SimIn.Units.ft2;% roll moment of inertia, slug.ft^2
c.Iyy_slugft2 = c.Iyy_kgm2/SimIn.Units.slug/SimIn.Units.ft2;% pitch moment of inertia, slug.ft^2
c.Izz_slugft2 = c.Izz_kgm2/SimIn.Units.slug/SimIn.Units.ft2;% yaw moment of inertia, slug.ft^2
c.Ixz_slugft2 = c.Ixz_kgm2/SimIn.Units.slug/SimIn.Units.ft2;% product of inertia, slug.ft^2

c.J_kgm2 = [c.Ixx_kgm2 0 c.Ixz_kgm2;0 c.Iyy_kgm2 0; c.Ixz_kgm2 0 c.Izz_kgm2];

c.J_slugft2 = c.J_kgm2/SimIn.Units.slug/SimIn.Units.ft2;

c.CM_Nom   = [0;0;0.001]; % (m) Define center of gravity 

% number of propellers
c.n_p=4;

% propeller diameter
c.prop_diam_ft = 10/12;
c.prop_diam_m = c.prop_diam_ft/SimIn.Units.m;

% propeller locations
c.L_m = 0.25;% arm length [m]
c.L_ft = c.L_m/SimIn.Units.ft; %* c.m2ft;% arm length [ft]
c.prop_z_offset_m = 0;% z distance between propeller and CG [m]
c.prop_z_offset_ft = c.prop_z_offset_m/SimIn.Units.ft; %* c.m2ft;% z distance between propeller and CG [ft]
c.prop_locations_ft=[+c.L_ft*cosd(45),+c.L_ft*sind(45), c.prop_z_offset_ft  %M1
                     -c.L_ft*cosd(45),-c.L_ft*sind(45), c.prop_z_offset_ft  %M2
                     +c.L_ft*cosd(45),-c.L_ft*sind(45), c.prop_z_offset_ft  %M3
                     -c.L_ft*cosd(45),+c.L_ft*sind(45), c.prop_z_offset_ft];%M4

% Propeller rotation direction (CW=+1, CCW=-1, as viewed from motors perspective)
%             1   2   3   4        
c.prop_dir = [+1, +1, -1, -1]'; % change to column vector due to issues in Simulink.Bus.createMATLABStruct

% motor time constant
c.tau_m = 0.01;

% *************************************************************************
% This model is used to generate the trim tables using the "older"
% aero-propulsive modeling.  It is not used in the simulation
c.CT0 = 0.094 * ones(c.n_p,1);
c.CQ0 = 0.0053 * ones(c.n_p,1);
SimIn.c = c;

% The below "motor/prop information" matrix format and drag parameters
% incorporated into the BAM Simulation are a derivative of work by Jan
% Vervoost from a Master's thesis "A Modular Simulation Environment for the
% Improved Dynamic Simulation of Multirotor Unmanned Aerial Vehicles" at
% University of Illinois at Urbana-Champaign (2016). A MATLAB file exchange
% download is available at:
% https://www.mathworks.com/matlabcentral/fileexchange/59705-simulation-environment-for-multirotor-uavs
%

% Parameters for aerodynamic drag computation
SimIn.aero.a = 0.060/SimIn.Units.ft2;                      % Surface area of airframe visible along the Body x axis [m^2] convert to ft^2
SimIn.aero.b = 0.060/SimIn.Units.ft2;                      % Surface area of airframe visible along the Body y axis [m^2]
SimIn.aero.c = 0.060/SimIn.Units.ft2;                      % Surface area of airframe visible along the Body z axis [m^2]
SimIn.aero.C_D = 0.40;                     % Drag coefficient [dimensionless]
SimIn.aero.Surface_params = [SimIn.aero.a;SimIn.aero.b;SimIn.aero.c]./SimIn.Units.ft2;       % Surface area parameters

% Set up relevant mass properties
SimIn.MP.mass       = c.mass_kg;      % Complete airframe mass [kg], Nomimal or (real) mass used in EOM dynamics)
SimIn.MP.I          = c.J_kgm2;
SimIn.MP.CM_Nom     = c.CM_Nom;         % Location of center of mass w.r.t. geometric center (in Body axes) [m]


% ************************************************************************ 
% prop.ParamsNom contains the motor/prop information.  Each row describes
% one motor and prop combination
% 1      : Motor arm angle measured clockwise (looking from above) from the positive X axis (forward direction) [deg]
% 2      : Distance of prop/motor in X/Y plane from the geometric center of the airframe [m]
% 3      : Distance of prop/motor in Z direction from the geometric center of the airframe [m]
% 4      : Direction of prop rotation: -1 for CW, +1 for CCW [unitless]
% 5      : Control effectiveness of the actuator (nominally 1.0)
% 6      : First-order motor transfer function time constant [sec]
% 7..8   : Quadratic polynomial coefficients [a1 a2] that relate RPM to thrust
%          Thrust = a1 * RPM + a2 * RPM^2
% 9..10  : Quadratic polynomial coefficients [b1 b2] that relate RPM to torque
%          Torque = b1 * RPM + b2 * RPM^2
% 11     : Minimum actuator RPM
% 12     : Maximum actuator RPM 
% 13..15 : Euler angles (deg) for rotation of thrust vector to body-fixed axes
%          Nominal direction of thrust vector is [0;0;-1]
% 16     : Propeller diameter [m]
% 17     : Propeller mass [kg]

% *************************************************************************
% Pol_AeroProp Model: Use the following SimIn.prop.ParamsNom when using the
% polynomial aero propulsive model (comment out whichever model parameters
% not in use as the aeropropulsive model)
SimIn.prop.ParamsNom = [045 , c.L_m , -0.028 , -1 , 1.0 , c.tau_m , [0 ,-1.8248e-06]./SimIn.Units.lbf, [0 , 0] , 0 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0; ...
                        225 , c.L_m , -0.028 , -1 , 1.0 , c.tau_m , [0 ,-1.5221e-06]./SimIn.Units.lbf , [0 , 0] , 0 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0; ...
                        315 , c.L_m , -0.028 , +1 , 1.0 , c.tau_m , [0 ,-1.8699e-06]./SimIn.Units.lbf , [0 , 0] , 0 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0; ...
                        135 , c.L_m , -0.028 , +1 , 1.0 , c.tau_m , [0 ,-1.6581e-06]./SimIn.Units.lbf , [0 , 0] , 0 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0];

% *************************************************************************
% Nom_AeroProp Model: Use the following SimIn.prop.ParamsNom when using the
% nominal aero propulsive model (comment out whichever model parameters
% not in use for the aeropropulsive model)
% SimIn.prop.ParamsNom = [045 , c.L_m , -0.028 , -1 , 1.0 , c.tau_m , [-5.4243e-05 , 1.4141e-07] , [7.0500e-07 , 1.8220e-09] , 3000 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0; ...
%                                225 , c.L_m , -0.028 , -1 , 1.0 , c.tau_m , [-5.4243e-05 , 1.4141e-07] , [7.0500e-07 , 1.8220e-09] , 3000 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0; ...
%                                315 , c.L_m , -0.028 , +1 , 1.0 , c.tau_m , [-5.4243e-05 , 1.4141e-07] , [7.0500e-07 , 1.8220e-09] , 3000 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0; ...
%                                135 , c.L_m , -0.028 , +1 , 1.0 , c.tau_m , [-5.4243e-05 , 1.4141e-07] , [7.0500e-07 , 1.8220e-09] , 3000 , 10000 , [0.0 , 0.0 , 0.0] , c.prop_diam_m , 0];

return
