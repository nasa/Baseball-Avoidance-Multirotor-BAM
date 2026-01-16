% function FCS = initFCS(SimIn,Setup_Type)
% This script initializes a series of parameters needed for the flight control 
% system for the BAM simulation.  In particular, this script establishes
% vehicle and motor configuration parameters necessary for use in the
% control system. 

% 
% Created by:
% K. Ackerman <kasey.a.ackerman@nasa.gov>
% NASA LaRC - Dynamic Systems and Control Branch
% 26 March 2018
%
% 22 JAN 2025, modified for use with BAM simulation (instead of Impact 
% simulation).  Modification includes using SimIn.Units which was previously defined using 
%   setUnits('m','kg'), modified by Michael J. Acheson
% *************************************************************************

function FCS = initFCS(SimIn)
FCS.t_control = 1/200;       % Controller rate 200Hz [s]
FCS.coriolisCorrection = 1;  % Set to 1 to cancel omega x (J*omega) term in control law, 0 if not

% *************************************************************************
% Nominal airframe data (without disturbance/uncertainties)
FCS.CM_Nom = SimIn.MP.CM_Nom;      % Location of center of gravity w.r.t. geometric center (in Body axes) [m]
FCS.mass_nominal = SimIn.MP.mass;   % Complete airframe mass [kg]
FCS.J_nominal = SimIn.MP.I; % Moment of inertia matrix
FCS.k_torque = 3/200;               % Factor how thrust and induced moment are related (from motor test data)
FCS.thrustReserve = 0.07;          % Limit total commanded thrust to 10...90% of max thrust to still allow maneuverability
FCS.prop.ParamsNom = SimIn.prop.ParamsNom;
FCS.nAct = size(FCS.prop.ParamsNom,1);  
% *************************************************************************

% *************************************************************************
%             Compute Thrust Min/Max, offset and gain factor                           %
% *************************************************************************
FCS.maxThrust = 0;
FCS.minThrust = 0;
FCS.maxThrust = abs(sum(FCS.prop.ParamsNom(:,7).*FCS.prop.ParamsNom(:,12)+FCS.prop.ParamsNom(:,8).*FCS.prop.ParamsNom(:,12).^2));
FCS.minThrust = abs(sum(FCS.prop.ParamsNom(:,7).*FCS.prop.ParamsNom(:,11)+FCS.prop.ParamsNom(:,8).*FCS.prop.ParamsNom(:,11).^2));

% 
% Set the max min thrust either to the vehicle performance limits or
% tighten the limits to allow for thrust reserve for maneuvring on top of
% lift thrust generation
% FCS.UppThrustLimit = FCS.maxThrust; % Limit to maximum allowable total thrust
% FCS.LwrThrustLimit = FCS.minThrust; % Limit to minimum allowable total thrust
% 
% Decrease total max thrust by reserve % times difference in max and min total thrusts
FCS.UppThrustLimit = FCS.maxThrust - (FCS.maxThrust-FCS.minThrust)*FCS.thrustReserve; 
% Increase total min thrust by reserve % times difference in max and min total thrusts
FCS.LwrThrustLimit = FCS.minThrust + (FCS.maxThrust-FCS.minThrust)*FCS.thrustReserve; 

% *************************************************************************
%               Define the motor control effectiveness matrix                      %
% *************************************************************************
% The controls effectiveness matrix maps desired torques to motor thrusts
for i = 1:size(FCS.prop.ParamsNom,1)
    % Find vector from nominal center of mass to nominal prop location
    FCS.prop_vector(i,1:3) = [abs(FCS.prop.ParamsNom(i,2)) * [cos(FCS.prop.ParamsNom(i,1)*SimIn.Units.deg);sin(FCS.prop.ParamsNom(i,1)*SimIn.Units.deg)];FCS.prop.ParamsNom(i,3)] - FCS.CM_Nom;
    % The nominal direction in which each motor generates thrust force
    FCS.thrust_direction(i,1:3) = -[cos(FCS.prop.ParamsNom(i,13)*SimIn.Units.deg)*sin(FCS.prop.ParamsNom(i,14)*SimIn.Units.deg)*cos(FCS.prop.ParamsNom(i,15)*SimIn.Units.deg)+sin(FCS.prop.ParamsNom(i,13)*SimIn.Units.deg)*sin(FCS.prop.ParamsNom(i,15)*SimIn.Units.deg); ...
                                    cos(FCS.prop.ParamsNom(i,13)*SimIn.Units.deg)*sin(FCS.prop.ParamsNom(i,14)*SimIn.Units.deg)*sin(FCS.prop.ParamsNom(i,15)*SimIn.Units.deg)-sin(FCS.prop.ParamsNom(i,13)*SimIn.Units.deg)*cos(FCS.prop.ParamsNom(i,15)*SimIn.Units.deg); ...
                                    cos(FCS.prop.ParamsNom(i,13)*SimIn.Units.deg)*cos(FCS.prop.ParamsNom(i,14)*SimIn.Units.deg)];
    % The control effectiveness matrix (B) converts individual motor thrust 
    % to combined body torques and thrust.One column for each motor and 
    % row elements are: roll/pitch/yaw torques and thrust    
	FCS.B(1:3,i) = cross(FCS.prop_vector(i,:)',FCS.thrust_direction(i,:)') + FCS.k_torque*FCS.thrust_direction(i,:)'*FCS.prop.ParamsNom(i,4);
    FCS.B(4,i) = 1.0;    
end
% Compute the the Moore-Penrose pseudoinverse 
% of the controls effectiveness matrix B
FCS.PseudoInv = FCS.B'/(FCS.B*FCS.B');

% *********** Define Geometric Controller Gains ***************************
% Vertical axis vector
FCS.e3                  = [0;0;1];
% Gains for attitude control law
FCS.Cont.Inner.Kp_1     = [7; 7; 7]; %15*[1;1;0.75];
FCS.Cont.Inner.Ki_1     = [0.8; 0.8; 0.8];% ones(3,1); %10*[1;1;0.75];
FCS.Cont.Inner.Kff_1    = [0.1; 0.1; 0.1];%0.7*[1;1;1]; %0.95->1
FCS.maxRateCmd          = [360;360;120] * SimIn.Units.deg; % 3x1 vector of maximum body rates

% Gains for path following control law
FCS.Cont.Outer.KL_1 = 1;
FCS.Cont.Outer.KX_1 = 2.75;
FCS.Cont.Outer.KV_1 = 2.75;
FCS.Cont.Outer.KR_1 = 1.25;
FCS.tan_angle_max   = tand(45); % maximum angle for b3d vector from vertical