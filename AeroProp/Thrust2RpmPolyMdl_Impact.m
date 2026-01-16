% function rpm = Thrust2RpmPolyMdl_Impact(ThrustCmds, ParamsNom, DesMom, vel_bIb, omega)
% This function takes desired individual motor thrusts and the given
% polynomial aeropropulsive model (in this case quadratic) and then solves 
% for the rpm necessary to achieve the desired thrust.  This is performed 
% for each motor which may have different polynomial coefficients 
% (i.e. allows for different propeller combinations). The polynomial here 
% is based on the aero-propulsive model (located in the AeroProp folder): 
% IMPACT_AeroProp_RSE_V1_10Jun25_180239.m
% This function actually has two methods to take desired inputs to produce
% desired outputs.  This function can either take:
% 1) Three desired moments and total thrust or
% 2) Four desired thrusts (one for each motor)
% The first method uses the control allocation block to convert
% three moments + total thrust into four desired moments. The first method
% makes direct use of the rpm^2 term (and the constant offsets) and
% directly computes the desired rpms.
% The second method uses the moments directly (without the need for control
% allocation).  This method requires a matrix inverse and also incorporates
% the aero-propulsive model effects body velocities and omega.  This method
% is more accurate but requires a matrix inverse.  
%
% The first method (simple polynomial model using control allocation) is
% used as it represents the "limited" knowledge that is likely available
% with simplified modeling.  The second method is provided merely to
% demonstrate an alternate method directly using moments (without control
% allocation) but is not used.

% 
% Created by:
% Michael J. Acheson
% NASA Langley Research Center
% Dynamic Systems and Control Branch D-316
% 5 March 2025
% 
% 6/23/2025 MJA: update to original version to align with new
% aeropropulsive database (polynomial model fit from flight data)
%
% *************************************************************************
% Compute desired motor rpm(s) from thrust command(s)
% In this case, a second order approximation is used
% *************************************************************************

function rpm = Thrust2RpmPolyMdl_Impact(ThrustCmds, ParamsNom, DesComp, vel_bIb, omega)

% ***************** First method **********************************

% ***************** Polynomial Coefficients *******************************
% Z-force (lbf) model is:
% Z = -4.3675 -5.6056e-2*w_vel -1.8248e-6*(r1^2-6.4e5) -1.5221e-6*(r2^2-6.4e5)
% -1.8699e-6*(r3^2-6.4e5) -1.6581e-6*(r4^2-6.4e5)

r1 = sqrt(abs((-ThrustCmds(1)+4.36754/4)/ParamsNom(1,8)+6.4e5));
r2 = sqrt(abs((-ThrustCmds(2)+4.36754/4)/ParamsNom(2,8)+6.4e5));
r3 = sqrt(abs((-ThrustCmds(3)+4.36754/4)/ParamsNom(3,8)+6.4e5));
r4 = sqrt(abs((-ThrustCmds(4)+4.36754/4)/ParamsNom(4,8)+6.4e5));

rpm = [r1;r2;r3;r4]*60/2/pi; % Output as rpm not rad/sec

% If any rpm is < rpm min or greater than max, then set it respectively
rpm_min_ind = rpm<ParamsNom(:,11);
rpm_max_ind = rpm>ParamsNom(:,12);
rpm(rpm_min_ind) = ParamsNom(rpm_min_ind,11);
rpm(rpm_max_ind) = ParamsNom(rpm_max_ind,11);
disp('')
% ************ Second method ************************************
% Convert units
% zf = DesComp(4);
% Lm = DesComp(1);
% Mm = DesComp(2);
% Nm = DesComp(3);
% 
% u = vel_bIb(1);
% v = vel_bIb(2);
% w = vel_bIb(3);
% 
% p = omega(1);
% q = omega(2);
% r = omega(3);
% 
% LHS = [(zf-(-4.36745106668932e+00 -5.60556215824423e-02*w));...
%     (Lm -(+1.92915796404590e-02 -2.42503854398595e-02*v -3.63123844135711e-02*p));...
%     (Mm -(+5.63069484179306e-02 +2.34685418652257e-02*u -3.50057789690019e-02*q));...
%     (Nm -(+1.26875248435228e-02 -2.01877916018307e-03*v -1.48375164425736e-02*r))];
% 
% RHS = [-1.82479511069024e-06 -1.52209885631848e-06 -1.86992909458542e-06 -1.65811696316834e-06;...
%     -8.83850959583441e-07 +8.67901867868840e-07 +8.67901867868840e-07 -9.45906857270267e-07;...
%     +9.11525805423114e-07 -8.50309526807694e-07 +9.14928871287047e-07 -8.83240550717913e-07;...
%     +8.01879372449960e-08 +1.60763413998182e-07 -7.89331643756855e-08 -1.47425188164639e-07];
% 
% rot_sq = (RHS'*RHS)\(RHS'*LHS);
% 
% rot_radsec = (abs(rot_sq+6.4e5)).^(1/2);
% rpm= rot_radsec*60/2/pi;


