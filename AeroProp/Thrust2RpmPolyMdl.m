% function rpm = Thrust2RpmPolyMdl(DesThrust,propParamsEst)
% This function takes desired individual motor thrusts and a given
% polynomial (in this case quadratic) and solves for the rpm necessary to
% achieve the desired thrust.  This is performed for each motor which may
% have different polynomial coefficients (i.e. allows for different
% propeller combinations). The polynomial here is based on the
% Aero-propulsive model

% 
% Created by:
% Michael J. Acheson
% NASA Langley Research Center
% Dynamic Systems and Control Branch D-316
% 5 March 2025
%
% *************************************************************************
% Compute desired motor rpm(s) from thrust command(s)
% In this case, a second order approximation is used
% *************************************************************************

function rpm = Thrust2RpmPolyMdl(DesThrust,propParamsEst)

% Polynomial Coefficients
a     = propParamsEst(:,2);  % Quadratic polynomial coeffient
b     = propParamsEst(:,1);  % Linear polynomial Coefficient
c     = zeros(size(propParamsEst,1),1); % No offset provided in current model

% *************************************************************************
% Motor RPM and thrust are related by a quadratic function of the form
% thrust = a * RPM^2 + b * RPM + c
% Rearranging and solving for the zeros (using the quadratic formula 
% for the positive rpm case yields):
% 0 = a * RPM^2 + b * RPM + (c-thrust)
rpm = (-b + real((b.^2-4.*a.*(c-DesThrust)).^(1/2)) )/2./a;
% *************************************************************************
