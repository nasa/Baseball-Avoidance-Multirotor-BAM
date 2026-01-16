function SimIn = setupIC(SimIn,userStruct)
% Initial conditions for vehicle include:
%  LatGeod, Long, GrndAltMSL, Euler, Omega_BIb, Vel_bIb, Pos_bii, Q_i2b, rpm_init
% setupInitialConditionsTable: uses trim table 
vel_bIi = userStruct.simulation_defaults.trim.vel_bIi;
acc_i   = userStruct.simulation_defaults.trim.acc_i;
psi     = userStruct.simulation_defaults.trim.psi;
pos     = userStruct.simulation_defaults.trim.pos_i;
psi_d   = userStruct.simulation_defaults.trim.psi_d;

% Original Aero-propulsive model 
% SimIn = setupInitialConditionsTable(SimIn, './Trim/T_map.mat', vel_bIi, acc_i, psi, pos,...
%     psi_d, SimIn.Units); % Note NED (inertial frame velocity provided)

% Polynomial Aero-propulsive model
SimIn = setupInitialConditionsTable(SimIn, './Trim/Poly_T_map.mat', vel_bIi, acc_i, psi, pos,...
    psi_d, SimIn.Units); % Note NED (inertial frame velocity provided)

end