% This script is used to develop the rpm to thrust curve fit

rpm_array = [0:250:7000];

SimIn = struct();
SimIn.Units = setUnits('m','kg');
SimIn.Environment.Atmos = setupAtmosphere(SimIn);
c = IMPACT_props2(SimIn.Units);
SimIn = initMulti(SimIn,c);
rho_slugpft3 = SimIn.Environment.Atmos.rho0/SimIn.Units.slug*SimIn.Units.ft^3; %0.002378; 
vel_bWb         = [0;0;0];
omega_bWb       = [0;0;0];

cnt = 0;
Thrust_array = nan(length(rpm_array),4);
for rpm = rpm_array
    cnt = cnt + 1;
    n_rpm = [rpm;0;0;0];
    [F,M] = ...
        compute_multirotor_FM(vel_bWb, omega_bWb, n_rpm, rho_slugpft3, c, SimIn.Units.ft);
    Thrust_array(cnt,1) = F(3);
    n_rpm = [0;rpm;0;0];
    [F,M] = ...
        compute_multirotor_FM(vel_bWb, omega_bWb, n_rpm, rho_slugpft3, c, SimIn.Units.ft);
    Thrust_array(cnt,2) = F(3);
    n_rpm = [0;0;rpm;0];
    [F,M] = ...
        compute_multirotor_FM(vel_bWb, omega_bWb, n_rpm, rho_slugpft3, c, SimIn.Units.ft);
    Thrust_array(cnt,3) = F(3);
    n_rpm = [0;0;0;rpm];
    [F,M] = ...
        compute_multirotor_FM(vel_bWb, omega_bWb, n_rpm, rho_slugpft3, c, SimIn.Units.ft);
    Thrust_array(cnt,4) = F(3);
end

figure
plot(rpm_array, Thrust_array);
grid on;

% ************** Perform Least Squares Regression Analysis ****************
% Get the desired fit coefficients in revolutions/SECOND not MINUTE
RHS = [ ones(length(rpm_array),1) (rpm_array') (rpm_array').^2 ];
quad_coef = (RHS'*RHS)\(RHS'*Thrust_array);

% *************** Store the results ***************************************
save('./Trim/RPM2Thrust_poly.mat','quad_coef','-v7.3');