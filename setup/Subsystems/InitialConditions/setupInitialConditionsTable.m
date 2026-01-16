function SimIn = setupInitialConditionsTable(SimIn, Trim_fname, vel_bIi_in, acc_bIi_in, psi_des, pos, psi_d, units)

% This function is used to specify various initial conditions.  Additionally, 
% the user provides a desired inertial frame velocity (NED) and desired heading angle.
% Based on these user provided inertial frame velocities, iterations are performed to 
% determine the roll and pitch angles (and corresponding body frame velocities) which
% which trim the vehicle. The output of the interations are the
% corresponding pitch & roll Euler angles, as well as the trim 4 rotor
% propeller speeds.
% *************************************************************************

%% *********** Establish vehicle Initial Conditions ************************
SimIn.IC.LatGeod =  37.102769 * SimIn.Units.deg;    % Initial GPS latitude [rad]
SimIn.IC.Lon = -76.386649 * SimIn.Units.deg;        % Initial GPS longitude [rad]
IC_alt = pos(3); % altitude above MSL..

SimIn.IC.GrndAltMSL = IC_alt;

% Now compute quaternions which may be need to express quantities below
qe2i = QrotZ(0)';% inital ECEF to ECI transform (4x1), ASSUMES THE ROTATING EARTH IS THE INERTIAL FRAME
%qe2h = Qmult(QrotZ(SimIn.IC.Lon),QrotY(-0*pi/2 - SimIn.IC.LatGeod))';% Transform ECEF to NED
qe2h = [1;0;0;0]; % Qmult(QrotZ(SimIn.IC.Lon),QrotY(-0*pi/2 - SimIn.IC.LatGeod))';% Transform ECEF to NED


% ******* Iterate for the necessary phi and theta to achieve trim *********
% Temporarily load in the trim map table (based on body frame velocites) and interpolate it for given body frame velocity
ang_conv = 1e-4; % Angle convergence criteria in radians
tm = load(Trim_fname);
psi0    = psi_des;

phi0_cur = 0;
theta0_cur = 0;
conv_flag = 0;
while ~conv_flag
    qh2b = QmultSeq(QrotZ(psi0),QrotY(theta0_cur),QrotX(phi0_cur))';
    vel_bIb = Qtrans(qh2b,vel_bIi_in);
    phi0    = interp3(tm.u_vec, tm.v_vec, tm.w_vec, tm.phi, vel_bIb(1)/units.ft, vel_bIb(2)/units.ft, vel_bIb(3)/units.ft); % rad
    theta0  = interp3(tm.u_vec, tm.v_vec, tm.w_vec, tm.theta, vel_bIb(1)/units.ft, vel_bIb(2)/units.ft, vel_bIb(3)/units.ft); % rad
    phi_err = abs(phi0_cur-phi0);
    theta_err = abs(theta0_cur-theta0);
    if phi_err < ang_conv && theta_err < ang_conv
        conv_flag   = 1;
    else
        phi0_cur    = phi0;
        theta0_cur  = theta0;
    end
end

% Interpolate the trim table to get the rotor rpms for trim
p1_0    = interp3(tm.u_vec, tm.v_vec, tm.w_vec, tm.p1, vel_bIb(1)/units.ft, vel_bIb(2)/units.ft, vel_bIb(3)/units.ft); % rpms
p2_0    = interp3(tm.u_vec, tm.v_vec, tm.w_vec, tm.p2, vel_bIb(1)/units.ft, vel_bIb(2)/units.ft, vel_bIb(3)/units.ft); % rpms
p3_0    = interp3(tm.u_vec, tm.v_vec, tm.w_vec, tm.p3, vel_bIb(1)/units.ft, vel_bIb(2)/units.ft, vel_bIb(3)/units.ft); % rpms
p4_0    = interp3(tm.u_vec, tm.v_vec, tm.w_vec, tm.p4, vel_bIb(1)/units.ft, vel_bIb(2)/units.ft, vel_bIb(3)/units.ft); % rpms
% ******* End of Iterate for the necessary phi and theta to achieve trim *********

SimIn.IC.Euler321 = [phi0;theta0;psi0] * SimIn.Units.rad; % Initial airframe attitude in Euler angles Roll/Pitch/Yaw (provide in deg, converted to base unit rad)
qi2h = Qmult(Qinvert(qe2i),qe2h);    % Transform from Inertial to NED
qi2b = QmultSeq(Qinvert(qe2i),qe2h,qh2b);          % Transform from ECI to body
SimIn.EOM.Q_e2h = qe2h; % IC required for Topodetic position in World-Relative, EOM block
% Position
Pos_bee = llh2ecef(SimIn.IC.LatGeod, SimIn.IC.Lon, IC_alt, SimIn.Environment.Earth);% ECEF position
SimIn.EOM.Pos_bee = Pos_bee; % IC required for Toptodetic position in World-Relative, EOM block
%Pos_bei = Qtrans(qe2i,Pos_bee);

% *********** Prescribe ICs **********************************************
%SimIn.IC.Omega_BIb = [0;0;psi_d]; % Initial angular rates in Body frame (provide in rad/s)
SimIn.IC.Omega_BIb = [0;0;0] * SimIn.Units.deg; % Initial angular rates in Body frame (provide in deg/s, converted to base unit rad/s)
SimIn.IC.Vel_bIb = vel_bIb * SimIn.Units.m; % Initial velocity in Body frame for 6DOF block [m/s]
SimIn.EOM.Accel_bIi = acc_bIi_in; %[0;0;0]; % Initial acceleration of body wrt inertial frame, expressed in the inertial frame
SimIn.IC.Pos_bii = pos*SimIn.Units.m; % Initial position in Inertial frame for 6DOF block [m]
SimIn.IC.Q_i2b = qi2b;

% Optional ICs (only used if check box selected in EOM, Rigid Body Dynamics mask)
g_i = Qtrans(Qmult(Qinvert(qe2h),qe2i),[0;0;SimIn.Units.g0]);% NED gravity vector to ECI
Asensed_bIb = Qtrans(Qinvert(qi2b),SimIn.EOM.Accel_bIi - g_i);% initial sensed acceleration 
SimIn.EOM.Asensed_bIb = Asensed_bIb;
SimIn.EOM.OmegDtI_BIb = [0;0;0]; % Angular acceleration of body wrt inertial, expressed in inertial

% Prescribe initial rotor rpms (obtained from trim table lookup)
SimIn.IC.rpm_init = [p1_0; p2_0; p3_0; p4_0]; % Trim table gives in rpms