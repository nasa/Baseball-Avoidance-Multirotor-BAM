function [X_lbf,Y_lbf,Z_lbf,L_ftlbf,M_ftlbf,N_ftlbf, dx, du] = ...
        IMPACT_AeroProp_RSE_V1_10Jun25_180239( ... 
        u_fps, v_fps, w_fps, p_rps, q_rps, r_rps, ... 
        Om1_rps, Om2_rps, Om3_rps, Om4_rps, ... 
        dOm1_rps2, dOm2_rps2, dOm3_rps2, dOm4_rps2, AnalyticDeriv)
% IMPACT_AeroProp_RSE_V1_10Jun25_180239 - IMPACT aero-propulsive model
% 
% DESCRIPTION:  
%   This function contains a response surface model describing the
%   aero-propulsive characteristics of the NASA Langley IMPACT multirotor
%   around its hover flight condition.  The function outputs the baseline
%   forces and moments for given state and control inputs.  Additionally,
%   the function can provide the analytic state and control derivative
%   matrices if the user sets the analytic derivative flag to true.
%
% INPUTS: 
%   u_fps,v_fps,w_fps - body-axis translational velocity components [ft/s]
%   p_rps,q_rps,r_rps - body-axis angular velocity components [rad/s]
%   Om1_rps,Om2_rps,Om3_rps,Om4_rps - rotor rotational speeds [rad/s]
%   dOm1_rps2,dOm2_rps2,dOm3_rps2,dOm4_rps2 - rotor rotational accelerations [rad/s^2]
%   AnalDeriv - logical flag to output analytic derivatives or not.  true => provide analytic derivatives
% 
% OUTPUTS:
%   X_lbf,Y_lbf,Z_lbf - body-axis applied forces [lbf]
%   L_ftlbf,M_ftlbf,N_ftlbf - body-axis applied moments [ft-lbf]
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
%   10 JUN 2025 - Model created for IMPACT Simulation, BMS
%   16 JUN 2025 - Model modified for BAM Simulation to include optional 
%                 analytic derivatives and changed rpm input to rpm^1 
%                 (not ^2) to align with control derivative output, MJA

arguments
    u_fps double; % Vehicle body frame u vel (ft/sec)
    v_fps double; % Vehicle body frame v vel (ft/sec)
    w_fps double; % Vehicle body frame w vel (ft/sec)
    p_rps double; % Vehicle body frame roll rate (rad/sec)
    q_rps double; % Vehicle body frame pitch rate (rad/sec)
    r_rps double; % Vehicle body frame yaw rate (rad/sec)
    Om1_rps double; % Rotor 1 speed (rad/sec)
    Om2_rps double; % Rotor 2 speed (rad/sec)
    Om3_rps double; % Rotor 3 speed (rad/sec)
    Om4_rps double; % Rotor 4 speed (rad/sec)
    dOm1_rps2  double; % Rotor 1 (d/dt)speed (rad/sec^2)
    dOm2_rps2 double; % Rotor 2 (d/dt)speed (rad/sec^2)
    dOm3_rps2 double; % Rotor 3 (d/dt)speed (rad/sec^2)
    dOm4_rps2 double; % Rotor 4 (d/dt)speed (rad/sec^2)
    AnalyticDeriv double = 0; % Flag to output analytic derivatives (true => output anal derivatives)
end

% center rotor speeds
Om1sq_r2ps2 = Om1_rps^2 - 640000; 
Om2sq_r2ps2 = Om2_rps^2 - 640000; 
Om3sq_r2ps2 = Om3_rps^2 - 640000; 
Om4sq_r2ps2 = Om4_rps^2 - 640000; 

% response surface equations

X_lbf = ... % R^2 = 86.16 %, NRMSE = 5.52 %
   +8.10302116822059e-03 *            1 + ... % Std. Error = 1.668e-03, Per. Error = 20.59
   -5.56632192839278e-02 *        u_fps + ... % Std. Error = 9.089e-04, Per. Error =  1.63
   +3.87511335238782e-02 *        q_rps + ... % Std. Error = 2.781e-03, Per. Error =  7.18
   -1.26914659054609e-06 *  Om1sq_r2ps2 + ... % Std. Error = 2.502e-08, Per. Error =  1.97
   +1.19027136730041e-06 *  Om2sq_r2ps2 + ... % Std. Error = 2.030e-08, Per. Error =  1.71
   -1.30158940657146e-06 *  Om3sq_r2ps2 + ... % Std. Error = 2.165e-08, Per. Error =  1.66
   +1.25718211602667e-06 *  Om4sq_r2ps2 ;     % Std. Error = 1.775e-08, Per. Error =  1.41

Y_lbf = ... % R^2 = 90.26 %, NRMSE = 4.26 %
   +9.68365224185690e-02 *            1 + ... % Std. Error = 1.521e-03, Per. Error =  1.57
   -6.28023724570568e-02 *        v_fps + ... % Std. Error = 7.822e-04, Per. Error =  1.25
   -2.82086191493782e-02 *        p_rps + ... % Std. Error = 2.441e-03, Per. Error =  8.65
   -1.31685034134974e-06 *  Om1sq_r2ps2 + ... % Std. Error = 2.230e-08, Per. Error =  1.69
   +1.30652672508409e-06 *  Om2sq_r2ps2 + ... % Std. Error = 1.868e-08, Per. Error =  1.43
   +1.39240415645199e-06 *  Om3sq_r2ps2 + ... % Std. Error = 1.839e-08, Per. Error =  1.32
   -1.39500917503523e-06 *  Om4sq_r2ps2 ;     % Std. Error = 1.544e-08, Per. Error =  1.11

Z_lbf = ... % R^2 = 99.13 %, NRMSE = 1.33 %
   -4.36745106668932e+00 *            1 + ... % Std. Error = 1.629e-03, Per. Error =  0.04
   -5.60556215824423e-02 *        w_fps + ... % Std. Error = 1.392e-03, Per. Error =  2.48
   -1.82479511069024e-06 *  Om1sq_r2ps2 + ... % Std. Error = 2.410e-08, Per. Error =  1.32
   -1.52209885631848e-06 *  Om2sq_r2ps2 + ... % Std. Error = 1.989e-08, Per. Error =  1.31
   -1.86992909458542e-06 *  Om3sq_r2ps2 + ... % Std. Error = 1.931e-08, Per. Error =  1.03
   -1.65811696316834e-06 *  Om4sq_r2ps2 ;     % Std. Error = 1.636e-08, Per. Error =  0.99

L_ftlbf = ... % R^2 = 93.66 %, NRMSE = 3.40 %
   +1.92915796404590e-02 *            1 + ... % Std. Error = 7.476e-04, Per. Error =  3.88
   -2.42503854398595e-02 *        v_fps + ... % Std. Error = 3.845e-04, Per. Error =  1.59
   -3.63123844135711e-02 *        p_rps + ... % Std. Error = 1.200e-03, Per. Error =  3.30
   -8.83850959583441e-07 *  Om1sq_r2ps2 + ... % Std. Error = 1.096e-08, Per. Error =  1.24
   +8.67901867868840e-07 *  Om2sq_r2ps2 + ... % Std. Error = 9.185e-09, Per. Error =  1.06
   +8.67901867868840e-07 *  Om3sq_r2ps2 + ... % Std. Error = 9.038e-09, Per. Error =  0.95
   -9.45906857270267e-07 *  Om4sq_r2ps2 ;     % Std. Error = 7.593e-09, Per. Error =  0.80

M_ftlbf = ... % R^2 = 93.21 %, NRMSE = 3.59 %
   +5.63069484179306e-02 *            1 + ... % Std. Error = 7.440e-04, Per. Error =  1.32
   +2.34685418652257e-02 *        u_fps + ... % Std. Error = 4.054e-04, Per. Error =  1.73
   -3.50057789690019e-02 *        q_rps + ... % Std. Error = 1.240e-03, Per. Error =  3.54
   +9.11525805423114e-07 *  Om1sq_r2ps2 + ... % Std. Error = 1.116e-08, Per. Error =  1.22
   -8.50309526807694e-07 *  Om2sq_r2ps2 + ... % Std. Error = 9.054e-09, Per. Error =  1.06
   +9.14928871287047e-07 *  Om3sq_r2ps2 + ... % Std. Error = 9.658e-09, Per. Error =  1.06
   -8.83240550717913e-07 *  Om4sq_r2ps2 ;     % Std. Error = 7.916e-09, Per. Error =  0.90

N_ftlbf = ... % R^2 = 99.36 %, NRMSE = 1.01 %
   +1.26875248435228e-02 *            1 + ... % Std. Error = 1.675e-04, Per. Error =  1.32
   -2.01877916018307e-03 *        v_fps + ... % Std. Error = 8.057e-05, Per. Error =  3.99
   -1.48375164425736e-02 *        r_rps + ... % Std. Error = 2.814e-04, Per. Error =  1.90
   +8.01879372449960e-08 *  Om1sq_r2ps2 + ... % Std. Error = 2.441e-09, Per. Error =  3.04
   +1.60763413998182e-07 *  Om2sq_r2ps2 + ... % Std. Error = 2.041e-09, Per. Error =  1.27
   -7.89331643756855e-08 *  Om3sq_r2ps2 + ... % Std. Error = 2.060e-09, Per. Error =  2.61
   -1.47425188164639e-07 *  Om4sq_r2ps2 + ... % Std. Error = 1.705e-09, Per. Error =  1.16
   +2.51454000273013e-05 *    dOm1_rps2 + ... % Std. Error = 4.101e-07, Per. Error =  1.63
   +2.61051571124931e-05 *    dOm2_rps2 + ... % Std. Error = 3.407e-07, Per. Error =  1.31
   -2.21621210729144e-05 *    dOm3_rps2 + ... % Std. Error = 3.636e-07, Per. Error =  1.64
   -2.63958764441350e-05 *    dOm4_rps2 ;     % Std. Error = 2.996e-07, Per. Error =  1.14

if ~AnalyticDeriv % Output the analytic derivatives if AnalDeriv flag is true
    dx = zeros(6,6);
    du = zeros(6,8);
else
    % State derivatives (forces)
    dx_du = -5.56632192839278e-02;
    dx_dq = +3.87511335238782e-02;
    dy_dv = -6.28023724570568e-02;
    dy_dp = -2.82086191493782e-02;
    dz_dw = -5.60556215824423e-02;
    % State derivatives (moments)
    dL_dv = -2.42503854398595e-02;
    dL_dp = -3.63123844135711e-02;
    dM_du = +2.34685418652257e-02;
    dM_dq = -3.50057789690019e-02;
    dN_dv = -2.01877916018307e-03;
    dN_dr = -1.48375164425736e-02;

    % Control derivatives (forces)
    dx_dr = 2*[-1.26914659054609e-06 +1.19027136730041e-06 -1.30158940657146e-06 +1.25718211602667e-06];
    dy_dr = 2*[-1.31685034134974e-06 +1.30652672508409e-06 +1.39240415645199e-06 -1.39500917503523e-06];
    dz_dr = 2*[-1.82479511069024e-06 -1.52209885631848e-06 -1.86992909458542e-06 -1.65811696316834e-06];
    % Control derivatives (moments)
    dL_drpm = 2*[-8.83850959583441e-07 +8.67901867868840e-07 +8.67901867868840e-07 -9.45906857270267e-07];
    dM_drpm = 2*[+9.11525805423114e-07 -8.50309526807694e-07 +9.14928871287047e-07 -8.83240550717913e-07];
    dN_drpm = 2*[+8.01879372449960e-08 +1.60763413998182e-07 -7.89331643756855e-08 -1.47425188164639e-07];
    dN_drpmdt = [+2.51454000273013e-05 +2.61051571124931e-05 -2.21621210729144e-05 -2.63958764441350e-05];

    % State derivatives
    dx = [dx_du 0 0 0 dx_dq 0; 0 dy_dv 0 dy_dp 0 0; 0 0 dz_dw 0 0 0;...
          0 dL_dv 0 dL_dp 0 0; dM_du 0 0 0 dM_dq 0; 0 dN_dv 0 0 dN_dr 0];
    du = [dx_dr zeros(1,4);dy_dr zeros(1,4); dz_dr zeros(1,4); ...
          dL_drpm zeros(1,4); dM_drpm zeros(1,4); dN_drpm dN_drpmdt];
end
