function param = setupParameters(SimIn)
%% Define parameters
% Define parameters below that need to change without recompiling.  For any 
% desired value that you want to be "tunable" before or during simulation, 
% you will need to replace "SimIn" with "SimPar" in the Simulink diagram.
% 

% Define tunable simulation executive parameters 
SimPar.model_params.stop_time = SimIn.model_params.stop_time;
SimPar.model_params.sim_rate  = SimIn.model_params.sim_rate;
SimPar.model_params.rt_pace   = SimIn.model_params.rt_pace;

%% Define Model Properties 
SimPar.vehicle.propellers.ParamsNom = SimIn.vehicle.propellers.ParamsNom;
SimPar.vehicle.mass.CM_Nom = SimIn.vehicle.mass.CM_Nom;
SimPar.vehicle.mass.mass = SimIn.vehicle.mass.mass; 
SimPar.vehicle.mass.I = SimIn.vehicle.mass.I;

FCS = SimIn.FCS;
SimPar.FCS.prop.ParamsNom   = FCS.prop.ParamsNom;
SimPar.FCS.CM_Nom           = FCS.CM_Nom;
SimPar.FCS.mass_nominal     = FCS.mass_nominal;
SimPar.FCS.J_nominal        = FCS.J_nominal;
SimPar.FCS.Cont             = SimIn.FCS.Cont;

% Below is no longer needed. Used with OG aero-propulsive model
SimPar.vehicle.aero.Surface_params =        SimIn.vehicle.aero.Surface_params;
SimPar.vehicle.aero.C_D =                   SimIn.vehicle.aero.C_D;
SimPar.vehicle.constants =                  SimIn.vehicle.constants;

%% Define IC Parameters
SimPar.IC           = SimIn.IC;

%% Define Sensor Parameters
% SimPar.sensor = SimIn.sensor; % Map all sensor noise parameters

%% Define EOM IC Parameters
SimPar.EOM = SimIn.EOM;


%% Define Turbulence 
SimPar.Environment.Turbulence.RandomSeedLong    = SimIn.Environment.Turbulence.RandomSeedLong;
SimPar.Environment.Turbulence.RandomSeedLat     = SimIn.Environment.Turbulence.RandomSeedLat;
SimPar.Environment.Turbulence.RandomSeedVert    = SimIn.Environment.Turbulence.RandomSeedVert;
SimPar.Environment.Turbulence.intensity         = SimIn.Environment.Turbulence.intensity; % Intensity 1= none, 2=light, 3=moderate, 4=severe
SimPar.Environment.Turbulence.dT                = SimIn.Environment.Turbulence.dT    ;
% SimPar.Environment.Turbulence.Vehicle_span_m        = SimIn.Environment.Turbulence.Vehicle_span_m;

%% Define Winds Parameters
SimPar.Environment.Turbulence.WindAt5kft            = SimIn.Environment.Turbulence.WindAt5kft;
SimPar.Environment.Turbulence.WindDirectionAt5kft   = SimIn.Environment.Turbulence.WindDirectionAt5kft;

%% Define the Bezier Curve
% Non-vectorized form of bezier curves (no longer used)
% SimPar.RefInputs.Bezier.waypointsX = SimIn.RefInputs.Bezier.waypointsX;
% SimPar.RefInputs.Bezier.waypointsY = SimIn.RefInputs.Bezier.waypointsY;
% SimPar.RefInputs.Bezier.waypointsZ = SimIn.RefInputs.Bezier.waypointsZ;
% 
% SimPar.RefInputs.Bezier.time_wptsX = SimIn.RefInputs.Bezier.time_wptsX;
% SimPar.RefInputs.Bezier.time_wptsY = SimIn.RefInputs.Bezier.time_wptsY;
% SimPar.RefInputs.Bezier.time_wptsZ = SimIn.RefInputs.Bezier.time_wptsZ;

% *********** Vectorized form of Bezier curves ****************************
SimPar.RefInputs.trajectory.wptVect     = SimIn.RefInputs.trajectory.wptVect;
SimPar.RefInputs.trajectory.timeVect    = SimIn.RefInputs.trajectory.timeVect;
SimPar.RefInputs.trajectory.parseVect   = SimIn.RefInputs.trajectory.parseVect'; % force to be column vector for use by writePfile.m

if isfield(SimIn.RefInputs,'trajectoryBball')
    SimPar.RefInputs.trajectoryBball.wptVect     = SimIn.RefInputs.trajectoryBball.wptVect;
    SimPar.RefInputs.trajectoryBball.timeVect    = SimIn.RefInputs.trajectoryBball.timeVect;
    SimPar.RefInputs.trajectoryBball.parseVect   = SimIn.RefInputs.trajectoryBball.parseVect'; % force to be column vector for use by writePfile.m
end


%% ************************************************************************
props.Description = 'Impact Quadrotor Tunable Parameters';
props.DataScope   = 'Exported';
props.HeaderFile  = 'Parameters.h';
props.Alignment   = -1;% Not used
% % props.Elements will be defined in buildBusObject.m.
options.names = false;
b = buildBusObject(SimPar, 'BUS_PARAM_', 'SimPar', props, options);
assignin('base',['BUS_PARAM_','SimPar'],b);% % Rename the bus object with prefix

%% Simulink Parameter Object
param = Simulink.Parameter;
param.Value = SimPar;
param.CoderInfo.StorageClass = 'ExportedGlobal';
param.CoderInfo.Alignment = -1;% Not used
param.Description = 'Impact Quadrotor Tunable Parameters';
param.DataType =  'Bus: BUS_PARAM_SimPar';
param.Min = [];
param.Max = [];
param.DocUnits = '';

% assignin('base','SimPar', param);

end
