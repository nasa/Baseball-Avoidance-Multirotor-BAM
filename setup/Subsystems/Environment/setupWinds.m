function SimIn = setupWinds(SimIn,userStruct)
temp = userStruct.simulation_defaults.Environment.Turbulence;
%% Define Turbulence 
SimIn.Environment.Turbulence.RandomSeedLong     = temp.RandomSeedLong; % Known good seeds 3366+521
SimIn.Environment.Turbulence.RandomSeedLat      = temp.RandomSeedLat; % Known good seeds 23+521
SimIn.Environment.Turbulence.RandomSeedVert     = temp.RandomSeedVert; % Known good seeds 1369+521
SimIn.Environment.Turbulence.intensity          = temp.intensity; % Intensity 1= none, 2=light, 3=moderate, 4=severe
SimIn.Environment.Turbulence.dT                 = 2*userStruct.model_params.sim_rate; 
% SimIn.Environment.Turbulence.Vehicle_span_m    

%% Define Winds Parameters
SimIn.Environment.Turbulence.WindAt5kft             = temp.WindAt5kft; % Specify in knots;
SimIn.Environment.Turbulence.WindDirectionAt5kft    = temp.WindDirectionAt5kft; % specify in degrees from North
end