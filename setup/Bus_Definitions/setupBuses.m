% setup bus objects
clear  BUS_ENVIRONMENT BUS_EOM BUS_FORCE_MOMENT BUS_PROP_ACTUATOR BUS_REF_INPUT BUS_MDL_C;

% define the bus objects
%ControlBus(true, SimIn.numEngines, SimIn.numSurfaces);
EnvironmentBus;
EomBus;
ForceMomentBus(true, 6);
%PropActuatorBus(true,4);
PropCmdBus(true,4);
RefInputBus;
RefInputInerBus;
SimOutBus;

% create user-specified SimOut bus
[BUS_USER_SIM_OUT, SimIn.lengthUserOut] = setupUserSimOutBus(userStruct);

%RefInputBus;
%PowerSystemInBus(true, SimIn.numEngines, SimIn.numSurfaces);
%PowerSystemOutBus(true, SimIn.numEngines, SimIn.numSurfaces);
%PropActuatorBus(true, SimIn.numEngines);
%SurfaceActuatorBus(true, SimIn.numSurfaces);
%SensorBus;
%TrimInputBus(true, SimIn.Trim.numEngines, SimIn.Trim.numSurfaces);
%FailureBusSurf(true,0, SimIn.numSurfaces); 
%FailureBusEng(true,SimIn.numEngines, 0); 

% must go after all other vehicle buses are defined
% VehicleOutBus;
% must go after all other buses defined
% SimOutBus;

% remove default return variable
clear ans;
