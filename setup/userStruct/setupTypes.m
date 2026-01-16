%% Setup Sensors
if ~isfield(userStruct.types,'sensors')
    userStruct.types.sensors =[];
end
if ~(isfield(userStruct.types.sensors,'useNoise'))
    userStruct.types.sensors.useNoise = 0;
end