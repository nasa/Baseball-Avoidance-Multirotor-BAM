function [b, numEl] = setupUserSimOutBus(userStruct)
% *************************************************************************
% This function is used to create a user defined bus from the down-selected
% elements of SimOut. The bus that is created is assigned to the 
% output block to facilitate execution of the model using the abbreviated output
% 
% INPUTS:
%   userStruct: The 'outputFunc' field contains the name of the user-specified
%               output function.
%
% OUTPUTS:
%   b:     the user simulation output bus (BUS_USER_SIM_OUT) definition
%   numEl: total number of data elements in the BUS_USER_SIM_OUT bus

% Created by Thomas C. Britton, thomas.c.britton@nasa.gov
% Science and Technology Corporation (STC), RSES Contract
% NASA Langley Research Center (LaRC), Dynamic Systems and Control Branch (D-316)
% 
% Modifications:
% 9.27.2023, TCB: Initial version
% 4.16.2025, TCB: Modified to support BAM simulation
% *************************************************************************

% Create function handle
outFuncHandle = str2func(userStruct.outputFunc);
localSimOut = Simulink.Bus.createMATLABStruct('BUS_SIM_OUT');
UserSimOut = outFuncHandle(localSimOut);

%%-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
%% Bus Objects
props.Description = 'User-defined SimOut';
props.DataScope   = 'Exported';
props.HeaderFile  = 'UserSimOut.h';
props.Alignment   = -1;% Not used
% props.Elements will be defined in buildBusObject.m.
options.names = false;
[b, numEl] = buildBusObject(UserSimOut, 'BUS_USER_', 'SIM_OUT', props, options);

%assignin('base',['BUS_USER_','SIM_OUT'],b);% % Rename the bus object with prefix

end