% Script used to generate an executable for the model specified in the 
% 'model_name' workspace variable.   
%
% AUTHORS:
%   Thomas C. Britton
%     Science and Technology Corporation (STC), RSES Contract,
%     NASA Langley Research Center
% 
% Revisions:
%   tbritton 20250320 - Initial version

% save original value for dataOutType variant
origDataOutType = SimIn.variants.dataOutType;

% set data output type for code generation
SimIn.variants.dataOutType = DataOutEnum.CODEGEN;

% get just the filename with no extension, requires that model_name to be
% defined in the workspace and point to the Simulink model file to perform
% code generation on
[~, modelFile, ~] = fileparts(model_name);

% build an exe, using C as target language
buildDiagram = sprintf('%s_app', modelFile);
build(buildDiagram, modelFile, true);

% reset dataOutType to the original value
SimIn.variants.dataOutType = origDataOutType;

clear origDataOutType buildDiagram modelFile;
