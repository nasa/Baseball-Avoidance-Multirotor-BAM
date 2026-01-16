% Script used to generate an executable for the ROS2 dependent model file
% specified in the 'model_name' workspace variable.   
%
% AUTHORS:
%   Thomas C. Britton
%     Science and Technology Corporation (STC), RSES Contract,
%     NASA Langley Research Center
% 
% Revisions:
%   tbritton 20250519 - Initial version
%   tbritton 20250606 - Changed the rosPackages initialization to be an
%                       empty string array

% save original value for dataOutType variant
origDataOutType = SimIn.variants.dataOutType;

% set data output type for code generation
SimIn.variants.dataOutType = DataOutEnum.CODEGEN;

% get just the filename with no extension, requires that model_name to be
% defined in the workspace and point to the Simulink model file to perform
% code generation on
[~, modelFile, ~] = fileparts(model_name);

% if string array of user-specified ROS2 packages doesn't exist, create it
if ~exist('rosPackages','var')
  rosPackages = strings(0,1);
  endIdx = 0;
  % check known userStruct variants for which s-functions are enabled
  if (userStruct.variants.pubType == PubEnum.ROS2) || ...
     (userStruct.variants.pubTypeBball == PubEnum.ROS2)
    if ~any(contains(rosPackages, "rclcpp"))
      rosPackages(endIdx + 1,:) = "rclcpp";
      endIdx = endIdx + 1;
    end

    if ~any(contains(rosPackages, "geometry_msgs"))
      rosPackages(endIdx + 1,:) = "geometry_msgs";
      endIdx = endIdx + 1;
    end
  end

  if userStruct.variants.subType == SubEnum.PHASE_SPACE
    if ~any(contains(rosPackages, "rclcpp"))
      rosPackages(endIdx + 1,:) = "rclcpp";
      endIdx = endIdx + 1;
    end

    if ~any(contains(rosPackages, "std_msgs"))
      rosPackages(endIdx + 1,:) = "std_msgs";
      endIdx = endIdx + 1;
    end
  end

end

% build an exe, using C as target language
buildDiagram = sprintf('%s_app', modelFile);
buildWithRos(buildDiagram, modelFile, rosPackages, true);

% reset dataOutType to the original value
SimIn.variants.dataOutType = origDataOutType;

clear origDataOutType buildDiagram modelFile;
