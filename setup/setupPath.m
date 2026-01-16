% Add desired folders in the BAM repo to the current path

% save root dir off to ease directory related items further along in the
% process (mainly to get rid of several relative pathing items).
setenv("ActiveRootDir", pwd);

curPath = pwd;
path(genpath('AutoCodeModels'), path);  % Add in the code generation functions
addpath(fullfile(curPath,"AeroProp")); % Add in the Aero Propulsion folder
addpath(genpath(fullfile(curPath, "Bez_Functions"))); % Add in required Simulink Reference Models
addpath(genpath(fullfile(curPath, "Examples")));
addpath(genpath(fullfile(curPath, "lib"))); % Add in the library functions
addpath(genpath(fullfile(curPath, "Ref_Models"))); % Add in required Simulink Reference Models
addpath(genpath(fullfile(curPath, "Trim")));
addpath(genpath(fullfile(curPath, "Util"))); % Add in utilities folder
addpath(genpath(fullfile(curPath, "ChallengeProblem"))); % Add in collision avoidance challenge problem folder
addpath(genpath(fullfile(curPath, "BamEcho")));  % Postprocess Visualization Tool, has M script for generating data.
% add to path the folders for sfunction source files
addpath(genpath(fullfile(curPath,'ROS2_ws/s_function_pkgs/publishers')));
addpath(genpath(fullfile(curPath,'ROS2_ws/s_function_pkgs/subscribers')));

% Retrieve the ROS_WORKSPACE environment variable
ros_workspace = getenv('ROS_WORKSPACE');

% Look for mex files in ROS_WORKSPACE if it has been set
if ~isempty(ros_workspace)
    % Construct the build directory path
    install_dir = fullfile(ros_workspace, 'install', 'lib');
    
    % Check if the build directory exists
    if exist(install_dir, 'dir')
        % Add the build directory and all its subdirectories to the path
        addpath(genpath(install_dir));
        fprintf('Successfully added ROS_WORKSPACE build directory and all subdirectories to the path.\n');
    else
        warning('Build directory does not exist: %s', install_dir);
    end
else
  addpath(genpath(fullfile(curPath,'ROS2_ws/bin')));
end

clear curPath;