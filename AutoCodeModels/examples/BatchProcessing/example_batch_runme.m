%{
example_batch_runme.m

DESCRIPTION: This script demonstrates how to programmatically generate a
series of parameter sets and evaluate them using a compiled executable.

FUNCTION: After verifying the initial path, this code generates a series of
parameter files, each with a unique random reference trajectory for the
multirotor to follow. These parameter files are then executed in parallel
using a system call to a shell script that passes them to the BAM_app
executable. The results are then read back into matlab and the trajectories
are plotted.
    
DEPENDENCIES:
    example_batch_runme.m (this file) moved to the root directory
    BAM_app executable created by running buildExe.m
    aRunParallel.sh and BAM_app<.exe> moved to a '_data' directory under root

NOTES:
    The path verification process should catch any other path errors and
    provide instructions for correction.
    
    This code relies of system commands, creates directories, and runs an
    external shell script, these will likely have to be modified for your
    environment.
    

AUTHORS:
Andrew Patterson, NASA Langley Research Center
Tenavi Nakamura-Zimmerer, NASA Langley Research Center

HISTORY:
2025-03-28 - created, APP
2025-05-22 - modified for Mac, TNZ

    
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
%}

%% Clean Workspace
clearvars;
clc;

% Writing pFiles...done (Execution time: 6.665s)
% Running sim...done	(Execution time: 11.780s)
% Loading results...done    (Execution time: 10.703s)
% Plotting results...done   (Execution time: 0.071s)

% System-specific options, defaults are set up for unix/windows but may
% need to be modified for your system
if isunix
    % Shell syntax to run multiple commands from a string
    shell_cmd = 'bash -c';
    % Name of the executable BAM file
    app_name = 'BAM_app';
else
    % Shell syntax to run multiple commands from a string
    shell_cmd = 'cmd /c';
    % Name of the executable BAM file
    app_name = 'BAM_app.exe';
end

directory_name = '_data';
script_name = 'aRunParallel.sh';

N_cores = 8;

% number of test cases per core
N_batches = 10;

N_runs = N_cores*N_batches;

%% Verify path setup
% check that we are in the root directory
if ~exist('setup.m', 'file')
    error('Please move example_batch_runme.m into and run from the root directory.');
end

% initialize data directory
data_dir = append(directory_name,filesep);
if ~exist(data_dir, 'dir'), mkdir(data_dir); end
workspace_name = 'workspace.mat';
workspace_file_name = append(data_dir,filesep,workspace_name);

% check for dependencies (executable/shell script)
exePath = fullfile(data_dir, app_name);
shellScriptPath = fullfile(data_dir, script_name);
exeDefaultPath = append('AutoCode',filesep,app_name);
shellDefaultPath = append('AutoCodeModels',filesep,'examples',filesep,'BatchProcessing');
if ~exist(exePath, 'file')
    error('Executable not found in directory: %s \nPlease add it from %s', exePath,exeDefaultPath);
end

if ~exist(shellScriptPath, 'file')
    error('Shell script not found in directory: %s\nPlease add it from %s', shellScriptPath,shellDefaultPath);
end


% Check for .bin files and warn if they exist
bin_files = dir(fullfile(directory_name, '*.bin'));
if ~isempty(bin_files)
    warning('Data directory contains .bin files. These may be run if new .bin files fail to overwrite them.');
end
% Check for .mat files and warn if they exist
mat_files = dir(fullfile(directory_name, '*.mat'));
if ~isempty(mat_files)
    warning('Data directory contains .mat files. These may be run if new .mat files fail to overwrite them.');
end


%% Setup
% Run setup and create workspace file to capture data structures.
% The workspace file includes buses and userStruct, SimIn, SimPar.
% writePfile depends on BUS_PARAM*, and SimPar
% convertArrayToStruct depends on BUS_USER*

if ~exist(workspace_file_name, 'file')
    % run setup and create dependencies file for future use
    % we often do not want to run setup every time we create a parameter
    % file, so it is convenient to have the workspace variable for record
    % keeping and creating new pFiles with the correct structure.
    setup;
    busVars = [who('BUS_PARAM*')' who('BUS_USER*')'];
    workspaceVars = [busVars 'SimPar' 'SimIn' 'userStruct'];
    save(workspace_file_name,workspaceVars{:});
else
    % setup the path varables and load the workspace file
    addpath('setup')
    setupPath;
    load(workspace_file_name)
end

%% Create new parameters
% create random numbers all at once, changing order will change numbers
rng(15)
randomSeedVals = randi(1e4,[N_runs,3]); % preallocate random turbulence seeds

% Pre-generate random vectors for myTraj.m function
% [x, y, z, xd, yd, zd]
sigma2 = [10 10 50 1 1 1]; 
randVector = sigma2.*randn(N_runs,6); % random end position/velocity in 3d


%% Set params
% The safest way to assign parameter values is to rerun setup.m or create 
% an appropriate example file because the setup process will assign all 
% variables correctly. In this case, the variables are carefully assigned 
% for correct initialization.
% constant parameters
SimPar.Value.Environment.Turbulence.intensity = 3; %1-off, 4-severe
fprintf('Writing pFiles...')
tic
% variable across runs
for n=1:N_runs
    % update turbulence parameters
    SimPar.Value.Environment.Turbulence.RandomSeedLong = randomSeedVals(n,1);
    SimPar.Value.Environment.Turbulence.RandomSeedLat = randomSeedVals(n,2);
    SimPar.Value.Environment.Turbulence.RandomSeedVert = randomSeedVals(n,3);

    % update trajectory parameters
    [wptVect, timeVect, parseVect] = myTraj(randVector(n,:));
    % [wptVect, timeVect, parseVect] = zerosTraj();
    SimPar.Value.RefInputs.trajectory.wptVect = wptVect;
    SimPar.Value.RefInputs.trajectory.timeVect = timeVect;
    SimPar.Value.RefInputs.trajectory.parseVect = parseVect';

    % write file
    file_name = sprintf('pFile%03d.bin',n);
    writePfile(SimPar.Value,append(data_dir,file_name),1);
end
tval = toc;
fprintf('done\t(Execution time: %.3fs)\n',tval)

%% RUN
 if ~exist('N_cores','var'),N_cores=1;end

 % build shell command string

command_str = sprintf("%s %s %d", fullfile('.', script_name), ...
                      fullfile('.', app_name), N_cores);
if isunix
    command_str = sprintf("chmod +x %s && %s", script_name, command_str);
end

command_str = sprintf('%s "cd %s && %s"', shell_cmd, data_dir, ...
                      command_str);

% run system command
fprintf('Running sim...')

warning(['Running system command\n "%s" \n Please verify this works ' ...
         'for your system and application. System errors are not sent ' ...
         'to matlab.'], command_str)

tic

system(command_str);

tval = toc;
fprintf('done\t(Execution time: %.3fs)\n',tval)


%% Load Data
sim_data = cell(N_runs,1);
fprintf('Loading results...')
tic
for n=1:N_runs
    file_name = sprintf('bam_out%03d.mat',n);
    % load data structure based on BUS_USER* structures
    sim_data{n}.Values = convertArrayToStruct(append(data_dir,file_name));
end
tval = toc;
fprintf('done\t(Execution time: %.3fs)\n',tval)

%% Plot Data

fprintf('Plotting results...')
tic

plot_trajectories(sim_data, plot_reference=false);

tval = toc;
fprintf('done\t(Execution time: %.3fs)\n',tval)
