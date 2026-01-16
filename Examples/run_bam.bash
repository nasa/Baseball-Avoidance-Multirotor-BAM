#!/bin/bash
#########################################################################
# BAM Simulink Model Runner
#########################################################################
# 
# This script runs the BAM Simulink model with MATLAB in command-line mode.
# It handles setting up the necessary paths, executing the model, and
# providing flexible output options.
#
# Author: Newton Campbell
# Email: newton.h.campbell@nasa.gov
#
# Usage:
# This script allows you to run the BAM Simulink model with different
# ROS workspace paths and output options. It can display output to the
# console, save to a log file, or both.
#
# Dependencies:
# - MATLAB with Simulink
# - ROS environment
# - BAM.slx Simulink model
# - mat_airsim_pub package
#
#########################################################################

# Function to display usage information
show_usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  -w, --workspace=PATH  : Path to your ROS workspace"
    echo "  -c, --console         : Display output in console only (default)"
    echo "  -l, --log=FILE        : Save output to specified log file only"
    echo "  -b, --both=FILE       : Display output in console and save to specified log file"
    echo "  -h, --help            : Show this help message"
    echo ""
    echo "If no workspace is specified, the script will attempt to detect common ROS"
    echo "workspace locations, use the ROS_WS_HOME environment variable if set,"
    echo "or prompt you to enter a path."
    echo ""
    echo "Examples:"
    echo "  $0 --workspace=/path/to/ros_ws"
    echo "  $0 -w /path/to/ros_ws -b simulation.log"
}

# Function to search for common ROS workspace directories
find_ros_workspace() {
    # List of common ROS workspace locations to check
    local possible_workspaces=(
        # Standard naming
        "$HOME/ros_ws"
        "$HOME/ros2_ws"
        "$HOME/colcon_ws"
        "$HOME/catkin_ws"
        
        # ROS2 distro-specific workspaces
        "$HOME/ros_jazzy"
        "$HOME/ros_iron"
        "$HOME/ros_humble"
        "$HOME/ros_galactic"
        "$HOME/ros_foxy"
        "$HOME/ros_rolling"
        
        # Alternative locations
        "$HOME/workspace/ros_ws"
        "$HOME/workspace/ros2_ws"
        "$HOME/dev/ros_ws"
        "$HOME/dev/ros2_ws"
        "$HOME/projects/ros_ws"
        "$HOME/projects/ros2_ws"
        
        # Root directory locations
        "/opt/ros_ws"
        "/opt/ros2_ws"
        "/usr/local/ros_ws"
        "/usr/local/ros2_ws"
        
        # Windows-style paths (when running on WSL)
        "/mnt/c/ros_ws"
        "/mnt/c/ros2_ws"
        "/mnt/c/dev/ros_ws"
        "/mnt/c/dev/ros2_ws"
        
        # Other common alternatives
        "$HOME/robostack"
        "$HOME/ros_workspace"
        "$HOME/ros2_workspace"
        "/opt/robostack"
        "$HOME/colcon_workspace"
    )
    
    # Look for a setup file in each possible workspace
    for ws in "${possible_workspaces[@]}"; do
        if [ -d "$ws" ] && [ -f "$ws/install/setup.bash" ]; then
            echo "$ws"
            return 0
        fi
    done
    
    # If no workspace found with install/setup.bash, look for ones with just a build directory
    for ws in "${possible_workspaces[@]}"; do
        if [ -d "$ws" ] && [ -d "$ws/build" ]; then
            echo "$ws"
            return 0
        fi
    done
    
    # If no workspace found, return empty string
    echo ""
    return 1
}

# Parse arguments
ROS_WORKSPACE=""
OUTPUT_OPTION="--console"
LOG_FILE=""

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -w=*|--workspace=*)
            ROS_WORKSPACE="${1#*=}"
            shift
            ;;
        -w|--workspace)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "Error: Workspace path required after $1 option"
                show_usage
                exit 1
            fi
            ROS_WORKSPACE="$2"
            shift 2
            ;;
        -c|--console)
            OUTPUT_OPTION="--console"
            shift
            ;;
        -l=*|--log=*)
            OUTPUT_OPTION="--log"
            LOG_FILE="${1#*=}"
            shift
            ;;
        -l|--log)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "Error: Log file path required after $1 option"
                show_usage
                exit 1
            fi
            OUTPUT_OPTION="--log"
            LOG_FILE="$2"
            shift 2
            ;;
        -b=*|--both=*)
            OUTPUT_OPTION="--both"
            LOG_FILE="${1#*=}"
            shift
            ;;
        -b|--both)
            if [[ -z "$2" || "$2" == -* ]]; then
                echo "Error: Log file path required after $1 option"
                show_usage
                exit 1
            fi
            OUTPUT_OPTION="--both"
            LOG_FILE="$2"
            shift 2
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Flag to track if we need to source the setup.bash file
NEED_SOURCE_SETUP=false

# Check if ROS workspace is provided, otherwise try to find it
if [ -z "$ROS_WORKSPACE" ]; then
    # Check if ROS_WS_HOME environment variable is set
    if [ -n "$ROS_WS_HOME" ]; then
        ROS_WORKSPACE="$ROS_WS_HOME"
        echo "Using ROS workspace from environment variable: $ROS_WORKSPACE"
        if [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
            NEED_SOURCE_SETUP=true
        fi
    else
        # Try to auto-detect ROS workspace
        echo "Searching for ROS workspace..."
        ROS_WORKSPACE=$(find_ros_workspace)
        
        if [ -n "$ROS_WORKSPACE" ]; then
            echo "Found ROS workspace at: $ROS_WORKSPACE"
            read -p "Use this workspace? (y/n): " USE_FOUND_WS
            if [[ "$USE_FOUND_WS" =~ ^[Yy]$ ]]; then
                if [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
                    NEED_SOURCE_SETUP=true
                    echo "Will source $ROS_WORKSPACE/install/setup.bash"
                fi
            else
                # User doesn't want to use the found workspace - prompt for input
                read -p "Please enter the path to your ROS workspace: " ROS_WORKSPACE
                
                # Validate input
                if [ -z "$ROS_WORKSPACE" ]; then
                    echo "Error: No workspace path provided. Exiting."
                    exit 1
                fi
                
                if [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
                    NEED_SOURCE_SETUP=true
                fi
            fi
        else
            # No workspace found automatically - prompt for input
            echo "No ROS workspace found automatically."
            read -p "Please enter the path to your ROS workspace: " ROS_WORKSPACE
            
            # Validate input
            if [ -z "$ROS_WORKSPACE" ]; then
                echo "Error: No workspace path provided. Exiting."
                exit 1
            fi
            
            if [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
                NEED_SOURCE_SETUP=true
            fi
        fi
    fi
elif [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
    NEED_SOURCE_SETUP=true
fi

# Verify the workspace directory exists
if [ ! -d "$ROS_WORKSPACE" ]; then
    echo "Warning: The specified workspace directory does not exist: $ROS_WORKSPACE"
    read -p "Do you want to continue anyway? (y/n): " CONTINUE
    if [[ ! "$CONTINUE" =~ ^[Yy]$ ]]; then
        echo "Operation cancelled."
        exit 1
    fi
fi

echo "Using ROS workspace: $ROS_WORKSPACE"

# Source the ROS workspace setup file if needed
if [ "$NEED_SOURCE_SETUP" = true ]; then
    echo "Sourcing ROS workspace setup file: $ROS_WORKSPACE/install/setup.bash"
    source "$ROS_WORKSPACE/install/setup.bash"
    
    # Check if sourcing was successful
    if [ $? -ne 0 ]; then
        echo "Warning: Failed to source the ROS workspace setup file."
        read -p "Continue anyway? (y/n): " CONTINUE
        if [[ ! "$CONTINUE" =~ ^[Yy]$ ]]; then
            echo "Operation cancelled."
            exit 1
        fi
    else
        echo "ROS workspace environment successfully loaded."
        echo "ROS_DISTRO=$ROS_DISTRO"
    fi
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
echo "Script directory: $SCRIPT_DIR"

# Create a temporary directory and MATLAB script file
TEMP_DIR=$(mktemp -d)
TEMP_SCRIPT="$TEMP_DIR/run_bam_temp.m"

# Ensure temporary directory is removed on exit
trap 'rm -rf "$TEMP_DIR"' EXIT

# Write MATLAB commands to the script file
cat > "$TEMP_SCRIPT" << EOL
cd $SCRIPT_DIR

% Add the current directory to path (where setup.m and BAM.slx are)
addpath(genpath('$SCRIPT_DIR'));

% Add the ROS workspace path
addpath(genpath('$ROS_WORKSPACE/install'));

% Handle Linux-specific library path for MEX files
if isunix && ~ismac
    % Set LD_LIBRARY_PATH environment variable for ROS libraries on Linux
    setenv('LD_LIBRARY_PATH', '${LD_LIBRARY_PATH}');
    disp('Set LD_LIBRARY_PATH for ROS 2 libraries');
    
    % Add specific paths for MEX files
    mex_paths = {
        fullfile('$ROS_WORKSPACE', 'install', 'lib', 'mat_airsim_pub'),
        fullfile('$ROS_WORKSPACE', 'bin', 'linux')
    };
    
    for i = 1:length(mex_paths)
        if exist(mex_paths{i}, 'dir')
            addpath(mex_paths{i});
            disp(['Added MEX path: ' mex_paths{i}]);
        end
    end
    
    % Find all publisher/subscriber MEX directories
    try
        [~, all_dirs] = system('find "$ROS_WORKSPACE/install" -path "*/lib/*" -type d | grep -E "(_pub|_sub)$"');
        if ~isempty(all_dirs)
            dir_list = strsplit(all_dirs, '\n');
            for i = 1:length(dir_list)
                if ~isempty(dir_list{i})
                    addpath(dir_list{i});
                    disp(['Found and added MEX directory: ' dir_list{i}]);
                end
            end
        end
    catch
        disp('Note: Unable to automatically find MEX directories');
    end
end

% Display current directory and path for debugging
disp('Current directory:');
pwd

% Run setup
disp('Running setup...');

% Run the setup script
setup;

% Configure variants
userStruct.variants.pubType = PubEnum.ROS2_NONLIB; % Use ROS2 for publishing
userStruct.variants.pubTypeBball = PubEnum.ROS2_NONLIB; % Turn on the baseball

% Specify the desired own-ship trajectory and the baseball trajectory
% numbers to execute in simulation and initialize the userStruct with their
% BP waypoints.
own_traj_num = 50; % Select desired own-ship traj number
bball_traj_num = 50; % Select desired bball traj number

% Run setup again
setup;

% Display confirmation message
disp('Path added and setup completed successfully.');

% Run the Simulink model
disp('Starting BAM.slx Simulink model...');
open_system('BAM');
sim('BAM');
disp('Simulation completed.');

% Exit MATLAB
close all;

exit;
EOL

# Ensure the script is readable
chmod +r "$TEMP_SCRIPT"

# Run MATLAB with appropriate output redirection
echo "Starting MATLAB simulation..."
echo "MATLAB will run script: $TEMP_SCRIPT"

# Change to the script directory before running MATLAB
cd "$SCRIPT_DIR"

case "$OUTPUT_OPTION" in
    --console)
        matlab -nodisplay -nosplash -nodesktop -r "run('$TEMP_SCRIPT')"
        ;;
    --log)
        if [ -z "$LOG_FILE" ]; then
            echo "Error: No log file specified with --log option."
            exit 1
        fi
        echo "Output will be saved to: $LOG_FILE"
        matlab -nodisplay -nosplash -nodesktop -r "run('$TEMP_SCRIPT')" > "$LOG_FILE" 2>&1
        echo "MATLAB execution completed. Check $LOG_FILE for details."
        ;;
    --both)
        if [ -z "$LOG_FILE" ]; then
            echo "Error: No log file specified with --both option."
            exit 1
        fi
        echo "Output will be displayed and saved to: $LOG_FILE"
        matlab -nodisplay -nosplash -nodesktop -r "run('$TEMP_SCRIPT')" 2>&1 | tee "$LOG_FILE"
        ;;
esac

echo "MATLAB execution completed."