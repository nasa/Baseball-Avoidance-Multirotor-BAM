#!/bin/bash

#=============================================================================
# install_bam_ros_packages.bash
#
# Authors:
#   Newton Campbell
#   Tenavi Nakamura-Zimmerer
# History:
#   2025-04-17: Created (NC)
#   2025-07-17: Updated for mac and allowing colcon build in
#     BAM_ROOT_DIR/ROS2_ws
#
# Description:
#   This script installs BAM (Baseball Avoidance MultiRotor) ROS2 packages 
#   into a ROS workspace. It creates symbolic links from the BAM repository 
#   to the ROS workspace, builds the packages using colcon, and sources the
#   workspace setup file.
#
# Usage:
#   source install_bam_ros_packages.bash [options]
#
# Options:
#   -w, --workspace <PATH>  Specify ROS workspace path directly
#   -a, --airsim            Install the bam_2_airsim package (optional)
#   -e, --editable          Call colcon build with --symlink-install (optional)
#   -s, --sudo              Initialize rosdep using sudo (optional)
#   -h, --help              Display help message
#
# If no workspace is specified, the script will attempt to find an existing
# ROS2 workspace in common locations. If no workspace is found, it will create
# one under the script directory with the name "ros_ws".
#
#=============================================================================

# Store the current directory to return to it at the end
CURRENT_DIR=$(pwd)

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname -- "$0" )" && pwd )"

# Define the root directory of the BAM project (assuming this script is in ROS2_ws/config/unix)
echo "Script directory is: $SCRIPT_DIR"
BAM_ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
echo "BAM_ROOT_DIR is: $BAM_ROOT_DIR"

# Verify the directory structure
if [ ! -d "$BAM_ROOT_DIR/ROS2_ws" ]; then
    echo "ERROR: ROS2 directory not found at $BAM_ROOT_DIR/ROS2_ws"
    echo "Please check directory structure."
    exit 1
fi

#=============================================================================
# Parse command line arguments
#=============================================================================

usage() {
    echo "Usage: source$(basename "$0") [options]"
    echo ""
    echo "Options:"
    echo "  -w, --workspace    Path to ROS2 workspace (optional)"
    echo "  -a, --airsim       Install the bam_2_airsim package (optional)"
    echo "  -e, --editable     Call colcon build with --symlink-install (optional)"
    echo "  -s, --sudo         Initialize rosdep using sudo (optional)"
    echo "  -h, --help         Display this help message"
    echo ""
    echo "If no workspace is specified, the script will attempt to find it automatically."
    echo "If no workspace is found, it will create one under the script directory."
    exit 0
}

ROS_WS_DIR=""
INSTALL_AIRSIM=""
SUDO_ROSDEP=""
EDITABLE_INSTALL=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--workspace)
            ROS_WS_DIR="$2"
            shift 2
            ;;
        -a|--airsim)
            INSTALL_AIRSIM=1
            shift
            ;;
        -e|--editable)
            EDITABLE_INSTALL="--symlink-install"
            shift
            ;;
        -s|--sudo)
            SUDO_ROSDEP=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 0
            ;;
    esac
done

#=============================================================================
# Workspace detection and creation
#=============================================================================

# If no workspace specified, try to find it
if [ -z "$ROS_WS_DIR" ]; then
    echo "No workspace specified, attempting to find ROS2 workspace..."
    
    # Check common workspace locations with common naming patterns
    FOUND_WS=""
    
    # List of likely workspace locations
    POTENTIAL_WS=(
        "$HOME/ros_kilted"
        "$HOME/ros_jazzy"
        "$HOME/ros_humble"
        "$HOME/ros_iron"
        "$HOME/ros_rolling"
        "$HOME/ros2_ws"
        "$HOME/ros_ws" 
        "/opt/ros2_ws"
        "/opt/ros_ws"
        "/workspaces/ros2_ws"
        "/workspaces/ros_ws"
        "$HOME/workspaces/ros2_ws"
        "$HOME/workspaces/ros_ws"
        "$HOME/dev/ros2_ws"
        "$HOME/dev/ros_ws"
        "$BAM_ROOT_DIR/ros2_ws"
        "$BAM_ROOT_DIR/ros_ws"
    )
    
    # Check each potential workspace location
    for WS in "${POTENTIAL_WS[@]}"; do
        if [ -d "$WS/src" ]; then
            ROS_WS_DIR="$WS"
            FOUND_WS=1
            echo "Found ROS2 workspace at: $WS"
        fi
    done
    
    # If still not found, use default location
    if [ -z "$FOUND_WS" ]; then
        ROS_WS_DIR="$SCRIPT_DIR/ros_ws"
        echo "No existing ROS2 workspace found, creating one at: $ROS_WS_DIR"
    fi
else
    # Catch relative paths like "." or ".."
    ROS_WS_DIR="$( cd "$ROS_WS_DIR" && pwd )"
    echo "Using specified ROS2 workspace: $ROS_WS_DIR"
fi

# Create the src directory if it doesn't exist
if [ ! -d "$ROS_WS_DIR" ]; then
    echo "Creating workspace directory: $ROS_WS_DIR"
    mkdir -p "$ROS_WS_DIR"
fi

echo "Installing BAM ROS2 packages into workspace: $ROS_WS_DIR"

#=============================================================================
# Create package links for all required packages
#=============================================================================

if [ ! -d "$ROS_WS_DIR/src" ]; then
    echo "Creating src directory: $ROS_WS_DIR/src"
    mkdir -p "$ROS_WS_DIR/src"
fi

echo "Creating package links for BAM ROS2 packages..."

# Function to create a package link (using symbolic links only)
create_package_link() {
    local SOURCE_DIR="$1"

    # Check for empty source
    if [ -z "$SOURCE_DIR" ]; then
        echo "ERROR: Empty source directory passed to create_package_link"
        return 1
    fi
    
    # Get the package name from the directory path
    local PACKAGE_NAME=$(basename "$SOURCE_DIR")
    local TARGET_DIR="$ROS_WS_DIR/src/$PACKAGE_NAME"
    
    # Check if source directory exists
    if [ -d "$SOURCE_DIR" ]; then
        echo "Creating symbolic link for package: $PACKAGE_NAME ($SOURCE_DIR -> $TARGET_DIR)"
        
        # Remove existing directory/symlink if it exists
        if [ -e "$TARGET_DIR" ] || [ -L "$TARGET_DIR" ]; then
            echo "Removing existing target: $TARGET_DIR"
            rm -rf "$TARGET_DIR"
        fi
        
        # Create symbolic link for the entire package
        ln -sf "$SOURCE_DIR" "$TARGET_DIR"
    else
        echo "Warning: Source directory not found: $SOURCE_DIR"
        echo "Current directory is: $(pwd)"
    fi
}

# Process all package categories
process_packages() {
    local DIR="$1"
    local CATEGORY="$2"
    
    echo "Processing $CATEGORY packages..."
    if [ -d "$DIR" ]; then
        for PKG_DIR in "$DIR"/*; do
            if [ -d "$PKG_DIR" ]; then
                echo "Found package: $PKG_DIR"
                create_package_link "$PKG_DIR"
            fi
        done
    else
        echo "Directory not found: '$DIR' - Skipping"
    fi
}

#=============================================================================
# Install dependencies
#=============================================================================

install_dependencies() {
    echo "Installing dependencies for BAM ROS2 packages..."
    
    # Check if ROS 2 is sourced
    if [ -z "$ROS_DISTRO" ]; then
        echo "Error: ROS 2 environment not sourced. Please source your ROS 2 installation."
        echo "For example: source /opt/ros/rolling/setup.bash"
        exit 1
    fi

    # Check if rosdep is installed
    if ! command -v rosdep &> /dev/null; then
        echo "rosdep not found. Attempting to install rosdep..."
        
        # Detect the Linux distribution
        if [ -f /etc/os-release ]; then
            . /etc/os-release
            DISTRO=$ID
        elif type lsb_release >/dev/null 2>&1; then
            DISTRO=$(lsb_release -si)
        elif [ -f /etc/lsb-release ]; then
            . /etc/lsb-release
            DISTRO=$DISTRIB_ID
        else
            DISTRO=$(uname -s)
        fi

        # Convert to lowercase
        DISTRO=${DISTRO,,}

        case $DISTRO in
            ubuntu|debian)
                sudo apt update
                sudo apt install -y python3-rosdep
                ;;
            fedora)
                sudo dnf install -y python3-rosdep
                ;;
            centos|rhel)
                sudo yum install -y python3-rosdep
                ;;
            arch|manjaro)
                sudo pacman -Sy python-rosdep
                ;;
            *)
                echo "Unsupported distribution: $DISTRO"
                echo "Please install rosdep manually and run this script again."
                exit 1
                ;;
        esac

        if [ $? -ne 0 ]; then
            echo "Failed to install rosdep. Please install it manually and run this script again."
            exit 1
        fi
    fi
    
    # Initialize rosdep if needed
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        if [ "$SUDO_ROSDEP" != 1 ]; then
            echo "Initializing rosdep with sudo..."
            sudo rosdep init
        else
            echo "Initializing rosdep..."
            rosdep init
        fi

        echo "Updating rosdep..."
        rosdep update
    fi

    # Installing dependencies from src directory
    rosdep install --from-paths src --ignore-src -y
        
    if [ $? -ne 0 ]; then
        echo "Error: Failed to install dependencies."
        exit 1
    fi

    echo "Dependencies installed successfully."
}

# First, process and build interface packages
echo "Processing and building interface packages..."
process_packages "$BAM_ROOT_DIR/ROS2_ws/interfaces_pkgs" "interface"

# Build interface packages
echo "Building interface packages..."
cd "$ROS_WS_DIR"

colcon build --packages-select $(ls -1 "$BAM_ROOT_DIR/ROS2_ws/interfaces_pkgs") --merge-install $EDITABLE_INSTALL --cmake-clean-cache --cmake-args -DDISABLE_AUTOCODING=ON

if [ $? -ne 0 ]; then
    echo "Error: Interface packages build failed."
    exit 1
fi

echo "Interface packages built successfully"
# Source the workspace to make the interface packages available
cd "$ROS_WS_DIR/install"
source "setup.bash"
cd "$ROS_WS_DIR"

# Now process the rest of the packages
echo "Processing remaining packages..."
process_packages "$BAM_ROOT_DIR/ROS2_ws/analysis_pkgs" "analysis"
process_packages "$BAM_ROOT_DIR/ROS2_ws/launch_pkgs" "launch"
process_packages "$BAM_ROOT_DIR/ROS2_ws/s_function_pkgs/publishers" "publisher"
process_packages "$BAM_ROOT_DIR/ROS2_ws/s_function_pkgs/subscribers" "subscriber"
if [ "$INSTALL_AIRSIM" = 1 ]; then
    process_packages "$BAM_ROOT_DIR/ROS2_ws/visualization_pkgs" "visualization"
else
    create_package_link "$BAM_ROOT_DIR/ROS2_ws/visualization_pkgs/drone_plotter_py"
fi

#=============================================================================
# Install dependencies
#=============================================================================
install_dependencies

#=============================================================================
# Build the workspace using colcon
#=============================================================================
echo "Building remaining ROS2 packages in workspace at $ROS_WS_DIR..."
cd "$ROS_WS_DIR"

# Get list of all packages
all_packages=$(colcon list --names-only)

# Get list of interface packages
interface_packages=$(ls -1 "$BAM_ROOT_DIR/ROS2_ws/interfaces_pkgs")

# Create a list of packages to build (all packages except interface packages)
packages_to_build=$(comm -23 <(echo "$all_packages" | sort) <(echo "$interface_packages" | sort))

# If not installing airsim, remove this from the package list
if [ "$INSTALL_AIRSIM" != 1 ]; then
    packages_to_build=$(comm -23 <(echo "$packages_to_build" | sort) <(echo "bam_2_airsim_pkg"))
    echo "$packages_to_build"
fi

# Build with merge-install to simplify the install space structure
# and disable autocoding for all packages, excluding already built interface packages
echo "Running: colcon build --merge-install $EDITABLE_INSTALL --cmake-args -DISABLE_AUTOCODING=ON"

colcon build --merge-install $EDITABLE_INSTALL --cmake-clean-cache --cmake-args -DDISABLE_AUTOCODING=ON --packages-select $(echo $packages_to_build)

if [ $? -ne 0 ]; then
    echo "Error: Build failed with exit code $?"
    echo "Note: If the build failed due to autocoding issues, try running without the DISABLE_AUTOCODING flag:"
    echo "colcon build --merge-install"
    exit 1
fi

echo "Build completed"


#=============================================================================
# Source the workspace setup file
#=============================================================================

echo "Sourcing the built workspace..."
if [ -f "$ROS_WS_DIR/install/setup.bash" ]; then
    cd "$ROS_WS_DIR/install"
    source "setup.bash"
    echo "Workspace has been sourced. ROS2 packages are now available."
else
    echo "Warning: setup.bash not found. Build may have failed."
fi

#=============================================================================
# Return to the original directory and display completion message
#=============================================================================

# Return to the original directory where the script was called from
cd "$CURRENT_DIR"

echo "====================================================================="
echo ""
echo "BAM ROS2 packages installation complete!"
echo ""
echo "Workspace location: $ROS_WS_DIR"
echo "The workspace has been sourced in this terminal session."
echo "For new terminal sessions, run:"
echo "cd $ROS_WS_DIR/install"
echo "source setup.bash"
echo ""
echo "You can now run the BAM system from any directory using:"
echo "ros2 launch bam_launcher bam_launch.py"
echo "If this fails, try manually sourcing the workspace again"
echo ""
echo "====================================================================="
