#!/bin/bash

#==========================================================================
# ROS Workspace Cleaner
# 
# Authors:
#   Newton Campbell
#   Tenavi Nakamura-Zimmerer
# History:
#   2025-04-25: Created (NC)
#   2025-07-18: Refactoring for simplicity, code reuse
#
# Description:
#   This shell script provides a user-friendly way to clean ROS workspaces
#   by removing build, install, log, and src directories. It can be useful for
#   cleanly testing ROS nodes against the simulator. It includes automatic
#   workspace detection in common locations with comprehensive naming patterns,
#   and safeguards to prevent accidental data loss. Also offers automatic
#   rebuilding of the workspace after cleaning.
#
# Usage:
#   bash ros_workspace_cleaner.bash [options]
#
# Options:
#   -w, --workspace <PATH>  Specify ROS workspace path directly
#   -a, --airsim            When rebuilding, install the bam_2_airsim package (optional)
#   -e, --editable          When rebuilding, call colcon build with --symlink-install (optional)
#   -s, --sudo              When rebuilding, initialize rosdep using sudo (optional)
#   -h, --help              Display help message
#==========================================================================

# Enable job control and exit on error
set -m

# Terminal colors for better UX
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Set terminal title (if supported)
echo -e "\033]0;ROS Workspace Cleaner\007"

# Display welcome banner
echo -e "${BOLD}========================================================${NC}"
echo -e "${BOLD}${BLUE}             ROS Workspace Cleaner Utility${NC}"
echo -e "${BOLD}========================================================${NC}"
echo ""

#==========================================================================
# Function definitions
#==========================================================================

# Print function for formatted output
print_msg() {
    local type=$1
    local msg=$2
    
    case $type in
        "info")    echo -e "[${BLUE}INFO${NC}] $msg" ;;
        "success") echo -e "[${GREEN}✓${NC}] $msg" ;;
        "warning") echo -e "[${YELLOW}!${NC}] $msg" ;;
        "error")   echo -e "[${RED}✗${NC}] $msg" ;;
        "header")  echo -e "\n${BOLD}${BLUE}$msg${NC}\n" ;;
        "section") echo -e "\n${BOLD}========================================================${NC}"
                   echo -e "${BOLD}$msg${NC}"
                   echo -e "${BOLD}========================================================${NC}\n" ;;
        *)         echo -e "$msg" ;;
    esac
}

# Display usage information
usage() {
    print_msg "section" "ROS Workspace Cleaner - Help Information"
    
    echo "Usage: bash $(basename "$0") [options]"
    echo ""
    echo "Options:"
    echo "  -w, --workspace PATH   Specify ROS workspace path directly"
    echo "  -a, --airsim           When rebuilding, install the bam_2_airsim package (optional)"
    echo "  -e, --editable         When rebuilding, call colcon build with --symlink-install (optional)"
    echo "  -s, --sudo             When rebuilding, initialize rosdep using sudo (optional)"
    echo "  -h, --help             Show this help message"
    echo ""
    echo "Examples:"
    echo "  $(basename "$0")                  Search for workspaces automatically"
    echo "  $(basename "$0") -w ~/ros2_ws     Clean the workspace at ~/ros2_ws"
    echo ""
    echo "Description:"
    echo "  This script helps maintain ROS workspaces by cleaning build"
    echo "  artifacts, installation files, and logs. This can resolve"
    echo "  build issues and reclaim disk space."
    echo ""
    echo "  The script will search for ROS workspaces with common naming patterns:"
    echo "    - Standard: ros_ws, ros2_ws, dev_ws, colcon_ws, workspace"
    echo "    - Distribution-specific: ros2_foxy, ros2_galactic, ros2_humble, "
    echo "                             ros2_iron, ros2_jazzy, ros2_kilted, ros2_rolling,"
    echo "                             foxy_ws, galactic_ws, humble_ws, iron_ws,"
    echo "                             jazzy_ws, kilted_ws, rolling_ws"
    echo "  If none of these are found, the current directory can be used."
    echo ""
    echo "  After cleaning, the script can optionally rebuild the workspace"
    echo "  automatically."
    echo ""
}

# Prompt user for yes/no choice
ask_yes_no() {
    local prompt="$1"
    local default="${2:-}"
    local response
    
    # If default is provided, show it in the prompt
    if [[ -n "$default" ]]; then
        if [[ "$default" == "y" ]]; then
            prompt="$prompt [Y/n]"
        else
            prompt="$prompt [y/N]"
        fi
    else
        prompt="$prompt [y/n]"
    fi
    
    while true; do
        read -p "$prompt: " response
        
        # Handle default if no input provided
        if [[ -z "$response" && -n "$default" ]]; then
            response=$default
        fi
        
        case "$response" in
            [Yy]* ) return 0;;  # Yes
            [Nn]* ) return 1;;  # No
            * ) echo -e "${YELLOW}Please answer y or n.${NC}";;
        esac
    done
}

# Prompt user for multi-choice selection
ask_choice() {
    local prompt="$1"
    local options=("${@:2}")
    local choice
    
    echo -e "${CYAN}$prompt${NC}"
    for i in "${!options[@]}"; do
        echo -e "${CYAN}[$((i+1))]${NC} ${options[i]}"
    done
    echo ""
    
    while true; do
        read -p "Enter choice [1-${#options[@]}]: " choice
        
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le "${#options[@]}" ]; then
            return "$choice"
        else
            echo -e "${YELLOW}Please enter a number between 1 and ${#options[@]}.${NC}"
        fi
    done
}

#==========================================================================
# Command-line argument handling
#==========================================================================

# Initialize variables for workspace specification
SPECIFIED_WORKSPACE=""
SKIP_SEARCH=0
INSTALL_AIRSIM=""
EDITABLE_INTSTALL=""
SUDO_ROSDEP=""

# Check for command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -w|--workspace)
            if [[ -n "$2" ]]; then
                SPECIFIED_WORKSPACE="$2"
                SKIP_SEARCH=1
                print_msg "info" "Using specified workspace: $SPECIFIED_WORKSPACE"
                shift 2
            else
                print_msg "error" "Workspace path must be provided with -w flag."
                usage
                exit 1
            fi
            ;;
        -a|--airsim)
            INSTALL_AIRSIM="-a"
            shift
            ;;
        -e|--editable)
            EDITABLE_INTSTALL="-e"
            shift
            ;;
        -s|--sudo)
            SUDO_ROSDEP="-s"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            print_msg "error" "Unknown parameter: $1"
            usage
            exit 1
            ;;
    esac
done

#==========================================================================
# Workspace verification (when specified via command line)
#==========================================================================

# If workspace is specified via arguments, verify it exists
if [ "$SKIP_SEARCH" -eq 1 ]; then
    # Check if the directory exists
    if [ ! -d "$SPECIFIED_WORKSPACE" ]; then
        print_msg "error" "The specified workspace directory does not exist: $SPECIFIED_WORKSPACE"
        print_msg "info" "Please check the path and try again."
        exit 1
    fi

    # Catch relative paths like "." or ".."
    SPECIFIED_WORKSPACE="$( cd "$SPECIFIED_WORKSPACE" && pwd )"
    
    # Check if it has a src directory (basic validation for ROS workspace)
    if [ ! -d "$SPECIFIED_WORKSPACE/src" ]; then
        print_msg "warning" "The specified directory does not contain a 'src' folder."
        print_msg "warning" "Are you sure this is a ROS workspace?"
        
        if ! ask_yes_no "Continue anyway" "n"; then
            exit 0
        fi
    fi
    
    # Use the specified workspace
    WORKSPACE_PATH="$SPECIFIED_WORKSPACE"
    
    # Skip to workspace confirmation
    CONFIRM_WORKSPACE=true
else
    #==========================================================================
    # Workspace auto-detection
    #==========================================================================
    
    print_msg "info" "Searching for ROS workspaces in common locations..."
    print_msg "info" "This may take a moment..."
    echo ""
    
    # Define common ROS distributions for comprehensive search
    ROS2_DISTROS=("foxy" "galactic" "humble" "iron" "jazzy" "kilted" "rolling")
    
    # Define common base directory locations to search
    BASE_DIRS=(
        "$HOME"
        "/opt/ros"
        "/opt"
        "/workspaces"
        "$HOME/ros"
        "$HOME/workspaces"
        "/usr/local/workspaces"
    )

    # Define common workspace naming patterns
    WS_PATTERNS=("workspace" "ros_ws" "ros2_ws" "dev_ws" "colcon_ws")
    
    # Initialize arrays to store found workspaces
    declare -a WORKSPACE_PATHS
    declare -a WORKSPACE_NAMES
    WORKSPACE_COUNT=0
    
    # First pass: Search for distribution-specific workspace names
    for base_dir in "${BASE_DIRS[@]}"; do
        echo "$base_dir"

        # Skip if base directory doesn't exist
        [ ! -d "$base_dir" ] && continue
        
        # Distribution-specific workspaces
        for distro in "${ROS2_DISTROS[@]}"; do
            if [ -d "$base_dir/ros2_$distro" ]; then
                WORKSPACE_PATHS[$WORKSPACE_COUNT]="$base_dir/ros2_$distro"
                WORKSPACE_NAMES[$WORKSPACE_COUNT]="ros2_$distro"
                ((WORKSPACE_COUNT++))
            fi
            
            if [ -d "$base_dir/ros_$distro" ]; then
                WORKSPACE_PATHS[$WORKSPACE_COUNT]="$base_dir/ros_$distro"
                WORKSPACE_NAMES[$WORKSPACE_COUNT]="ros_$distro"
                ((WORKSPACE_COUNT++))
            fi
        done
        
        # Generic workspace patterns
        for pattern in "${WS_PATTERNS[@]}"; do
            if [ -d "$base_dir/$pattern" ]; then
                WORKSPACE_PATHS[$WORKSPACE_COUNT]="$base_dir/$pattern"
                WORKSPACE_NAMES[$WORKSPACE_COUNT]="$pattern"
                ((WORKSPACE_COUNT++))
            fi
        done
        
        # ROS2 with workspace suffix
        for distro in "${ROS2_DISTROS[@]}"; do
            if [ -d "$base_dir/${distro}_ws" ]; then
                WORKSPACE_PATHS[$WORKSPACE_COUNT]="$base_dir/${distro}_ws"
                WORKSPACE_NAMES[$WORKSPACE_COUNT]="${distro}_ws"
                ((WORKSPACE_COUNT++))
            fi
        done
    done

    echo ""

    # If no workspaces were found, add script directory to list of potential workspaces
    if [ $WORKSPACE_COUNT -eq 0 ]; then
        BAM_ROS_DIR="$( cd "$( dirname -- "$0" )/../../.." && pwd )"
        if [ -d "$BAM_ROS_DIR/ROS2_ws" ]; then
            print_msg "warning" "No ROS workspaces found in common locations. Considering script directory as a potential workspace."

            WORKSPACE_PATHS[$WORKSPACE_COUNT]="$BAM_ROS_DIR/ROS2_ws"
            WORKSPACE_NAMES[$WORKSPACE_COUNT]="$BAM_ROS_DIR/ROS2_ws"
            ((WORKSPACE_COUNT++))
        fi
    fi
    
    # If no workspaces were found, add current directory to list of potential workspaces
    if [ $WORKSPACE_COUNT -eq 0 ]; then
        print_msg "warning" "No ROS workspaces found in common locations. Considering current directory as a potential workspace."

        WORKSPACE_PATHS[$WORKSPACE_COUNT]="$(pwd)"
        WORKSPACE_NAMES[$WORKSPACE_COUNT]="$(pwd)"
        ((WORKSPACE_COUNT++))
    else
        print_msg "info" "Workspace search complete. Found $WORKSPACE_COUNT potential workspaces."
    fi

    echo ""

    #==========================================================================
    # Workspace selection
    #==========================================================================

    WORKSPACE_INDEX=0
    CONFIRM_WORKSPACE=false

    while [ $CONFIRM_WORKSPACE = false ]; do
        # Increment the workspace index to show the next candidate
        ((WORKSPACE_INDEX++))

        # Check if we've gone through all candidates
        if [ $WORKSPACE_INDEX -gt $WORKSPACE_COUNT ]; then
            print_msg "warning" "No more workspaces found."
            print_msg "info" "Please specify a workspace path manually with the -w flag."
            exit 0
        fi

        # Get current workspace candidate details
        WORKSPACE_PATH="${WORKSPACE_PATHS[$((WORKSPACE_INDEX-1))]}"
        WORKSPACE_NAME="${WORKSPACE_NAMES[$((WORKSPACE_INDEX-1))]}"

        # Display workspace information and options
        echo -e "${BOLD}--------------------------------------------------------${NC}"
        print_msg "info" "Found potential ROS workspace [$WORKSPACE_INDEX/$WORKSPACE_COUNT]:"
        echo -e "  Path: ${CYAN}$WORKSPACE_PATH${NC}"
        echo -e "  Name: ${CYAN}$WORKSPACE_NAME${NC}"

        echo -e "${BOLD}--------------------------------------------------------${NC}"
        echo ""

        # Check for build, install, log directories to show what would be cleaned
        echo "Directories present:"

        # Check build directory (handle both regular and symlink directories)
        if [ -e "$WORKSPACE_PATH/build" ]; then
            if [ -L "$WORKSPACE_PATH/build" ]; then
                local target=$(readlink -f "$WORKSPACE_PATH/build")
                echo -e "  - build:   ${GREEN}Yes${NC} (symlink → $target)"
            else
                echo -e "  - build:   ${GREEN}Yes${NC}"
            fi
        else
            echo -e "  - build:   ${RED}No${NC}"
        fi

        # Check install directory (handle both regular and symlink directories)
        if [ -e "$WORKSPACE_PATH/install" ]; then
            if [ -L "$WORKSPACE_PATH/install" ]; then
                local target=$(readlink -f "$WORKSPACE_PATH/install")
                echo -e "  - install: ${GREEN}Yes${NC} (symlink → $target)"
            else
                echo -e "  - install: ${GREEN}Yes${NC}"
            fi
        else
            echo -e "  - install: ${RED}No${NC}"
        fi

        # Check log directory (handle both regular and symlink directories)
        if [ -e "$WORKSPACE_PATH/log" ]; then
            if [ -L "$WORKSPACE_PATH/log" ]; then
                local target=$(readlink -f "$WORKSPACE_PATH/log")
                echo -e "  - log:     ${GREEN}Yes${NC} (symlink → $target)"
            else
                echo -e "  - log:     ${GREEN}Yes${NC}"
            fi
        else
            echo -e "  - log:     ${RED}No${NC}"
        fi

        echo ""

        # Ask for user selection
        options=("Yes, clean this workspace" "No, show the next workspace" "Quit")
        ask_choice "Clean this workspace?" "${options[@]}"
        choice=$?

        case $choice in
            1)
                CONFIRM_WORKSPACE=true
                ;;
            2)
                continue
                ;;
            3)
                print_msg "info" "Operation cancelled."
                exit 0
                ;;
        esac
    done
fi

#==========================================================================
# Workspace cleaning confirmation
#==========================================================================

print_msg "section" "WORKSPACE CLEANING CONFIRMATION"
print_msg "info" "Workspace to clean: ${CYAN}$WORKSPACE_PATH${NC}"
echo ""
print_msg "info" "This will clean the following directories if they exist:"
echo -e "  - ${YELLOW}$WORKSPACE_PATH/build${NC}    :: Build artifacts and CMake files"
echo -e "  - ${YELLOW}$WORKSPACE_PATH/install${NC}  :: Installed executables and libraries"
echo -e "  - ${YELLOW}$WORKSPACE_PATH/log${NC}      :: Build and runtime logs"
echo ""
print_msg "warning" "IMPORTANT: For symbolic links, only the links themselves will be removed,"
print_msg "warning" "           not the directories or files they point to."
echo ""
print_msg "warning" "These actions cannot be undone. Any compiled code and logs will be lost."
echo ""

if ! ask_yes_no "Are you sure you want to proceed" "n"; then
    print_msg "info" "Operation cancelled by user."
    exit 0
fi

#==========================================================================
# Workspace cleaning process
#==========================================================================

print_msg "section" "CLEANING WORKSPACE"

# Clean build directory - preserve symlink targets
if [ -e "$WORKSPACE_PATH/build" ] || [ -L "$WORKSPACE_PATH/build" ]; then
    print_msg "header" "Cleaning build directory..."
    rm -rf "$WORKSPACE_PATH/build"
else
    print_msg "info" "Build directory doesn't exist. Skipping."
fi

# Clean install directory - preserve symlink targets
if [ -e "$WORKSPACE_PATH/install" ] || [ -L "$WORKSPACE_PATH/install" ]; then
    print_msg "header" "Cleaning install directory..."
    rm -rf "$WORKSPACE_PATH/install"
else
    print_msg "info" "Install directory doesn't exist. Skipping."
fi

# Clean log directory - preserve symlink targets
if [ -e "$WORKSPACE_PATH/log" ] || [ -L "$WORKSPACE_PATH/log" ]; then
    print_msg "header" "Cleaning log directory..."
    rm -rf "$WORKSPACE_PATH/log"
else
    print_msg "info" "Log directory doesn't exist. Skipping."
fi

#==========================================================================
# Source packages cleaning (optional)
#==========================================================================

print_msg "section" "SOURCE PACKAGES"

if [ -d "$WORKSPACE_PATH/src" ]; then
    print_msg "warning" "Would you like to clean packages in the src directory?"
    print_msg "warning" "This will clean ALL packages in ${CYAN}$WORKSPACE_PATH/src${NC}."
    print_msg "info" "NOTE: For symlinks, only the links will be removed, not their targets."
    echo ""

    if ask_yes_no "Clean source packages" "n"; then
        # User chose to clean source packages

        # List all packages before cleaning for information
        echo "Packages to be cleaned:"
        for pkg in "$WORKSPACE_PATH"/src/*/; do
            if [ -d "$pkg" ]; then
                pkg_name=$(basename "$pkg")
                echo -n "  - $pkg_name"

                # Show indicator for symlink packages
                if [ -L "$pkg" ]; then
                    target=$(readlink -f "$pkg")
                    echo -e " ${YELLOW}(symlink → $target)${NC}"
                else
                    echo ""
                fi
            fi
        done
        echo ""

        # Double-check with a stronger warning
        print_msg "warning" "WARNING: This will clean ALL contents in $WORKSPACE_PATH/src"
        print_msg "warning" "Are you ABSOLUTELY SURE you want to continue?"
        print_msg "warning" "This action cannot be undone and will delete source code."
        echo ""
        
        if ask_yes_no "I understand this will clean ALL packages" "n"; then
            # Proceed with source cleaning
            print_msg "header" "Cleaning all packages in source directory..."
            
            # Loop through each package in the src folder and clean it
            for pkg in "$WORKSPACE_PATH"/src/*/; do
                if [ -d "$pkg" ]; then
                    pkg_name=$(basename "$pkg")
                    unlink "$WORKSPACE_PATH/src/$pkg_name"
                fi
            done

            print_msg "success" "Source packages cleaned."

            # Try to remove the now-empty directory
            if [ -z "$(ls -A "$WORKSPACE_PATH/src")" ]; then
                rmdir "$WORKSPACE_PATH/src"
                print_msg "success" "Empty source directory removed."
            else
                print_msg "info" "Source directory is not empty."
            fi
        else
            # User cancelled source cleaning
            print_msg "info" "Source cleaning cancelled."
        fi
    else
        # User chose not to clean source packages
        print_msg "info" "Source packages will be preserved."
    fi
else
    print_msg "info" "Source directory doesn't exist. Nothing to clean."
fi

#==========================================================================
# Cleanup completion
#==========================================================================

print_msg "section" "CLEANUP COMPLETE"
print_msg "success" "ROS workspace $WORKSPACE_PATH has been cleaned."
print_msg "info" "Symlinks were handled properly - only links were removed, not their targets."

#==========================================================================
# Offer to rebuild the workspace
#==========================================================================

print_msg "section" "REBUILD WORKSPACE"
print_msg "info" "Would you like to rebuild the workspace now?"
echo ""

if ask_yes_no "Rebuild workspace" "n"; then
    # User wants to rebuild
    SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
    source "$SCRIPT_DIR/install_bam_ros_packages.bash" -w "$WORKSPACE_PATH" $INSTALL_AIRSIM $EDITABLE_INTSTALL $SUDO_ROSDEP
else
    # User does not want to rebuild
    echo ""
    print_msg "success" "Rebuild skipped. Thank you for using the ROS Workspace Cleaner utility."
    echo ""
fi