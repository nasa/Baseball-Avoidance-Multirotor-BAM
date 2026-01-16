#!/bin/bash
#
# install_psw_sub.sh
#
# This script installs the updated psw_sub package files into a ROS workspace.
#
# Expected Git repository structure (to be run from within this directory):
#
#  psw_sub/
#    ├── CMakeLists.txt
#    ├── install_psw_sub.sh   <-- (this script)
#    ├── package.xml
#    └── src
#         └── PSWSubscriber.cpp
#
# The script will:
#  1. Determine the target workspace by (a) command-line argument or (b) searching for a directory
#     in $HOME matching "ros_*". You'll be prompted to confirm the selection.
#  2. Ensure that the workspace has a "src" folder.
#  3. Check if the package folder exists in "src". If not, it will create the package using ros2 pkg create.
#  4. Copy CMakeLists.txt, package.xml, and the src folder into the package folder.
#
# Usage:
#   ./install_psw_sub.sh [TARGET_WORKSPACE_DIRECTORY]
#
set -e

# Function to print an error message and exit.
error_exit() {
    echo "Error: $1" >&2
    exit 1
}

# Function to search for a directory in $HOME matching "ros_*"
find_ros_directory() {
    local found_dir
    found_dir=$(ls -d "$HOME"/ros_* 2>/dev/null | head -n 1 || echo "")
    echo "$found_dir"
}

# Determine the target workspace (TARGET_WS)
if [ -n "$1" ]; then
    TARGET_WS="$1"
else
    CANDIDATE=$(find_ros_directory)
    if [ -z "$CANDIDATE" ]; then
        echo "No directory matching 'ros_*' was found in your home folder ($HOME)."
        echo "Please specify the workspace directory via the command-line argument."
        exit 1
    else
        echo "Found workspace directory candidate: $CANDIDATE"
        read -p "Is this the correct workspace directory? (y/n): " CONFIRM
        if [ -z "$CONFIRM" ]; then
            echo "No response provided. Exiting."
            exit 1
        elif [ "$CONFIRM" != "y" ]; then
            echo "Please specify the correct workspace directory using the command-line argument."
            exit 1
        else
            TARGET_WS="$CANDIDATE"
        fi
    fi
fi

# Confirm TARGET_WS exists and is accessible.
if [ ! -d "$TARGET_WS" ]; then
    error_exit "The specified workspace directory '$TARGET_WS' does not exist or cannot be accessed."
fi

echo "Using ROS workspace directory: ${TARGET_WS}"

# Ensure the workspace has a src folder.
if [ ! -d "${TARGET_WS}/src" ]; then
    echo "Creating src folder in the workspace..."
    mkdir -p "${TARGET_WS}/src" || error_exit "Failed to create ${TARGET_WS}/src."
fi

# Set package name (in this instance: psw_sub) and target package directory.
PKG_NAME="psw_sub"
TARGET_PKG_DIR="${TARGET_WS}/src/${PKG_NAME}"

# If the package directory does not exist, create it using ros2 pkg create.
if [ ! -d "$TARGET_PKG_DIR" ]; then
    echo "Package directory '${TARGET_PKG_DIR}' not found."
    echo "Creating the package '${PKG_NAME}' in the workspace using ros2 pkg create..."
    ros2 pkg create --build-type ament_cmake ${PKG_NAME} \
        || error_exit "Failed to create package '${PKG_NAME}' via ros2 pkg create."
fi

# Copy updated package files.
echo "Copying updated files from the repository to ${TARGET_PKG_DIR}"
# Copy CMakeLists.txt and package.xml
cp -f ./CMakeLists.txt "${TARGET_PKG_DIR}/" || error_exit "Failed to copy CMakeLists.txt."
cp -f ./package.xml "${TARGET_PKG_DIR}/" || error_exit "Failed to copy package.xml."

# Remove existing src folder in the package (if present) and copy new one.
if [ -d "${TARGET_PKG_DIR}/src" ]; then
    rm -rf "${TARGET_PKG_DIR}/src" || error_exit "Failed to remove existing src folder in ${TARGET_PKG_DIR}"
fi
cp -R ./src "${TARGET_PKG_DIR}/" || error_exit "Failed to copy the src folder."

echo "Package '${PKG_NAME}' was successfully updated in the workspace."

cat <<EOF

Next steps:
1. Navigate to your workspace:
   cd ${TARGET_WS}
2. Build the workspace using colcon:
   colcon build --symlink-install --packages-select ${PKG_NAME}
3. Source the install setup file after building:
   source ${TARGET_WS}/install/setup.bash
4. If you haven't set the PATH variable to include ${TARGET_WS}/build/${PKG_NAME}, do so. Otherwise, from MATLAB, run the following prior to simulation setup:
   addpath(genpath('${TARGET_WS}/build/${PKG_NAME}'))
5. In your Simulink model, add an S-Function block and set its name to "PSWSubscriber"
   with the topic name as a parameter.
EOF

echo "Installation complete."
