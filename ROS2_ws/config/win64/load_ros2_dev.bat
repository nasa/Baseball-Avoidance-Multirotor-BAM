@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo ROS2 Development Environment Setup Script
echo ========================================================
echo.

:: Check for Visual Studio Developer Command Prompt
set "VS_CMD_FOUND=0"
set "VS_CMD_PATH="

:: Check for VS 2022
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" (
    set "VS_CMD_PATH=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"
    set "VS_CMD_FOUND=1"
)
if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" (
    set "VS_CMD_PATH=C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat"
    set "VS_CMD_FOUND=1"
)
if exist "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat" (
    set "VS_CMD_PATH=C:\Program Files\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat"
    set "VS_CMD_FOUND=1"
)
if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" (
    set "VS_CMD_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
    set "VS_CMD_FOUND=1"
)

:: Check for VS 2019 if 2022 not found
if "!VS_CMD_FOUND!"=="0" (
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat" (
        set "VS_CMD_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\Tools\VsDevCmd.bat"
        set "VS_CMD_FOUND=1"
    )
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\Tools\VsDevCmd.bat" (
        set "VS_CMD_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019\Professional\Common7\Tools\VsDevCmd.bat"
        set "VS_CMD_FOUND=1"
    )
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\Common7\Tools\VsDevCmd.bat" (
        set "VS_CMD_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019\Enterprise\Common7\Tools\VsDevCmd.bat"
        set "VS_CMD_FOUND=1"
    )
    if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat" (
        set "VS_CMD_PATH=C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\Common7\Tools\VsDevCmd.bat"
        set "VS_CMD_FOUND=1"
    )
)

:: Check for ROS2 Robostack environment
set "ROS_ENV_FOUND=0"
set "ROS_ENV_PATH="

:: Common locations for Robostack
if exist "%USERPROFILE%\robostack\activate_ros2_jazzy.bat" (
    set "ROS_ENV_PATH=%USERPROFILE%\robostack\activate_ros2_jazzy.bat"
    set "ROS_ENV_FOUND=1"
)
if exist "C:\robostack\activate_ros2_jazzy.bat" (
    set "ROS_ENV_PATH=C:\robostack\activate_ros2_jazzy.bat"
    set "ROS_ENV_FOUND=1"
)
if exist "C:\Users\%USERNAME%\robostack\activate_ros2_jazzy.bat" (
    set "ROS_ENV_PATH=C:\Users\%USERNAME%\robostack\activate_ros2_jazzy.bat"
    set "ROS_ENV_FOUND=1"
)

:: Report findings and load environments if found
echo Checking for required components:
echo.

if "!VS_CMD_FOUND!"=="1" (
    echo [✓] Visual Studio Developer Command Prompt found at:
    echo     !VS_CMD_PATH!
) else (
    echo [✗] Visual Studio Developer Command Prompt not found.
    echo     Please install Visual Studio 2019 or 2022 with C++ development tools.
    echo     Download from: https://visualstudio.microsoft.com/downloads/
)

if "!ROS_ENV_FOUND!"=="1" (
    echo [✓] ROS2 Robostack environment found at:
    echo     !ROS_ENV_PATH!
) else (
    echo [✗] ROS2 Robostack environment not found.
    echo     Please install Robostack following the instructions at:
    echo     https://robostack.github.io/GettingStarted.html
)

echo.

:: Load environments if both components are found
if "!VS_CMD_FOUND!"=="1" if "!ROS_ENV_FOUND!"=="1" (
    echo Loading development environment...
    echo.
    
    :: Load Visual Studio environment
    echo Initializing Visual Studio Developer Command Prompt...
    call "!VS_CMD_PATH!"
    
    :: Load ROS environment
    echo Initializing ROS2 environment...
    call "!ROS_ENV_PATH!"
    
    echo.
    echo ========================================================
    echo ROS2 Development Environment loaded successfully!
    echo.
    echo You can now use colcon to build ROS2 packages.
    echo Example: colcon build --merge-install
    echo ========================================================
) else (
    echo.
    echo ========================================================
    echo Unable to load complete development environment.
    echo Please install the missing components listed above.
    echo ========================================================
)

endlocal