@echo off
setlocal enabledelayedexpansion

echo ===================================================
echo ROS2 Jazzy Installation using Pixi
echo ===================================================
echo.

:: Set installation directory
set "INSTALL_DIR=%USERPROFILE%\robostack"

:: Check if pixi is installed
where pixi >nul 2>&1
if errorlevel 1 (
    echo Pixi not found. Installing Pixi...
    powershell -Command "Invoke-Expression (Invoke-RestMethod -Uri https://pixi.sh/install.ps1)"
    
    :: Update PATH for current session
    set "PATH=%LOCALAPPDATA%\pixi\bin;%PATH%"
    echo Pixi installed successfully.
) else (
    echo Pixi is already installed.
)

:: Create installation directory if it doesn't exist
if not exist "%INSTALL_DIR%" (
    mkdir "%INSTALL_DIR%"
    echo Created directory: %INSTALL_DIR%
) else (
    echo Directory already exists: %INSTALL_DIR%
)

:: Navigate to installation directory
cd /d "%INSTALL_DIR%"

:: Initialize pixi project
echo Initializing RoboStack project...
pixi init robostack --quiet

:: Create pixi.toml configuration
echo Creating pixi.toml configuration...
(
echo [project]
echo name = "robostack"
echo version = "0.1.0"
echo description = "Development environment for RoboStack ROS packages"
echo authors = ["ROS2 User <%USERNAME%@example.com>"]
echo channels = ["https://fast.prefix.dev/conda-forge"]
echo platforms = ["win-64"]
echo.
echo [dependencies]
echo python = "*"
echo compilers = "*"
echo cmake = "*"
echo pkg-config = "*"
echo make = "*"
echo ninja = "*"
echo.
) > pixi.toml

:: Check for Visual Studio 2022
echo Checking for Visual Studio 2022...
if exist "C:\Program Files\Microsoft Visual Studio\2022" (
    echo Visual Studio 2022 found, adding support to pixi.toml
    (
    echo [target.win-64.dependencies]
    echo vs2022_win-64 = "*"
    echo.
    ) >> pixi.toml
) else (
    echo Visual Studio 2022 not found, continuing without VS support.
)

:: Add environments and features to pixi.toml
(
echo [environments]
echo jazzy = { features = ["jazzy"] }
echo.
echo [feature.jazzy]
echo channels = ["https://prefix.dev/robostack-jazzy"]
echo.
echo [feature.jazzy.dependencies]
echo ros-jazzy-desktop = "*"
echo colcon-common-extensions = "*"
echo rosdep = "*"
) >> pixi.toml

:: Install ROS2 Jazzy
echo Installing ROS2 Jazzy (this may take some time)...
pixi install

:: Create activation shortcut
echo @echo off > activate_ros2_jazzy.bat
echo cd /d "%INSTALL_DIR%" >> activate_ros2_jazzy.bat
echo pixi shell -e jazzy >> activate_ros2_jazzy.bat

echo.
echo ===================================================
echo Installation Complete!
echo ===================================================
echo.
echo To activate ROS2 Jazzy:
echo   1. Navigate to %INSTALL_DIR%
echo   2. Run: pixi shell -e jazzy
echo.
echo Or simply run: %INSTALL_DIR%\activate_ros2_jazzy.bat
echo.
echo To test your installation, run 'rviz2' after activating the environment.
echo To exit the environment, type 'exit' or press Ctrl+D.
echo.

:: Offer to activate the environment now
set /p ACTIVATE_NOW="Would you like to activate ROS2 Jazzy now? (Y/N): "
if /i "%ACTIVATE_NOW%"=="Y" (
    pixi shell -e jazzy
)

endlocal