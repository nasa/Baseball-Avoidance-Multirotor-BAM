@echo off
setlocal enabledelayedexpansion

REM ########################################################################
REM # BAM Simulink Model Runner - Windows Version
REM ########################################################################
REM # 
REM # This script runs the BAM Simulink model with MATLAB in command-line mode.
REM # It handles setting up the necessary paths, executing the model, and
REM # providing flexible output options.
REM #
REM # Author: Newton Campbell
REM # Email: newton.h.campbell@nasa.gov
REM # 
REM # Newton Campbell is a Computer Engineer at NASA Goddard Space Flight Center
REM # with expertise in artificial intelligence, machine learning, and robotics.
REM # He works on autonomous systems and intelligent robotics for space applications.
REM # His research interests include human-robot interaction, machine learning,
REM # and autonomous systems.
REM #
REM # Usage:
REM # This script allows you to run the BAM Simulink model with different
REM # ROS workspace paths and output options. It can display output to the
REM # console, save to a log file, or both.
REM #
REM # Dependencies:
REM # - MATLAB with Simulink
REM # - ROS environment
REM # - BAM.slx Simulink model
REM # - mat_airsim_pub package
REM #
REM ########################################################################

REM Initialize variables
set "ROS_WORKSPACE="
set "OUTPUT_OPTION=--console"
set "LOG_FILE="
set "MATLAB_FOUND=0"
set "MATLAB_PATH="
set "ROS_TYPE=none"

REM Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

REM Parse command line arguments
:parse_args
if "%~1"=="" goto :end_parse_args

set "ARG=%~1"
if "%ARG:~0,2%"=="-w" (
    if "%ARG:~2,1%"=="=" (
        set "ROS_WORKSPACE=%ARG:~3%"
    ) else (
        set "ROS_WORKSPACE=%~2"
        shift
    )
) else if "%ARG:~0,11%"=="--workspace" (
    if "%ARG:~11,1%"=="=" (
        set "ROS_WORKSPACE=%ARG:~12%"
    ) else (
        set "ROS_WORKSPACE=%~2"
        shift
    )
) else if "%ARG%"=="-h" (
    goto :show_usage
) else if "%ARG%"=="--help" (
    goto :show_usage
)

shift
goto :parse_args

:end_parse_args

REM Check if ROS_WS_HOME is set or argument is provided, otherwise prompt user
if "%ROS_WORKSPACE%"=="" (
    if not "%ROS_WS_HOME%"=="" (
        set "ROS_WORKSPACE=%ROS_WS_HOME%"
        echo Using ROS workspace from environment variable: %ROS_WORKSPACE%
    ) else (
        REM Try to auto-detect
        if exist "C:\ros2_ws" (
            set "ROS_WORKSPACE=C:\ros2_ws"
            echo Auto-detected ROS workspace at: %ROS_WORKSPACE%
        ) else if exist "%USERPROFILE%\ros2_ws" (
            set "ROS_WORKSPACE=%USERPROFILE%\ros2_ws"
            echo Auto-detected ROS workspace at: %ROS_WORKSPACE%
        ) else (
            REM Prompt user for workspace path
            set /p ROS_WORKSPACE="Please enter the path to your ROS workspace: "
            
            REM Validate input
            if "%ROS_WORKSPACE%"=="" (
                echo Error: No workspace path provided. Exiting.
                exit /b 1
            )
        )
    )
)

REM Verify the workspace directory exists
if not exist "%ROS_WORKSPACE%" (
    echo Warning: The specified workspace directory does not exist: %ROS_WORKSPACE%
    set /p CONTINUE="Do you want to continue anyway? (y/n): "
    if /i not "!CONTINUE!"=="y" (
        echo Operation cancelled.
        exit /b 1
    )
)

echo Using ROS workspace: %ROS_WORKSPACE%
echo Script directory: %SCRIPT_DIR%

REM Check for ROS2 environment setup script
set "ROS2_SETUP=%SCRIPT_DIR%\ROS2\win64\load_ros2_dev.bat"
if exist "%ROS2_SETUP%" (
    echo Found ROS2 setup script at: %ROS2_SETUP%
) else (
    echo Warning: ROS2 setup script not found at: %ROS2_SETUP%
    echo Will continue without loading ROS2 environment.
)

REM Check for different ROS installation types
set "ROS_FOUND=0"

REM 1. Check for Robostack with pixi
if exist "%USERPROFILE%\robostack\activate_ros2_jazzy.bat" (
    set "ROS_PATH=%USERPROFILE%\robostack\activate_ros2_jazzy.bat"
    set "ROS_FOUND=1"
    set "ROS_TYPE=robostack_pixi"
    set "ROS_DISTRO=jazzy"
    echo [✓] Robostack pixi environment found at: %ROS_PATH%
)

REM Check for MATLAB on PATH first
where matlab >nul 2>&1
if %ERRORLEVEL% EQU 0 (
    set "MATLAB_FOUND=1"
    for /f "tokens=*" %%i in ('where matlab') do set "MATLAB_PATH=%%i"
    echo Found MATLAB on PATH: %MATLAB_PATH%
) else (
    REM Try to find MATLAB in common locations
    for %%V in (R2024a R2024b R2023b R2023a R2022b R2022a R2021b R2021a) do (
        if exist "C:\Program Files\MATLAB\%%V\bin\matlab.exe" (
            set "MATLAB_FOUND=1"
            set "MATLAB_PATH=C:\Program Files\MATLAB\%%V\bin\matlab.exe"
            echo Found MATLAB at: !MATLAB_PATH!
            goto :matlab_found
        )
    )
    
    echo Error: MATLAB not found. Please ensure MATLAB is installed and in your PATH.
    exit /b 1
)

:matlab_found

REM Create a MATLAB script file in the Windows temp directory
set "TEMP_DIR=%TEMP%"
set "TEMP_SCRIPT=%TEMP_DIR%\run_bam_temp_%RANDOM%.m"
echo Creating temporary MATLAB script at: %TEMP_SCRIPT%

REM Convert backslashes to forward slashes for MATLAB paths
set "SCRIPT_DIR_MATLAB=%SCRIPT_DIR:\=/%"
set "ROS_WORKSPACE_MATLAB=%ROS_WORKSPACE:\=/%"

REM Create the MATLAB script using a more reliable method
(
echo %% BAM Simulink Model Runner - Generated MATLAB Script
echo %% Created: %DATE% %TIME%
echo.
echo %% First change to the script directory to ensure proper file access
echo cd('%SCRIPT_DIR_MATLAB%'^);
echo disp('Changed working directory to:'^);
echo pwd
echo.
echo %% Add the script directory and all subdirectories to path
echo addpath(genpath('%SCRIPT_DIR_MATLAB%'^)^);
echo.
echo %% Add the ROS workspace path
echo addpath(genpath('%ROS_WORKSPACE_MATLAB%/install'^)^);
echo.
echo %% Display current directory and path for debugging
echo disp('Current directory:'^);
echo pwd
echo.
echo %% Configure for ROS2 and run setup
echo disp('Configuring for ROS2...'^);
echo.
echo % Run the setup script
echo setup;
echo.
echo % Configure variants
echo userStruct.variants.pubType = PubEnum.ROS2_NONLIB; % Use ROS2 for publishing
echo userStruct.variants.pubTypeBball = PubEnum.ROS2_NONLIB; % Turn on the baseball
echo.
echo % Specify the desired own-ship trajectory and the baseball trajectory
echo % numbers to execute in simulation and initialize the userStruct with their
echo % BP waypoints.
echo own_traj_num = 50; % Select desired own-ship traj number
echo bball_traj_num = 50; % Select desired bball traj number
echo.
echo % Run setup again
echo setup;
echo.
echo.
echo %% Display confirmation message
echo disp('Path added and setup completed successfully.'^);
echo.
echo %% Run the Simulink model
echo disp('Starting BAM.slx Simulink model...'^);
echo open_system('BAM'^);
echo sim('BAM'^);
echo disp('Simulation completed.'^);
echo.
echo %% Exit MATLAB
echo exit;
) > "%TEMP_SCRIPT%"

REM Verify the script was created
if not exist "%TEMP_SCRIPT%" (
    echo Error: Failed to create MATLAB script at %TEMP_SCRIPT%
    exit /b 1
)

REM Display the first few lines of the script to verify content
echo.
echo Verifying MATLAB script content:
type "%TEMP_SCRIPT%" | findstr /C:"cd" /C:"addpath" /C:"setup" /C:"BAM"
echo.

REM Run MATLAB from the script directory
cd /d "%SCRIPT_DIR%"
echo Current directory before running MATLAB: %CD%

REM Handle different ROS installation types
if "%ROS_TYPE%"=="robostack_pixi" (
    echo ========================================================
    echo Robostack with pixi detected
    echo ========================================================
    echo.
    echo To run the BAM simulation with Robostack:
    echo.
    echo 1. Open a new command prompt
    echo 2. Run the following commands:
    echo.
    echo    cd /d "%SCRIPT_DIR%"
    echo    "%MATLAB_PATH%" -nosplash -nodesktop -r "run('%TEMP_SCRIPT:\=/%')"
    echo.
    echo ========================================================
    echo.
    echo The MATLAB script has been created at: %TEMP_SCRIPT%
    echo You must manually run MATLAB with the Robostack environment activated.
) else (
    REM Load ROS2 environment if the setup script exists
    if exist "%ROS2_SETUP%" (
        echo Loading ROS2 environment...
        call "%ROS2_SETUP%"
        echo ROS2 environment loaded.
    )
    
    echo Running MATLAB...
    "%MATLAB_PATH%" -nodisplay -nosplash -nodesktop -r "run('%TEMP_SCRIPT:\=/%')"
    
    echo MATLAB execution completed.
)

echo Temporary script location (not removed): %TEMP_SCRIPT%

goto :eof

:show_usage
echo Usage: %~nx0 [OPTIONS]
echo Options:
echo   -w, --workspace=PATH  : Path to your ROS workspace
echo   -h, --help            : Show this help message
echo.
echo If no workspace is specified, the script will use the ROS_WS_HOME environment
echo variable if set, or prompt you to enter a path.
echo.
echo Examples:
echo   %~nx0 --workspace=C:\path\to\ros_ws
echo   %~nx0 -w C:\path\to\ros_ws

endlocal