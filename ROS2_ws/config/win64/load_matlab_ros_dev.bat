@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo MATLAB with ROS2 Environment - Multi-ROS Support
echo ========================================================
echo.

:: Get current working directory
set "WORK_DIR=%CD%"
echo Current directory: %WORK_DIR%

:: Find ROS2 workspace in common locations
echo Searching for ROS2 workspace...
SET FOUND_WORKSPACE=0
SET WORKSPACE_PATH=

:: Check common workspace paths
FOR %%W IN (
    "C:\ros2_ws"
    "C:\dev_ws"
    "C:\ros_ws"
    "C:\colcon_ws"
    "C:\workspace"
    "C:\ros2\workspace"
    "C:\users\%USERNAME%\ros2_ws"
    "C:\users\%USERNAME%\dev_ws"
    "C:\users\%USERNAME%\ros_ws"
    "%USERPROFILE%\ros2_ws"
    "%USERPROFILE%\dev_ws"
    "%USERPROFILE%\ros_ws"
    "C:\ros_jazzy"
    "C:\ros_humble"
    "C:\ros_foxy"
    "C:\ros_galactic"
    "C:\ros_iron"
    "C:\ros_rolling"
    "D:\ros2_ws"
    "D:\dev_ws"
) DO (
    IF EXIST "%%~W\install\setup.bat" (
        SET FOUND_WORKSPACE=1
        SET WORKSPACE_PATH=%%~W
        echo [✓] ROS2 workspace found at: %%~W
        GOTO :WORKSPACE_FOUND
    )
)

:WORKSPACE_FOUND
IF %FOUND_WORKSPACE%==1 (
    echo [✓] ROS2 workspace will be added to MATLAB environment
) ELSE (
    echo [!] No ROS2 workspace found in common locations.
    echo     If you have a workspace in a custom location, custom packages may not be available.
)

:: Find Visual Studio Developer Command Prompt
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

if "!VS_CMD_FOUND!"=="1" (
    echo [✓] Visual Studio Developer Command Prompt found at:
    echo     !VS_CMD_PATH!
) else (
    echo [✗] Visual Studio Developer Command Prompt not found.
    echo     Continuing without VS environment, which may cause issues.
)

:: Check for different ROS installation types
set "ROS_FOUND=0"
set "ROS_TYPE=none"
set "ROS_PATH="
set "ROS_BIN_PATH="
set "ROS_LIB_PATH="
set "ROS_DISTRO="

:: 1. Check for Robostack with pixi
if exist "%USERPROFILE%\robostack\activate_ros2_jazzy.bat" (
    set "ROS_PATH=%USERPROFILE%\robostack\activate_ros2_jazzy.bat"
    set "ROS_BIN_PATH=%USERPROFILE%\robostack\bin"
    set "ROS_LIB_PATH=%USERPROFILE%\robostack\lib"
    set "ROS_PYTHON_PATH=%USERPROFILE%\robostack\Lib\site-packages"
    set "ROS_PREFIX_PATH=%USERPROFILE%\robostack"
    set "ROS_FOUND=1"
    set "ROS_TYPE=robostack_pixi"
    set "ROS_DISTRO=jazzy"
)

:: 2. Check for standard ROS2 installation
if "!ROS_FOUND!"=="0" (
    for %%d in (humble iron jazzy rolling) do (
        if exist "C:\dev\ros2_%%d\setup.bat" (
            set "ROS_PATH=C:\dev\ros2_%%d\setup.bat"
            set "ROS_BIN_PATH=C:\dev\ros2_%%d\bin"
            set "ROS_LIB_PATH=C:\dev\ros2_%%d\lib"
            set "ROS_PYTHON_PATH=C:\dev\ros2_%%d\Lib\site-packages"
            set "ROS_PREFIX_PATH=C:\dev\ros2_%%d"
            set "ROS_FOUND=1"
            set "ROS_TYPE=standard"
            set "ROS_DISTRO=%%d"
        )
    )
)

:: 3. Check for ROS2 installation with local_setup.bat
if "!ROS_FOUND!"=="0" (
    for %%d in (humble iron jazzy rolling) do (
        if exist "C:\opt\ros\%%d\local_setup.bat" (
            set "ROS_PATH=C:\opt\ros\%%d\local_setup.bat"
            set "ROS_BIN_PATH=C:\opt\ros\%%d\bin"
            set "ROS_LIB_PATH=C:\opt\ros\%%d\lib"
            set "ROS_PYTHON_PATH=C:\opt\ros\%%d\Lib\site-packages"
            set "ROS_PREFIX_PATH=C:\opt\ros\%%d"
            set "ROS_FOUND=1"
            set "ROS_TYPE=windows_install"
            set "ROS_DISTRO=%%d"
        )
    )
)

if "!ROS_FOUND!"=="1" (
    echo [✓] ROS2 environment found:
    echo     Type: !ROS_TYPE!
    echo     Path: !ROS_PATH!
    echo     Distro: !ROS_DISTRO!
) else (
    echo [✗] ROS2 environment not found.
    echo     Cannot proceed without ROS environment.
    goto :EOF
)

:: Find MATLAB - use most recent version
set "MATLAB_FOUND=0"
set "MATLAB_PATH="
set "LATEST_VER=0"
set "LATEST_PATH="

:: Look for MATLAB installations and find the latest one
for /d %%i in ("C:\Program Files\MATLAB\R*") do (
    if exist "%%i\bin\matlab.exe" (
        set "TEMP_VER=%%~nxi"
        set "TEMP_VER=!TEMP_VER:~1!"
        
        if !TEMP_VER! GTR !LATEST_VER! (
            set "LATEST_VER=!TEMP_VER!"
            set "LATEST_PATH=%%i\bin\matlab.exe"
            set "MATLAB_FOUND=1"
        )
    )
)

:: Also check x86 program files (less common)
for /d %%i in ("C:\Program Files (x86)\MATLAB\R*") do (
    if exist "%%i\bin\matlab.exe" (
        set "TEMP_VER=%%~nxi"
        set "TEMP_VER=!TEMP_VER:~1!"
        
        if !TEMP_VER! GTR !LATEST_VER! (
            set "LATEST_VER=!TEMP_VER!"
            set "LATEST_PATH=%%i\bin\matlab.exe"
            set "MATLAB_FOUND=1"
        )
    )
)

if "!MATLAB_FOUND!"=="1" (
    set "MATLAB_PATH=!LATEST_PATH!"
    echo [✓] Latest MATLAB found at:
    echo     !MATLAB_PATH!
) else (
    echo [✗] MATLAB not found. Cannot proceed.
    goto :EOF
)


:: Create the startup file directly
set "STARTUP_FILE=%TEMP%\ros_startup_%RANDOM%.m"

:: Write MATLAB script content directly to the file
(
echo %% ROS2 Environment Setup
echo disp^('Setting up ROS2 environment variables...'^);
echo.
echo %% Set environment variables for ROS2
echo setenv^('PATH', [getenv^('PATH'^) ';%ROS_BIN_PATH%;%ROS_LIB_PATH%']^);
echo setenv^('PYTHONPATH', [getenv^('PYTHONPATH'^) ';%ROS_PYTHON_PATH%']^);
echo setenv^('AMENT_PREFIX_PATH', '%ROS_PREFIX_PATH%'^);
echo setenv^('ROS_DISTRO', '%ROS_DISTRO%'^);
echo.
) > "%STARTUP_FILE%"

:: Add workspace setup if found
if "%FOUND_WORKSPACE%"=="1" (
    (
    echo %% Setup ROS2 Workspace
    echo disp^('Setting up ROS2 workspace: %WORKSPACE_PATH%'^);
    echo setenv^('ROS_WORKSPACE', '%WORKSPACE_PATH%'^);
    echo addpath^(genpath^('%WORKSPACE_PATH%\build'^)^);
    echo disp^('Added workspace to path: %WORKSPACE_PATH%\install'^);
    echo if exist^('%WORKSPACE_PATH%\install\share', 'dir'^)
    echo     setenv^('AMENT_PREFIX_PATH', [getenv^('AMENT_PREFIX_PATH'^) ';%WORKSPACE_PATH%\install']^);
    echo     disp^('Added workspace to AMENT_PREFIX_PATH'^);
    echo end
    echo disp^('ROS2 workspace setup complete.'^);
    echo.
    ) >> "%STARTUP_FILE%" 
)

:: Add verification section
(
echo %% Verify setup
echo disp^(['ROS_DISTRO: ', getenv^('ROS_DISTRO'^)]^);
echo disp^(['PATH includes ROS: ', num2str^(contains^(getenv^('PATH'^), '%ROS_DISTRO%'^)^)]^);
) >> "%STARTUP_FILE%"

if "%FOUND_WORKSPACE%"=="1" (
    (
    echo disp^(['ROS_WORKSPACE: ', getenv^('ROS_WORKSPACE'^)]^);
    ) >> "%STARTUP_FILE%"
)

:: Add final section
(
echo.
echo %% Change to original working directory and refresh path cache
echo cd^('%WORK_DIR%'^);
echo rehash;
echo disp^('ROS2 environment setup complete.'^);
echo disp^(['Current directory: ', pwd]^);
) >> "%STARTUP_FILE%" 

echo Created startup file: !STARTUP_FILE!
echo.

:: Handle different ROS installation types
if "!ROS_TYPE!"=="robostack_pixi" (
    echo ========================================================
    echo Robostack with pixi detected
    echo ========================================================
    echo.
    
    :: Create a launcher for pixi with simple unique name
    set "LAUNCHER=%TEMP%\matlab_ros_launcher_%RANDOM%.bat"
    
    (
    echo @echo off
    echo title MATLAB with ROS2 Environment - Pixi Shell
    if "!VS_CMD_FOUND!"=="1" (
        echo echo Loading Visual Studio environment...
        echo call "!VS_CMD_PATH!"
        echo echo.
    )
    echo echo Changing to ROS directory...
    echo cd /d "!ROS_PREFIX_PATH!"
    echo echo.
    echo echo Activating pixi shell...
    echo echo.
    echo echo ========================================================
    echo echo IMPORTANT: Run these exact commands after the pixi shell loads:
    echo echo ========================================================
    echo echo.
    echo echo cd "!WORK_DIR!"
    echo echo "!MATLAB_PATH!" -r "run('!STARTUP_FILE!')"
    echo echo.
    echo echo ========================================================
    echo echo Starting pixi shell now...
    echo echo ========================================================
    echo echo.
    echo pixi shell -e jazzy
    ) > "!LAUNCHER!"
    
    echo Created pixi launcher: !LAUNCHER!
    echo.
    echo ========================================================
    echo Executing launcher script for pixi...
    echo ========================================================
    echo This will open the pixi shell with the ROS environment.
    echo You will need to manually start MATLAB once the environment loads.
    echo The command to start MATLAB will be displayed in the window.
    echo.
    
    :: Run the launcher
    start "" "!LAUNCHER!"
) else (
    :: For standard ROS installations, we can load and then launch MATLAB directly
    echo ========================================================
    echo Standard ROS2 installation detected
    echo ========================================================
    echo.
    echo Creating direct launcher that will automatically start MATLAB...
    
    set "DIRECT_LAUNCHER=%TEMP%\matlab_ros_direct_%RANDOM%.bat"
    
    (
    echo @echo off
    if "!VS_CMD_FOUND!"=="1" (
        echo echo Loading Visual Studio environment...
        echo call "!VS_CMD_PATH!"
        echo echo.
    )
    echo echo Loading ROS2 !ROS_DISTRO! environment...
    echo call "!ROS_PATH!"
    echo echo.
    if "%FOUND_WORKSPACE%"=="1" (
        echo echo Loading ROS2 workspace from !WORKSPACE_PATH!...
        echo call "!WORKSPACE_PATH!\install\setup.bat"
        echo echo.
    )
    echo echo Starting MATLAB with ROS2 environment...
    echo "!MATLAB_PATH!" -r "run('!STARTUP_FILE!')"
    ) > "!DIRECT_LAUNCHER!"
    
    echo Created direct launcher: !DIRECT_LAUNCHER!
    echo.
    echo ========================================================
    echo Executing direct launcher...
    echo ========================================================
    echo.
    
    :: Run the direct launcher
    start "" "!DIRECT_LAUNCHER!"
)

echo.
echo Instructions for future use:
echo.

if "!ROS_TYPE!"=="robostack_pixi" (
    echo For Robostack with pixi:
    echo 1. Open Visual Studio Developer Command Prompt
    echo 2. Run: cd /d "!ROS_PREFIX_PATH!"
    echo 3. Run: pixi shell -e jazzy
    echo 4. Run: "!MATLAB_PATH!" -r "run('!STARTUP_FILE!')"
) else (
    echo For standard ROS2 installation:
    echo 1. Open Visual Studio Developer Command Prompt
    echo 2. Run: call "!ROS_PATH!"
    if "%FOUND_WORKSPACE%"=="1" (
        echo 3. Run: call "!WORKSPACE_PATH!\install\setup.bat"
        echo 4. Run: "!MATLAB_PATH!" -r "run('!STARTUP_FILE!')"
    ) else (
        echo 3. Run: "!MATLAB_PATH!" -r "run('!STARTUP_FILE!')"
    )
)

echo.
echo You can also re-run this script whenever you need to launch MATLAB with ROS.
echo.

endlocal