@echo off
setlocal enabledelayedexpansion

::=============================================================================
:: install_bam_ros_packages.bat
::
:: Author: Newton Campbell
:: Date: 2025-04-17
::
:: Description:
::   This script installs BAM (Baseball Avoidance MultiRotor) ROS2 packages 
::   into a ROS workspace. It copies packages from the BAM repository 
::   to the ROS workspace, builds the packages using colcon, and sources the
::   workspace setup file.
::
:: Usage:
::   install_bam_ros_packages.bat [-w|--workspace <ros_workspace_path>]
::
:: Options:
::   -w, --workspace    Path to ROS2 workspace (optional)
::   -h, --help         Display help message
::
:: If no workspace is specified, the script will attempt to find an existing
:: ROS2 workspace in common locations. If no workspace is found, it will create
:: one under the script directory with the name "ros_ws".
::=============================================================================

:: Store the current directory to return to it at the end
set "CURRENT_DIR=%CD%"

:: Get the directory where this script is located
set "SCRIPT_DIR=%~dp0"
:: Remove trailing backslash
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

:: Define the root directory of the BAM project (assuming this script is in ROS2_ws\config\win64)
echo Script directory is: %SCRIPT_DIR%
for %%A in ("%SCRIPT_DIR%\..\..\..\") do set "BAM_ROOT_DIR=%%~fA"
echo BAM_ROOT_DIR is: %BAM_ROOT_DIR%

:: Verify the directory structure
if not exist "%BAM_ROOT_DIR%\ROS2_ws" (
    echo ERROR: ROS2 directory not found at %BAM_ROOT_DIR%\ROS2_ws
    echo Please check directory structure.
    exit /b 1
)

::=============================================================================
:: Parse command line arguments
::=============================================================================
set "ROS_WS_DIR="
set "SHOW_HELP="

:parse_args
if "%~1"=="" goto :args_done
if /i "%~1"=="-w" (
    set "ROS_WS_DIR=%~2"
    shift
    shift
    goto :parse_args
)
if /i "%~1"=="--workspace" (
    set "ROS_WS_DIR=%~2"
    shift
    shift
    goto :parse_args
)
if /i "%~1"=="-h" (
    set "SHOW_HELP=1"
    shift
    goto :parse_args
)
if /i "%~1"=="--help" (
    set "SHOW_HELP=1"
    shift
    goto :parse_args
)
echo Unknown option: %~1
set "SHOW_HELP=1"
shift
goto :parse_args

:args_done

:: Display help if requested
if defined SHOW_HELP (
    echo Usage: %~nx0 [-w^|--workspace ^<ros_workspace_path^>]
    echo   -w, --workspace    Path to ROS2 workspace ^(optional^)
    echo   -h, --help         Display this help message
    echo.
    echo If no workspace is specified, the script will attempt to find it automatically.
    echo If no workspace is found, it will create one under the script directory.
    exit /b 0
)

::=============================================================================
:: Workspace detection and creation
::=============================================================================
:: If no workspace specified, try to find it
if not defined ROS_WS_DIR (
    echo No workspace specified, attempting to find ROS2 workspace...
    
    :: Check common workspace locations with common naming patterns
    set "FOUND_WS="
    
    :: List of potential workspace locations
    :: Include common Windows-specific locations like C:\ros2_ws
    set "POTENTIAL_WS[0]=%SCRIPT_DIR%\ros2_ws"
    set "POTENTIAL_WS[1]=%SCRIPT_DIR%\ros_ws"
    set "POTENTIAL_WS[2]=%USERPROFILE%\ros2_ws"
    set "POTENTIAL_WS[3]=%USERPROFILE%\ros_ws"
    set "POTENTIAL_WS[4]=%USERPROFILE%\ros_jazzy"
    set "POTENTIAL_WS[5]=%USERPROFILE%\ros_humble"
    set "POTENTIAL_WS[6]=%USERPROFILE%\ros_iron"
    set "POTENTIAL_WS[7]=%USERPROFILE%\ros_rolling"
    set "POTENTIAL_WS[8]=%BAM_ROOT_DIR%\ros2_ws"
    set "POTENTIAL_WS[9]=%BAM_ROOT_DIR%\ros_ws"
    set "POTENTIAL_WS[10]=C:\ros2_ws"
    set "POTENTIAL_WS[11]=C:\ros_ws"
    set "POTENTIAL_WS[12]=C:\ros_jazzy"
    set "POTENTIAL_WS[13]=C:\ros_humble"
    set "POTENTIAL_WS[14]=C:\ros_iron"
    set "POTENTIAL_WS[15]=C:\ros_rolling"
    set "POTENTIAL_WS[16]=C:\dev\ros2_ws"
    set "POTENTIAL_WS[17]=C:\dev\ros_ws"
    
    :: Check each potential workspace location
    for /L %%i in (0,1,17) do (
        set "WS=!POTENTIAL_WS[%%i]!"
        if exist "!WS!\src" (
            echo Checking workspace: !WS!
            if exist "!WS!\src\COLCON_IGNORE" (
                set "ROS_WS_DIR=!WS!"
                set "FOUND_WS=1"
                echo Found ROS2 workspace at: !WS!
                goto :workspace_found
            ) else if exist "!WS!\src\CMakeLists.txt" (
                set "ROS_WS_DIR=!WS!"
                set "FOUND_WS=1"
                echo Found ROS2 workspace at: !WS!
                goto :workspace_found
            ) else if exist "!WS!\build" (
                set "ROS_WS_DIR=!WS!"
                set "FOUND_WS=1"
                echo Found ROS2 workspace at: !WS!
                goto :workspace_found
            )
        )
    )
    
    :: If still not found, use default location
    set "ROS_WS_DIR=%SCRIPT_DIR%\ros_ws"
    echo No existing ROS2 workspace found, creating one at: %ROS_WS_DIR%
) else (
    echo Using specified ROS2 workspace: %ROS_WS_DIR%
)

:workspace_found

:: Create the src directory if it doesn't exist
if not exist "%ROS_WS_DIR%" (
    echo Creating workspace directory: %ROS_WS_DIR%
    mkdir "%ROS_WS_DIR%"
)

if not exist "%ROS_WS_DIR%\src" (
    echo Creating src directory: %ROS_WS_DIR%\src
    mkdir "%ROS_WS_DIR%\src"
)

echo Installing BAM ROS2 packages into workspace: %ROS_WS_DIR%

::=============================================================================
:: Create package links/copies for all required packages
::=============================================================================
echo Creating package links for BAM ROS2 packages...

:: Function to create a package link (shortcut or copy)
:: Usage: call :create_package_link <source_dir>
goto :after_create_package_link_function

:create_package_link
set "SOURCE_DIR=%~1"

:: Normalize the path separators
set "SOURCE_DIR=%SOURCE_DIR:/=\%"
set "SOURCE_DIR=%SOURCE_DIR:\\=\%"

if "%SOURCE_DIR%"=="" (
    echo ERROR: Empty source directory passed to create_package_link
    exit /b 1
)

:: Remove any trailing backslashes
if "%SOURCE_DIR:~-1%"=="\" set "SOURCE_DIR=%SOURCE_DIR:~0,-1%"

for %%F in ("%SOURCE_DIR%") do set "PACKAGE_NAME=%%~nxF"
set "TARGET_DIR=%ROS_WS_DIR%\src\%PACKAGE_NAME%"

:: Create symbolic links (junctions) for packages; if it fails, 
:: it'll copy the package directories over
if exist "%SOURCE_DIR%\" (
    echo Creating link for package: %PACKAGE_NAME% (%SOURCE_DIR% -^> %TARGET_DIR%)
    
    :: Remove existing directory/symlink if it exists
    if exist "%TARGET_DIR%" (
        echo Removing existing target: %TARGET_DIR%
        rmdir /S /Q "%TARGET_DIR%"
    )
    
    :: Try to create a junction point
    mklink /J "%TARGET_DIR%" "%SOURCE_DIR%" >nul 2>&1
    
    if errorlevel 1 (
        echo Junction creation failed, falling back to directory copy...
        
        :: Fall back to straight file copy
        echo Copying directory contents...
        xcopy /E /I /Y "%SOURCE_DIR%" "%TARGET_DIR%"
        
        :: Create a marker file to indicate this is a copy
        echo This is a copied package directory, not a link > "%TARGET_DIR%\.copied_package"
        
        echo Package copied successfully.
    ) else (
        echo Successfully created junction point for: %TARGET_DIR%
    )
) else (
    echo Warning: Source directory not found: %SOURCE_DIR%
    echo Current directory is: "%CD%"

)

exit /b

:after_create_package_link_function

:: Analysis packages - Contains algorithms for analyzing simulation data
echo Processing analysis packages...
if exist "%BAM_ROOT_DIR%\ROS2_ws\analysis_pkgs" (
    for /D %%D in ("%BAM_ROOT_DIR%\ROS2_ws\analysis_pkgs\*") do (
        if exist "%%D" (
            echo Found package: %%D
            call :create_package_link "%%D"
        )
    )
) else (
    echo Directory not found: "%BAM_ROOT_DIR%\ROS2_ws\analysis_pkgs" - Skipping
)

:: Interface packages - Contains ROS2 message and service definitions
echo Processing interface packages...
if exist "%BAM_ROOT_DIR%\ROS2_ws\interfaces_pkgs" (
    for /D %%D in ("%BAM_ROOT_DIR%\ROS2_ws\interfaces_pkgs\*") do (
        if exist "%%D" (
            echo Found package: %%D
            call :create_package_link "%%D"
        )
    )
) else (
    echo Directory not found: "%BAM_ROOT_DIR%\ROS2_ws\interfaces_pkgs" - Skipping
)

:: Launch packages - Contains ROS2 launch files
echo Processing launch packages...
if exist "%BAM_ROOT_DIR%\ROS2_ws\launch_pkgs" (
    for /D %%D in ("%BAM_ROOT_DIR%\ROS2_ws\launch_pkgs\*") do (
        if exist "%%D" (
            echo Found package: %%D
            call :create_package_link "%%D"
        )
    )
) else (
    echo Directory not found: "%BAM_ROOT_DIR%\ROS2_ws\launch_pkgs" - Skipping
)

:: Publisher packages
echo Processing publisher packages...
if exist "%BAM_ROOT_DIR%\ROS2_ws\s_function_pkgs\publishers" (
    for /D %%D in ("%BAM_ROOT_DIR%\ROS2_ws\s_function_pkgs\publishers\*") do (
        if exist "%%D" (
            echo Found package: %%D
            call :create_package_link "%%D"
        )
    )
) else (
    echo Directory not found: "%BAM_ROOT_DIR%\ROS2_ws\s_function_pkgs\publishers" - Skipping
)

:: Subscriber packages
echo Processing subscriber packages...
if exist "%BAM_ROOT_DIR%\ROS2_ws\s_function_pkgs\subscribers" (
    for /D %%D in ("%BAM_ROOT_DIR%\ROS2_ws\s_function_pkgs\subscribers\*") do (
        if exist "%%D" (
            echo Found package: %%D
            call :create_package_link "%%D"
        )
    )
) else (
    echo Directory not found: "%BAM_ROOT_DIR%\ROS2_ws\s_function_pkgs\subscribers" - Skipping
)

:: Visualization packages - Contains nodes for visualizing simulation data
echo Processing visualization packages...
if exist "%BAM_ROOT_DIR%\ROS2_ws\visualization_pkgs" (
    for /D %%D in ("%BAM_ROOT_DIR%\ROS2_ws\visualization_pkgs\*") do (
        if exist "%%D" (
            echo Found package: %%D
            call :create_package_link "%%D"
        )
    )
) else (
    echo Directory not found: "%BAM_ROOT_DIR%\ROS2_ws\visualization_pkgs" - Skipping
)

::=============================================================================
:: Build the workspace using colcon
::=============================================================================
echo Building ROS2 workspace at %ROS_WS_DIR%...
cd /d "%ROS_WS_DIR%"

:: Build with merge-install to simplify the install space structure
:: and disable autocoding for all packages
echo Running: colcon build --merge-install --cmake-args -DDISABLE_AUTOCODING=ON
call colcon build --merge-install --cmake-args -DDISABLE_AUTOCODING=ON
if errorlevel 1 (
    echo Error: Build failed with errorlevel %errorlevel%
    echo Note: If the build failed due to autocoding issues, try running without the DISABLE_AUTOCODING flag:
    echo colcon build --merge-install
) else (
    echo Build completed successfully
)

::=============================================================================
:: Source the workspace setup file
::=============================================================================
echo Sourcing the built workspace...
if exist "%ROS_WS_DIR%\install\setup.bat" (
    call "%ROS_WS_DIR%\install\setup.bat"
    echo Workspace has been sourced. ROS2 packages are now available.
) else (
    echo Warning: setup.bat not found. Build may have failed.
)

::=============================================================================
:: Return to the original directory and display completion message
::=============================================================================
:: Return to the original directory where the script was called from
cd /d "%CURRENT_DIR%"

echo BAM ROS2 packages installation complete!
echo Workspace location: %ROS_WS_DIR%
echo The workspace has been sourced in this terminal session.
echo For new terminal sessions, run: call "%ROS_WS_DIR%\install\setup.bat"
echo.
echo =====================================================================
echo You can now run the BAM system from any directory using:
echo ros2 launch bam_launcher bam_launch.py
echo =====================================================================

endlocal