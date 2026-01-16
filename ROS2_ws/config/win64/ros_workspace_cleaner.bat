@echo off
setlocal enabledelayedexpansion

::==========================================================================
:: ROS Workspace Cleaner
:: 
:: Author: Newton Campbell
:: Created: April 25, 2025
::
:: Description:
::   This batch script provides a user-friendly way to clean ROS workspaces
::   by removing build, install, and log directories. Its useful for cleanly
::   testing ROS nodes against the simulator. It includes automatic
::   workspace detection in common locations with comprehensive naming patterns,
::   manual path entry, and safeguards to prevent accidental data loss.
::   Also offers automatic rebuilding of the workspace after cleaning.
::
:: Usage:
::   ros_workspace_cleaner.bat [options]
::
:: Options:
::   -w, --workspace PATH   Specify ROS workspace path directly
::   -h, --help, /?         Show this help message
::==========================================================================

title ROS Workspace Cleaner

:: Display welcome banner
echo ========================================================
echo             ROS Workspace Cleaner Utility
echo                
echo ========================================================
echo.

:: -------------------------------------------------------------
:: Command-line argument handling
:: -------------------------------------------------------------

:: Initialize variables for workspace specification
set "SPECIFIED_WORKSPACE="
set "SKIP_SEARCH=0"

:: Check for command line arguments
if not "%~1"=="" (
    if /i "%~1"=="-w" (
        :: -w flag requires a workspace path
        if not "%~2"=="" (
            set "SPECIFIED_WORKSPACE=%~2"
            set "SKIP_SEARCH=1"
            echo Using specified workspace: %SPECIFIED_WORKSPACE%
        ) else (
            echo [ERROR] Workspace path must be provided with -w flag.
            goto :usage
        )
    ) else if /i "%~1"=="--workspace" (
        :: --workspace flag requires a workspace path
        if not "%~2"=="" (
            set "SPECIFIED_WORKSPACE=%~2"
            set "SKIP_SEARCH=1"
            echo Using specified workspace: %SPECIFIED_WORKSPACE%
        ) else (
            echo [ERROR] Workspace path must be provided with --workspace flag.
            goto :usage
        )
    ) else if /i "%~1"=="/?" (
        :: Windows-style help flag
        goto :usage
    ) else if /i "%~1"=="-h" (
        :: Unix-style help flag (short)
        goto :usage
    ) else if /i "%~1"=="--help" (
        :: Unix-style help flag (long)
        goto :usage
    ) else (
        :: Invalid parameter
        echo [ERROR] Unknown parameter: %~1
        goto :usage
    )
)

:: -------------------------------------------------------------
:: Workspace verification (when specified via command line)
:: -------------------------------------------------------------

:: If workspace is specified via arguments, verify it exists
if "%SKIP_SEARCH%"=="1" (
    :: Check if the directory exists
    if not exist "%SPECIFIED_WORKSPACE%" (
        echo [ERROR] The specified workspace directory does not exist: %SPECIFIED_WORKSPACE%
        echo Please check the path and try again.
        exit /b 1
    )
    
    :: Check if it has a src directory (basic validation for ROS workspace)
    if not exist "%SPECIFIED_WORKSPACE%\src" (
        echo [WARNING] The specified directory does not contain a 'src' folder.
        echo Are you sure this is a ROS workspace?
        choice /c YN /m "Continue anyway"
        if errorlevel 2 goto :eof
    )
    
    :: Use the specified workspace
    set "WORKSPACE_PATH=%SPECIFIED_WORKSPACE%"
    goto :confirm_workspace
)

:: -------------------------------------------------------------
:: Workspace auto-detection
:: -------------------------------------------------------------

echo Searching for ROS workspaces in common locations...
echo This may take a moment...
echo.

:: Initialize workspace tracking variables
set "WORKSPACE_FOUND=0"
set "WORKSPACE_INDEX=0"
set "WORKSPACE_COUNT=0"

:: Define common ROS distributions for comprehensive search
set "ROS1_DISTROS=melodic noetic"
set "ROS2_DISTROS=foxy galactic humble iron jazzy rolling"

:: Define common base directory locations to search
set "BASE_DIRS=%USERPROFILE% %HOMEDRIVE%%HOMEPATH% C:\ C:\dev C:\ros C:\work C:\workspace C:\users\%USERNAME% D:\ D:\dev D:\ros D:\workspace E:\"

:: Define common workspace naming patterns
set "WS_PATTERNS=workspace ros_ws ros2_ws dev_ws colcon_ws"

echo Searching with distribution-specific patterns...

:: Initialize array to store found workspaces
set "WORKSPACE_COUNT=0"

:: First pass: Search for distribution-specific workspace names
for %%b in (%BASE_DIRS%) do (
    :: ROS1 distribution-specific workspaces
    for %%d in (%ROS1_DISTROS%) do (
        if exist "%%b\ros_%%d" (
            if exist "%%b\ros_%%d\src" (
                set /a WORKSPACE_COUNT+=1
                set "WORKSPACE_PATHS[!WORKSPACE_COUNT!]=%%b\ros_%%d"
                set "WORKSPACE_NAMES[!WORKSPACE_COUNT!]=ros_%%d"
                echo Found: %%b\ros_%%d
            )
        )
    )
    
    :: ROS2 distribution-specific workspaces
    for %%d in (%ROS2_DISTROS%) do (
        if exist "%%b\ros2_%%d" (
            if exist "%%b\ros2_%%d\src" (
                set /a WORKSPACE_COUNT+=1
                set "WORKSPACE_PATHS[!WORKSPACE_COUNT!]=%%b\ros2_%%d"
                set "WORKSPACE_NAMES[!WORKSPACE_COUNT!]=ros2_%%d"
                echo Found: %%b\ros2_%%d
            )
        )
    )
    
    :: Generic workspace patterns
    for %%p in (%WS_PATTERNS%) do (
        if exist "%%b\%%p" (
            if exist "%%b\%%p\src" (
                set /a WORKSPACE_COUNT+=1
                set "WORKSPACE_PATHS[!WORKSPACE_COUNT!]=%%b\%%p"
                set "WORKSPACE_NAMES[!WORKSPACE_COUNT!]=%%p"
                echo Found: %%b\%%p
            )
        )
    )
    
    :: ROS1 with workspace suffix
    for %%d in (%ROS1_DISTROS%) do (
        if exist "%%b\%%d_ws" (
            if exist "%%b\%%d_ws\src" (
                set /a WORKSPACE_COUNT+=1
                set "WORKSPACE_PATHS[!WORKSPACE_COUNT!]=%%b\%%d_ws"
                set "WORKSPACE_NAMES[!WORKSPACE_COUNT!]=%%d_ws"
                echo Found: %%b\%%d_ws
            )
        )
    )
    
    :: ROS2 with workspace suffix
    for %%d in (%ROS2_DISTROS%) do (
        if exist "%%b\%%d_ws" (
            if exist "%%b\%%d_ws\src" (
                set /a WORKSPACE_COUNT+=1
                set "WORKSPACE_PATHS[!WORKSPACE_COUNT!]=%%b\%%d_ws"
                set "WORKSPACE_NAMES[!WORKSPACE_COUNT!]=%%d_ws"
                echo Found: %%b\%%d_ws
            )
        )
    )
)

echo.
echo Workspace search complete. Found %WORKSPACE_COUNT% potential workspaces.
echo.

:: If no workspaces were found, prompt for manual entry
if %WORKSPACE_COUNT% EQU 0 (
    echo [!] No ROS workspaces found in common locations.
    echo     Please specify a workspace path manually.
    echo.
    set /p MANUAL_PATH="Enter full path to your ROS workspace: "
    
    :: Validate manual entry
    if not exist "!MANUAL_PATH!" (
        echo [ERROR] The specified path does not exist: !MANUAL_PATH!
        exit /b 1
    )
    
    :: Warn if missing src directory
    if not exist "!MANUAL_PATH!\src" (
        echo [WARNING] The specified directory does not contain a 'src' folder.
        echo Are you sure this is a ROS workspace?
        choice /c YN /m "Continue anyway"
        if errorlevel 2 exit /b 1
    )
    
    set "WORKSPACE_PATH=!MANUAL_PATH!"
    goto :confirm_workspace
)

:: -------------------------------------------------------------
:: Workspace selection
:: -------------------------------------------------------------

:display_workspace
:: Increment the workspace index to show the next candidate
set /a WORKSPACE_INDEX+=1

:: Check if we've gone through all candidates
if %WORKSPACE_INDEX% GTR %WORKSPACE_COUNT% (
    echo [!] No more workspaces found.
    echo     Please specify a workspace path manually.
    echo.
    set /p MANUAL_PATH="Enter full path to your ROS workspace: "
    
    :: Validate manual entry
    if not exist "!MANUAL_PATH!" (
        echo [ERROR] The specified path does not exist: !MANUAL_PATH!
        exit /b 1
    )
    
    :: Warn if missing src directory
    if not exist "!MANUAL_PATH!\src" (
        echo [WARNING] The specified directory does not contain a 'src' folder.
        echo Are you sure this is a ROS workspace?
        choice /c YN /m "Continue anyway"
        if errorlevel 2 exit /b 1
    )
    
    set "WORKSPACE_PATH=!MANUAL_PATH!"
    goto :confirm_workspace
)

:: Get current workspace candidate details
set "WORKSPACE_PATH=!WORKSPACE_PATHS[%WORKSPACE_INDEX%]!"
set "WORKSPACE_NAME=!WORKSPACE_NAMES[%WORKSPACE_INDEX%]!"

:: Determine if it's likely ROS1 or ROS2 based on naming convention
set "WORKSPACE_TYPE=unknown"
if "!WORKSPACE_NAME:~0,4!"=="ros_" set "WORKSPACE_TYPE=ros1"
if "!WORKSPACE_NAME!"=="ros_ws" set "WORKSPACE_TYPE=ros1"
if "!WORKSPACE_NAME:~0,5!"=="ros2_" set "WORKSPACE_TYPE=ros2"
if "!WORKSPACE_NAME!"=="ros2_ws" set "WORKSPACE_TYPE=ros2"

:: Check for distribution-specific naming that might indicate type
for %%d in (%ROS1_DISTROS%) do (
    if "!WORKSPACE_NAME!"=="ros_%%d" set "WORKSPACE_TYPE=ros1"
    if "!WORKSPACE_NAME!"=="%%d_ws" set "WORKSPACE_TYPE=ros1"
)
for %%d in (%ROS2_DISTROS%) do (
    if "!WORKSPACE_NAME!"=="ros2_%%d" set "WORKSPACE_TYPE=ros2"
    if "!WORKSPACE_NAME!"=="%%d_ws" set "WORKSPACE_TYPE=ros2"
)

:: Display workspace information and options
echo --------------------------------------------------------
echo Found potential ROS workspace [%WORKSPACE_INDEX%/%WORKSPACE_COUNT%]:
echo   Path: !WORKSPACE_PATH!
echo   Name: !WORKSPACE_NAME!
if "!WORKSPACE_TYPE!"=="ros1" echo   Type: ROS1 (likely)
if "!WORKSPACE_TYPE!"=="ros2" echo   Type: ROS2 (likely)
if "!WORKSPACE_TYPE!"=="unknown" echo   Type: Unknown
echo --------------------------------------------------------
echo.
:: Check for build, install, log directories to show what would be cleaned
set "HAS_BUILD="
set "HAS_INSTALL="
set "HAS_LOG="
if exist "!WORKSPACE_PATH!\build" set "HAS_BUILD=Yes"
if not defined HAS_BUILD set "HAS_BUILD=No"
if exist "!WORKSPACE_PATH!\install" set "HAS_INSTALL=Yes"
if not defined HAS_INSTALL set "HAS_INSTALL=No"
if exist "!WORKSPACE_PATH!\log" set "HAS_LOG=Yes"
if not defined HAS_LOG set "HAS_LOG=No"

echo Directories present:
echo   - build:   !HAS_BUILD!
echo   - install: !HAS_INSTALL!
echo   - log:     !HAS_LOG!
echo.
echo [Y] Yes, use this workspace
echo [N] No, show the next workspace
echo [Q] Quit
echo.
choice /c YNQ /m "Use this workspace"

:: Handle user selection
if errorlevel 3 goto :eof            :: Q - Quit
if errorlevel 2 goto :display_workspace  :: N - Show next workspace
if errorlevel 1 goto :confirm_workspace  :: Y - Use this workspace

:: -------------------------------------------------------------
:: Workspace cleaning confirmation
:: -------------------------------------------------------------

:confirm_workspace
echo.
echo ========================================================
echo               WORKSPACE CLEANING CONFIRMATION
echo ========================================================
echo.
echo Workspace to clean: %WORKSPACE_PATH%
echo.
echo This will remove the following directories if they exist:
echo  - %WORKSPACE_PATH%\build    :: Build artifacts and CMake files
echo  - %WORKSPACE_PATH%\install  :: Installed executables and libraries
echo  - %WORKSPACE_PATH%\log      :: Build and runtime logs
echo.
echo These actions cannot be undone. Any compiled code and logs will be lost.
echo.
choice /c YN /m "Are you sure you want to proceed"

:: If user selects "No", abort the cleaning process
if errorlevel 2 (
    echo Operation cancelled by user.
    goto :eof
)

:: -------------------------------------------------------------
:: Workspace cleaning process
:: -------------------------------------------------------------

echo.
echo ========================================================
echo                  CLEANING WORKSPACE
echo ========================================================
echo.

:: Clean build directory
if exist "%WORKSPACE_PATH%\build" (
    echo Removing build directory...
    rd /s /q "%WORKSPACE_PATH%\build" >nul 2>&1
    :: Verify removal was successful
    if exist "%WORKSPACE_PATH%\build" (
        echo [ERROR] Failed to remove build directory.
        echo         You may need to close any applications using files in this directory.
    ) else (
        echo [OK] Build directory removed successfully.
    )
) else (
    echo [INFO] Build directory doesn't exist. Skipping.
)

:: Clean install directory
if exist "%WORKSPACE_PATH%\install" (
    echo Removing install directory...
    rd /s /q "%WORKSPACE_PATH%\install" >nul 2>&1
    :: Verify removal was successful
    if exist "%WORKSPACE_PATH%\install" (
        echo [ERROR] Failed to remove install directory.
        echo         You may need to close any applications using files in this directory.
    ) else (
        echo [OK] Install directory removed successfully.
    )
) else (
    echo [INFO] Install directory doesn't exist. Skipping.
)

:: Clean log directory
if exist "%WORKSPACE_PATH%\log" (
    echo Removing log directory...
    rd /s /q "%WORKSPACE_PATH%\log" >nul 2>&1
    :: Verify removal was successful
    if exist "%WORKSPACE_PATH%\log" (
        echo [ERROR] Failed to remove log directory.
        echo         You may need to close any applications using files in this directory.
    ) else (
        echo [OK] Log directory removed successfully.
    )
) else (
    echo [INFO] Log directory doesn't exist. Skipping.
)

:: -------------------------------------------------------------
:: Source packages cleaning (optional)
:: -------------------------------------------------------------

echo.
echo ========================================================
echo                  SOURCE PACKAGES
echo ========================================================
echo.
echo Would you like to remove packages in the src directory?
echo This will delete ALL packages in %WORKSPACE_PATH%\src.
echo.
choice /c YN /m "Clean source packages"

if errorlevel 2 (
    :: User chose not to clean source packages
    echo Source packages will be preserved.
) else (
    :: User chose to clean source packages
    if exist "%WORKSPACE_PATH%\src" (
        :: Double-check with a stronger warning
        echo.
        echo [!] WARNING: This will remove ALL contents in %WORKSPACE_PATH%\src
        echo     Are you ABSOLUTELY SURE you want to continue?
        echo     This action cannot be undone and will delete all your source code.
        echo.
        choice /c YN /m "I understand this will delete ALL packages"
        
        if errorlevel 2 (
            :: User cancelled source deletion
            echo Source deletion cancelled.
        ) else (
            :: Proceed with source deletion
            echo.
            echo Removing all packages in source directory...
            
            :: List all packages before removal for information
            echo Packages to be removed:
            for /d %%d in ("%WORKSPACE_PATH%\src\*") do (
                echo   - %%~nxd
            )
            echo.
            
            :: Loop through each directory in the src folder
            for /d %%d in ("%WORKSPACE_PATH%\src\*") do (
                echo   - Removing package: %%~nxd
                rd /s /q "%%d" >nul 2>&1
                
                :: Verify each package was removed
                if exist "%%d" (
                    echo     [ERROR] Failed to remove %%~nxd
                    echo              Some files may be in use or protected.
                )
            )
            echo Source packages removed.
        )
    ) else (
        echo [INFO] Source directory doesn't exist. Nothing to clean.
    )
)

:: -------------------------------------------------------------
:: Cleanup completion
:: -------------------------------------------------------------

echo.
echo ========================================================
echo                  CLEANUP COMPLETE
echo ========================================================
echo.
echo ROS workspace %WORKSPACE_PATH% has been cleaned.
echo.

:: -------------------------------------------------------------
:: Offer to rebuild the workspace
:: -------------------------------------------------------------

echo ========================================================
echo                REBUILD WORKSPACE
echo ========================================================
echo.
echo Would you like to rebuild the workspace now?
echo.
choice /c YN /m "Rebuild workspace"

:: If user wants to rebuild
if errorlevel 2 (
    echo.
    echo Rebuild skipped. You can manually rebuild later using:
    echo   cd %WORKSPACE_PATH%
    if "!WORKSPACE_TYPE!"=="ros1" (
        echo   catkin_make
    ) else if "!WORKSPACE_TYPE!"=="ros2" (
        echo   colcon build
    ) else (
        echo   colcon build    ^(for ROS2^)
        echo   catkin_make     ^(for ROS1^)
    )
    goto :script_end
) else (
    :: Prepare to rebuild
    echo.
    echo ========================================================
    echo                REBUILDING WORKSPACE
    echo ========================================================
    echo.
    
    :: Change directory to the workspace
    pushd "%WORKSPACE_PATH%"
    
    :: Determine which build system to use
    if "!WORKSPACE_TYPE!"=="ros1" (
        :: For ROS1 workspaces
        echo Detected ROS1 workspace. Using catkin_make...
        echo.
        
        :: Check if catkin_make is available
        where catkin_make >nul 2>nul
        if !errorlevel! neq 0 (
            echo [ERROR] catkin_make command not found.
            echo         Make sure you have sourced the ROS1 environment.
            echo         Example: C:\opt\ros\noetic\setup.bat
            popd
            goto :build_failed
        )
        
        :: Run the build with output
        echo Running: catkin_make
        catkin_make
        
        if !errorlevel! neq 0 (
            echo.
            echo [ERROR] Build failed. See above for errors.
            popd
            goto :build_failed
        )
    ) else (
        :: For ROS2 or unknown workspaces, default to colcon
        echo Using colcon build system...
        echo.
        
        :: Check if colcon is available
        where colcon >nul 2>nul
        if !errorlevel! neq 0 (
            echo [ERROR] colcon command not found.
            echo         Make sure you have sourced the ROS2 environment.
            echo         Example: C:\dev\ros2_humble\local_setup.bat
            popd
            goto :build_failed
        )
        
        :: Ask about build options
        echo Build options:
        echo [1] Standard build                   (colcon build)
        echo [2] Build with tests                 (colcon build --merge-install --test-result-base=test_results)
        echo [3] Build with merge install         (colcon build --merge-install)
        echo [4] Build specific package only      (you'll be prompted for package name)
        echo.
        choice /c 1234 /m "Select build option"
        
        if errorlevel 4 (
            :: Build specific package
            set /p PKG_NAME="Enter package name to build: "
            echo.
            echo Running: colcon build --packages-select !PKG_NAME!
            colcon build --packages-select !PKG_NAME!
        ) else if errorlevel 3 (
            :: Build with symlinks
            echo.
            echo Running: colcon build --merge-install
            colcon build --merge-install
        ) else if errorlevel 2 (
            :: Build with tests
            echo.
            echo Running: colcon build --merge-install --test-result-base=test_results
            colcon build --merge-install --test-result-base=test_results
        ) else (
            :: Standard build
            echo.
            echo Running: colcon build
            colcon build
        )
        
        if !errorlevel! neq 0 (
            echo.
            echo [ERROR] Build failed. See above for errors.
            popd
            goto :build_failed
        )
    )
    
    :: Return to original directory
    popd
    
    echo.
    echo ========================================================
    echo                 BUILD SUCCESSFUL
    echo ========================================================
    echo.
    echo Workspace rebuilt successfully!
    echo.
    echo Remember to source the setup files before using:
    if "!WORKSPACE_TYPE!"=="ros1" (
        echo   call %WORKSPACE_PATH%\devel\setup.bat
    ) else (
        echo   call %WORKSPACE_PATH%\install\setup.bat
    )
    echo.
    goto :build_complete
)

goto :eof

:script_end
echo.
echo Thank you for using the ROS Workspace Cleaner utility.
echo.
goto :eof

:build_failed
echo.
echo ========================================================
echo                  BUILD FAILED
echo ========================================================
echo.
echo The workspace was cleaned successfully, but the rebuild process failed.
echo You may need to:
echo   1. Source your ROS environment first
echo   2. Check for dependency issues
echo   3. Manually rebuild with more specific options
echo.
goto :eof

:build_complete
echo Thank you for using the ROS Workspace Cleaner utility.
echo.
goto :eof

:: -------------------------------------------------------------
:: Help and usage information
:: -------------------------------------------------------------

:usage
echo.
echo ========================================================
echo         ROS Workspace Cleaner - Help Information
echo                
echo ========================================================
echo.
echo Usage: %~nx0 [options]
echo.
echo Options:
echo   -w, --workspace PATH   Specify ROS workspace path directly
echo   -h, --help, /?         Show this help message
echo.
echo Examples:
echo   %~nx0                  Search for workspaces automatically
echo   %~nx0 -w C:\ros2_ws    Clean the workspace at C:\ros2_ws
echo.
echo Description:
echo   This script helps maintain ROS workspaces by cleaning build
echo   artifacts, installation files, and logs. This can resolve
echo   build issues and reclaim disk space.
echo.
echo   The script will search for ROS workspaces with common naming patterns:
echo     - Standard: ros_ws, ros2_ws, dev_ws, colcon_ws, workspace
echo     - ROS1 distribution-specific: ros_melodic, ros_noetic, melodic_ws, noetic_ws
echo     - ROS2 distribution-specific: ros2_foxy, ros2_galactic, ros2_humble, 
echo                                  ros2_iron, ros2_jazzy, ros2_rolling,
echo                                  foxy_ws, galactic_ws, humble_ws, iron_ws,
echo                                  jazzy_ws, rolling_ws
echo.
echo   After cleaning, the script can optionally rebuild the workspace
echo   automatically using the appropriate build system (colcon or catkin).
echo.
exit /b 0

:eof
endlocal