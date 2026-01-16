cmake_minimum_required(VERSION 3.20)
project(psw_sub)

# turn off ZERO_CHECK
set (CMAKE_SUPPRESS_REGENERATION true)

set(AMENT_CMAKE_SYMLINK_INSTALL OFF)

# Get the real path of the source directory to handle symbolic links
file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" SRC_DIR)

# Get the real path to the ROS2 workspace installation
file(REAL_PATH "${SRC_DIR}/../../.." WS_ROOT_DIR)

# Ensure path separator consistency
# cmake_path(NATIVE_PATH WS_ROOT_DIR WS_ROOT_DIR)

# Output both paths in messages
message(STATUS "Source Directory (SRC_DIR): ${SRC_DIR}")
message(STATUS "Workspace Root Directory (WS_ROOT_DIR): ${WS_ROOT_DIR}")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Make sure we appropriately export symbols
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_CXX_VISIBILITY_PRESET default)
set(CMAKE_VISIBILITY_INLINES_HIDDEN OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find MATLAB
find_package(Matlab REQUIRED SIMULINK)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (Matlab_FOUND)
  message(STATUS "Matlab root directory found: ${Matlab_ROOT_DIR}.")
  matlab_add_mex(
    NAME ${PROJECT_NAME} 
    SRC src/${PROJECT_NAME}.cpp
  )
endif()

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

if(MSVC)
  # Windows with Visual Studio uses configuration-specific subdirectories
  set(MEX_FULLNAME ${CMAKE_BINARY_DIR}/Release/${PROJECT_NAME}.${Matlab_MEX_EXTENSION})
else()
  # Linux/macOS - MEX files don't use the lib prefix
  set(MEX_FULLNAME ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.${Matlab_MEX_EXTENSION})
endif()

# Use install() instead of ament_cmake_symlink_install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
# Install into a folder for use by Simulink
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${WS_ROOT_DIR}/bin
)

# Install package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()