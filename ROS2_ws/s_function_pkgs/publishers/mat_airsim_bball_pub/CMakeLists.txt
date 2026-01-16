cmake_minimum_required(VERSION 3.20)
project(mat_airsim_bball_pub)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get the real path of the source directory to handle symbolic links
file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" SRC_DIR)

# Get the real path to the ROS2 workspace installation
file(REAL_PATH "${SRC_DIR}/../../.." WS_ROOT_DIR)

# Ensure path separator consistency
# cmake_path(NATIVE_PATH WS_ROOT_DIR WS_ROOT_DIR)

# Output both paths in messages
message(STATUS "Source Directory (SRC_DIR): ${SRC_DIR}")
message(STATUS "Workspace Root Directory (WS_ROOT_DIR): ${WS_ROOT_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_msg_iface REQUIRED)

# Find MATLAB
find_package(Matlab REQUIRED SIMULINK) 

# Baseball publisher
if (Matlab_FOUND)
  message(STATUS "Matlab root directory found: ${Matlab_ROOT_DIR}.")
  matlab_add_mex(
    NAME ${PROJECT_NAME} 
    SRC src/${PROJECT_NAME}.cpp
  )
endif()

ament_target_dependencies(${PROJECT_NAME} ros_msg_iface)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install into a folder for use by Simulink
install(TARGETS ${PROJECT_NAME}
  DESTINATION ${WS_ROOT_DIR}/bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()