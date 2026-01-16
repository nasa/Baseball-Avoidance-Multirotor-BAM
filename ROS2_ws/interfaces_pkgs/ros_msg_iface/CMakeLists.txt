cmake_minimum_required(VERSION 3.20)
project(ros_msg_iface)

# Add position independent code flag
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create library target
add_library(${PROJECT_NAME} STATIC src/${PROJECT_NAME}.cpp)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
)

# Export dependencies and includes
ament_export_dependencies(rclcpp geometry_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the library WITHOUT exporting it through CMake
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Get the real path of the source directory to handle symbolic links
file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" SRC_DIR)

# Get the real path to the ROS2 workspace installation
file(REAL_PATH "${SRC_DIR}/../.." WS_ROOT_DIR)

# Ensure path separator consistency
# cmake_path(NATIVE_PATH WS_ROOT_DIR WS_ROOT_DIR)

# Output both paths in messages
message(STATUS "Source Directory (SRC_DIR): ${SRC_DIR}")
message(STATUS "Workspace Root Directory (WS_ROOT_DIR): ${WS_ROOT_DIR}")

# Custom installation
if(WIN32)
  set(DEST_DIR "win64")
elseif(APPLE)
  set(DEST_DIR "mac")
else()
  set(DEST_DIR "linux")
endif()

if(MSVC)
  set(LIB_PATH ${CMAKE_BINARY_DIR}/Release/${CMAKE_STATIC_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})
else()
  set(LIB_PATH ${CMAKE_BINARY_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}${PROJECT_NAME}${CMAKE_STATIC_LIBRARY_SUFFIX})
endif()

install(FILES ${LIB_PATH} DESTINATION ${WS_ROOT_DIR}/lib/${DEST_DIR})
install(DIRECTORY include/ DESTINATION ${WS_ROOT_DIR}/include)
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()