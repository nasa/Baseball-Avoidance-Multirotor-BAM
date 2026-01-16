
message(STATUS "Processing Bam2Airsim Node...")

set (AIRLIB_INC_DIR $ENV{AirSimLib}/include)
set (EIGN_INC_DIR $ENV{AirSimLib}/deps/eigen3)
set (RPC_INC_DIR $ENV{AirSimLib}/deps/rpclib/include)
set (MAVLINK_INC_DIR $ENV{AirSimLib}/deps/MavLinkCom/include)
set (AIRLIB_LIB_DIR $ENV{AirSimLib}/lib/x64/Release)
set (RPC_LIB_DIR $ENV{AirSimLib}/deps/rpclib/lib/x64/Release)
set (MAVLINK_LIB_DIR $ENV{AirSimLib}/deps/MavLinkCom/lib/x64/Release)

FILE(GLOB PROJECT_SOURCE_FILES "src/*.cpp")
FILE(GLOB PROJECT_HEADER_FILES "include/*.hpp")

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(Bam2Airsim
                ${PROJECT_SOURCE_FILES}
                ${PROJECT_HEADER_FILES})

include_directories(${AIRLIB_INC_DIR}
                ${EIGN_INC_DIR}
                ${RPC_INC_DIR}
                ${MAVLINK_INC_DIR}
                ${HOME_DIR}/spdlog/include
                ${HOME_DIR}/Bam2Airsim/include
               )

ament_target_dependencies(Bam2Airsim rclcpp geometry_msgs)

IF (WIN32)
  target_link_options(Bam2Airsim PRIVATE "/ignore:4044")
ELSE ()
  target_link_options(Bam2Airsim PRIVATE)
ENDIF()

target_link_libraries(Bam2Airsim
                      ${AIRLIB_LIB_DIR}/AirLib.lib
                      ${RPC_LIB_DIR}/rpc.lib
                      ${MAVLINK_LIB_DIR}/MavLinkCom.lib
                      spdlog::spdlog
                      )