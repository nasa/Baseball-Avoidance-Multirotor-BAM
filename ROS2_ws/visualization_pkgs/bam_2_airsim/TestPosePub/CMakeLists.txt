
message(STATUS "Processing TestPosePub Node...")


FILE(GLOB PROJECT_SOURCE_FILES "src/*.cpp")
FILE(GLOB PROJECT_HEADER_FILES "include/*.hpp")

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(TestPosePub
                ${PROJECT_SOURCE_FILES}
                ${PROJECT_HEADER_FILES})

include_directories(
                ${HOME_DIR}/spdlog/include
                ${HOME_DIR}/TestPosePub/include
               )

ament_target_dependencies(TestPosePub rclcpp geometry_msgs)

IF (WIN32)
  target_link_options(TestPosePub PRIVATE "/ignore:4044")
ELSE ()
  target_link_options(TestPosePub PRIVATE)
ENDIF()

target_link_libraries(TestPosePub
                      spdlog::spdlog
                      )