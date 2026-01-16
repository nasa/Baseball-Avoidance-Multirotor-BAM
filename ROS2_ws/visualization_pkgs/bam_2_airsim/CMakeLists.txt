cmake_minimum_required(VERSION 3.20)
project(bam_2_airsim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (HOME_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spdlog REQUIRED)

# set some library options ( need dynamic release to match with Airsim's build)
set(MD_dynamicrelease)


#set(CMAKE_BUILD_TYPE RelWithDebInfo)

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

add_subdirectory(Bam2Airsim)
add_subdirectory(TestPosePub)
#ament_target_dependencies(Bam2Airsim rclcpp geometry_msgs)

install(TARGETS Bam2Airsim TestPosePub
  DESTINATION lib/${PROJECT_NAME})


ament_package()
