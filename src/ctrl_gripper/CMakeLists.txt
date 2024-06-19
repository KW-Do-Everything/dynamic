cmake_minimum_required(VERSION 3.8)
project(ctrl_gripper)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(open_manipulator_x_libs REQUIRED)
find_package(robotis_manipulator REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

set(dependencies
  "geometry_msgs" 
  "open_manipulator_msgs"
  "open_manipulator_x_libs" 
  "rclcpp" 
  "robotis_manipulator" 
  "sensor_msgs" 
  "std_msgs" 
  "trajectory_msgs"
)

add_executable(gripper src/gripper.cpp)
ament_target_dependencies(gripper rclcpp open_manipulator_msgs ${dependencies})

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server rclcpp open_manipulator_msgs ${dependencies})


install(TARGETS gripper
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS server
  DESTINATION lib/${PROJECT_NAME})

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

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(open_manipulator_msgs)
ament_export_dependencies(open_manipulator_x_libs)
ament_export_dependencies(robotis_manipulator)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(trajectory_msgs)
ament_package()
