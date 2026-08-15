cmake_minimum_required(VERSION 3.8)
project(robim_ur20_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(waypoint_mover src/waypoint_mover.cpp)
ament_target_dependencies(waypoint_mover rclcpp moveit_ros_planning_interface)

add_executable(sweep_mover src/sweep_mover.cpp)
ament_target_dependencies(sweep_mover rclcpp moveit_ros_planning_interface)

install(TARGETS waypoint_mover sweep_mover
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
