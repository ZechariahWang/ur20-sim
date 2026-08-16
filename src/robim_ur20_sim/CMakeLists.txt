cmake_minimum_required(VERSION 3.8)
project(robim_ur20_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(sweep_mover src/sweep_mover.cpp)
ament_target_dependencies(sweep_mover rclcpp moveit_ros_planning_interface)

add_executable(room_publisher src/room_publisher.cpp)
ament_target_dependencies(room_publisher rclcpp moveit_ros_planning_interface visualization_msgs)

install(TARGETS sweep_mover room_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
