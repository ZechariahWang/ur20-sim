cmake_minimum_required(VERSION 3.8)
project(ur20_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_library(ur20_utils src/utils.cpp)
ament_target_dependencies(ur20_utils rclcpp moveit_ros_planning_interface)

add_executable(main_sweep src/main.cpp)
ament_target_dependencies(main_sweep rclcpp moveit_ros_planning_interface)
target_link_libraries(main_sweep ur20_utils)

add_executable(sweep_mover src/calibration_tests/sweep_mover.cpp)
ament_target_dependencies(sweep_mover rclcpp moveit_ros_planning_interface)
target_link_libraries(sweep_mover ur20_utils)

add_executable(room_publisher src/room_publisher.cpp)
ament_target_dependencies(room_publisher rclcpp moveit_ros_planning_interface visualization_msgs)

add_executable(tcp_orientate src/calibration_tests/tcp_orientate.cpp)
ament_target_dependencies(tcp_orientate rclcpp moveit_ros_planning_interface)
target_link_libraries(tcp_orientate ur20_utils)

add_executable(axis_test src/calibration_tests/axis_test.cpp)
ament_target_dependencies(axis_test rclcpp moveit_ros_planning_interface)
target_link_libraries(axis_test ur20_utils)

install(TARGETS main_sweep sweep_mover room_publisher tcp_orientate axis_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
