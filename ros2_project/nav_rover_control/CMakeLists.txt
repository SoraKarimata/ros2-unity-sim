cmake_minimum_required(VERSION 3.5)
project(gazebo_simulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

################################################################################
# Build
################################################################################
set(dependencies
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

################################################################################
# Install
################################################################################
install(
  DIRECTORY
    launch
    config
    worlds
    urdf
    meshes
    models
  DESTINATION share/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(
  PROGRAMS
  launch/ares8_rover.launch.py
  launch/mapviz.launch.py
  launch/robot_state_publisher.launch.py
  launch/manual_controller.launch.py
  launch/gps_waypoint_follower.launch.py
  launch/odometry.launch.py
  launch/tf_broadcast.launch.py
  launch/rover_control.launch.py
  DESTINATION lib/${PROJECT_NAME}
)


################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
