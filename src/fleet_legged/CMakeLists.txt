cmake_minimum_required(VERSION 3.8)
project(fleet_legged)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gait_node.py
  scripts/envelope.py
  scripts/check_lidar_plane.py
  scripts/walk_test.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
