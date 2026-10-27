cmake_minimum_required(VERSION 3.8)
project(fleet_components)

find_package(ament_cmake REQUIRED)

install(DIRECTORY components urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
