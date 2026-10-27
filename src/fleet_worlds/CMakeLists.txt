cmake_minimum_required(VERSION 3.8)
project(fleet_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models maps DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/fleet_worlds.dsv.in")

ament_package()
