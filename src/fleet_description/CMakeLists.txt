cmake_minimum_required(VERSION 3.8)
project(fleet_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME})

# Make package://fleet_description/... and model://fleet_description/... resolvable by Gazebo
# after `source install/setup.bash`.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz_sim_resource_path.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py TIMEOUT 300)
endif()

ament_package()
