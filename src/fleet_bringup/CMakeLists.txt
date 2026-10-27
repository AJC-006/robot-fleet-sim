cmake_minimum_required(VERSION 3.8)
project(fleet_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/patrol.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # short headless run of one robot; set FLEET_SIM_TESTS=0 to skip, FLEET_SIM_LAUNCH_PREFIX to wrap
  ament_add_pytest_test(test_sim_statistics test/test_sim_statistics.py TIMEOUT 300)
endif()

ament_package()
