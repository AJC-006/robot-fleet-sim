cmake_minimum_required(VERSION 3.16)
project(fleet_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)

# Coulomb friction cone for DART contacts (see README.md)
add_library(fleet_coulomb_friction SHARED src/coulomb_friction.cpp)
target_link_libraries(fleet_coulomb_friction
  gz-sim::gz-sim gz-plugin::register gz-msgs::gz-msgs gz-common::gz-common)

install(TARGETS fleet_coulomb_friction LIBRARY DESTINATION lib)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_LINELENGTH 100)
  ament_lint_auto_find_test_dependencies()

  # closed-form Coulomb checks, stepping gz-sim in-process with the plugin from this build tree
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_coulomb_friction test/test_coulomb_friction.py
    APPEND_ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 300)
endif()

ament_package()
