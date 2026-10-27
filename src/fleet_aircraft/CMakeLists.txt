cmake_minimum_required(VERSION 3.16)
project(fleet_aircraft)

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
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

include_directories(include)

add_library(fleet_gnss_error SHARED src/gnss_error.cpp)
target_link_libraries(fleet_gnss_error gz-sim::gz-sim gz-plugin::register
  gz-transport::core gz-msgs::core gz-math::core)

add_executable(test_gnss_core test/test_gnss_core.cpp)
install(TARGETS test_gnss_core RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fleet_gnss_error LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_LINELENGTH 100)
  ament_lint_auto_find_test_dependencies()
  add_test(NAME gnss_core COMMAND test_gnss_core)
endif()

ament_package()
