cmake_minimum_required(VERSION 3.5)
project(omron_os32c_driver LANGUAGES CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(odva_ethernetip REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

add_library(omron_os32c src/os32c.cpp)
target_include_directories(omron_os32c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  omron_os32c
  Boost
  odva_ethernetip
  sensor_msgs
  rclcpp
)

add_executable(scanner_node src/scanner_node.cpp)
target_link_libraries(scanner_node
  omron_os32c
)

add_executable(omron_os32c_node src/os32c_node.cpp)
target_link_libraries(omron_os32c_node
  omron_os32c
)
ament_target_dependencies(omron_os32c_node
  diagnostic_updater
)

install(TARGETS
    omron_os32c_node
    scanner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
