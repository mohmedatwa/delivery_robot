cmake_minimum_required(VERSION 3.8)
project(delivery_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_library(delivery_robot_hardware SHARED
  src/delivery_robot_hardware.cpp
)

target_include_directories(delivery_robot_hardware PRIVATE include)
ament_target_dependencies(delivery_robot_hardware
  hardware_interface
  rclcpp_lifecycle
  pluginlib
  serial
)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS delivery_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()