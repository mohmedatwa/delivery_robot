cmake_minimum_required(VERSION 3.8)
project(delivery_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# =====================
# C++ Hardware Plugin
# =====================
add_library(${PROJECT_NAME} SHARED
  src/delivery_firmware.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# =====================
# Python Package
# =====================
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  delivery_firmware/mpu6050_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Launch Files
# =====================
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# =====================
# Export
# =====================
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()