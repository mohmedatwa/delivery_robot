cmake_minimum_required(VERSION 3.8)
project(delivery_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_lint_auto REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(
  DIRECTORY delivery_scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  ${PROJECT_NAME}/draw_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
