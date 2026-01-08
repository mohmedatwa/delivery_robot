cmake_minimum_required(VERSION 3.8)
project(delivery_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)




find_package(ament_lint_auto REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(
  DIRECTORY config launch delivery_twist
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
  ${PROJECT_NAME}/twist_relay.py
  DESTINATION lib/${PROJECT_NAME}
)
set(ament_cmake_copyright_FOUND TRUE)

set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()


ament_package()
