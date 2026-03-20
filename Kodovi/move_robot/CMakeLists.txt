cmake_minimum_required(VERSION 3.8)
project(move_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveRobotService.srv"
)

install(PROGRAMS
  src/move_robot_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_robot_node
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
