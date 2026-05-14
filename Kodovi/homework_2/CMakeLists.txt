cmake_minimum_required(VERSION 3.8)
project(homework_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RobotCommand.srv"
)

install(PROGRAMS
  src/homework_2_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME homework_2_controller
)

install(PROGRAMS
  src/homework_2_command.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME homework_2_command
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
