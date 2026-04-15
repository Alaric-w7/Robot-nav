cmake_minimum_required(VERSION 3.5)
project(elderbot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/auto_dock_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
