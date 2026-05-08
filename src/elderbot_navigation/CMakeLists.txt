cmake_minimum_required(VERSION 3.5)
project(elderbot_navigation)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AutoDock.action"
  "action/Patrol.action"
)

install(
  DIRECTORY action config launch maps rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/auto_dock_node.py
    scripts/patrol_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
