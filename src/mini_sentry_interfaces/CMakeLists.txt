cmake_minimum_required(VERSION 3.10)
project(mini_sentry_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GameStatus.msg"
  "srv/SetMode.srv"
  "action/GoToPoint.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
