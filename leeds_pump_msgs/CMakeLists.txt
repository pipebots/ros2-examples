cmake_minimum_required(VERSION 3.5)
project(leeds_pump_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ACTION_FILES
  action/LeedsPumpPump.action
)

set(MSG_FILES
  msg/LeedsPumpStatus.msg
)

set(SRV_FILES
  srv/LeedsPumpGimbal.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  ${MSG_FILES}
  ${SRV_FILES}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
