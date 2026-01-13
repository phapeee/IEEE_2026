cmake_minimum_required(VERSION 3.5)
project(limit_switch_calibration_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/LimitSwitchCalibration.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
