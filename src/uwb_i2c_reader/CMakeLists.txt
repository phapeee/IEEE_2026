cmake_minimum_required(VERSION 3.8)
project(uwb_i2c_reader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(i2c_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uwb_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/uwb_i2c_reader_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  i2c_manager
  rclcpp
  uwb_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
