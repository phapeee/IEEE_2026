cmake_minimum_required(VERSION 3.5)
project(isaac_sim_gpio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(smacc2_msgs REQUIRED)

add_executable(isaac_sim_gpio_node src/isaac_sim_gpio_node.cpp)
ament_target_dependencies(isaac_sim_gpio_node rclcpp sensor_msgs smacc2_msgs)
target_compile_options(isaac_sim_gpio_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS isaac_sim_gpio_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
