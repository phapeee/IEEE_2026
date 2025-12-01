cmake_minimum_required(VERSION 3.8)
project(laser_scan_merger)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(laser_scan_merger_node src/laser_scan_merger_node.cpp)
ament_target_dependencies(laser_scan_merger_node rclcpp sensor_msgs)

target_compile_features(laser_scan_merger_node PUBLIC c_std_99 cxx_std_17)

install(TARGETS laser_scan_merger_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
