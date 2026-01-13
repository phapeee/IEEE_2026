cmake_minimum_required(VERSION 3.5)
project(pointcloud_filter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/pointcloud_filter_node.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

rclcpp_components_register_nodes(${PROJECT_NAME} "pointcloud_filter::PointcloudFilterNode")

add_executable(pointcloud_filter_node src/main.cpp)
target_link_libraries(pointcloud_filter_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  pointcloud_filter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
