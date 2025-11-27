cmake_minimum_required(VERSION 3.8)
project(uwb_triangulaion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uwb_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(uwb_triangulaion_node src/uwb_triangulation_node.cpp)
ament_target_dependencies(uwb_triangulaion_node
  rclcpp
  geometry_msgs
  uwb_msgs
)

add_executable(uwb_covariance_estimator_node src/uwb_covariance_estimator_node.cpp)
ament_target_dependencies(uwb_covariance_estimator_node
  rclcpp
  geometry_msgs
)

target_link_libraries(uwb_triangulaion_node yaml-cpp)
target_link_libraries(uwb_covariance_estimator_node yaml-cpp)

target_include_directories(uwb_triangulaion_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(uwb_covariance_estimator_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  uwb_triangulaion_node
  uwb_covariance_estimator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
