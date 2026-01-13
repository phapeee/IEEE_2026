cmake_minimum_required(VERSION 3.5)
project(smacc_button_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(limit_switch_calibration_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(smacc2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(smacc2_msgs REQUIRED)

include_directories(include)

add_executable(button_waypoint_sm src/smacc_button_nav.cpp)

ament_target_dependencies(
  button_waypoint_sm
  controller_manager_msgs
  geometry_msgs
  limit_switch_calibration_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_localization
  smacc2
  smacc2_msgs
  std_msgs
  std_srvs
)

# ---- Link Boost::thread (plain signature) ----
target_link_libraries(button_waypoint_sm Boost::thread)

# ---- Link yaml-cpp (also plain signature) ----
if(TARGET yaml-cpp)
  target_link_libraries(button_waypoint_sm yaml-cpp)
elseif(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(button_waypoint_sm yaml-cpp::yaml-cpp)
elseif(YAML_CPP_LIBRARIES)
  target_link_libraries(button_waypoint_sm ${YAML_CPP_LIBRARIES})
else()
  message(FATAL_ERROR "yaml-cpp target not found after find_package(yaml-cpp)")
endif()

install(
  TARGETS button_waypoint_sm
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
