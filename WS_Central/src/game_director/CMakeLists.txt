cmake_minimum_required(VERSION 3.8)
project(game_director)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ieee_fleet_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_task_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(game_director src/game_director_node.cpp)
target_compile_features(game_director PUBLIC cxx_std_17)
ament_target_dependencies(
  game_director
  ament_index_cpp
  ieee_fleet_msgs
  rclcpp
  rmf_task_msgs
  std_msgs
  std_srvs
)
target_link_libraries(
  game_director
  yaml-cpp
  nlohmann_json::nlohmann_json
)

install(TARGETS game_director DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()
install(FILES README.md package.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
