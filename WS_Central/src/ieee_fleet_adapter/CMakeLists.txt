cmake_minimum_required(VERSION 3.8)
project(ieee_fleet_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ieee_fleet_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(fleet_adapter
  src/fleet_adapter.cpp
)

ament_target_dependencies(fleet_adapter
  ieee_fleet_msgs
  rclcpp
  rclcpp_action
  rmf_fleet_msgs
  std_msgs
  yaml-cpp
)

# rmf_fleet_adapter exports executables and a shared library in the same export
# set; link directly to the library target to avoid attempting to link executables.
if(TARGET rmf_fleet_adapter::rmf_fleet_adapter)
  target_link_libraries(fleet_adapter rmf_fleet_adapter::rmf_fleet_adapter)
elseif(TARGET rmf_fleet_adapter)
  target_link_libraries(fleet_adapter rmf_fleet_adapter)
else()
  message(FATAL_ERROR
    "Could not find rmf_fleet_adapter library target. Expected "
    "'rmf_fleet_adapter::rmf_fleet_adapter' or 'rmf_fleet_adapter'.")
endif()

target_include_directories(fleet_adapter
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS fleet_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  ADAPTER_CONTRACT.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
