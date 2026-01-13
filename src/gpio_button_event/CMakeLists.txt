cmake_minimum_required(VERSION 3.5)
project(gpio_button_event)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smacc2_msgs REQUIRED)

find_path(GPIOD_INCLUDE_DIR gpiod.h)
find_library(GPIOD_LIBRARY gpiod)
if(NOT GPIOD_INCLUDE_DIR OR NOT GPIOD_LIBRARY)
  message(FATAL_ERROR "libgpiod headers/library not found. Install libgpiod-dev before building gpio_button_event.")
endif()

add_executable(gpio_button_event_node src/gpio_button_node.cpp)
target_include_directories(gpio_button_event_node PRIVATE ${GPIOD_INCLUDE_DIR})
ament_target_dependencies(gpio_button_event_node rclcpp std_msgs smacc2_msgs)
target_compile_definitions(gpio_button_event_node PRIVATE HAVE_LIBGPIOD)
target_link_libraries(gpio_button_event_node ${GPIOD_LIBRARY})

target_compile_options(gpio_button_event_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS gpio_button_event_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES package.xml README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
