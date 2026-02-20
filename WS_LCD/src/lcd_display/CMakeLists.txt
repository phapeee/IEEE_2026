cmake_minimum_required(VERSION 3.8)
project(st7789_ros_wrapper)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_library(GPIOD_LIBRARY NAMES gpiod REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TextOverlay.msg"
  "msg/ImageOverlay.msg"
  DEPENDENCIES sensor_msgs
)

add_library(st7789_core
  src/hw/SpiBus.cpp
  src/hw/GpioLine.cpp
  src/driver/St7789.cpp
  src/gfx/Canvas565.cpp
  src/gfx/DirtyTracker.cpp
  src/gfx/FontBitmap.cpp
  src/assets/ImageLoader.cpp
  src/assets/FrameSequence.cpp
  src/app/FrameQueue.cpp
  src/app/Screen.cpp
  src/app/ScreenRuntimeConfig.cpp
  src/app/AcceptanceSmokeCli.cpp
  src/app/AnimationCli.cpp
  src/app/ShowImageCli.cpp
  src/app/TextDemoCli.cpp
)
target_include_directories(st7789_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(st7789_core PUBLIC cxx_std_17)
target_include_directories(st7789_core SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(st7789_core PUBLIC ${GPIOD_LIBRARY} ${OpenCV_LIBS})

add_library(st7789_ros_bridge
  src/nodes/DisplayNodeRuntimeConfig.cpp
  src/nodes/ImageMessageConverter.cpp
)
target_include_directories(st7789_ros_bridge
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(st7789_ros_bridge PUBLIC cxx_std_17)
target_link_libraries(st7789_ros_bridge st7789_core)
ament_target_dependencies(st7789_ros_bridge sensor_msgs std_msgs)

add_executable(st7789_display_node src/nodes/st7789_display_node.cpp)
ament_target_dependencies(st7789_display_node rclcpp sensor_msgs std_msgs)
target_link_libraries(st7789_display_node st7789_core st7789_ros_bridge)
target_compile_features(st7789_display_node PRIVATE cxx_std_17)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(st7789_display_node "${cpp_typesupport_target}")

add_executable(lcd_acceptance_smoke src/tools/lcd_acceptance_smoke.cpp)
target_link_libraries(lcd_acceptance_smoke st7789_core)
target_compile_features(lcd_acceptance_smoke PRIVATE cxx_std_17)

add_executable(lcd_play_animation src/tools/lcd_play_animation.cpp)
target_link_libraries(lcd_play_animation st7789_core)
target_compile_features(lcd_play_animation PRIVATE cxx_std_17)

add_executable(lcd_show_image src/tools/lcd_show_image.cpp)
target_link_libraries(lcd_show_image st7789_core)
target_compile_features(lcd_show_image PRIVATE cxx_std_17)

add_executable(lcd_cli src/tools/lcd_cli.cpp)
target_link_libraries(lcd_cli st7789_core)
target_compile_features(lcd_cli PRIVATE cxx_std_17)
ament_target_dependencies(lcd_cli ament_index_cpp rclcpp)

add_executable(lcd_text_demo src/tools/lcd_text_demo.cpp)
target_link_libraries(lcd_text_demo st7789_core)
target_compile_features(lcd_text_demo PRIVATE cxx_std_17)

add_executable(lcd_publish_image_overlay src/tools/lcd_publish_image_overlay.cpp)
ament_target_dependencies(lcd_publish_image_overlay rclcpp sensor_msgs)
target_include_directories(lcd_publish_image_overlay SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(lcd_publish_image_overlay ${OpenCV_LIBS} "${cpp_typesupport_target}")
target_compile_features(lcd_publish_image_overlay PRIVATE cxx_std_17)

install(TARGETS
  st7789_core
  st7789_ros_bridge
  EXPORT export_st7789_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  st7789_display_node
  lcd_acceptance_smoke
  lcd_play_animation
  lcd_show_image
  lcd_cli
  lcd_text_demo
  lcd_publish_image_overlay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  README.md
  instructions.md
  software_structure.md
  verification_checklist.md
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../config/display.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_targets(export_st7789_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(st7789_core st7789_ros_bridge)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_spibus test/test_spibus.cpp)
  if(TARGET test_spibus)
    target_link_libraries(test_spibus st7789_core)
    target_compile_features(test_spibus PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_gpioline test/test_gpioline.cpp)
  if(TARGET test_gpioline)
    target_link_libraries(test_gpioline st7789_core)
    target_compile_features(test_gpioline PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_st7789 test/test_st7789.cpp)
  if(TARGET test_st7789)
    target_link_libraries(test_st7789 st7789_core)
    target_compile_features(test_st7789 PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_screen_runtime_config test/test_screen_runtime_config.cpp)
  if(TARGET test_screen_runtime_config)
    target_link_libraries(test_screen_runtime_config st7789_core)
    target_compile_features(test_screen_runtime_config PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_frame_queue test/test_frame_queue.cpp)
  if(TARGET test_frame_queue)
    target_link_libraries(test_frame_queue st7789_core)
    target_compile_features(test_frame_queue PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_acceptance_smoke_cli test/test_acceptance_smoke_cli.cpp)
  if(TARGET test_acceptance_smoke_cli)
    target_link_libraries(test_acceptance_smoke_cli st7789_core)
    target_compile_features(test_acceptance_smoke_cli PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_image_loader test/test_image_loader.cpp)
  if(TARGET test_image_loader)
    target_link_libraries(test_image_loader st7789_core ${OpenCV_LIBS})
    target_include_directories(test_image_loader SYSTEM PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_compile_features(test_image_loader PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_font_bitmap test/test_font_bitmap.cpp)
  if(TARGET test_font_bitmap)
    target_link_libraries(test_font_bitmap st7789_core)
    target_compile_features(test_font_bitmap PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_canvas565_text test/test_canvas565_text.cpp)
  if(TARGET test_canvas565_text)
    target_link_libraries(test_canvas565_text st7789_core)
    target_compile_features(test_canvas565_text PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_canvas565_blit test/test_canvas565_blit.cpp)
  if(TARGET test_canvas565_blit)
    target_link_libraries(test_canvas565_blit st7789_core)
    target_compile_features(test_canvas565_blit PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_dirty_tracker test/test_dirty_tracker.cpp)
  if(TARGET test_dirty_tracker)
    target_link_libraries(test_dirty_tracker st7789_core)
    target_compile_features(test_dirty_tracker PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_screen_dirty_present test/test_screen_dirty_present.cpp)
  if(TARGET test_screen_dirty_present)
    target_link_libraries(test_screen_dirty_present st7789_core)
    target_compile_features(test_screen_dirty_present PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_screen_threaded_present test/test_screen_threaded_present.cpp)
  if(TARGET test_screen_threaded_present)
    target_link_libraries(test_screen_threaded_present st7789_core)
    target_compile_features(test_screen_threaded_present PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_frame_sequence test/test_frame_sequence.cpp)
  if(TARGET test_frame_sequence)
    target_link_libraries(test_frame_sequence st7789_core)
    target_compile_features(test_frame_sequence PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_animation_cli test/test_animation_cli.cpp)
  if(TARGET test_animation_cli)
    target_link_libraries(test_animation_cli st7789_core)
    target_compile_features(test_animation_cli PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_show_image_cli test/test_show_image_cli.cpp)
  if(TARGET test_show_image_cli)
    target_link_libraries(test_show_image_cli st7789_core)
    target_compile_features(test_show_image_cli PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(test_text_demo_cli test/test_text_demo_cli.cpp)
  if(TARGET test_text_demo_cli)
    target_link_libraries(test_text_demo_cli st7789_core)
    target_compile_features(test_text_demo_cli PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(
    test_display_node_runtime_config
    test/test_display_node_runtime_config.cpp
  )
  if(TARGET test_display_node_runtime_config)
    target_link_libraries(test_display_node_runtime_config st7789_ros_bridge)
    target_compile_features(test_display_node_runtime_config PRIVATE cxx_std_17)
  endif()

  ament_add_gtest(
    test_image_message_converter
    test/test_image_message_converter.cpp
  )
  if(TARGET test_image_message_converter)
    target_link_libraries(test_image_message_converter st7789_ros_bridge)
    ament_target_dependencies(test_image_message_converter sensor_msgs)
    target_compile_features(test_image_message_converter PRIVATE cxx_std_17)
  endif()
endif()

ament_package()
