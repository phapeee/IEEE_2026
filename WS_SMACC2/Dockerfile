# syntax=docker/dockerfile:1.6
ARG BASE_IMAGE=ros2-jazzy-base-arm64
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-c"]

WORKDIR /tmp/ws
COPY src/SMACC2/.github/SMACC2.jazzy.repos ./src/SMACC2/.github/SMACC2.jazzy.repos
COPY src/SMACC2/smacc2/package.xml ./src/SMACC2/smacc2/package.xml
COPY src/SMACC2/smacc2_client_library/cl_foundation_pose/package.xml ./src/SMACC2/smacc2_client_library/cl_foundation_pose/package.xml
COPY src/SMACC2/smacc2_client_library/cl_gcalcli/package.xml ./src/SMACC2/smacc2_client_library/cl_gcalcli/package.xml
COPY src/SMACC2/smacc2_client_library/cl_generic_sensor/package.xml ./src/SMACC2/smacc2_client_library/cl_generic_sensor/package.xml
COPY src/SMACC2/smacc2_client_library/cl_http/package.xml ./src/SMACC2/smacc2_client_library/cl_http/package.xml
COPY src/SMACC2/smacc2_client_library/cl_isaac_apriltag/package.xml ./src/SMACC2/smacc2_client_library/cl_isaac_apriltag/package.xml
COPY src/SMACC2/smacc2_client_library/cl_keyboard/package.xml ./src/SMACC2/smacc2_client_library/cl_keyboard/package.xml
COPY src/SMACC2/smacc2_client_library/cl_lifecycle_node/package.xml ./src/SMACC2/smacc2_client_library/cl_lifecycle_node/package.xml
COPY src/SMACC2/smacc2_client_library/cl_mission_tracker/package.xml ./src/SMACC2/smacc2_client_library/cl_mission_tracker/package.xml
COPY src/SMACC2/smacc2_client_library/cl_modbus_tcp_relay/package.xml ./src/SMACC2/smacc2_client_library/cl_modbus_tcp_relay/package.xml
COPY src/SMACC2/smacc2_client_library/cl_moveit2z/package.xml ./src/SMACC2/smacc2_client_library/cl_moveit2z/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/cl_nav2z/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/cl_nav2z/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/backward_global_planner/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/backward_global_planner/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/backward_local_planner/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/backward_local_planner/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/forward_global_planner/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/forward_global_planner/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/forward_local_planner/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/forward_local_planner/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/nav2z_planners_common/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/nav2z_planners_common/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/pure_spinning_local_planner/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/pure_spinning_local_planner/package.xml
COPY src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/undo_path_global_planner/package.xml ./src/SMACC2/smacc2_client_library/cl_nav2z/custom_planners/undo_path_global_planner/package.xml
COPY src/SMACC2/smacc2_client_library/cl_ros2_timer/package.xml ./src/SMACC2/smacc2_client_library/cl_ros2_timer/package.xml
COPY src/SMACC2/smacc2_event_generator_library/eg_conditional_generator/package.xml ./src/SMACC2/smacc2_event_generator_library/eg_conditional_generator/package.xml
COPY src/SMACC2/smacc2_event_generator_library/eg_random_generator/package.xml ./src/SMACC2/smacc2_event_generator_library/eg_random_generator/package.xml
COPY src/SMACC2/smacc2_msgs/package.xml ./src/SMACC2/smacc2_msgs/package.xml
COPY src/SMACC2/smacc2_performance_tools/performance_tests/sm_atomic_performance_trace_1/package.xml ./src/SMACC2/smacc2_performance_tools/performance_tests/sm_atomic_performance_trace_1/package.xml
COPY src/SMACC2/smacc2_performance_tools/performance_tests/sm_atomic_subscribers_performance_test/package.xml ./src/SMACC2/smacc2_performance_tools/performance_tests/sm_atomic_subscribers_performance_test/package.xml
COPY src/SMACC2/smacc2_performance_tools/performance_tests/sm_coretest_transition_speed_1/package.xml ./src/SMACC2/smacc2_performance_tools/performance_tests/sm_coretest_transition_speed_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_advanced_recovery_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_advanced_recovery_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_atomic/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_atomic/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_atomic_http/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_atomic_http/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_atomic_lifecycle/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_atomic_lifecycle/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_atomic_mode_states/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_atomic_mode_states/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_branching/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_branching/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_cl_gcalcli_test_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_cl_gcalcli_test_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_cl_keyboard_unit_test_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_cl_keyboard_unit_test_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_cl_ros2_timer_unit_test_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_cl_ros2_timer_unit_test_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_modbus_tcp_relay_test_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_modbus_tcp_relay_test_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_multi_stage_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_multi_stage_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_multithread_test_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_multithread_test_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_nav2_gazebo_test_1/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_nav2_gazebo_test_1/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_pack_ml/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_pack_ml/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_panda_cl_moveit2z_cb_inventory/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_panda_cl_moveit2z_cb_inventory/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_panda_cl_moveit2z_cb_inventory_isaacsim/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_panda_cl_moveit2z_cb_inventory_isaacsim/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_simple_action_client/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_simple_action_client/package.xml
COPY src/SMACC2/smacc2_sm_reference_library/sm_three_some/package.xml ./src/SMACC2/smacc2_sm_reference_library/sm_three_some/package.xml
COPY src/SMACC2/smacc2_state_reactor_library/sr_all_events_go/package.xml ./src/SMACC2/smacc2_state_reactor_library/sr_all_events_go/package.xml
COPY src/SMACC2/smacc2_state_reactor_library/sr_conditional/package.xml ./src/SMACC2/smacc2_state_reactor_library/sr_conditional/package.xml
COPY src/SMACC2/smacc2_state_reactor_library/sr_event_countdown/package.xml ./src/SMACC2/smacc2_state_reactor_library/sr_event_countdown/package.xml
RUN vcs import src --skip-existing --input src/SMACC2/.github/SMACC2.jazzy.repos && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-example-interfaces && \
    rosdep update && \
    rosdep install --ignore-src --from-paths src -y -r --rosdistro "${ROS_DISTRO}" && \
    rm -rf /var/lib/apt/lists/*

COPY src ./src

WORKDIR /ws

CMD ["bash"]
