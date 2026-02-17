# RMF Integration Cautions (Fleet Adapter + Command Endpoints)

This note summarizes operational and configuration details that can break
integration between the central fleet adapter and the robot/drone command
endpoints.

## Cautions

- `RobotState.map_name` must match the RMF nav graph level name for that fleet.
  Ground uses `L1`; drone uses `AIR`. A mismatch will prevent RMF from placing
  the robot on the correct map.
- Robot namespaces must be consistent across:
  `rmf_fleet.robots` names, the command endpoint node namespace (or explicit
  `robot_name` parameter), and the action server path constructed by
  `use_robot_namespace`. If any of these differ, the adapter will never find the
  action server.
- The command endpoint `last_completed_topic` and the robot state publisher
  `last_completed_topic` must be identical (and remapped the same way). If they
  diverge, `RobotState.last_completed_request` never updates and RMF actions will
  time out.
- Battery SOC must be a 0.0–1.0 fraction in `RobotState`. If the drone publisher
  is configured with `battery_units: percent`, it will publish 0–100 and the
  adapter will treat it as invalid and use `battery_fallback_soc` instead.
- `state_best_effort: true` on the robot state publisher does not match the
  adapter’s reliable subscription. If enabled, the adapter may receive no state
  and decommission robots as stale.
- Ground `rmf_command_endpoint_node` with `hold_result: true` requires an
  external caller to invoke `mark_success` or `mark_failure`. If not, actions
  will time out in the adapter.
- Drone `drone_execute_command_server` depends on `command_done_topic` to
  complete actions. If no component publishes done messages, actions will remain
  active until the adapter times out.
- `rmf_fleet.actions` must include every `perform_action` category you intend to
  use; otherwise the adapter will reject them before sending to the robot.
- `fleet_manager.required_action_keys` must align with the payload schemas used
  by the SMACC2 state machine. Mismatches cause immediate rejection.
