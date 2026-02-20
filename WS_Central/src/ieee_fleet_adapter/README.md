# IEEE 2026 Fleet Adapters

This package provides two RMF fleet adapter instances (ground and drone) using
the C++ `rmf_fleet_adapter::agv::EasyFullControl` API with `perform_action`
support. Each adapter bridges RMF to robot/drone command endpoints via a ROS 2
action server (`ieee_fleet_msgs/action/ExecuteCommand`) and consumes robot
state updates from `ieee_fleet_msgs/msg/RobotState`.

Contract details: `ADAPTER_CONTRACT.md`.

## Quick Start

Launch both adapters:

```bash
ros2 launch ieee_fleet_adapter ieee_fleet_adapters.launch.xml
```

Run one adapter manually:

```bash
ros2 run ieee_fleet_adapter fleet_adapter -c /path/WS_Central/config/ground_fleet.yaml -n /path/WS_Central/config/ground_nav_graph.yaml
```

Use sim time:

```bash
ros2 run ieee_fleet_adapter fleet_adapter -c /path/WS_Central/config/ground_fleet.yaml -n /path/WS_Central/config/ground_nav_graph.yaml --use_sim_time
```

Optional schedule server override (ROS parameter `server_uri`):

```bash
ros2 run ieee_fleet_adapter fleet_adapter -c /path/WS_Central/config/ground_fleet.yaml -n /path/WS_Central/config/ground_nav_graph.yaml --ros-args -p server_uri:=http://SCHEDULE_SERVER:port
```

## Config + Nav Graphs

- `WS_Central/config/ground_fleet.yaml` + `WS_Central/config/ground_nav_graph.yaml`
- `WS_Central/config/drone_fleet.yaml` + `WS_Central/config/drone_nav_graph.yaml`

The config files follow the RMF `rmf_fleet` schema and include an `actions` list
with the `perform_action` categories supported by each fleet. If the list is
empty, all `perform_action` requests are rejected.

**RMF Fleet Settings**

- `rmf_fleet.actions`: Allowed `perform_action` categories.
- `rmf_fleet.reassign_task_interval`: Seconds between periodic
  `reassign_dispatched_tasks()` calls (default: 60 in the adapter).

**Fleet Manager Settings (`fleet_manager`)**

- `robot_state_topic`: Topic publishing `RobotState` for all robots (default:
  `/ieee_fleet/robot_state`).
- `command_action`: Action name exposed by each robot (default:
  `execute_command`).
- `use_robot_namespace`: If true, action servers are expected at
  `/<robot_name>/<command_action>`; otherwise `/<command_action>`.
- `action_server_wait_timeout`: Seconds to wait for an action server to appear
  (default: 2.0).
- `command_timeout`: Seconds to wait for a goal to be accepted (default: 5.0).
- `robot_state_update_frequency`: Hz for polling robot state from the latest
  `RobotState` messages.
- `state_stale_threshold_sec`: Mark robot unavailable when state is stale
  (default: 2.0).
- `battery_fallback_soc`: SOC fallback if missing/invalid (default: 1.0).
- `low_battery_threshold`: Decommission robot below this SOC (default: 0.2).
- `enable_battery_gating`: Enable low-battery gating (default: true).
- `critical_action_categories`: Actions allowed even when low on battery.
- `required_action_keys`: Per-category required keys for action payloads. The
  default map matches `ADAPTER_CONTRACT.md`.
- `action_timeouts`: Per-category timeouts (seconds).
- `default_action_timeout_sec`: Fallback action timeout (default: 30.0).
- `navigate_timeout_base_sec`: Base navigation timeout (default: 5.0).
- `navigate_timeout_per_meter_sec`: Per-meter navigation timeout (default: 3.0).
- `navigate_timeout_min_sec`: Min navigation timeout (default: 10.0).
- `navigate_timeout_max_sec`: Max navigation timeout (default: 300.0).
- `retry_backoff_sec`: Delay between command retry attempts (default: 1.0).
- `health_topic`: JSON health summary topic (default:
  `/ieee_fleet/adapter_health`).
- `health_publish_period_sec`: Health publish period (default: 2.0).
- `metrics_publish_period_sec`: Metrics log period (default: 30.0).
- `debug`: Enables extra action-client debug logging (default: false).

## ROS Interfaces

**Action Clients**

- `ieee_fleet_msgs/action/ExecuteCommand` on
  `/<robot_name>/<command_action>` or `/<command_action>`.

**Subscriptions**

- `ieee_fleet_msgs/msg/RobotState` on `fleet_manager.robot_state_topic`.
- `rmf_fleet_msgs/msg/LaneRequest` on `lane_closure_requests`.
- `rmf_fleet_msgs/msg/SpeedLimitRequest` on `speed_limit_requests`.
- `rmf_fleet_msgs/msg/ModeRequest` on `action_execution_notice`.

**Publishers**

- `rmf_fleet_msgs/msg/ClosedLanes` on `closed_lanes` (latched).
- `std_msgs/msg/String` on `fleet_manager.health_topic` (JSON map of
  `{robot: {age_sec, stale}}`).

## Command Payloads

The adapter sends navigation as `category=navigate` (or `dock`) with JSON:

```json
{"map_name":"L1","x":1.0,"y":2.0,"yaw":0.0,"speed_limit":0.0}
```

Docking includes the `dock` key:

```json
{"map_name":"L1","x":1.0,"y":2.0,"yaw":0.0,"speed_limit":0.0,"dock":"dock_name"}
```

RMF `perform_action` categories are forwarded as-is with JSON:

```json
{"category":"activate_antenna","description":{...}}
```

The command endpoint must update `last_completed_request` in `RobotState`
when a command finishes so the adapter can call `finished()` on the RMF
execution handle.

## Runtime Behavior

- Each robot has a monotonically increasing `command_id`; the adapter logs
  command lifecycle events as `CMD {...}`.
- New commands preempt active ones, issue a stop, and record a `preempted`
  outcome.
- If the action server is unavailable, the adapter retries the command every
  `retry_backoff_sec` until it is accepted.
- Navigation timeouts are computed from distance using the
  `navigate_timeout_*` settings; action timeouts come from `action_timeouts`
  or the action `description` (`duration_ms` or `duration_sec`), falling back
  to `default_action_timeout_sec`.
- When `RobotState.requires_replan` is true, the adapter triggers an RMF
  replan and reassigns dispatched tasks.
- When `RobotState` is stale or the battery is below `low_battery_threshold`,
  the adapter decommissions the robot (unstable) and recomissions it once
  healthy. Low-battery actions are rejected unless in
  `critical_action_categories`.
- A `ModeRequest` with `MODE_IDLE` on `action_execution_notice` will finish the
  current action for that robot if one is active.
- The adapter periodically calls `reassign_dispatched_tasks()` based on
  `rmf_fleet.reassign_task_interval`.

## Command Endpoint Contract

Each robot/drone should provide:

- Action server: `ieee_fleet_msgs/action/ExecuteCommand`
- State topic: `ieee_fleet_msgs/msg/RobotState`
