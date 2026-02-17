# IEEE 2026 RMF Adapter Contract (Central Hub)

This document freezes the contract between the central RMF fleet adapter and
robot/drone command endpoints.

## Action Interface

**Action name:** `execute_command`

**Action type:** `ieee_fleet_msgs/action/ExecuteCommand`

### ExecuteCommand.Goal
- `command_id` (uint64)
  - Monotonic per robot.
  - Adapter increments for each command it sends to that robot.
- `category` (string)
  - `navigate`
  - `dock`
  - Any `perform_action` category (e.g., `activate_antenna`, `pickup_duck`).
- `description_json` (string)
  - JSON-encoded payload. See schemas below.

### Completion Semantics
Robots **must** update `last_completed_request` in `RobotState` to the most
recent `command_id` they have finished. The adapter uses this to call
`execution.finished()` in RMF.

## Robot State Interface

**Topic:** `/ieee_fleet/robot_state`

**Message:** `ieee_fleet_msgs/msg/RobotState`

**Required fields**
- `stamp`: ROS time of the state measurement.
- `robot_name`: matches RMF robot name.
- `map_name`: RMF map/level name (string).
- `x`, `y`, `yaw`: position in meters and radians.
- `battery_soc`: **0.0–1.0**, where 1.0 is full.
- `last_completed_request`: latest finished `command_id`.
- `requires_replan`: set true when the robot wants RMF to replan.

**Notes**
- If battery SOC is unknown, publish `NaN` or any out-of-range value
  (e.g., `-1.0`), and the adapter will fall back to a configured default.
- `requires_replan=true` should be used for local navigation failures, blocked
  paths, or localization issues that require RMF to issue a new plan.

## JSON Payload Schemas

### `navigate`
```json
{
  "map_name": "L1",
  "x": 1.0,
  "y": 2.0,
  "yaw": 0.0,
  "speed_limit": 0.0
}
```

### `dock`
```json
{
  "map_name": "L1",
  "x": 1.0,
  "y": 2.0,
  "yaw": 0.0,
  "speed_limit": 0.0,
  "dock": "dock_name"
}
```

### `perform_action` categories
The adapter forwards RMF perform_action categories directly. The payload is:

```json
{
  "category": "activate_antenna",
  "description": {
    "target": "antenna_A",
    "duration_sec": 3.0
  }
}
```

The `description` object is free-form JSON, but each action category should
follow a published schema agreed by robot and task teams.

The adapter enforces required keys per category. You can override the default
required-key map via `fleet_manager.required_action_keys` in the fleet config.

## Semantics

### `command_id`
- Monotonic per robot.
- Used to detect duplicates and correlate completion.

### `requires_replan`
- When true, the adapter calls `replan()` for that robot and logs the cause.
- The robot should return to `requires_replan=false` after the next state
  update once the issue is resolved.

## Ground Fleet Action Categories (default)

Each action payload is the `description` object in the `perform_action` schema.

| Category | Required keys | Optional keys |
| --- | --- | --- |
| `deploy` | `site` (string) | `duration_sec` (number) |
| `activate_antenna` | `antenna_id` (string) | `duration_sec` (number) |
| `pickup_duck` | `pickup_zone` (string) | `duck_id` (string), `duration_sec` (number) |
| `drop_duck` | `drop_zone` (string) | `duck_id` (string), `duration_sec` (number) |
| `crater_lap_leader` | `lap_count` (int) | `direction` (string), `duration_sec` (number) |
| `crater_lap_follower` | `leader_id` (string) | `duration_sec` (number) |
| `dock` | `dock` (string) | `duration_sec` (number) |

## Drone Fleet Action Categories (default)

| Category | Required keys | Optional keys |
| --- | --- | --- |
| `takeoff` | `altitude_m` (number) | `duration_sec` (number) |
| `scan_leds` | `pattern` (string) | `duration_sec` (number) |
| `send_ir` | `payload` (string) | `duration_sec` (number) |
| `land` | None | `duration_sec` (number) |
