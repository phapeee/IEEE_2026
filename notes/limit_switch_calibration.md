# Limit Switch Calibration Notes

## Overview
- ROS 2 Python node `limit_switch_calibration` (entry point `calibration_node`) runs calibration actions that move in 1-2 directions until a limit switch triggers, then hard-sets pose via `robot_localization/SetPose`.
- Legacy, file-driven state machines are still supported via `state_machine_file`.

## Interfaces
### Parameters
- `cmd_vel_topic` (TwistStamped), `cmd_vel_frame_id`
- `start_topic`, `status_topic`
- `smacc_event_topic`, `event_object_tag`, `event_source`, `success_event_type`, `failure_event_type`
- `set_pose_service`, `move_publish_rate_hz`, `default_timeout_sec`, `direction_timeout_sec`
- `action_name`, `linear_x_speed`, `linear_y_speed`, `pose_covariance_diagonal`, `default_pose_frame_id`
- `front_switch_topic`, `back_switch_topic`, `left_switch_topic`, `right_switch_topic`
- `state_machine_file`: optional YAML path for legacy state machines

### Action
- `limit_switch_calibration_msgs/LimitSwitchCalibration`
  - Goal: `directions[]` (`front|back|left|right`), `frame_id`, `x`, `y`, `yaw_deg`
  - Result: `success`, `message`, `failed_direction`
  - Feedback: `current_direction`, `status`

### Subscriptions
- `start_topic` (`std_msgs/String`): legacy state machine id to run
- `*_switch_topic` (`std_msgs/Bool`): front/back/left/right pressed state

### Publications
- `cmd_vel_topic` (`geometry_msgs/TwistStamped`): linear x/y only (yaw stays 0)
- `status_topic` (`std_msgs/String`): step/status messages
- `smacc_event_topic` (`smacc2_msgs/SmaccEvent`): success/failure events

### Service
- `set_pose_service` (`robot_localization/SetPose`): used by set_pose steps

## State Machine Format
- YAML top-level: `state_machines: { id: { description: "...", steps: [...] } }`
- Two modes:
  - Linear: steps list without `name` and no `start_state`. Runs in order.
  - Graph: any step has `name` or `start_state` is present. Uses explicit transitions.

### Step Types (Linear)
- `condition`: wait on `condition` (single), `any_of`, `all_of`, or `condition: timer` with `duration_sec`; uses `timeout_sec`.
- `move`: requires `duration_sec` > 0; sends `velocity_x`/`velocity_y` for the duration.
- `action`:
  - `set_pose`: uses `pose` (frame_id, x, y, yaw or yaw_deg, covariance_diagonal)
  - `terminate`: uses `result` and `message`

### Step Types (Graph)
- `move`: uses `velocity_x`/`velocity_y` and either:
  - `success_condition` or `success_any_of`/`success_all_of` + `timeout_sec` to branch, or
  - `duration_sec` for a fixed move
  - transitions: `next_successive_state`, `next_timeout_state`
- `set_pose`: calls SetPose; transitions `next_successive_state` or `next_failure_state`
- `condition`: same semantics as linear, then transitions to `next_successive_state`
- `terminate`: emits success/failure with `message`

### Conditions
- `front_pressed` / `front_released`
- `back_pressed` / `back_released`
- `left_pressed` / `left_released`
- `right_pressed` / `right_released`

## Runtime Behavior
- Action goals run sequential direction moves, then set pose on success.
- Start requests are ignored while an action is active.
- Publishes status per step and emits SMACC events on completion/termination.
- On shutdown, cancels the worker and publishes zero TwistStamped.
