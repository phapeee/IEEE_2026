# game_director

`game_director` is the mission executive for the IEEE 2026 central workspace.
It does not drive robots directly. Instead, it decides *what should happen next*,
submits RMF tasks, tracks outcomes, and adapts to failures.

This README documents how the package actually works from source:

- node lifecycle and mission phases
- task state machine and scheduler behavior
- RMF dispatch/cancel integration
- world model inputs and external event contracts
- `task_pool.yaml` schema and customization workflow

## 1) Package At A Glance

Path: `WS_Central/src/game_director`

Important files:

- `src/game_director_node.cpp`: main ROS 2 node (`GameDirector`) and runtime logic
- `config/task_pool.yaml`: default mission plan (15 tasks)
- `launch/game_director.launch.py`: launch entrypoint

## 2) Core Concept

The director combines three things:

1. **World state** (robots, antenna states, duck states, crater state, drone state).
2. **Declarative task pool** (all candidate mission tasks with constraints and policy).
3. **Periodic scheduler** (every `tick_period_sec`) that transitions task states and dispatches/cancels RMF work.

At runtime it continuously asks:

- Which tasks are now valid (`READY`)?
- Which ones should be skipped/done already via postconditions?
- Which robot can run each task safely?
- Should a stuck task be canceled/retried?
- Is it time to advance to the next mission phase?

## 3) Node Interfaces

### Subscriptions

- `robot_state_topic` (default `/ieee_fleet/robot_state`)  
  Type: `ieee_fleet_msgs/msg/RobotState`  
  Used to update robot pose, map, battery, heartbeat, and fault-related fields.

- `dispatch_states`  
  Type: `rmf_task_msgs/msg/DispatchStates` (transient local QoS)  
  Used to track assignment/dispatch status from RMF dispatcher.

- `task_summaries`  
  Type: `rmf_task_msgs/msg/TaskSummary`  
  Used to detect queued/active/completed/failed/canceled task outcomes.

- `external_event_topic` (default `/game_director/events`)  
  Type: `std_msgs/msg/String` JSON payload  
  Used to inject game events (antenna/duck/crater/drone/fault/capability updates).

### Publications

- `status_topic` (default `/game_director/status`)  
  Type: `std_msgs/msg/String` JSON snapshot of current phase/tasks/locks/robots.

- `final_report_topic` (default `/game_director/final_report`)  
  Type: `std_msgs/msg/String` JSON (transient local) published at `COMPLETE`.

### RMF API endpoints used

- Topic `task_api_requests` (`rmf_task_msgs/msg/ApiRequest`)
- Topic `task_api_responses` (`rmf_task_msgs/msg/ApiResponse`)
- Service `cancel_task` (`rmf_task_msgs/srv/CancelTask`)
- Service `start_game` (`std_srvs/srv/Trigger`, default `/game_director/start_game`)

## 3.1) ROS Parameters

Declared by `GameDirector`:

- `task_pool_path` (string, default `""`)  
  Path to task pool YAML. Resolution order:
  1) this parameter (if provided and exists)
  2) `./config/task_pool.yaml` from current working directory
  3) installed share file `game_director/config/task_pool.yaml`
  4) `config/task_pool.yaml` found by walking parent folders from the installed/source module path (covers `WS_Central/config/task_pool.yaml`)

- `boot_validation_requirements` (string list)  
  Enabled BOOT checks. Default list:
  - `dispatch_states_publisher`
  - `task_summaries_publisher`
  - `task_api_requests_subscriber`
  - `expected_robots_seen`
  - `expected_robots_online`
  - `valid_robot_maps`
  - `finite_robot_pose`
  - `sane_battery_values`
  Special values:
  - `none` (or `off`, `disable_all`) disables all BOOT checks.
  Note: prefer these sentinels instead of `[]`, because empty arrays can fail ROS 2 parameter parsing on some distros.

- `robot_state_topic` (string, default `/ieee_fleet/robot_state`)
- `external_event_topic` (string, default `/game_director/events`)
- `status_topic` (string, default `/game_director/status`)
- `final_report_topic` (string, default `/game_director/final_report`)
- `start_service_name` (string, default `/game_director/start_game`)
- `match_duration_sec` (double, default `0.0`)  
  If `> 0`, overrides `settings.match_duration_sec` from YAML.

- `use_sim_time` (bool, default `false`)

From launch file, `task_pool_path`, `params_file`, and `use_sim_time` are exposed as launch arguments.  
To override other parameters, use `--ros-args -p ...`.

## 4) Mission Phase State Machine

`MissionPhase` values:

- `BOOT`
- `SAFE_HOLD`
- `READY`
- `GAME_START`
- `COMPLETE`

Phase progression logic:

1. `BOOT -> READY` when boot checks pass.
2. `BOOT -> SAFE_HOLD` if checks do not pass within `boot_wait_sec`.
3. `SAFE_HOLD -> READY` when checks recover.
4. `READY -> GAME_START` only after `start_game` service is called.
5. `GAME_START -> COMPLETE` when all tasks are terminal.

Notes:

- There is no hardcoded ANTENNA/DUCK/CRATER/DRONE progression in code.
- Task ordering is controlled by `task_pool.yaml` preconditions and dependencies.
- Typical setup:
  - ANTENNA and DUCK can run mixed during `GAME_START`.
  - DRONE is gated by `all_antennas_terminal`.
  - CRATER timing is controlled by its own preconditions.

## 5) Boot Checks And SAFE_HOLD

Boot validation checks are controlled by `boot_validation_requirements` (string list in `config/game_director.yaml`).
With the default list, BOOT requires all of the following:

- at least one publisher on `dispatch_states`
- at least one publisher on `task_summaries`
- at least one subscriber on `task_api_requests`
- all expected robots seen in world model
- all expected robots online (heartbeat not stale)
- all robots have a valid `map_name`
- all robots have finite `x, y, yaw`
- battery values are finite and in `[0.0, 1.0]`

If boot times out, director enters `SAFE_HOLD` and starts canceling in-flight tasks with hold semantics.  
When checks recover, it returns to `READY`.

In `READY`, no game tasks are dispatched. The only path to `GAME_START` is the start service:

```bash
ros2 service call /game_director/start_game std_srvs/srv/Trigger "{}"
```

## 6) Task Lifecycle State Machine

Each task instance (`TaskRuntime`) has lifecycle:

- `PENDING`
- `READY`
- `SUBMITTED`
- `RUNNING`
- `CANCELING`
- `DONE`
- `FAILED`
- `SKIPPED`
- `CANCELED` (enum exists; default logic mostly uses `FAILED/SKIPPED/DONE`)

High-level transitions:

- `PENDING -> READY` when phase, dependencies, preconditions, and retry/backoff checks pass.
- `READY -> SUBMITTED` on successful RMF API submit.
- `SUBMITTED -> RUNNING` when RMF summary becomes active.
- `RUNNING/SUBMITTED -> CANCELING` on timeout, robot offline, or safe-hold.
- `RUNNING/SUBMITTED -> DONE/FAILED` mainly from RMF completion/failure streams.
- `PENDING/READY -> DONE/SKIPPED` can happen immediately if success/dismiss conditions are already true.
- `FAILED -> PENDING` if attempts remain and backoff expires.
- terminal (`DONE/FAILED/SKIPPED/CANCELED`) stops regular scheduling for that task.

## 7) Scheduler Tick Flow

Every tick (`settings.tick_period_sec`, default `0.5s`) the node runs:

1. Refresh robot heartbeat freshness.
2. Drop expired dynamic priority boosts.
3. Handle `BOOT`, `SAFE_HOLD`, or `READY` special behavior.
4. Consume RMF API responses (submit accepted/rejected/timeouts).
5. Sync from RMF dispatch states + task summaries.
6. Enforce watchdogs:
   - queue timeout
   - running timeout
   - assigned robot offline
   - cancel retry/timeout handling
7. Recompute task readiness/dismiss/success/dependency outcomes.
8. Advance mission phase if criteria are met.
9. Dispatch up to 4 `READY` tasks (priority order).
10. Publish status JSON; publish final report when needed.

## 8) Robot Selection And Prioritization

Priority score for a ready task:

- `base` from YAML
- plus `deadline_boost_per_sec * elapsed_match_time`
- plus temporary dynamic boost (if active)
- plus `+10` when task phase equals current mission phase
- plus `+30` hard bonus for `CLEAR_BLOCKING_DUCK`

Robot selection modes:

- `dispatch.mode: robot_targeted`
  - uses `dispatch.robot` first, then preferred/allowed list fallback
  - requires resolved fleet+robot to submit
- `dispatch.mode: best_available`
  - director chooses best robot among candidates using health + penalties

Availability filters include:

- robot exists and online
- allowed/excluded robot filters
- required capabilities present
- battery above `min_battery_soc`
- not degraded for this task type
- per-robot in-flight limits (`major` and `micro`)
- duck tasks blocked if robot already carrying a duck

## 9) Locks, Dependencies, And Pairing

### Locks

`lock_keys` are per-task mutex keys (templated strings supported).  
A task dispatch proceeds only if all requested locks are free or already owned by same task.

Special lock behavior:

- key `crater_token` also updates world crater token owner.

### Dependencies

`depends_on` + policies:

- `dependency_policy: require_done` (default)
- `dependency_policy: require_terminal`

On dependency failure:

- `skip`
- `fail`
- `run_anyway`

### CRATER pair guard

For `CRATER_LAP_PAIR`, director checks that all crater-pair tasks are concurrently dispatchable before allowing one to submit.  
If one paired task fails, partner is canceled (`crater_partner_failed`).

## 10) RMF Dispatch Integration Details

### Submit request types

- `best_available` -> `dispatch_task_request`
- `robot_targeted` -> `robot_task_request` (requires `fleet` and `robot`)

Each outgoing request is rendered from YAML template and auto-populated with:

- `unix_millis_request_time` (if missing)
- `unix_millis_earliest_start_time` (if missing)
- `requester` from settings (if missing)
- `fleet_name` for best-available when a fleet is resolved
- labels:
  - `game_task:<task_id>`
  - `game_type:<task_type>`
  - `game_phase:<phase>`

### Response handling

- accepted submit must include `state.booking.id` -> becomes `rmf_task_id`
- rejected or timeout submit marks task failed/retry path
- RMF dispatch/summary streams then drive runtime state transitions

### Cancel handling

On cancel, director sends:

- `cancel_task_request` API message
- optional `cancel_task` service call when dispatch state is queued/selected

Cancel retries happen every `cancel_retry_period_sec` until success or `cancel_timeout_sec`.

## 11) World Model

`WorldModel` tracks:

- `robots`: pose, map, battery, heartbeat freshness, replanning flag, fault flags, carried duck
- `antennas`: `UNKNOWN/ACTIVATED/ATTEMPTED_FAILED/BLOCKED`
- `ducks`: `UNKNOWN/DETECTED/CLAIMED/CARRIED/IN_DROP_ZONE/LOST`
- `crater`: token owner, lap in progress, entry/exit clear, paired robots
- `drone_states`: per drone state
- expected robot lists and drop zone

World model updates come from:

- robot heartbeat topic
- director task outcomes (`on_task_success/on_task_failure`)
- external event JSON topic

## 12) External Event Contract (`/game_director/events`)

Topic type: `std_msgs/msg/String` with JSON in `data`.

Supported `event` values:

- `antenna_state`
- `duck_detected`
- `duck_claimed`
- `duck_carried`
- `duck_in_drop_zone`
- `duck_lost`
- `crater_state`
- `drone_state`
- `robot_fault`
- `robot_capability`

Examples:

```bash
ros2 topic pub /game_director/events std_msgs/msg/String \
  "data: '{\"event\":\"antenna_state\",\"antenna_id\":\"A1\",\"state\":\"ACTIVATED\"}'"
```

```bash
ros2 topic pub /game_director/events std_msgs/msg/String \
  "data: '{\"event\":\"duck_detected\",\"duck_id\":\"D1\",\"x\":1.2,\"y\":0.8,\"confidence\":0.9}'"
```

```bash
ros2 topic pub /game_director/events std_msgs/msg/String \
  "data: '{\"event\":\"robot_capability\",\"robot_name\":\"bot_keypad\",\"capability\":\"duck_pickup\",\"set\":true}'"
```

Invalid JSON is ignored with a warning.

## 13) Status And Final Report Payloads

### `/game_director/status` payload

Published every tick:

```json
{
  "phase": "GAME_START",
  "elapsed_match_sec": 42.5,
  "tasks": {"PENDING": 3, "RUNNING": 2, "DONE": 6},
  "locks": {"duck:D1": "collect_duck_d1"},
  "robots": {
    "bot_button": {"online": true, "map": "ws_map", "soc": 0.84, "faults": []}
  },
  "safe_hold_reason": ""
}
```

### `/game_director/final_report` payload

Published on `COMPLETE`:

```json
{
  "timestamp_unix_s": 1730000000.0,
  "phase": "COMPLETE",
  "antennas": {"A1": "ACTIVATED"},
  "ducks": {"D1": "IN_DROP_ZONE"},
  "crater": {
    "lap_in_progress": false,
    "entry_clear": true,
    "exit_clear": true,
    "paired_robots": []
  },
  "drone": {"drone1": "ON_GROUND"},
  "task_results": {
    "activate_antenna_a1": {"state": "DONE", "attempts": 1, "last_error": "", "rmf_task_id": ""}
  }
}
```

## 14) `task_pool.yaml` Reference

Default file locations:

- source copy: `WS_Central/src/game_director/config/task_pool.yaml`
- workspace convenience copy: `WS_Central/config/task_pool.yaml`
- install share copy: `install/game_director/share/game_director/config/task_pool.yaml`

### Top-level structure

```yaml
settings: {}
world: {}
tasks: []
```

### `settings` fields

- `requester`: requester tag inserted into RMF requests
- `tick_period_sec`: scheduler period
- `boot_wait_sec`: max boot validation wait before `SAFE_HOLD`
- `stale_robot_timeout_sec`: heartbeat stale threshold
- `min_battery_soc`: minimum SoC allowed for assignment
- `major_inflight_limit_per_robot`
- `micro_inflight_limit_per_robot`
- `failure_degrade_threshold`: consecutive failures before robot-task type is degraded
- `match_duration_sec`: used in deadline-based scoring
- `queued_timeout_default_sec`
- `running_timeout_default_sec`
- `api_response_timeout_sec`
- `cancel_retry_period_sec`
- `cancel_timeout_sec`

### `world` fields

- `antennas`: map `{antenna_id: initial_state}` or list of IDs
- `ducks`: list of duck IDs
- `crater_pair`: list of pair robot names (currently parsed but not actively used by scheduler)
- `drop_zone`: symbolic drop zone name
- `expected_ground_robots`: list used by boot checks and candidate selection
- `expected_drone_robots`: list used by boot checks and candidate selection

### Task object fields

- `id`, `type`, `phase`, `enabled`, `major`
- `target`: free-form map injected into template context
- `preconditions`, `dismiss_conditions`, `success_conditions`
- `lock_keys`
- `priority.base`, `priority.deadline_boost_per_sec`, `priority.opportunistic_boost`
- `retry.max_attempts`, `retry.backoff_sec`
- `timeouts.queued_sec`, `timeouts.running_sec`
- `dispatch.mode` (`best_available` or `robot_targeted`)
- `dispatch.fleet`, `dispatch.robot`
- `dispatch.request` (must include `category` and `description`)
- `allowed_robots`, `preferred_robots`, `required_capabilities`, `excluded_robots`
- `depends_on`
- `dependency_policy` (`require_done` or `require_terminal`)
- `on_dependency_failure` (`skip`, `fail`, `run_anyway`)
- `metadata` (free-form)

For this package version, runnable tasks should typically use `phase: GAME_START`.

### Supported condition kinds

- `always`
- `phase_is`
- `phase_in`
- `any_antenna_done`
- `all_antennas_terminal`
- `all_ducks_in_drop_zone`
- `antenna_state_in`
- `duck_state_in`
- `task_done`
- `task_terminal`
- `crater_token_free`
- `lock_free`
- `robot_online`
- `any_ground_robot_available`
- `any_drone_available`
- `match_time_before_sec`
- `match_time_after_sec`

Unknown condition kinds evaluate as false.

`phase_is` and `phase_in` evaluate against mission phase (`BOOT`, `SAFE_HOLD`, `READY`, `GAME_START`, `COMPLETE`).
Legacy phase names (`ANTENNA`, `DUCK`, `CRATER`, `DRONE`, `ENDGAME`) are treated as `GAME_START`.

### Template placeholders

String placeholders use `{{path}}` resolution against context:

- `task_id`, `task_type`
- `selected_robot`, `selected_fleet`
- `drop_zone`
- `now_unix_ms`
- `target` map
- flattened target keys (`{{duck_id}}`, `{{pickup_zone}}`, etc.)

If an entire string is one placeholder, original type is preserved.

### Minimal task example

```yaml
- id: example_activate_a1
  type: ACTIVATE_ANTENNA
  phase: GAME_START
  target:
    antenna_id: A1
    staging_waypoint: antenna
  preconditions:
    - kind: any_ground_robot_available
  dismiss_conditions:
    - kind: antenna_state_in
      antenna_id: A1
      states: [ACTIVATED, ATTEMPTED_FAILED]
  success_conditions:
    - kind: antenna_state_in
      antenna_id: A1
      states: [ACTIVATED]
  lock_keys: ["antenna:A1"]
  priority:
    base: 100.0
  retry:
    max_attempts: 3
    backoff_sec: 2.0
  timeouts:
    queued_sec: 20.0
    running_sec: 45.0
  dispatch:
    mode: best_available
    fleet: ground
    request:
      category: compose
      description:
        category: activate_antenna
        phases:
          - activity:
              category: sequence
              description:
                activities:
                  - category: go_to_place
                    description: "{{staging_waypoint}}"
                  - category: perform_action
                    description:
                      unix_millis_action_duration_estimate: 10000
                      category: activate_antenna
                      description:
                        antenna_id: "{{antenna_id}}"
```

## 15) Default Task Pool Walkthrough

The provided task pool defines 15 tasks:

- 4 antenna activation tasks: `activate_antenna_a1..a4`
- 5 duck collection tasks: `collect_duck_d1..d5`
- 2 crater paired tasks: `crater_lap_leader`, `crater_lap_follower`
- 4 drone tasks: `drone_takeoff`, `drone_scan_leds`, `drone_send_ir`, `drone_land`

Default assumptions encoded:

- ground robots: `bot_button`, `bot_keypad`, `bot_rotary`, `bot_crater`
- drone robot: `drone1`
- crater pair is targeted to `bot_button` + `bot_keypad`
- duck tasks use duck-specific pickup waypoints and common `drop_zone`
- all tasks are set to `phase: GAME_START`
- DRONE execution is gated by `all_antennas_terminal` preconditions
- CRATER timing is configured via crater task preconditions

## 16) Launch And Usage

Prerequisite: RMF dispatcher and fleet adapters should already be running.

Launch:

```bash
ros2 launch game_director game_director.launch.py
```

Override task pool path:

```bash
ros2 launch game_director game_director.launch.py \
  task_pool_path:=/absolute/path/to/task_pool.yaml
```

Override node params file (contains BOOT check toggles):

```bash
ros2 launch game_director game_director.launch.py \
  params_file:=/absolute/path/to/game_director.yaml
```

Use sim time:

```bash
ros2 launch game_director game_director.launch.py use_sim_time:=true
```

Observe runtime:

```bash
ros2 topic echo /game_director/status
ros2 topic echo /game_director/final_report
```

## 17) Tuning And Extension Workflow

Recommended way to adapt behavior:

1. Edit `task_pool.yaml` first (priorities, preconditions, dispatch templates, retries).
2. Validate condition logic and lock keys for deadlock/race risk.
3. Validate robot naming and fleet naming align with your adapter.
4. Use `/game_director/status` while testing to confirm lifecycle transitions.
5. Only change C++ logic when YAML-level policy is insufficient.

When adding a new task type:

1. Add task(s) in YAML.
2. Ensure adapters understand the `perform_action` category/description used.
3. Add/adjust world model success/failure side-effects if needed.
4. Add condition kinds in `src/game_director_node.cpp` only if existing kinds are insufficient.

## 18) Known Implementation Notes

- `world.crater.entry_clear`, `exit_clear`, and `paired_robots` are tracked and reported but not currently used in scheduler preconditions by default.
- `world.crater_pair` from YAML is parsed into the seed but not used directly by scheduling logic.
- Dynamic priority boosts for `CLEAR_BLOCKING_DUCK` exist in code; default task pool currently has only `COLLECT_DUCK` duck-clearing behavior.

These are useful extension points, not errors by themselves.
