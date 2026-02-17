## 1) What the Game Director is responsible for

The Game Director is a **mission executive** that:

* Maintains a **live world model** of the game
* Maintains a **task pool** (all tasks that could/should happen)
* Decides **what tasks to submit/cancel/retry** and **when**
* Encodes strategy: priorities, robot preferences, hard constraints, and fallback rules
* Monitors progress through RMF dispatch states + robot reports
* Handles failures: reassigns, reroutes, escalates, and safely degrades

It does **not** drive motors, path planning, or grippers. It tells RMF “please execute this plan” and uses RMF’s outcomes to advance the mission.

---

## 2) Core architecture (internal modules)

### A) World Model (state aggregation)

Continuously fuses inputs into a single coherent state:

* **Robot state** (per robot): pose, mode, availability, fault flags, battery SOC, last_completed_request
* **Antenna state** (per antenna): unknown / activated / attempted_failed / blocked
* **Duck state** (per duck): unknown / detected (pose+confidence) / claimed / carried_by(robot) / in_drop_zone / lost
* **Crater state**: token free/busy, entry/exit clear, lap in progress, robots paired
* **Drone state**: on ground / airborne / scanning / reported / battery SOC (optional)
* **Clock**: match time, phase deadlines, timeouts

The World Model exposes clean query functions like:

* `antenna_remaining()`
* `unclaimed_ducks()`
* `robots_capable_of(task_type)`
* `robot_health_score(robot)`
* `is_phase_complete(phase)`

### B) Task Pool (declarative task objects)

Holds all tasks that might happen in the match, each as a structured object:

Each task includes:

* **Task ID** (stable)
* **Task type** (`ACTIVATE_ANTENNA`, `COLLECT_DUCK`, `CRATER_LAP_PAIR`, `DRONE_SCAN_LEDS`, `DRONE_SEND_IR`, etc.)
* **Preconditions** (must be true to run)
* **Dismiss conditions** (if true, task is canceled/ignored)
* **Success conditions** (postconditions; used for idempotency)
* **Priority function** (dynamic, can depend on match time, robot health, etc.)
* **Hard constraints** (allowed robots)
* **Soft preferences** (priority list of robots, or a scoring function)
* **Retry policy** (max attempts, backoff, escalation)
* **RMF request template** (how to translate to RMF “compose task” + perform_action)
* **Ownership/claim locks** (duck claim, crater token, etc.)
* **Runtime state** (PENDING/READY/SUBMITTED/RUNNING/DONE/FAILED/SKIPPED)

The package should read a task_pool.yaml, which contains all tasks, from config/ folder.

### C) Scheduler / Planner (decides actions)

Runs continuously (event-driven + periodic tick). It:

1. Updates which tasks are READY
2. Selects a set of tasks to submit based on:

   * priorities
   * per-robot queue limits
   * resource locks
   * phase strategy
3. Chooses dispatch mode:

   * robot-targeted (preferred list / hard assignment)
   * best-available (RMF chooses)
4. Issues RMF submit/cancel calls
5. Tracks tasks through RMF state transitions

### D) Dispatch Tracker (RMF integration)

Maintains the “truth” of what RMF is doing:

* Records which RMF task IDs correspond to which Game Director task IDs
* Watches dispatch state updates (queued/selected/dispatched/completed/failed/canceled)
* Converts RMF outcomes into task lifecycle transitions
* Detects “stuck” conditions (too long queued, too long running, no robot heartbeat)

### E) Fault Manager (robustness)

Centralizes failure handling:

* Robot offline / no heartbeat
* Battery too low
* Repeated execution failure for a task category
* Deadlock/congestion behaviors (too many robots trying to reach same region)
* Task-level escalation logic (reassign, change priority, skip, degrade strategy)

---

## 3) High-level flow: Match lifecycle

### Stage 0 — Boot / validation (pre-match)

Goal: ensure the system is ready and consistent.

1. **Wait for RMF core and fleet adapters** to be online
2. **Discover participants**

   * Ground robots: r1..r4 present, publishing state
   * Drone: drone1 present, publishing state
3. Validate invariants:

   * All robots have valid `map_name`
   * Battery SOC is sane
   * Robot poses are inside the field
   * No robot is already “busy” with a stale command_id
4. Initialize world model:

   * Antennas: `UNKNOWN`
   * Ducks: `UNKNOWN` (or create 5 duck IDs if you have them)
5. Enter match state: `RUNNING`

**If anything fails here:** enter a “SAFE HOLD” state (no tasks dispatched) and only resume when stable.

---

## 4) Phase-based strategy (with opportunistic overrides)

A robust director is **phase-driven** but still opportunistic. You’ll use “primary goals” per phase, but allow exceptions (like duck blocking an antenna).

### Phase 1 — Antenna activation (primary objective)

**Primary goal:** activate all 4 antennas (or attempt and mark failure).

#### Task pool initialization

Create 4 tasks:

* `ACTIVATE_ANTENNA(A1..A4)`
  Each has:
* Hard assignment or preferred robot list (your heuristic “each robot takes one antenna”)
* Preconditions:

  * robot healthy and SOC above threshold
  * antenna not activated and not already attempted_failed
* Dismiss:

  * antenna already activated
* Retry:

  * limited retries; escalate if blocked
* RMF template:

  * travel to antenna staging point
  * perform_action `activate_antenna` with antenna_id

#### Opportunistic “duck-in-the-way” override

If a robot reports:

* “activation failed: duck blocking”
  the director can:

1. Create a high-priority `CLEAR_BLOCKING_DUCK` (or a normal `COLLECT_DUCK`) targeted to that robot (or nearest robot)
2. Temporarily pause/requeue antenna activation after clearing

This keeps your heuristic intact but doesn’t get stuck.

#### Completion criteria

Phase 1 completes when each antenna is either:

* `ACTIVATED` OR
* `ATTEMPTED_FAILED` (after defined retry policy)

---

### Phase 2 — Duck collection (continuous objective)

**Primary goal:** move all 5 ducks to the drop zone.

This phase can start **as soon as a robot finishes its antenna** (don’t wait for all antennas).

#### Duck task generation

As perception detects ducks, the director:

* Creates/updates `COLLECT_DUCK(Dj)` tasks
* Maintains a **duck claim lock** so only one robot targets a duck at a time

#### Robot assignment policy

For each duck task, choose robot based on scoring:

* Distance / ETA to duck
* Current load (busy robots penalized)
* Battery SOC
* Capability flags (gripper health)
* Heuristic preferences (e.g., “robot that finished antenna first gets first duck”)

#### Task template (typical)

* Navigate near duck pose
* perform_action `pickup_duck` (includes duck_id, optional pose hint)
* Navigate to drop zone
* perform_action `drop_duck`

#### Dismiss and postconditions

* If duck already in drop zone → dismiss the task
* If duck lost / not seen → downgrade priority until re-detected
* If robot already carrying a duck → don’t assign another pickup task

#### Completion criteria

All ducks are `IN_DROP_ZONE` (or confirmed absent if rules allow).

---

### Phase 3 — Crater lap (special cooperative objective)

**Primary goal:** two specific robots perform a coordinated lap and exit.

This is implemented as a **parent task** that spawns two synchronized child tasks.

#### Preconditions for starting

* Both designated robots are healthy and above SOC threshold
* Crater token is free
* Field conditions acceptable (optional: low congestion)

#### Resource lock

Acquire `crater_token` to prevent other crater-related tasks from starting.

#### Dispatch pattern

Dispatch two tasks nearly simultaneously:

* `CRATER_LAP(robot_leader)`
* `CRATER_LAP(robot_follower)`

Each task is a travel + perform_action sequence where SMACC2 handles the real cooperation.

#### Failure handling

If one robot fails mid-lap:

* Abort the partner safely
* Mark crater attempt as failed (or retry if time allows)
* Release crater token
* Optionally reassign to backup pair if permitted

#### Completion criteria

Both child tasks report completed and both robots are out of crater.

---

### Phase 4 — Drone scan and report (endgame)

**Primary goal:** drone takes off, scans antenna LEDs, sends IR report, lands.

This phase triggers when:

* All antennas are `ACTIVATED` or `ATTEMPTED_FAILED`
  (and optionally when robots are no longer actively moving near antennas for better visibility)

#### Drone task sequence

* `DRONE_TAKEOFF`
* `DRONE_SCAN_LEDS` (returns LED colors/status)
* `DRONE_SEND_IR` (includes scan result payload)
* `DRONE_LAND`

The director can package this as:

* One compose task with multiple perform_action steps, or
* Separate tasks in sequence (easier to recover per-step)

#### Failure handling

* If scan fails: retry scan N times (different hover positions if supported)
* If IR send fails: retry send; if persistent, store result locally as “partial success”
* Always attempt landing as a safety action

#### Completion criteria

IR report sent (or defined “attempted_failed” state reached) and drone landed.

---

## 5) How tasks move through the system (end-to-end)

### Step 1 — Task becomes READY

Preconditions are satisfied and not dismissed.

### Step 2 — Director chooses dispatch mode

* Robot-targeted: when you have a preferred list / hard assignment
* Best-available: when any ground robot can do it

### Step 3 — Submit to RMF

Director generates the RMF request (compose + perform_action data), attaches:

* priority
* target robot (optional)
* labels/metadata (useful for logs)

### Step 4 — RMF dispatches

RMF assigns the task to a robot and triggers your fleet adapter.

### Step 5 — Execution happens via endpoints

Fleet adapter sends `ExecuteCommand(command_id, category, description_json)` to the robot/drone endpoint.

* Endpoint triggers SMACC2
* SMACC2 calls Nav2 (ground) or executes flight actions (drone)

### Step 6 — Completion signal flows back

* Endpoint updates `last_completed_request` and returns success/failure
* Fleet adapter reports completion to RMF
* RMF publishes task completion state
* Director updates its task object (DONE/FAILED/etc.) and triggers next tasks

---

## 6) Robustness features the Director must include

### A) Idempotency everywhere

Director assumes tasks can be retried safely.

* “activate antenna” can return “already activated”
* “pickup duck” can return “duck not present” and director will re-detect and retry

### B) Explicit timeouts

For every task category:

* `queued_timeout` (RMF didn’t dispatch quickly enough)
* `run_timeout` (SMACC2 not finishing)
  Timeout action:
* cancel RMF task
* release locks
* requeue with different robot or degrade strategy

### C) Per-robot inflight limits

Prevent flooding RMF. Example:

* At most 1 active “major task” per robot
* Allow 1 “opportunistic microtask” (like clearing a blocking duck) only if safe

### D) Failure scoring and capability downgrades

If a robot fails repeatedly at a task type:

* mark capability degraded (e.g., gripper unreliable)
* remove robot from eligible list for pickup tasks
* keep it doing navigation-only tasks if possible

### E) Heartbeat watchdog

If robot state stops updating:

* mark robot OFFLINE
* cancel or abandon tasks assigned to it
* release any locks it holds (duck claim, crater token)
* reassign tasks

### F) Congestion control

If too many robots target the same area:

* throttle tasks for that region
* use staging points
* reduce max speed or stagger dispatch

### G) Safe fallback actions

On major fault:

* send “stop/hold” to all robots
* park the drone
* switch to reduced goals (e.g., only antennas, skip crater)

---

## 7) Recommended “director tick” algorithm (conceptual)

Every cycle (or on relevant events):

1. Update World Model
2. Recompute task statuses:

   * DONE via postconditions
   * SKIPPED via dismiss rules
   * READY if preconditions satisfied
3. Update resource locks:

   * release if owners finished/failed/offline
4. Build candidate list of READY tasks and sort by:

   * priority (dynamic)
   * deadline proximity
   * strategic phase weights
5. For each candidate (highest first):

   * choose eligible robots based on hard constraints + capabilities + SOC
   * apply preference ordering
   * check per-robot inflight limits
   * submit to RMF (robot-targeted or best-available)
6. Monitor RMF dispatch updates:

   * if failure to assign → fallback to next robot / different plan
   * if queued too long → cancel + resubmit
7. Log everything with stable IDs:

   * task_id, rmf_task_id, robot_name, command_id

---

## 8) End-of-game behavior

When the director declares “mission complete”:

* optionally issue docking/parking tasks for robots
* ensure drone lands
* publish final report: antenna results, duck results, crater lap results, drone IR sent status
* freeze further dispatch unless an operator restarts the match
