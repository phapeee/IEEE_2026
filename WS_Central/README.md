# IEEE 2026 Central Hub Workspace

This workspace is the trimmed central hub ROS 2 workspace. The `central_main` launcher brings up the RMF core stack, fleet adapters, and optional GPIO button events.

It includes RMF integration packages and IEEE 2026 fleet adapter tooling:

- `ieee_fleet_msgs`: action + message definitions for adapter ↔ robot command endpoints.
- `ieee_fleet_adapter`: RMF fleet adapter instances for the ground fleet and drone.
- `game_director`: mission executive that manages phase/task strategy and RMF dispatch lifecycle.
- `game_initiator`: GPIO watcher that calls `/game_director/start_game` when a configured start input is triggered.

## Launch

After building and sourcing this workspace, this launch will start:

- `rmf_traffic_schedule`
- `rmf_task_dispatcher`
- `ieee_fleet_adapter` (ground + drone)
- any optional components enabled in `config/central_main.yaml`

```bash
source install/setup.bash
ros2 launch central_main central_main.launch.py
```

Run the mission director separately:

```bash
source install/setup.bash
ros2 launch game_director game_director.launch.py
```

Run the GPIO-based game initiator separately:

```bash
source install/setup.bash
ros2 launch game_initiator game_initiator.launch.py
```

### Adapter Contract

See `WS_Central/src/ieee_fleet_adapter/ADAPTER_CONTRACT.md` for the frozen
command/state contract, action categories, and JSON schemas.

`central_main.launch.py` reads `config/central_main.yaml` by default. To override:

```bash
ros2 launch central_main central_main.launch.py \
  central_main_config:=/absolute/path/to/central_main.yaml
```

## Configuration

- `config/central_main.yaml` toggles components and RMF stack settings.
- `config/game_initiator.yaml` configures GPIO chip/lines, trigger edges, and pull-up priming for `game_initiator`.
