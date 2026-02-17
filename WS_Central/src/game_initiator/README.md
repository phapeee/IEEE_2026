# game_initiator

`game_initiator` monitors one or more GPIO inputs and calls the game director
`start_game` service (`std_srvs/srv/Trigger`) whenever a configured trigger edge
is detected.

Default target service: `/game_director/start_game`

## Launch

```bash
source install/setup.bash
ros2 launch game_initiator game_initiator.launch.py
```

To use a custom parameter file:

```bash
ros2 launch game_initiator game_initiator.launch.py \
  params_file:=/absolute/path/to/game_initiator.yaml
```

## Parameters

Main parameters:

- `gpio_chip` (`string`): GPIO chip name (for example `gpiochip4`)
- `polling_frequency_hz` (`double`): polling rate
- `debounce_duration_ms` (`double`): debounce duration in milliseconds
- `start_service_name` (`string`): Trigger service path
- `shutdown_on_success` (`bool`): if true, shuts down this node after a successful service call
- `inputs` (`string[]`): list of input names
- `inputs.<name>.gpio_line` (`int`)
- `inputs.<name>.active_low` (`bool`)
- `inputs.<name>.pullup` (`bool`)
- `inputs.<name>.trigger_on` (`string`): `press`, `release`, or `both`
- `prime_pullups` (`bool`): if true, primes pull-up lines at startup
- `prime_pullups_script` (`string`): script path, defaults to
  `scripts/prime_gpio_pullups.sh`

## Configuration File

Workspace-level default file:

- `WS_Central/config/game_initiator.yaml`
