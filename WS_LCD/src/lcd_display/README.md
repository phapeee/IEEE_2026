# ST7789 ROS Wrapper (Pi 5, Ubuntu 24.04)

`st7789_ros_wrapper` provides a C++ ST7789 display stack (spidev + libgpiod) and ROS 2 integration for a Waveshare 1.69" 240x280 SPI LCD on Raspberry Pi 5.

This package does not use `/dev/mem`, `bcm2835`, `wiringPi`, or register-poke GPIO modes.

## Hardware Wiring (BCM + Physical Pins)

| LCD Pin | Signal | Pi BCM | Pi Physical Pin |
|---|---|---:|---:|
| `VCC` | 3.3V power | - | `1` |
| `GND` | Ground | - | `6` |
| `DIN` | SPI MOSI | `GPIO10` | `19` |
| `CLK` | SPI SCLK | `GPIO11` | `23` |
| `CS` | SPI CE0 | `GPIO8` | `24` |
| `DC` | Data/Command | `GPIO13` | `33` |
| `RST` | Reset | `GPIO16` | `36` |
| `BL` | Backlight | `GPIO26` | `37` |

## Ubuntu 24.04 SPI Enable

1. Edit `/boot/firmware/config.txt`.
2. Ensure these entries exist:
   - `dtparam=spi=on`
   - Optional: `dtoverlay=spi0-1cs`
3. Reboot.
4. Verify device node:

```bash
ls /dev/spidev*
```

Expected output should include `/dev/spidev0.0`.

## Dependencies

Install build tools and GPIO dependencies:

```bash
sudo apt update
sudo apt install -y g++ cmake make build-essential libgpiod-dev gpiod libopencv-dev
```

If you are building inside a ROS 2 environment, ensure your ROS distro is installed and sourced.
Image decoding for `PNG/JPG/BMP` uses OpenCV `imgcodecs`.

## Permissions (No Sudo Runtime)

Grant the current user access to SPI/GPIO:

```bash
sudo usermod -aG spi,gpio "$USER"
```

Then restart your shell session:
- log out/in, or
- run `newgrp spi` and `newgrp gpio` in a fresh shell.

Verify:

```bash
id
```

Groups should include `spi` and `gpio`.

## Build (CMake Workflow via Colcon)

From the workspace root:

```bash
cd WS_LCD
source /opt/ros/jazzy/setup.bash
colcon build --merge-install --packages-select st7789_ros_wrapper \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

This runs a standard CMake configure/build flow through `colcon`.

Alternative (project helper script):

```bash
cd WS_LCD
./build.sh
```

## Run

Source the workspace first:

```bash
cd WS_LCD
source install/setup.bash
```

Run the ROS node with package config:

```bash
ros2 run st7789_ros_wrapper st7789_display_node --ros-args \
  --params-file config/display.yaml
```

Or start it via launch file:

```bash
ros2 launch st7789_ros_wrapper display.launch.py
```

ROS topic interfaces exposed by the node:
- `~/image` (`sensor_msgs/msg/Image`): render image frames.
- `~/image_overlay` (`st7789_ros_wrapper/msg/ImageOverlay`): render image frames at per-message `x`, `y`, and `scale` (`float32`).
- `~/text` (`std_msgs/msg/String`): render text overlay using default `text_x`, `text_y`, and `text_scale` parameters.
- `~/text_overlay` (`st7789_ros_wrapper/msg/TextOverlay`): render text overlay with per-message `x`, `y`, and `scale` (`float32`).
- `~/backlight` (`std_msgs/msg/Bool`): turn panel backlight on/off.

Note: text rendering currently uses bitmap glyphs, so `text_overlay.scale` is rounded to the nearest integer (minimum `1`).

GPIO chip selection (ROS param):
- `gpiochip` (`string`, default `gpiochip4`): GPIO chip name or absolute device path (for example `gpiochip4` or `/dev/gpiochip4`).

Panel color controls (ROS params):
- `color_order_bgr` (`bool`, default `true`): when `true`, driver sets ST7789 MADCTL BGR bit; set `false` if red/blue channels appear swapped.
- `display_inversion` (`bool`, default `true`): when `true`, sends `INVERSION ON` (`0x21`); set `false` to send `INVERSION OFF` (`0x20`).

Dirty-rectangle render controls (ROS params):
- `use_dirty_rects` (`bool`, default `true`): enable partial-region SPI updates when possible.
- `dirty_rect_full_frame_threshold` (`double`, default `0.35`): if dirty-region coverage is at or above this ratio, use full-frame transfer instead.

Threaded presentation controls (ROS params):
- `use_threads` (`bool`, default `false`): enable background IO thread for asynchronous LCD transfers.
- `frame_queue_capacity` (`int`, default `3`): max queued frames before oldest frames are dropped.

Topic examples:

```bash
ros2 topic pub --once /st7789_display_node/text std_msgs/msg/String "{data: 'Hello ST7789'}"
ros2 topic pub --once /st7789_display_node/text_overlay st7789_ros_wrapper/msg/TextOverlay \
  "{text: 'Hello ST7789', x: 40, y: 120, scale: 1.5}"
ros2 topic pub --once /st7789_display_node/image_overlay st7789_ros_wrapper/msg/ImageOverlay \
  "{x: 40, y: 20, scale: 4.0, image: {height: 1, width: 1, encoding: 'rgb8', is_bigendian: 0, step: 3, data: [255, 0, 0]}}"
ros2 run st7789_ros_wrapper lcd_publish_image_overlay -- \
  --image /path/to/image.png \
  --x 40 --y 20 --scale 0.75
ros2 topic pub --once /st7789_display_node/backlight std_msgs/msg/Bool "{data: true}"
ros2 run image_tools cam2image --ros-args -r image:=/st7789_display_node/image
```

Run acceptance smoke test (backlight + reset path + full-frame benchmark):

```bash
ros2 run st7789_ros_wrapper lcd_acceptance_smoke -- \
  --spi-device /dev/spidev0.0 \
  --spi-speed-hz 32000000 \
  --dc-gpio 13 \
  --rst-gpio 16 \
  --bl-gpio 26 \
  --rotation-degrees 0 \
  --backlight-cycles 3 \
  --benchmark-frames 120
```

Run animation playback from a frame directory:

```bash
ros2 run st7789_ros_wrapper lcd_play_animation -- \
  --frames-dir /path/to/frames \
  --fps 12 \
  --loop
```

Run the unified non-ROS CLI command (image / animation / stop):

```bash
ros2 run st7789_ros_wrapper lcd_cli -- image --image /path/to/image.png --center
ros2 run st7789_ros_wrapper lcd_cli -- animation --frames-dir /path/to/frames --fps 12 --loop --center
ros2 run st7789_ros_wrapper lcd_cli -- stop
```

By default, `lcd_cli` loads display parameters from `config/display.yaml` (same schema as
`st7789_display_node`). You can override or disable this behavior:

```bash
ros2 run st7789_ros_wrapper lcd_cli -- --config /path/to/display.yaml image --image /path/to/image.png
ros2 run st7789_ros_wrapper lcd_cli -- --no-config animation --frames-dir /path/to/frames --fps 12
```

`lcd_cli -- image --center` and `lcd_cli -- animation --center` render content centered on the
canvas without scaling. Input dimensions must be less than or equal to display dimensions.

Run single-image display (auto fit/crop to panel size by default):

```bash
ros2 run st7789_ros_wrapper lcd_show_image -- \
  --image /path/to/image.png \
  --fit-to-screen
```

Run text rendering demo:

```bash
ros2 run st7789_ros_wrapper lcd_text_demo -- \
  --text "Hello ST7789" \
  --x 8 \
  --y 12 \
  --scale 2 \
  --text-r 255 --text-g 255 --text-b 0
```

## Examples

Implemented example binaries:
- `lcd_show_image`
- `lcd_text_demo`
- `lcd_play_animation`
- `lcd_cli`
- `lcd_publish_image_overlay`

Current status:
- Core text rendering is implemented via bitmap ASCII glyphs on `Canvas565::drawText(...)` and `Screen::drawText(...)`.
- ROS 2 runtime rendering is now available through topic-driven `st7789_display_node` subscriptions (`~/image`, `~/image_overlay`, `~/text`, `~/text_overlay`, `~/backlight`) with a timer-based present loop.
- `lcd_show_image` supports:
  - CLI parsing/validation in `ShowImageCli`
  - image decode for `PNG/JPG/BMP`
  - optional `--fit-to-screen` path that applies aspect-preserving scale + centered crop to panel size
- `lcd_text_demo` supports:
  - configurable text, position, scale, text color, and background color from CLI
- Animation playback toolchain is implemented:
  - CLI parsing/validation in `AnimationCli`
  - frame discovery from folder in `FrameSequence`
  - runtime playback executable `lcd_play_animation`
- Unified CLI tool `lcd_cli` is implemented:
  - `image` subcommand: load one image path to display
  - `animation` subcommand: play frames from a folder
  - auto-loads display runtime defaults from `config/display.yaml` (`--config`/`--no-config`)
  - `--center` option (image/animation): draw content centered without scaling
  - `stop` subcommand: stop an active `lcd_cli animation` playback process
- Current playback expects frame dimensions to match the configured display canvas size unless
  `lcd_cli animation --center` is used.

## Troubleshooting

### Missing `/dev/spidev*`

- Re-check `/boot/firmware/config.txt` contains `dtparam=spi=on`.
- Reboot after editing.
- Verify kernel device nodes again with `ls /dev/spidev*`.

### Permission errors on SPI/GPIO

- Ensure user is in `spi` and `gpio` groups.
- Start a new login shell after `usermod`.
- Re-run without `sudo`.

### Blank or shifted display

- Confirm wiring and backlight (`BL`) pin behavior first.
- Try rotation overrides:
  - `--ros-args -p rotation_degrees:=0|90|180|270`
- If image is shifted/clipped, adjust offsets:
  - `--ros-args -p x_offset:=<n> -p y_offset:=<n>`
- Start from `x_offset=0`, `y_offset=0` and tune incrementally per panel variant.

### Red/blue colors swapped or colors look wrong

- Toggle panel color order:
  - `--ros-args -p color_order_bgr:=false` (or `true`, depending on panel variant)
- Toggle display inversion:
  - `--ros-args -p display_inversion:=false` (or `true`)
- For `lcd_cli`, set these in `config/display.yaml` (or pass `--no-config` and rely on code defaults).

## Verification Checklist

See `verification_checklist.md` for Step 6 acceptance checks, expected outcomes, and performance reporting template.
