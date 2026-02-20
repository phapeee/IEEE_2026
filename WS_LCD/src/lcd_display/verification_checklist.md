# ST7789 Verification and Acceptance Checklist (Step 6)

This checklist defines the acceptance workflow and expected behavior for the current implementation state.

## Preconditions

1. Wiring matches the pin map in `instructions.md`.
2. SPI is enabled and `/dev/spidev0.0` exists.
3. User has `spi` and `gpio` group access (no `sudo` required for runtime).
4. Package is built:

```bash
cd WS_LCD
./build.sh
```

## Hardware Test

### Backlight toggle + reset smoke

Run:

```bash
cd WS_LCD
source install/setup.bash
ros2 run st7789_ros_wrapper lcd_acceptance_smoke -- \
  --spi-device /dev/spidev0.0 \
  --spi-speed-hz 32000000 \
  --dc-gpio 13 \
  --rst-gpio 16 \
  --bl-gpio 26 \
  --rotation-degrees 0 \
  --backlight-cycles 3 \
  --backlight-delay-ms 250 \
  --benchmark-frames 120
```

Expected behavior:
- Program initializes display without crashing.
- Backlight visibly toggles off/on for each cycle.
- Program exits with code `0` and prints `Acceptance smoke test passed.`

Notes:
- Reset line toggling is exercised during `Screen::begin()` through the ST7789 init sequence.

## SPI Test

Use the same smoke command above.

Expected behavior:
- SPI device opens without `sudo`.
- Frame benchmark runs without SPI write failures.

Failure indicators:
- `Failed to initialize display hardware.`
- `Frame present failed at frame index ...`

## Functional Tests

The following test definitions are fixed now and should be executed once the corresponding features are implemented:

1. Image display:
- Run `lcd_show_image` with a known 240x280 test image.
- Expected: correct orientation, no clipping artifacts.

2. Text rendering:
- Run `lcd_text_demo` with multiple lines.
- Expected: stable text drawing, no tearing/flicker at normal refresh.

3. Animation:
- Run `lcd_play_animation` at 10-15 FPS on a folder of frames.
- Expected: continuous playback, no crashes, consistent pacing.

## Performance Baseline

Recommended default:
- `spi_speed_hz = 32000000` (32 MHz)

Measurement method:
- Use `lcd_acceptance_smoke --benchmark-frames <N> --min-fps <target>`
- Record measured FPS from:
  - `Benchmark result: <N> frames in <seconds> s -> <fps> FPS`

Report template:
- Device: Raspberry Pi 5 + Waveshare 1.69" ST7789
- SPI speed: ___ Hz
- Frames: ___
- Measured FPS: ___
- Min-FPS threshold used: ___
- Pass/Fail: ___
