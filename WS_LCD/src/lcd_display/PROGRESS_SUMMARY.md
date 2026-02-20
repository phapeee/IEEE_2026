# ST7789 LCD Package Progress Summary

## Scope completed so far
This work established the ROS 2 package foundation, implemented Linux hardware I/O, implemented the ST7789 driver protocol layer, added runtime-editable display configuration via ROS parameters/YAML, implemented Step 6 verification assets (checklist + smoke test app), added Step 7 package README documentation, implemented Step 8 image decoding in `ImageLoader` (PNG/JPG/BMP), implemented Step 9 bitmap-font ASCII text rendering in the graphics/app layers, implemented Step 10 animation playback toolchain foundations (frame discovery + CLI + playback executable), implemented Step 11 example binaries (`lcd_show_image`, `lcd_text_demo`) plus image fit-to-screen support, implemented Step 12 ROS topics + launch integration for runtime display control, implemented Step 13 dirty-rectangle rendering support with full-frame fallback, implemented Step 14 optional threaded presentation with bounded frame queue, and added comprehensive unit tests for all implemented logic.

## Step 1: ROS 2 package scaffold
Completed:
- Created package root at `WS_LCD/src/lcd_display/` with package name `st7789_ros_wrapper`.
- Added `package.xml` and `CMakeLists.txt`.
- Added placeholder node: `src/nodes/st7789_display_node.cpp`.
- Added install rules for core docs (`instructions.md`, `software_structure.md`).

Build infrastructure update:
- Fixed stale package path check in `WS_LCD/build.sh`.
- Updated from `src/lcd_display/st7789_ros_wrapper/package.xml` to `src/lcd_display/package.xml`.

Verification:
- `./build.sh --ws WS_LCD` succeeded after the path fix.

## Step 2: Core C++ library structure
Completed:
- Added C++ module structure and skeleton implementations:
  - `hw`: `SpiBus`, `GpioLine`
  - `driver`: `St7789`
  - `gfx`: `Color565`, `Canvas565`
  - `assets`: `ImageLoader` (stub at this stage)
  - `app`: `Screen`
- Added `st7789_core` library target and linked node against it.

Verification:
- `./build.sh --ws WS_LCD` succeeded.

## Step 3: Real Linux hardware I/O implementation
Completed:
- Implemented `SpiBus` using Linux `spidev` syscalls:
  - open/close device
  - ioctl config (`mode`, `bits_per_word`, `speed_hz`)
  - robust write loop with partial-write handling
  - clear `lastError()` reporting
- Implemented `GpioLine` using `libgpiod`:
  - open chip, get line, request output
  - set value, release resources
  - clear `lastError()` reporting
- Added backend abstraction interfaces for both classes to enable full mock-based tests without hardware.

Build/dependency updates:
- Linked `st7789_core` with `gpiod` in CMake.
- Added package dependencies:
  - `libgpiod-dev`
  - `libgpiod2`
  - `ament_cmake_gtest`

Tests added:
- `test/test_spibus.cpp`
- `test/test_gpioline.cpp`

Test coverage includes:
- success and failure open/config paths
- invalid configuration validation
- write success/partial writes/failures
- request/set/release semantics
- re-request and destructor cleanup behavior

Verification:
- `./build.sh --ws WS_LCD` succeeded.
- Docker `colcon test` succeeded.

## Step 4: ST7789 protocol driver layer
Completed:
- Expanded `St7789` public API:
  - `initialize()`
  - `isInitialized()`
  - `setRotation()`
  - `setBacklight()`
  - `present()` (full frame)
  - `presentRegion()` (partial region)
  - `lastError()`
- Implemented protocol details:
  - reset pulse and recovery timing
  - initialization command sequence (`SWRESET`, `SLPOUT`, pixel format, MADCTL, inversion, display on)
  - command/data switching via DC line
  - address-window writes (`CASET`, `RASET`)
  - pixel stream write (`RAMWR`) with RGB565 big-endian byte order
  - rotation-aware logical width/height handling
  - panel offset support with overflow validation
- Updated `Screen` construction so canvas dimensions follow rotation.

Tests added:
- `test/test_st7789.cpp`

Driver test coverage includes:
- successful init command flow
- invalid rotation rejection
- SPI and GPIO bring-up failure handling
- reset-line toggle failures
- cleanup/release behavior on failure
- backlight pre/post initialization behavior
- full-frame write correctness
- region write correctness
- out-of-bounds and invalid-argument checks
- offset overflow guard checks

Verification:
- `./build.sh --ws WS_LCD` succeeded.
- Docker `colcon test` succeeded.
- Current test result: 25 tests, 0 failures.

## Step 5: Runtime configuration mechanism (no code edits required)
Completed:
- Added a single runtime config mechanism based on ROS 2 parameters (supports both CLI `--ros-args -p ...` and YAML parameter files).
- Added typed and validated runtime config translation:
  - new `app::ScreenRuntimeConfig`
  - new `buildScreenConfig(...)` validator/converter
- Wired node startup to declare/read these parameters:
  - `spi_device`, `spi_speed_hz`, `spi_mode`, `spi_bits_per_word`
  - `dc_gpio`, `rst_gpio`, `bl_gpio`
  - `rotation_degrees` (accepted values: `0`, `90`, `180`, `270`)
  - `x_offset`, `y_offset`
- Added sample configuration file:
  - `config/display.yaml`
- Updated install rules so config ships with the package.

Tests added:
- `test/test_screen_runtime_config.cpp`

Config test coverage includes:
- successful mapping of runtime values to `ScreenConfig`
- rejection of empty SPI device path
- SPI speed validation (non-positive and out-of-range)
- SPI mode validation (range `0..3`)
- bits-per-word validation
- GPIO offset validation
- rotation degree validation and conversion
- x/y offset bounds validation

Verification:
- `./build.sh` (inside `WS_LCD/`) succeeded.
- Docker `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 34 tests, 0 failures.

## Step 6: Verification / acceptance tests
Completed:
- Added a dedicated smoke-test executable:
  - `lcd_acceptance_smoke`
- Smoke-test flow uses public `Screen` API and performs:
  - display init (includes reset sequence),
  - backlight off/on cycling,
  - full-frame present benchmark,
  - optional minimum FPS threshold gate.
- Added reusable CLI/config utilities:
  - `AcceptanceSmokeConfig`
  - argument parser + validator
  - FPS calculation helper
- Added explicit Step 6 acceptance checklist document:
  - `verification_checklist.md`
  - includes commands, expected behavior, failure indicators, and performance report template.

Tests added:
- `test/test_acceptance_smoke_cli.cpp`

Test coverage includes:
- help/valid/invalid CLI argument parsing behavior
- validation rules for benchmark and backlight settings
- FPS computation edge cases
- usage text contains required options

Verification:
- Docker `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Docker `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 42 tests, 0 failures.

## Step 7: README and usage documentation
Completed:
- Added package README:
  - `README.md`
- README now includes:
  - wiring table with BCM + physical pin mapping
  - Ubuntu 24.04 SPI enable steps
  - dependency install commands
  - permissions setup for `spi` and `gpio` groups
  - build instructions using CMake workflow through `colcon`
  - run instructions for node and acceptance smoke tool
  - examples section (implemented vs planned)
  - troubleshooting for:
    - missing `/dev/spidev*`
    - permission errors
    - blank/shifted display with rotation/offset guidance
- Updated install rules so README is installed with package docs.

Verification:
- Documentation-only update for this step; no runtime logic changed.

## Step 8: Image decoding and format handling in `ImageLoader`
Completed:
- Implemented real image decoding in `ImageLoader::loadFile(...)`.
- Added support for `PNG`, `JPG/JPEG`, and `BMP` files.
- Added RGB565 conversion from decoded RGB888 image data.
- Added dimension validation and unsupported-extension/missing-file handling.

Build/dependency updates:
- Added OpenCV image decoding dependency in CMake (`core`, `imgcodecs`).
- Added package dependency entries for `libopencv-dev`.
- Updated README dependency instructions to include OpenCV.

Tests added:
- `test/test_image_loader.cpp`

Test coverage includes:
- empty path handling
- unsupported extension rejection
- missing-file handling
- successful decode + RGB565 conversion for PNG
- successful decode + RGB565 conversion for BMP
- successful decode for JPEG with expected dimensions
- invalid image payload rejection

Verification:
- Docker `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Docker `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 50 tests, 0 failures.

## Step 9: Text rendering/font subsystem (bitmap ASCII)
Completed:
- Added a bitmap font module:
  - `gfx::FontBitmap`
  - built-in public-domain 8x8 glyph table for ASCII (`U+0000..U+007F`)
  - glyph lookup helpers (`glyphRows`, `bitIsSet`, printable-range check)
- Extended `Canvas565` text drawing capabilities:
  - `drawChar(...)` with scaling and clipping behavior
  - `drawText(...)` with newline (`\\n`) and carriage-return (`\\r`) handling
- Added app-layer convenience API:
  - `Screen::drawText(...)`
- Updated README example status to reflect that core text rendering is now implemented (while standalone example binary remains pending).

Tests added:
- `test/test_font_bitmap.cpp`
- `test/test_canvas565_text.cpp`

Test coverage includes:
- glyph row lookup for known printable characters
- non-ASCII fallback behavior (`?`)
- glyph bit coordinate checks
- printable ASCII bound checks
- `drawChar(...)` pixel coverage at scale 1 and scale 2
- newline/carriage-return behavior in `drawText(...)`
- invalid input handling (`scale=0`, empty text)
- clipping behavior when characters are partially out of bounds

Verification:
- Docker `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Docker `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 62 tests, 0 failures.

## Step 10: Animation playback toolchain foundation
Completed:
- Added image blit support in the graphics/app layers:
  - `gfx::Canvas565::blit(...)`
  - `app::Screen::drawImage(...)`
- Added frame discovery utility:
  - `assets::listFramePaths(...)` in `FrameSequence`
  - scans a directory, filters supported image extensions, and returns sorted frame paths.
- Added animation playback CLI/config module:
  - `app::AnimationCliConfig`
  - parser/validator/usage helpers in `AnimationCli`
- Added standalone animation playback executable:
  - `lcd_play_animation`
  - loads frames from a folder, renders via public `Screen` API, supports FPS, loop/non-loop, and optional frame limit.
- Updated README run/examples sections to include current animation playback status.

Tests added:
- `test/test_canvas565_blit.cpp`
- `test/test_frame_sequence.cpp`
- `test/test_animation_cli.cpp`

Test coverage includes:
- pixel blit placement, clipping, and invalid-input behavior
- frame-directory validation, extension filtering, and deterministic sorting
- animation CLI parsing (help/valid/invalid), config validation, and usage text content

Verification:
- Docker `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Docker `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 78 tests, 0 failures.

## Step 11: Example binaries + image fit-to-screen support
Completed:
- Added standalone image example executable:
  - `lcd_show_image`
  - supports image path input, optional fit/no-fit mode, and display runtime config overrides.
- Added standalone text example executable:
  - `lcd_text_demo`
  - supports text content, x/y position, scale, text color, background color, and display runtime config overrides.
- Added new CLI/config modules:
  - `app::ShowImageCliConfig` + parser/validator/usage helpers
  - `app::TextDemoCliConfig` + parser/validator/usage helpers
- Extended image assets layer with fit-to-size loading:
  - `ImageLoader::loadFileFitToSize(...)`
  - performs aspect-preserving scale + centered crop to target display dimensions.
- Updated README run/examples sections with executable usage and new example status.

Tests added:
- `test/test_show_image_cli.cpp`
- `test/test_text_demo_cli.cpp`
- extended `test/test_image_loader.cpp` for fit-to-size behavior.

Test coverage includes:
- show-image CLI parsing (help/valid/invalid), config validation, and usage text content
- text-demo CLI parsing (help/valid/invalid), config validation, and usage text content
- image fit-to-size validation for invalid dimensions, requested output dimensions, and centered crop behavior

Verification:
- Docker `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Docker `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 95 tests, 0 failures.

## Step 12: ROS topics + launch integration for runtime display control
Completed:
- Upgraded `st7789_display_node` to provide timer-based topic rendering using the public `Screen` API:
  - `~/image` (`sensor_msgs/msg/Image`) for image frames.
  - `~/text` (`std_msgs/msg/String`) for text overlay updates.
  - `~/backlight` (`std_msgs/msg/Bool`) for backlight control.
- Added ROS image conversion support:
  - new `ros::convertImageMessageToRgb565(...)`
  - supports `rgb8`, `bgr8`, `rgba8`, `bgra8`, and `mono8`.
  - supports exact-size mode and aspect-preserving fit + centered crop mode.
- Added node render-parameter validation utilities:
  - new `ros::DisplayNodeRuntimeConfig` + `ros::buildDisplayNodeConfig(...)`.
  - validates `render_hz`, text placement/scale, and RGB565 color parameters.
- Added launch integration:
  - new `launch/display.launch.py`
  - installs launch directory with package.
- Updated package/runtime configuration:
  - expanded `config/display.yaml` with render and text topic defaults.
  - updated package dependencies for `sensor_msgs`, `std_msgs`, and launch.
- Updated README with launch usage and topic interface examples.

Tests added:
- `test/test_display_node_runtime_config.cpp`
- `test/test_image_message_converter.cpp`

Test coverage includes:
- valid/invalid node render parameter conversion and validation
- image conversion for supported encodings (`rgb8`, `bgr8`, `mono8`)
- center-crop fit behavior correctness
- unsupported encoding and invalid layout rejection
- exact-size dimension mismatch rejection

Verification:
- Containerized `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Containerized `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 106 tests, 0 failures.

## Step 13: Dirty-rectangle presentation with configurable full-frame fallback
Completed:
- Added graphics dirty-region utility:
  - new `gfx::DirtyTracker`
  - supports clipped rectangle marking, region union, full-canvas mark, and coverage metrics.
- Upgraded app screen rendering path:
  - `Screen` now tracks dirty regions from `clear`, `drawImage`, and `drawText`.
  - `Screen::present()` now selects between partial-region present and full-frame present.
  - full-frame fallback is triggered when dirty-region coverage exceeds a configurable threshold.
  - when dirty tracking is enabled and no new drawing occurred, `present()` now avoids unnecessary SPI transfer.
- Added runtime toggles to ROS display node config:
  - `use_dirty_rects`
  - `dirty_rect_full_frame_threshold` (range `[0.0, 1.0]`)
- Updated package default config:
  - expanded `config/display.yaml` with dirty-rectangle parameters.

Tests added:
- `test/test_dirty_tracker.cpp`
- `test/test_screen_dirty_present.cpp`
- extended `test/test_display_node_runtime_config.cpp` for dirty-rectangle parameter validation.

Test coverage includes:
- dirty-rectangle clipping, union, clear/reset, and coverage calculation
- partial update window selection for small text updates
- threshold-based fallback from partial update to full-frame push
- no-op present behavior when no new draw commands were issued
- runtime validation for dirty-rectangle threshold bounds

Verification:
- Containerized `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Containerized `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 119 tests, 0 failures.

## Step 14: Optional render+IO threaded presentation with bounded frame queue
Completed:
- Added new app-layer queue module:
  - `app::FrameQueue`
  - bounded queue semantics with drop-oldest behavior when full
  - blocking consumer pop + explicit stop signaling for clean shutdown.
- Upgraded `Screen` rendering path to support optional background display IO:
  - new `ScreenConfig` fields:
    - `use_threads`
    - `frame_queue_capacity`
  - `Screen::begin()` now starts an IO worker thread when `use_threads=true`.
  - `Screen::present()` now enqueues full-frame or dirty-region transfer payloads in threaded mode.
  - synchronous mode remains default behavior and remains fully supported.
  - dirty-region behavior and threshold fallback continue to apply in both sync and threaded modes.
  - added thread-safe access around direct driver operations (`initialize`, `setBacklight`, sync present path).
- Extended runtime configuration plumbing:
  - `ScreenRuntimeConfig` now supports:
    - `use_threads`
    - `frame_queue_capacity`
  - validation added for positive/non-overflow queue capacity.
- Updated ROS node/runtime defaults and docs:
  - `st7789_display_node` now declares `use_threads` and `frame_queue_capacity` parameters.
  - startup log includes threaded-mode settings.
  - `config/display.yaml` includes default values for both new parameters.
  - README documents threaded presentation parameters.

Tests added:
- `test/test_frame_queue.cpp`
- `test/test_screen_threaded_present.cpp`
- extended `test/test_screen_runtime_config.cpp` for queue-capacity and threaded runtime config validation.

Test coverage includes:
- queue push/pop ordering, drop-oldest behavior, and stop/unblock semantics
- asynchronous full-frame and dirty-region transfer path verification in threaded mode
- runtime config validation for `frame_queue_capacity` bounds and `use_threads` propagation

Verification:
- Containerized `colcon build --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Containerized `colcon test --merge-install --packages-select st7789_ros_wrapper` succeeded.
- Current test result: 108 tests, 0 failures.

## Files currently present (high-level)
Core files now include:
- `CMakeLists.txt`
- `package.xml`
- `README.md`
- `src/nodes/st7789_display_node.cpp`
- `config/display.yaml`
- `verification_checklist.md`
- `include/st7789_ros_wrapper/hw/SpiBus.hpp`
- `include/st7789_ros_wrapper/hw/GpioLine.hpp`
- `include/st7789_ros_wrapper/driver/St7789.hpp`
- `include/st7789_ros_wrapper/gfx/Color565.hpp`
- `include/st7789_ros_wrapper/gfx/Canvas565.hpp`
- `include/st7789_ros_wrapper/gfx/DirtyTracker.hpp`
- `include/st7789_ros_wrapper/gfx/FontBitmap.hpp`
- `include/st7789_ros_wrapper/assets/ImageLoader.hpp`
- `include/st7789_ros_wrapper/assets/FrameSequence.hpp`
- `include/st7789_ros_wrapper/app/Screen.hpp`
- `include/st7789_ros_wrapper/app/FrameQueue.hpp`
- `include/st7789_ros_wrapper/app/ScreenRuntimeConfig.hpp`
- `include/st7789_ros_wrapper/app/AcceptanceSmokeCli.hpp`
- `include/st7789_ros_wrapper/app/AnimationCli.hpp`
- `include/st7789_ros_wrapper/app/ShowImageCli.hpp`
- `include/st7789_ros_wrapper/app/TextDemoCli.hpp`
- `include/st7789_ros_wrapper/ros/DisplayNodeRuntimeConfig.hpp`
- `include/st7789_ros_wrapper/ros/ImageMessageConverter.hpp`
- `src/hw/SpiBus.cpp`
- `src/hw/GpioLine.cpp`
- `src/driver/St7789.cpp`
- `src/gfx/Canvas565.cpp`
- `src/gfx/DirtyTracker.cpp`
- `src/gfx/FontBitmap.cpp`
- `src/assets/ImageLoader.cpp`
- `src/assets/FrameSequence.cpp`
- `src/app/Screen.cpp`
- `src/app/FrameQueue.cpp`
- `src/app/ScreenRuntimeConfig.cpp`
- `src/app/AcceptanceSmokeCli.cpp`
- `src/app/AnimationCli.cpp`
- `src/app/ShowImageCli.cpp`
- `src/app/TextDemoCli.cpp`
- `src/nodes/DisplayNodeRuntimeConfig.cpp`
- `src/nodes/ImageMessageConverter.cpp`
- `src/tools/lcd_acceptance_smoke.cpp`
- `src/tools/lcd_play_animation.cpp`
- `src/tools/lcd_show_image.cpp`
- `src/tools/lcd_text_demo.cpp`
- `launch/display.launch.py`
- `test/test_spibus.cpp`
- `test/test_gpioline.cpp`
- `test/test_st7789.cpp`
- `test/test_screen_runtime_config.cpp`
- `test/test_frame_queue.cpp`
- `test/test_acceptance_smoke_cli.cpp`
- `test/test_image_loader.cpp`
- `test/test_font_bitmap.cpp`
- `test/test_canvas565_text.cpp`
- `test/test_canvas565_blit.cpp`
- `test/test_dirty_tracker.cpp`
- `test/test_screen_dirty_present.cpp`
- `test/test_screen_threaded_present.cpp`
- `test/test_frame_sequence.cpp`
- `test/test_animation_cli.cpp`
- `test/test_show_image_cli.cpp`
- `test/test_text_demo_cli.cpp`
- `test/test_display_node_runtime_config.cpp`
- `test/test_image_message_converter.cpp`

## Constraints respected during work
- C++ implementation only for package code (no Python package code added).
- No use of `/dev/mem`, `bcm2835`, `wiringPi`, or pigpio register pokes.
- Linux access model aligned with `spidev` + `libgpiod`.
