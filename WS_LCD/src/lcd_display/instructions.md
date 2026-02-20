Below is an objective, hand-off–ready instruction you can give to a developer to build the full solution (hardware + software). It assumes Raspberry Pi 5 + Ubuntu 24.04 and your chosen pins.

---

## Project Instruction: ST7789 SPI LCD on Raspberry Pi 5 (Ubuntu 24.04) using spidev + libgpiod

### Goal

Deliver a reusable, easy-to-use software stack for a Waveshare **1.69" 240×280 ST7789** SPI LCD module connected to a **Raspberry Pi 5** running **Ubuntu 24.04 (ARM64)**. The stack must support:

* Displaying full-screen and partial images (PNG/JPG/BMP)
* Drawing text
* Simple animations (frame-by-frame playback)
* A clean API suitable for integration into other projects

The solution **must not** depend on `/dev/mem` or the `bcm2835` library.

---

## 1) Hardware Requirements and Wiring

### LCD Module

* Waveshare 1.69" LCD module
* Resolution: 240×280
* Controller: ST7789 (ST7789V2)
* Interface pins present: `VCC, GND, DIN, CLK, CS, DC, RST, BL` (no touch)

### Raspberry Pi 5 Pin Assignments

Use BCM numbering and the 40-pin header.

**Power**

* LCD `VCC` → Pi `3V3` (Pin 1)
* LCD `GND` → Pi `GND` (Pin 6)

**SPI0**

* LCD `DIN` (MOSI) → Pi `GPIO10` (Pin 19)
* LCD `CLK` (SCLK) → Pi `GPIO11` (Pin 23)
* LCD `CS` → Pi `GPIO8 / CE0` (Pin 24)

**Control GPIO**

* LCD `DC` → Pi `GPIO13` (Pin 33)
* LCD `RST` → Pi `GPIO16` (Pin 36)
* LCD `BL` → Pi `GPIO26` (Pin 37) (on/off backlight)

### Hardware Deliverable

Provide a short hardware verification checklist that includes:

* Confirm SPI wiring
* Confirm BL toggles (backlight on/off)
* Confirm reset behavior (RST pulse)

---

## 2) OS and Kernel Configuration (Ubuntu 24.04) (You can skip this step since it would be done manually)

### Enable SPI

Instruct the user to:

1. Edit:

   * `/boot/firmware/config.txt`
2. Ensure these lines exist:

   * `dtparam=spi=on`
   * optionally `dtoverlay=spi0-1cs` (explicit enable CE0)
3. Reboot.

### Verify SPI Presence

Developer must provide verification commands and expected outputs:

* `ls /dev/spidev*` should show `/dev/spidev0.0`

### Install Dependencies

The deliverable must include installation steps for:

* C++ build tools
* `libgpiod` runtime + development headers
* `gpiod` CLI tools
* Optional: image decode dependencies (choose one approach and document it)

Minimum packages:

* `g++`, `cmake`, `make`
* `libgpiod-dev`, `gpiod`

### Permissions

Include steps to ensure a non-root user can access required devices:

* add user to `spi` and `gpio` groups
* explain session restart requirement (logout/login or `newgrp`)

---

## 3) Software Architecture Requirements (Use Layered Layout)

Implement the project using the layered structure previously defined:

* Hardware I/O layer: `spidev` + `libgpiod`
* ST7789 protocol driver layer
* Graphics/canvas framebuffer layer (RGB565)
* Asset loaders (images/fonts)
* Application-facing API layer

### Non-Functional Requirements

* Must build and run on Ubuntu 24.04 arm64 on Pi 5
* Must not require sudo for normal operation (after group permissions are set)
* Must be stable at typical SPI speeds (provide tested defaults and allow configuration)
* Must include clear logging or error messages when device access fails

---

## 4) Software Deliverables

### A) Core Library

Provide a C++ library that exposes:

1. **Display initialization**

   * panel init sequence (reset, sleep out, pixel format, display on)
   * configurable rotation (0/90/180/270)
   * optional x/y offsets if required by panel variant
2. **Backlight control**

   * on/off via GPIO26
3. **Frame presentation**

   * full-frame push (RGB565)
   * optionally partial update API (set window + push region)
4. **Graphics API**

   * a framebuffer/canvas object in RGB565 (240×280)
   * basic draw functions (clear, pixel, rect fill at minimum)
5. **Text**

   * at least one font mechanism:

     * either a simple bitmap font (fast, minimal deps) OR
     * FreeType-backed font rendering (better quality)
   * must support rendering ASCII text to the framebuffer
6. **Images**

   * ability to load PNG/JPG/BMP and display (scaled/cropped to 240×280)
   * image must be converted to RGB565 internally
7. **Animation**

   * frame-by-frame playback from a folder
   * configurable FPS
   * loop and non-loop modes
   * advise best practice: preconvert frames to RGB565 for speed (optional tool)

### B) Examples / Apps (must include)

Provide at least three runnable example programs:

1. `lcd_show_image`: loads and displays a specified image file
2. `lcd_text_demo`: renders text at specified coordinates
3. `lcd_play_animation`: plays frames from a folder at target FPS

Each example must:

* accept command-line arguments for file/folder and optional rotation/FPS
* use the public API (not internal classes directly)

### C) Tools (optional but recommended)

Provide a small utility tool to preconvert images/frames to raw RGB565 for fast playback:

* `png/jpg -> rgb565` conversion
* naming scheme for animation frames
* output format specification (endianness, width/height)

---

## 5) Configuration Parameters (Must be editable without code changes)

Provide a single config mechanism (CLI flags or a config file) for:

* SPI device path or bus/dev selection (`/dev/spidev0.0`)
* SPI speed (default + safe fallback)
* GPIO numbers for DC/RST/BL
* rotation
* optional offsets

---

## 6) Verification / Acceptance Tests

Developer must include a test checklist and “expected behavior”:

### Hardware Test

* BL toggles on/off from CLI tool or example app
* Screen powers on (backlight visible)
* Reset line toggling doesn’t crash program

### SPI Test

* Program can open and write to `/dev/spidev0.0` without sudo (after permissions)

### Functional Tests

1. Display a single full-screen image; verify correct orientation.
2. Render text; verify stable drawing with no tearing artifacts at normal refresh.
3. Animation:

   * run 10–15 FPS playback from folder
   * no crashes
   * consistent frame pacing

### Performance Baseline

Provide:

* recommended SPI speed default
* measured approximate FPS for full-frame updates at that speed

---

## 7) Documentation Requirements

Provide a README that includes:

* wiring table (BCM + physical pins)
* Ubuntu SPI enable steps
* dependency install steps
* build instructions (`cmake` workflow)
* how to run examples
* troubleshooting section:

  * missing `/dev/spidev*`
  * permission errors
  * blank/shifted display (rotation/offset guidance)

---

## 8) Constraints and Explicit Exclusions

* Do not use bcm2835, wiringPi, pigpio “register poke” modes, or `/dev/mem` mapping.
* Use only `spidev` for SPI and `libgpiod` for GPIO on Ubuntu 24.04.
* Do not assume Raspberry Pi OS tooling (`raspi-config`) exists.

---

## Definition of Done

The project is complete when:

* A clean build produces the library + example apps.
* A user can wire the LCD per the instructions, enable SPI, install deps, build, and:

  * display an image,
  * display text,
  * play an animation,
    without requiring root access (after permissions setup).

---

If you want, I can also rewrite this in a “contract style” format with explicit acceptance criteria and a short milestone plan (M1 hardware bring-up, M2 driver init, M3 framebuffer + text, M4 images + animation).
