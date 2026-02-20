Here’s a clean “software blueprint” that makes an SPI ST7789 LCD easy to use for **images, text, and animation** on **Pi 5 + Ubuntu** with **spidev + libgpiod**. No code—just the structure and responsibilities.

---

## 1) Layered architecture

### Layer A — Hardware I/O (Linux)

**Goal:** Provide simple, reliable primitives:

* SPI write bytes (bulk transfer)
* GPIO set/clear (DC, RST, BL)

Modules:

* `SpiBus` (wraps `/dev/spidev0.0`)
* `GpioLine` (wraps configurable `gpiochip*` line requests)
* `Delay/Time` (sleep, frame pacing)

---

### Layer B — Display driver (ST7789)

**Goal:** Turn “draw this” into “send correct commands + pixel data”.

Responsibilities:

* `init()` (reset, sleep out, pixel format, orientation, display on)
* `setAddressWindow(x0,y0,x1,y1)` (CASET/RASET)
* `writeCommand(cmd)`, `writeData(bytes)`
* `pushPixelsRGB565(ptr, count)` (fast bulk blit)
* rotation + offsets (some panels need x/y offsets)
* backlight control (`setBacklight(bool)`)

This layer should be **purely about the ST7789 protocol**, not fonts or PNG decoding.

---

### Layer C — Framebuffer + drawing (graphics)

**Goal:** Let you draw shapes/text/images without thinking about SPI commands.

Core object:

* `Canvas` / `FrameBuffer`

  * Holds a pixel buffer in **RGB565** sized **240×280**
  * APIs like:

    * `clear(color)`
    * `setPixel(x,y,color)`
    * `drawLine/Rect/FillRect/Circle`
    * `blit(image, x, y)`
    * `drawText(font, x, y, "Hello")`

Then one method:

* `present()` → converts nothing (already RGB565) and calls driver `pushFrame(buffer)`

Key decision:

* Keep framebuffer in **RGB565 always** (fast, no conversions during present)

---

### Layer D — Assets (fonts, images, animation)

**Goal:** Load assets and convert to RGB565 buffers you can blit quickly.

Recommended components:

* `ImageLoader`

  * Load PNG/JPEG/BMP → decode into RGB888
  * Resize/crop/rotate (once)
  * Convert to RGB565 (once) → return `Image565`
* `Font`

  * bitmap fonts (fast, tiny) or FreeType-based fonts (better quality)
  * rasterize glyphs into a monochrome/alpha bitmap
  * blend into RGB565 framebuffer

For animation:

* `FrameSource`

  * from folder of frames (preconverted .rgb565 files, or PNG frames decoded once)
  * provides `nextFrame()` returning `Image565` or pointer to pixels

Performance note:

* Best animation setup is **preconvert frames to raw RGB565 files** and stream them.

---

### Layer E — Application-level API (what you call)

**Goal:** The “easy” interface your project uses.

Examples of top-level classes:

* `DisplayApp` or `Screen`

  * `showImage(path, fitMode)`
  * `showText(lines, font, layout)`
  * `playAnimation(folder, fps, loop=true)`
  * `draw(callback(Canvas&))` (custom drawing)
  * `setBrightness()` (if later you add PWM; otherwise on/off)

This layer should hide all:

* SPI device paths
* pin numbers
* init sequences
* pixel formats

---

## 2) Suggested project layout (C++)

### Folder structure

```
lcd/
  CMakeLists.txt
  include/
    lcd/SpiBus.hpp
    lcd/GpioLine.hpp
    lcd/St7789.hpp
    gfx/Canvas.hpp
    gfx/Color.hpp
    gfx/Font.hpp
    gfx/Image565.hpp
    gfx/ImageLoader.hpp
    app/Screen.hpp
    app/AnimationPlayer.hpp
  src/
    lcd/SpiBus.cpp
    lcd/GpioLine.cpp
    lcd/St7789.cpp
    gfx/Canvas.cpp
    gfx/ImageLoader.cpp
    app/Screen.cpp
    app/AnimationPlayer.cpp
  assets/
    fonts/
    images/
    animations/
  tools/
    convert_frames/   (optional: convert PNGs → raw RGB565 offline)
  examples/
    hello_text/
    show_image/
    animate_frames/
```

---

## 3) Data types to standardize early

These decisions make everything easy later:

### Pixel format

* Use **RGB565** as your internal framebuffer format.

### Color type

* `Color565` (uint16_t)
* helpers:

  * `rgb(r,g,b) -> Color565`
  * common colors

### Image type

* `Image565 { width, height, std::vector<uint16_t> pixels }`
* Optional: support a `stride` for fast row blits.

---

## 4) Typical runtime flow

1. `SpiBus` opens `/dev/spidev0.0` and sets mode/speed
2. `GpioLine` requests DC/RST/BL output lines
3. `St7789.init()`
4. App creates a `Canvas(240,280)`
5. Loop:

   * draw onto canvas
   * `screen.present(canvas)` → driver pushes full frame (or dirty rectangles)
   * sleep for frame pacing

---

## 5) Features you can add later (without redesign)

* **Dirty-rectangle updates**: only send changed areas to the LCD (big speed win)
* **Double-buffering**: avoid tearing for animation
* **Brightness control**: if BL moved to PWM hardware (GPIO18/12) or external transistor
* **Input events**: buttons/encoder via libgpiod (separate module)

---

## 6) Minimal “easy API” you should aim for

Even without code, define what you want to call:

* `screen.begin()`
* `screen.backlight(true/false)`
* `screen.clear(color)`
* `screen.drawText("...", x, y, font, color)`
* `screen.drawImage(img, x, y)`
* `screen.present()`
* `screen.playAnimation(source, fps)`

Everything else stays internal.

---

Below is a **Phase 1 / Phase 2** software plan that keeps your ST7789 stack clean and easy to use now, and gives you clear upgrade paths later (threads, dirty rects, fonts, ROS 2 packaging). Still **no code**, just structure + decisions.

---

## Phase 1: Solid baseline (simple + reliable)

### Goals

* Get display working with **spidev + libgpiod**
* Support: **clear, pixels, basic shapes, text (simple), image blit, basic animation**
* Keep architecture clean so Phase 2 upgrades don’t cause a rewrite

### Core modules (Phase 1)

**Hardware**

* `SpiBus`

  * open `/dev/spidev0.0`, set mode/speed, chunked `write()`
* `GpioLine`

  * request DC/RST/BL lines, set values

**Driver**

* `St7789Driver`

  * init sequence
  * `setAddressWindow()`
  * `writeCommand() / writeData()`
  * `pushFrameRGB565()` (full-frame)

**Graphics**

* `Canvas565` (240×280 RGB565)

  * draw primitives + “blit Image565”
* `Image565` (RGB565 buffer)
* `ImageLoader` (PNG/JPG decode → RGB565, resize/crop)

**App API**

* `Screen`

  * owns driver + a canvas
  * `showImage()`, `drawText()`, `present()`, `playFrames(folder, fps)`

### Phase 1 file layout

```
st7789_screen/
  include/
    hw/SpiBus.hpp
    hw/GpioLine.hpp
    drv/St7789Driver.hpp
    gfx/Canvas565.hpp
    gfx/Image565.hpp
    gfx/ImageLoader.hpp
    gfx/FontBitmap.hpp
    app/Screen.hpp
  src/...
  assets/...
  examples/...
```

---

## Phase 2: Performance + polish upgrades (your requested topics)

### 2A) Single-threaded vs Render thread + IO thread

#### Option 1: Single-threaded (keep as default)

**When it’s best**

* Simple apps
* Low FPS (< ~20) animations
* You want minimal complexity

**Loop**

* draw → convert assets if needed → SPI push → sleep

**Pros**

* Easiest debugging
* No synchronization bugs
* Deterministic behavior

**Cons**

* If decoding images/fonts takes time, it stutters animation
* SPI transfer blocks drawing

**Structure impact**

* No change. Phase 1 stays as-is.

---

#### Option 2: Render thread + IO thread (recommended for smooth animation)

**Concept**

* Render thread builds frames in memory (Canvas)
* IO thread pushes frames to LCD over SPI
* Use a ring buffer of 2–3 framebuffers (RGB565)

**Pros**

* Smooth animation even if rendering occasionally spikes
* IO timing isolated and stable

**Cons**

* More complexity: buffer ownership + synchronization
* Need careful shutdown behavior

**Add these modules**

* `FrameQueue` (ring buffer)
* `Renderer` (produces frames)
* `DisplayIO` (consumes frames, calls driver)

**Public API stays the same**

* `Screen.present()` becomes “enqueue frame”
* IO thread pushes in the background

---

### 2B) Full-frame blit vs Dirty-rect optimized

#### Option 1: Full-frame blit (Phase 1 default)

**What it is**

* Always push the whole 240×280 RGB565 buffer

**Pros**

* Simple
* Predictable
* Works well at moderate FPS if SPI clock is decent

**Cons**

* Wastes bandwidth when only small areas change (text updates)

**Keep it as default mode**

* Good baseline for correctness.

---

#### Option 2: Dirty rectangles (Phase 2 optimization)

**What it is**

* Track regions that changed
* Only set address window and push those parts

**Two approaches**

1. **Mark-dirty drawing ops**
   Every drawing call updates a `DirtyRegion` accumulator.
2. **Framebuffer diff**
   Compare current frame with previous frame, compute changed blocks (more CPU).

**Recommended Phase 2 plan**

* Start with **mark-dirty** (low CPU, easy)
* Use a single `DirtyTracker`:

  * union of rectangles
  * optional “merge if overlap” heuristic
* Add a threshold:

  * if dirty area > X% of screen, fall back to full-frame push

**New modules**

* `gfx/DirtyTracker.hpp`
* Driver adds `pushRegion(x,y,w,h, ptr/stride)`

---

### 2C) Fonts: Bitmap vs FreeType

#### Option 1: Bitmap fonts (Phase 1 + still useful later)

**What it is**

* Prebaked glyph bitmaps (1-bit or 8-bit alpha)

**Pros**

* Tiny dependency footprint
* Very fast rendering
* Perfect for UI labels, debug text

**Cons**

* Limited font sizes and styles
* Scaling looks rough unless you provide multiple sizes

**Structure**

* `FontBitmap` + `TextRenderer` that blits glyphs into Canvas

---

#### Option 2: FreeType fonts (Phase 2 “nice text”)

**What it is**

* Load TTF/OTF and rasterize glyphs at runtime

**Pros**

* High quality text
* Any size, kerning, unicode support

**Cons**

* Heavier dependency
* Rasterization cost (mitigate with glyph cache)

**Phase 2 recommended structure**

* `FontFreeType` (wrap FT_Library, FT_Face)
* `GlyphCache`

  * map: (codepoint, size) → glyph bitmap + metrics
* `TextLayout`

  * baseline, wrap, alignment
* `TextRenderer`

  * alpha blend glyph bitmap into RGB565 canvas

**Build options**

* CMake option: `-DENABLE_FREETYPE=ON`
* Keep bitmap fonts available even if FreeType is enabled (best of both worlds)

---

### 2D) ROS 2-friendly packaging (ament_cmake + render node)

#### Goal

Make the display usable as a ROS 2 component:

* show status, telemetry, small UI
* render images from a topic
* update text from messages/parameters

#### Package layout (recommended)

Create a ROS 2 workspace package, for example:

```
st7789_display/
  CMakeLists.txt           (ament_cmake)
  package.xml
  include/st7789_display/...
  src/
    st7789_node.cpp
    Screen.cpp (from your core lib or linked as subdir)
  launch/
    display.launch.py
  config/
    display.yaml
```

#### Architecture decision: split into 2 libraries

1. **Core library (non-ROS)**

   * `libst7789_core`: hw/driver/gfx/app API
   * usable in non-ROS programs and tests

2. **ROS wrapper package**

   * `st7789_display` depends on `libst7789_core`
   * adds node, topics, parameters

#### Suggested ROS interfaces

**Parameters**

* `spi_device` (`/dev/spidev0.0`)
* `spi_hz`
* `dc_gpio`, `rst_gpio`, `bl_gpio`
* `width`, `height`, `rotation`
* `use_dirty_rects`
* `use_threads`
* font selection

**Topics**

* `~/image` (sensor_msgs/Image) — show image full screen or in a region
* `~/text` (std_msgs/String) — render text overlay
* `~/backlight` (std_msgs/Bool) — on/off
* `~/fps` (std_msgs/Float32) — optional control

**Timer-based rendering**

* Node runs a timer at N Hz:

  * consume latest messages
  * render into canvas
  * present/enqueue frame

#### QoS considerations

* For image topic: `SensorDataQoS` or “best effort” if high rate
* For text: reliable

#### Testing

* Unit tests for:

  * rgb888→rgb565 conversion
  * dirty tracker
  * font rasterization (bitmap and/or freetype)
* Integration test optional: “headless” mode that writes frames to a file

---

## Phase 2 toggles (build/runtime)

Make Phase 2 features optional with clear flags:

**Build-time**

* `ENABLE_FREETYPE`
* `ENABLE_ROS2` (or separate package)
* `ENABLE_GPIOD` (always on in your case)

**Runtime/params**

* `use_threads: true/false`
* `use_dirty_rects: true/false`
* `font_backend: bitmap|freetype`

---

## Recommended Phase 2 progression (least pain)

1. Add **dirty tracking** (mark-dirty + threshold fallback)
2. Add **render+IO threads** with frame queue (optional mode)
3. Add **FreeType + glyph cache**
4. Wrap as **ROS 2 node** (keep core lib standalone)
