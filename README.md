![EPD Painter logo](img/epdpainter_logo.png)

# EPD_Painter

A high-performance e-paper display driver for ESP32-S3, driving the **Soldered Inkplate 10** panel (ED097TC2, 9.7", 1200×825) through a combination of ESP32-S3 vector (SIMD) assembly, a custom LCD_CAM DMA pipeline, and a delta-update pixel system.

> **Port status.** The engine was developed on 960×540 boards (M5PaperS3, LilyGo T5 S3). This branch drives the Inkplate 10 only. The waveform tables are untuned seeds for this panel, so black and white are solid and the two greys need a tuning session (`examples/other/tuneup`). The timing figures in the deeper docs were measured on the 960×540 boards.

---

## What makes this different

Most ESP32-S3 e-paper drivers use the standard `esp_lcd_panel_io_i80` interface for pushing pixels. That interface has significant overhead and is the main bottleneck for high-speed EPD updates. EPD_Painter bypasses it entirely and drives the LCD_CAM peripheral directly — an approach inspired by [this Adafruit blog post](https://blog.adafruit.com/2022/06/21/esp32uesday-more-s3-lcd-peripheral-hacking-with-code/).

On top of that:

- **ESP32-S3 vector (SIMD) assembly** — pixel packing, waveform conversion, and delta detection are done 64 pixels at a time using 128-bit vector registers
- **Delta updates** — only pixels that have changed since the last frame are driven.
- **Dual-core pipeline** — the waveform rendering runs in the background core while your code continues on the main core.
- **4-shade greyscale** (white, light grey, dark grey, black) — the 2bpp pixel format maps directly onto the EPD waveform encoding with minimal conversion, keeping the pipeline fast

---

## Supported boards

| Board | Resolution | Preset |
|---|---|---|
| Soldered Inkplate 10, ESP32-S3 breakout | 1200×825 | `EPD_PAINTER_PRESET` (the only one) |

Wiring is the InkplateLibrary `inkplate10-s3-test` breakout (`INKPLATE10_S3_WIRING.md` there):

| Signal | GPIO | | Signal | GPIO |
|---|---|---|---|---|
| D0–D7 | 38, 39, 40, 41, 42, 47, 48, 21 | | CKV | 10 |
| CL | 4 | | LE | 11 |
| SPV | 5 | | SPH | 12 |
| GMOD | 6 | | I2C SDA / SCL | 8 / 9 |
| OE | 7 | | | |

The I2C bus carries a PCAL6416A expander at 0x20 (TPS65186 WAKEUP/PWRUP/VCOM/INT/PWR_GOOD on A0–A4) and the TPS65186 PMIC at 0x48. LCD_CAM routes the bus through the GPIO matrix, so the data lines being split across two GPIO registers costs nothing. Every pin is in `EPD_Painter_presets.h`.

---

## Installation

1. Clone this repository into your Arduino `libraries` folder:
   ```
   git clone https://github.com/your-repo/EPD_Painter ~/Arduino/libraries/EPD_Painter
   ```
2. Install the **Adafruit GFX Library** (required) via Arduino Library Manager
3. Install **LVGL v9** (optional — needed for LVGL examples) via Arduino Library Manager
4. Select **ESP32S3 Dev Module** with **PSRAM: OPI PSRAM**, **USB Mode: Hardware CDC and JTAG** and **USB CDC On Boot: Enabled**, or from the command line:
   ```
   arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app,USBMode=hwcdc,CDCOnBoot=cdc <sketch>
   ```
5. Flash `examples/inkplate10/inkplate10_hello` first. It reports the pixel clock and the PMIC state on Serial and draws an orientation and edge test.

---

## Quick start

```cpp
#include "EPD_Painter_Adafruit.h"

EPD_PainterAdafruit display(EPD_PAINTER_PRESET);

void setup() {
    display.begin();

    display.fillScreen(0);      // white background
    display.setTextColor(3);    // black text
    display.setTextSize(3);
    display.setCursor(40, 40);
    display.print("Hello, EPD!");

    display.paint();
}

void loop() {}
```

`EPD_Painter_presets.h` is now imported automatically by `EPD_Painter.h`, `EPD_Painter_Adafruit.h`, and `EPD_Painter_LVGL.h`. You only need the binding header you actually use.

### Orientation and timing knobs

The preset defaults to `Rotation::ROTATION_180`, which puts (0,0) in the same corner as the Inkplate library. `EPD_PAINTER_PRESET.withRotation(...)` picks another orientation. Board timing lives in the Config:

| Field | Default | Meaning |
|---|---|---|
| `pclk_hz` | 4 000 000 | CL pixel clock. 4 MHz gives 125 ns CL-high, the shortest pulse the bit-banged Inkplate driver found working on the breakout. Raise it once the wiring is proven. |
| `vscan_skip_lines` | 3 | Gate clocks after SPV before the first row latches, matching Inkplate's `vscan_start()`. |
| `skip_idle_cols` | false | Experimental: end each row after the frame's last changed column (scan order). Saves the cut share of row time; turn off if stray marks appear past the cut-off. |
| `g16_pass_us_*` | 75 000 | 16-grey pass period. Must exceed one row loop (825 × 308 bytes / `pclk_hz`), so scale it with the clock. |

Every waveform table is only valid at the row timing it was tuned at, so retune after changing `pclk_hz`.

---

## Bindings

| Binding | Header | Description |
|---|---|---|
| Adafruit GFX | `EPD_Painter_Adafruit.h` | Full Adafruit GFX API — `print()`, `drawBitmap()`, shapes, custom fonts |
| LVGL v9 | `EPD_Painter_LVGL.h` | Complete LVGL widget library; flush callback wired automatically |
| Raw driver | `EPD_Painter.h` | Bring your own 8bpp framebuffer |

---

## Core API

| Function | Description |
|---|---|
| `begin()` | Allocate buffers, init hardware and I2C |
| `paint()` | Delta-update the panel; blocks until the paint task picks it up, second pass runs in background |
| `paintLater()` | Fully non-blocking paint — hands off to background task immediately |
| `paintPacked()` | Paint from a pre-packed 2bpp buffer, skipping compaction |
| `unpaintPacked()` | DC-balance pass — drives every pixel in a packed buffer back toward white |
| `clear()` | Drive the **physical panel** to full white; optional rect list and `ClearMode` (HARD/SOFT) |
| `clearDirtyAreas()` | Compact framebuffer, compute dirty rects, clear only those areas |
| `computeDirtyRects()` | Return dirty rectangles between current display state and new framebuffer |
| `fxClear()` | Animated sweep-bar clear effect |
| `clearBuffers()` | Zero all packed buffers — reset DC-balance baseline before power-off |
| `drawGray8()` | Draw an 8bpp greyscale image, Floyd-Steinberg dithered against the panel's measured level curve |
| `drawGray8Fit()` | As above, scaled to fit the canvas and centred — the photo case |
| `setVcomMv(mv, persist)` | Set VCOM in mV (e.g. `-1250`). RAM only and re-applied at every power-up, or `persist = true` to program the TPS65186 EEPROM. `Config::power.vcom_mv` sets it from the config. `readVcomMv()` reads it back |
| `setQuality(q)` | `QUALITY_HIGH` / `QUALITY_NORMAL` / `QUALITY_FAST` — trades refresh speed for black depth. `QUALITY_BW` snaps to black and white and drives `Config::bw_passes` (default 4) passes |

**Ghosting removal:**
```cpp
display.clear();   // blank the physical panel
display.paint();   // repaint current framebuffer onto the clean panel
```

See [reference_manual.md](reference_manual.md) for full documentation with copy-paste examples.

---

## Colour depth

EPD_Painter uses 4 shades of grey (2 bits per pixel). This is a deliberate performance choice — more grey levels require proportionally more waveform passes per frame, multiplying refresh time. At 4 shades the driver keeps updates fast enough to feel interactive.

| Value | Shade |
|---|---|
| `0` | White |
| `1` | Light grey |
| `2` | Dark grey |
| `3` | Black |

For the LVGL binding, use the provided colour constants (`EPD_PainterLVGL::WHITE`, `::LT_GREY`, `::DK_GREY`, `::BLACK`) rather than standard LVGL colour functions.

---

## Shutdown handling

EPD_Painter provides graceful shutdown via `EPD_BootCtl`, handling the DC-balance and screen-state reconciliation problems that arise from EPD panels retaining their image through power loss.

**Off by default on the Inkplate 10.** The board has no power latch and no charger to tell a power-button boot from a USB one. With `setAutoShutdown(true)` called before `begin()`, every other reset paints the shutdown image and deep-sleeps, and the next reset undraws it (DC balance) and runs normally.

**Custom behaviour:** Create your own `EPD_BootCtl` instance and check `shutdownPending()` in `setup()` or `loop()` to intercept and show a confirmation UI before deciding to call `shutdown()` or `cancelShutdown()`.

The fallback shutdown image is a built-in fractal pattern. To supply your own, implement `EPD_BootCtl::IImageProvider` and return a PSRAM-allocated packed 2bpp buffer with rows of `EPD_Painter::packedRowBytesFor(width)` bytes (304 for this panel).

See [reference_manual.md](reference_manual.md) for full shutdown API documentation and examples.

---

## Examples

| Example | Binding | Description |
|---|---|---|
| `inkplate10/inkplate10_hello` | Adafruit GFX | First light: PMIC status, orientation/edge test, grey bars |
| `inkplate10/inkplate10_fast_counter` | Adafruit GFX | Centred counter repainted as fast as possible: FAST quality, row window, 10 MHz |
| `inkplate10/inkplate10_gif_player` | Adafruit GFX | Animated GIF from flash, scaled to fit, 4 flat greys; `gif2h.py` converts your own (needs AnimatedGIF) |
| `adafruit/bounce` | Adafruit GFX | Bouncing ball animation |
| `adafruit/breakout` | Adafruit GFX | A breakout game animation |
| `adafruit/circles` | Adafruit GFX | Animated circles |
| `adafruit/diagonal_lines` | Adafruit GFX | Line drawing |
| `adafruit/page_text` | Adafruit GFX | Text rendering |
| `lvgl/lvgl_hello_world` | LVGL | Minimal LVGL setup |
| `lvgl/lvgl_lighting_control` | LVGL | UI control panel demo |
| `other/dashboard` | Raw | Dashboard layout |
| `other/elevated` | Raw | ESP32 port of the Elevated 4K intro |
| `other/drift` | Raw | Julia set morphing animation |
| `other/grey16_testcard` | Raw | 16-grey test card |
| `other/megademo` | Raw | Demo reel of the engine's modes |
| `other/power_ghost_test` | Raw | Power-cycle ghosting check |
| `other/tuneup` | Raw | Self-tuning against a scanner rig — the route to real tables for this panel |

---

## Architecture overview

```
 ┌─────────────────────────┐
 │  8bpp GFX canvas (PSRAM)│  ← Adafruit GFX / LVGL draws here
 └────────────┬────────────┘
              │ SIMD pixel packing (assembly)
 ┌────────────▼────────────┐
 │ 2bpp fastbuffer (IRAM)  │  ← 64-pixel chunks, bitmask delta detection
 └────────────┬────────────┘
              │ waveform lookup + DMA
 ┌────────────▼────────────┐
 │   LCD_CAM DMA → panel   │  ← row-by-row, SPV/CKV/LE timing signals
 └─────────────────────────┘
```

Memory usage on the 1200×825 panel (2bpp rows are padded from 300 to 304 bytes so every row is a whole number of 64-pixel SIMD chunks):

| Buffer | Location | Size |
|---|---|---|
| 8bpp GFX canvas | PSRAM | ~967 KB |
| 2bpp fastbuffer | Internal RAM if ≥96 KB would remain, else PSRAM | ~245 KB |
| 2bpp screen / paint / light buffers | PSRAM | 3 × ~245 KB |
| 16-grey state + spill planes (on first `setGreyLevels(16)`) | PSRAM | ~3.1 MB |
| DMA row buffers | Internal RAM | 2 × 308 B |
