# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

EPD_Painter is a high-performance e-paper (EPD) display driver for the ESP32-S3, driving the **Soldered Inkplate 10** panel (ED097TC2, 1200×825, 8-bit parallel) on the InkplateLibrary `inkplate10-s3-test` breakout. It uses ESP32-S3 vector (SIMD) assembly, custom LCD_CAM DMA, and a delta-update pixel pipeline. The engine was developed on 960×540 M5PaperS3/LilyGo boards; support for those was removed in the Inkplate 10 port, and timing figures in the deeper docs come from them.

## Build System

This is an **Arduino library** (and project). Build and flash via **Arduino IDE**:
- Open `src/src.ino` as the entry point
- Required library: **Adafruit GFX Library**
- Optional library: **LVGL v9** (for LVGL-based examples)
- Board: ESP32S3 Dev Module, OPI PSRAM — `esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app,USBMode=hwcdc,CDCOnBoot=cdc`
- First-light sketch: `examples/inkplate10/inkplate10_hello`

There is no Makefile or CMake. Arduino IDE is the primary build tool, and `arduino-cli compile` with the FQBN above works for checking builds.

## Architecture

### Three-Stage Pixel Pipeline

1. **8bpp GFX canvas** (PSRAM) — drawn to via Adafruit GFX or LVGL
2. **2bpp packed fastbuffer** (internal RAM if it fits with headroom, else PSRAM) — 64-pixel chunks with bitmasks
3. **Ink drive format** — sent row-by-row over LCD_CAM DMA to the EPD panel

Packed rows use a stride of `packedRowBytes()` = width/4 rounded up to 16 bytes (1200px → 304, not 300): every SIMD path and the per-row chunk mask need whole 64-pixel chunks. The padding columns are zero and are clocked out after the visible ones. Never size a packed buffer as `width*height/4`; use `packedSize()`. `_pack()` is the single place an 8bpp canvas becomes the physical layout, for every rotation and both depths.

### Key Source Files

- `src/EPD_Painter.h/.cpp` — Core driver: hardware init, LCD_CAM DMA setup and pixel clock (`Config::pclk_hz`), `paint()`, `clear()`, `sendRow()` row timing (SPV/CKV/LE signals, `vscan_skip_lines` lead-in), `_endPass()` tail latch, `_pack()`
- `src/EPD_Painter.S` — **Xtensa assembly**: all pixel packing, waveform conversion, `ink_on`/`ink_off` delta detection, 64-pixel SIMD chunks
- `src/EPD_Painter_Adafruit.h` — Wrapper exposing Adafruit GFX API; owns 8bpp framebuffer in PSRAM
- `src/EPD_Painter_LVGL.h` — LVGL v9 integration: flush callback + paint timer
- `src/EPD_Painter_presets.h` — The single board preset, `EPD_INKPLATE10_PRESET` (aliased `EPD_PAINTER_PRESET`): pins, I2C, PMIC pins, pixel clock, pass periods, default `ROTATION_180`
- `src/Inkplate10_Trains.h` — Waveform data: 4-level pairs only, seeded from the M5PaperS3 set and **not tuned on this panel**. No 16-grey or direct tables yet. Gathered by `EPD_Painter_trains.h`. Runtime precedence: NVS tuned blob → train file → formula/two-step fallback
- `src/EPD_Painter_tuned.h` — Tuned-blob format (build/validate/install; board-keyed, CRC-guarded). `begin()` auto-installs blobs from NVS namespace `epd_painter` (keys `fast`/`normal`/`high`), written by `examples/other/tuneup`
- `src/epd_painter_powerctl.h/.cpp` — TPS65186 PMIC behind a PCAL6416A expander (WAKEUP/PWRUP/VCOM/INT/PWR_GOOD, pull-ups on the open-drain inputs), GMOD and OE on native GPIOs. Sequence follows the InkplateLibrary Inkplate10 driver
- `src/epd_pin_driver.h` — `EPD_GpioPin` (direct register writes) and the `EPD_PowerDriver` interface
- `src/build_opt.h` — HAL macros for Arduino vs ESP-IDF compatibility

### Board Preset

One board, no `#define` needed: `EPD_PAINTER_PRESET` is the Inkplate 10 config. `EPD_Painter_presets.h` is imported automatically by `EPD_Painter.h`, `EPD_Painter_Adafruit.h`, and `EPD_Painter_LVGL.h`. There is no auto-detect or board probing.

### Delta Update System

`packed_screenbuffer` (PSRAM) tracks the physical display state. Each `paint()` call compares the new framebuffer against this state and only drives pixels that changed. Pixels needing lightening within a darkening chunk are deferred to a subsequent pass — a single `paint()` call is sufficient for complete coverage.

### Waveform System

7-pass waveforms (`lighter_waveform`, `darker_waveform` tables in `.S`) map 2bpp pixel values (00/01/10/11) to voltage drive sequences. Quality levels `QUALITY_HIGH / QUALITY_NORMAL / QUALITY_FAST` adjust latch delays.

### Memory Layout

| Buffer | Location | Size |
|---|---|---|
| 8bpp GFX canvas | PSRAM | ~967 KB |
| 2bpp fastbuffer | Internal RAM (if ≥96 KB headroom) else PSRAM | ~245 KB |
| 2bpp screen/paint/light buffers | PSRAM | 3 × ~245 KB |
| DMA row buffers | Internal RAM | 2 × 308 B |

## Hardware: Inkplate 10 on the ESP32-S3 breakout

- **Panel**: ED097TC2, 1200×825, 8-bit parallel. 2 bits per pixel on the bus: 01 darken, 10 whiten, 00 float
- **Data bus**: D0–D7 = GPIO 38, 39, 40, 41, 42, 47, 48, 21 (routed through the GPIO matrix to LCD_CAM, so the register split is free). GPIO39–42 boot as JTAG pads; `begin()` selects the GPIO function explicitly
- **Control**: CL=4 (LCD_CAM PCLK), SPV=5, GMOD=6, OE=7, CKV=10, LE=11, SPH=12
- **I2C**: SDA=8, SCL=9, the global `Wire`. Devices: 0x20 PCAL6416A, 0x48 TPS65186 (ACKs only while WAKEUP is high), 0x51 panel EEPROM
- **PMIC pins on the expander**: A0 WAKEUP, A1 PWRUP, A2 VCOM, A3 TPS INT, A4 PWR_GOOD. INT and PWR_GOOD are open drain and need the expander pull-ups
- **V3P3 (TPS65186 ENABLE bit 5) powers the panel logic**: every panel line must be low before it switches off. `powerOff()` handles this
- **Orientation**: the Inkplate library scans its buffer last-byte-first, so the preset defaults to `ROTATION_180` to keep the same (0,0)
- **No** touch, battery charger, RTC, SD or GPS on this board. GPIO48/38 may drive a devkit WS2812 sitting on a data line
- Source of truth for the wiring: `../InkplateLibrary/INKPLATE10_S3_WIRING.md` and `src/boards/Inkplate10/` on the `inkplate10-s3-test` branch


## Calibration and the tuning rig

`CALIBRATION.md` covers the self-tuning system, the scanner rig, and the
hardware gotchas that will otherwise cost hours. **Read it before touching the
tuner or the scanner.** In particular:

- Flash with `USBMode=hwcdc,CDCOnBoot=cdc` in the FQBN, or the sketch's
  `Serial` goes to UART pins and you silently get no telemetry
- Opening the serial port resets the board — attach before a run, never during
- Go through the eSCL bridge for scans, never `scanimage` directly
- `converged=1` does not mean a good table; it tests the applies only

## Technical Reference

`How_It_Works.md` contains deep technical documentation covering e-paper physics, the full pixel pipeline, DMA architecture, sendRow() timing, assembly internals, and memory layout. Read this before modifying core driver code.
