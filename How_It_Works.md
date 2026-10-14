# EPD Painter — How It Works

A guide for newcomers to the codebase. This document explains the big picture
first, then drills into each major subsystem.

> The engine was built and measured on 960×540 panels (M5PaperS3, LilyGo T5
> S3), and the worked numbers below — row bytes, pass times, 80 MHz PCLK — are
> theirs. The library now drives the Inkplate 10 (1200×825) only;
> [§13](#13-the-inkplate-10-port) lists everything that differs there.

---

## Table of Contents

1. [What Is an E-Paper Display?](#1-what-is-an-e-paper-display)
2. [Pixel Format — From Canvas to Panel](#2-pixel-format--from-canvas-to-panel)
3. [The Screen Buffer — Tracking Physical State](#3-the-screen-buffer--tracking-physical-state)
4. [Waveforms — How Greyscale Works](#4-waveforms--how-greyscale-works)
5. [The Chunk Bitmasks — 64-Pixel Work Blocks](#5-the-chunk-bitmasks--64-pixel-work-blocks)
6. [ink_dual — The Change Detection System](#6-ink_dual--the-change-detection-system)
7. [DMA and the LCD_CAM Peripheral](#7-dma-and-the-lcd_cam-peripheral)
8. [sendRow() — The Row Timing Protocol](#8-sendrow--the-row-timing-protocol)
9. [paint() — Full Frame Update Walk-Through](#9-paint--full-frame-update-walk-through)
10. [clear() — Full Panel Erase](#10-clear--full-panel-erase)
11. [Memory Map](#11-memory-map)
12. [File Overview](#12-file-overview)
13. [The Inkplate 10 Port](#13-the-inkplate-10-port)

---

## 1. What Is an E-Paper Display?

E-paper (electrophoretic) displays work by physically moving charged pigment
particles through a fluid. Unlike an LCD, the panel holds its image with **no
power** once set — pixels are mechanically latched.

Each pixel contains both black and white particles:

```
  Positive electrode (top)
  ┌───────────────────────┐
  │                       │
  │  ○ ○ ○ ● ● ● ● ○ ○ ●  │   ← white particles (○) and black particles (●)
  │     fluid medium      │      floating in a microcapsule
  └───────────────────────┘
  Negative electrode (bottom)

  Apply + voltage → black particles rise → pixel appears BLACK
  Apply - voltage → white particles rise → pixel appears WHITE
  No voltage      → particles hold position → image retained
```

Because the particles must physically move, driving the display requires
**multiple passes** of carefully timed voltage pulses — you cannot just write a
pixel value once and have it appear instantly at the right shade.

---

## 2. Pixel Format — From Canvas to Panel

The code uses **three different pixel representations** as data moves through
the pipeline:

### Stage 1 — GFX Canvas (8bpp, 1 byte per pixel)

The Adafruit GFX library works with one byte per pixel. Only the **2 least
significant bits** are meaningful here; the upper 6 bits are ignored.

```
  Byte in canvas:  [ x x x x x x P P ]
                                 └─┘
                             2-bit pixel value
```

| Value | Shade       |
|-------|-------------|
| `00`  | White       |
| `01`  | Light grey  |
| `10`  | Dark grey   |
| `11`  | Black       |

### Stage 2 — Packed Framebuffer (2bpp, 4 pixels per byte)

`epd_painter_compact_pixels()` compresses the canvas by packing 4 pixels into
each byte. This is what lives in `packed_fastbuffer` and `packed_screenbuffer`.

```
  4 canvas bytes:    [00PP] [00PP] [00PP] [00PP]
                       P3     P2     P1     P0

  → 1 packed byte:   [ P3 P2 P1 P0 ]   (MSB first)
```

This 4× compression matters because the 960×540 panel would otherwise need
518,400 bytes per framebuffer. At 2bpp it costs only 129,600 bytes — small
enough to fit in internal RAM for the fast path. The 2bpp format also matches
the ink drive format (described below) which makes the vector code to translate
between them much easier, and hence faster.

### Stage 3 — Ink Drive Format (for DMA output)

`epd_painter_convert_packed_fb_to_ink()` converts each packed byte into the
actual voltage drive pattern the panel expects, according to the current
waveform pass. Each 2bpp pixel becomes **two independent drive bits** — one
for each electrode:

```
  Packed byte:  [ P3 P3 P2 P2 P1 P1 P0 P0 ]
                  └─┬─┘ └─┬─┘ └─┬─┘ └─┬─┘
                    │     │     │     └── pixel 0: bit1=positive electrode, bit0=negative
                    │     │     └───────  pixel 1
                    │     └────────────   pixel 2
                    └─────────────────    pixel 3

  Drive codes:  00 = no drive (float)
                01 = drive negative  (whiten)
                10 = drive positive  (darken)
                11 = drive both      (DC balance)
```

---

## 3. The Screen Buffer — Tracking Physical State

The EPD panel has no way to report what is currently shown on screen. The code
maintains `packed_screenbuffer` as a **software model of the physical display**:

```
  packed_screenbuffer   ←→   What is physically on the panel right now
  packed_fastbuffer     ←→   What we WANT the panel to show next
```

The screen buffer starts as all `0x00` (all white) and is updated incrementally
by `ink_dual` as pixels change. It is stored in PSRAM (it only
needs to be read once per frame, so speed is less critical).

**Why is this necessary?**

If the driver doesn't know what colour a pixel currently is, it would need to
start from scratch every update, moving the pixel to full white, then to the
actual colour it needs to be. This causes the panel to flash.

**The screen buffer does not survive a power cycle.** On reboot it is
initialised to all-white, but the physical panel still shows whatever was last
displayed. The `EPD_PainterShutdown` class handles this: on shutdown it saves
a known image (the shutdown image) to LittleFS and paints it to the panel; on
the next boot it unpaints that image — driving all those pixels back toward
white — so that by the time user code starts, the screen buffer and the
physical panel are in agreement. See the Reference Manual for the full
shutdown handling documentation.

---

## 4. Waveforms — How Greyscale Works

Greyscale is achieved by driving each pixel with a **sequence of voltage pulses
across 7 passes**. The accumulated effect of these pulses moves the particles
to a position that reflects the target shade.

Two waveform tables are defined — `lighter_waveform` and `darker_waveform`.
Each has 3 rows (one per pixel value group: black `11`, dark grey `10`, light
grey `01`) and 7 columns (one per pass):

```c
  lighter_waveform[pixel_value][pass]:
               Pass: 0  1  2  3  4  5  6
  11 (black):      { 1, 2, 2, 0, 2, 0, 2 }
  10 (dark grey):  { 2, 2, 2, 2, 2, 3, 3 }
  01 (light grey): { 2, 2, 2, 2, 2, 2, 2 }

  Drive codes:  0 = float    1 = whiten (+)
                2 = darken (-)    3 = both simultaneously
```

### The Printing Press Analogy

This process is conceptually very close to how a traditional printing press
works. A press applies each ink colour in a separate pass — cyan, magenta,
yellow, black — and the final image emerges from their combination across
all passes.

Here, the three "inks" are the drive patterns for each greyscale shade
(black, dark grey, light grey). On each pass, all three are loaded
simultaneously and stamped onto the 64-pixel chunk at once, each one
landing only on the pixels it belongs to:

```
  Traditional press:                  EPD waveform pass:

  Cyan plate   → only cyan pixels     Waveform for 11  → only black pixels
  Magenta plate→ only magenta pixels  Waveform for 10  → only dark grey pixels
  Yellow plate → only yellow pixels   Waveform for 01  → only light grey pixels
        │                                     │
        └── all applied in one roller pass    └── all applied in one 64-pixel chunk
```

For each pass, the waveform bytes for all three pixel groups are broadcast
into Q registers in the assembly routine:

```
  lighter_wf[0] = lighter_waveform[2][pass] * 0x55   // for 01 pixels
  lighter_wf[1] = lighter_waveform[1][pass] * 0x55   // for 10 pixels
  lighter_wf[2] = lighter_waveform[0][pass] * 0x55   // for 11 pixels
```

The `* 0x55` multiplication spreads each 2-bit drive code across all pixel
positions in the byte, so the waveform can be applied with a simple AND/OR
mask against the pixel identity bits.

---

## 5. The Chunk Bitmasks — 64-Pixel Work Blocks

All pixel processing in EPD Painter operates on **64-pixel chunks** (16 packed
bytes), matching the ESP32-S3's 128-bit vector registers. Each row carries
**two** 32-bit chunk masks — one per drive plane (MSB = first chunk on the
left, 15 of 32 bits used for a 960-pixel row):

```
  Row of 960 pixels = 240 packed bytes = 15 chunks of 64 pixels

  bitmask[row]        — dark plane:  chunk contains pixels to DARKEN
  bitmask_light[row]  — light plane: chunk contains pixels to LIGHTEN

  A chunk may be flagged in either mask, both, or neither.
```

The chunk is purely a **work-skipping and SIMD-width unit** — it says nothing
about direction. Direction is decided per pixel: the delta detector emits two
pixel-disjoint drive planes, and both can be active inside the same chunk.
(Earlier versions of the driver forced each chunk to be entirely darkening or
entirely lightening, deferring opposite-direction pixels to later frames —
that constraint, and its motion artifacts, no longer exist.)

Per row, per waveform pass, the converter runs twice and the planes merge in
the DMA buffer:

```
  memset(dma_buf, 0, len);                             // float everything
  epd_painter_convert_packed_fb_to_ink   (fbD_row, dma_buf, len, darker_wf,  bitmask[row]);
  epd_painter_convert_packed_fb_to_ink_or(fbL_row, dma_buf, len, lighter_wf, bitmask_light[row]);
```

The first call writes ink for dark-plane chunks; the second computes ink for
light-plane chunks and **ORs** it in. Ink drive codes are per-pixel and the
planes are pixel-disjoint, so the OR composes them losslessly: one row
transmission carries darken codes, whiten codes and floats side by side.
Chunks in neither mask keep the memset floats and are skipped by both calls.

---

## 6. ink_dual — The Change Detection System

`epd_painter_ink_dual` is the heart of the delta-update system. Called once
per row, it compares the desired new frame (read directly from the PSRAM
`packed_paintbuffer`) against the current physical state
(`packed_screenbuffer`) and emits both drive planes in one sweep:

```
  For each 64-pixel chunk:

  dark plane  = new  & spread(screen == 00)      pixels appearing on white
                                                 ground, at their new value
  light plane = screen & spread(new != screen)   changed pixels that are
                                                 currently non-white, at
                                                 their CURRENT value

  screen'     = (screen & ~changed) | dark       bookkeeping: darkened pixels
                                                 take their new value,
                                                 lightened pixels become white
```

The two planes are pixel-disjoint by construction: the dark plane is masked
to `screen == 00` pixels, and the light plane is `screen & changed`, which is
zero wherever the screen is white. There is no direction priority, no
fallback path, and no deferral — a chunk with work in both directions simply
sets its bit in both masks.

The dark plane lands in the internal-RAM `packed_fastbuffer` (read every
pass); the light plane lands in PSRAM and is only touched — by ink_dual's
stores and the converter's loads — for chunks actually flagged, so its PSRAM
traffic is sparse.

The spread/invert mask logic is unchanged from the original design: a 2bpp
pixel's bits are smeared across both positions and inverted, yielding `11`
only for pixels that were `00`; XOR-then-spread flags changed pixels:

```
  screen pixel = 01       screen pixel = 00      desired=01, screen=10
  spread:  11             spread:  00            XOR = 11 → differs
  invert:  00 (blocked)   invert:  11 (allowed)  spread = 11 → drive it
```

### The Two-Frame Grey Transition

A pixel changing between two non-white shades still takes **two cycles**,
because darkening only starts from white ground:

```
  Cycle N:  pixel is in the light plane — driven toward white,
            screen buffer records 00.
  Cycle N+1: pixel is in the dark plane — driven to its new value.
```

This is why a submitted frame runs two paint cycles (`paintStage = 2`): the
second cycle picks up these two-step transitions. A cycle in which no chunk
has any work skips its drive passes entirely.

### Why One Transmission — the Charge-Retention Rule

The obvious alternative for a mixed chunk — transmit the row twice per pass,
once per plane, latching both onto the same gate row — **does not work**, and
the reason is worth recording:

A row's ink dose does not come from its ~30 µs transmission slot. When a row
is latched, each pixel stores the drive voltage on its own capacitance and
keeps driving the ink **until that row is next latched** — normally a full
pass period (milliseconds). Retention, not the row slot, is where the dose
comes from; it is also why 540 rows can effectively drive in parallel.

Latching a second data set onto the same row therefore discharges the first
after one row slot — starving it of ~99.8% of its dose. This was verified
optically (`debugRowTest()` drives a stripe pattern three ways and scans the
result): a second latch on the same gate row lands perfectly aligned — the
hardware supports it — but whichever data is latched *first* leaves almost no
ink behind. Hence the per-pixel OR-merge into a single transmission, which
gives every pixel the full retention dose.

---

## 7. DMA and the LCD_CAM Peripheral

The ESP32-S3's LCD_CAM peripheral can clock data out on a parallel bus at high
speed using DMA, without CPU involvement during the transfer. This is how one
row of pixel data is sent to the EPD panel quickly.

### Hardware Setup

```
  ESP32-S3
  ┌─────────────────────────────────────────────────────────┐
  │                                                         │
  │  Internal RAM                                           │
  │  ┌──────────────┐   ┌──────────────────┐                │
  │  │  DMA buffer1 │   │  DMA descriptor1 │                │
  │  │  (row data)  │←──│  .buffer = buf1  │                │
  │  └──────────────┘   │  .suc_eof = 1    │                │
  │                      │  .next = nullptr │                │
  │  ┌──────────────┐   └──────────────────┘                │
  │  │  DMA buffer2 │   ┌──────────────────┐                │
  │  │  (row data)  │←──│  DMA descriptor2 │                │
  │  └──────────────┘   │  .suc_eof = 1    │                │
  │                      │  .next = nullptr │                │
  │                      └──────────────────┘                │
  │         ↑                                               │
  │       GDMA ──────────────────────────────────────────►  │
  │                        LCD_CAM peripheral                │
  │                        (i8080 8-bit mode, 80 MHz PCLK)  │
  └─────────────────────────┬───────────────────────────────┘
                             │  8-bit parallel bus + PCLK
                             ▼
                    EPD data lines (D0–D7 + CL)
```

### Double Buffering

Two DMA buffers are used so the CPU can fill one while the DMA engine is
streaming the other — there is no gap between rows:

```
  Time →

  CPU:  [fill buf1] [fill buf2] [fill buf1] [fill buf2] ...
  DMA:             [send buf1] [send buf2] [send buf1]  ...
```

Each DMA descriptor has `suc_eof = 1` and `.next = nullptr`, so the DMA engine
stops after completing each buffer. `sendRow()` explicitly resets the DMA
channel and points it at the correct descriptor before each transfer, ensuring
the DMA always starts from byte 0 of the intended buffer. This avoids any
synchronisation issues between the DMA engine and the LCD_CAM peripheral.

### Row-End Padding — Flushing the Source-Driver Shift Chain

The panel's source driver is a shift-register chain running along the top edge
of the glass: each CL pulse shifts 4 pixels (one byte) further along, and LE
latches the whole chain onto the column outputs at the end of the row.

The chain has slightly **more stages than visible columns** (dummy cells,
cascade margin between driver chips). Clocking in exactly `width` pixels and
latching leaves the data about one 32-bit word short of its final position at
the far end of the chain — so the last ~16 columns latch data belonging to
pixels ~16px to their left, and the image visibly repeats at the right-hand
edge. This is a property of the glass, not the MCU: independent drivers (e.g.
FastEPD) show the same artifact when they clock exactly `width` pixels.

The fix: every row's DMA transfer carries `Config::row_pad_bytes` (default 4)
trailing zero bytes, and `lcd_dout_cyclelen` is extended to match, giving the
chain 16 extra clocks to carry the real pixels to the end before LE latches.
Zero bytes are neutral drive, so the pad is electrically harmless, and it
costs only ~50ns per row at 80 MHz PCLK.

Diagnostics that pinpointed this (July 2026, M5PaperS3): GDMA underflow flags
stayed clear across full paints (ruling out DMA starvation), and a 3px-pitch
vertical grating showed the affected band's lines doubled at +1px phase — a
byte-multiple displacement, i.e. pixels arriving short in the shift chain.
`examples/adafruit/diagonal_lines` is the test pattern for this artifact.

---

## 8. sendRow() — The Row Timing Protocol

The EPD panel uses a shift-register architecture for row selection. Three
control signals manage the row pointer:

| Signal | Purpose                                           |
|--------|---------------------------------------------------|
| `SPV`  | Frame start — resets row pointer to line 0        |
| `CKV`  | Row clock — falling edge advances the row pointer |
| `LE`   | Latch enable — commits shift register to the row  |

### Timing for the First Row

```
  SPV  ──┐            ┌──────────────────────
         └────────────┘
  CKV  ────────┐ ┌──────────┐ ┌───┐ ┌───┐ ┌──
               └─┘          └─┘   └─┘   └─┘
                ↑            └ vscan_skip_lines lead-in clocks ┘
  Start token enters the gate chain here
```

SPV is held low for `vscan_hold_us` across one CKV rising edge, then
`vscan_skip_lines` more CKV clocks step the token past gate outputs that are
not visible lines (three on the Inkplate 10, matching its `vscan_start()`).

### Timing for Subsequent Rows

```
  LE   ──┐       ┌────
         └───────┘
  CKV  ──┐      ┌────
         └──────┘
            ↑
         Row pointer advances here; previous row's data is latched
```

The `gdma_reset()` call between pin transitions serves double duty: it resets
the DMA channel for the next transfer while simultaneously providing the
timing delay required by the CKV/LE signals — eliminating the need for a
separate `delayMicroseconds()` call.

### Full Sequence Per Row

```
  1. Wait for previous DMA transfer to complete (lcd_start == 0)
  2. Issue timing signals (SPV pulse on row 0, LE+CKV pulse otherwise)
     — gdma_reset() is used as the timing delay between pin transitions
  3. Point DMA at the correct descriptor (alternating desc1/desc2)
  4. Start DMA transfer (LCD_CAM clocks data onto D0–D7 at Config::pclk_hz)
  5. On the last row: wait for DMA to finish, then issue final latch pulse
```

Each send latches the row transmitted *before* it. The last row of a pass
would therefore never be latched (the next pass's first row overwrites it in
the source chain), so the pass loop, `clear()` and `fxClear()` finish every
pass with `_endPass()`: one float row send that latches the real last row and
clocks the gate onto its line.

---

## 9. paint() — Full Frame Update Walk-Through

This is what happens every time you call `paint()`:

```
  GFX canvas (8bpp, PSRAM)
         │
         │  epd_painter_compact_pixels()
         ▼
  packed_paintbuffer (2bpp, PSRAM — the desired frame)
         │
         │  For each row:
         ├──── ink_dual() ──────────────────────────────────────────┐
         │     compares paintbuffer with packed_screenbuffer        │
         │     dark plane  = new values on white ground             │
         │       → packed_fastbuffer (internal RAM)                 │
         │     light plane = screen values of changed non-white px  │
         │       → packed_lightbuffer (PSRAM, sparse)               │
         │     screen buffer updated (darkened marked, lightened    │
         │       recorded as white)                                 │
         │     returns bitmask[] (dark) + bitmask_light[] per row   │
         │                                                          │
         │  (a cycle with no flagged chunks skips the passes)       │
         │
         │  For each of 7 waveform passes:
         │    For each row:
         │      memset(dma_buf, 0)                                  ← float default
         │      convert_packed_fb_to_ink   (darker_wf,  bitmask)    ← dark chunks write
         │      convert_packed_fb_to_ink_or(lighter_wf, bitmask_light) ← light chunks OR in
         │      sendRow()                                           ← one DMA transmission
         │
         └──► Neutralise: send all-zero frame (stops all pixel movement)
```

### Performance

At 80 MHz pixel clock on a 960×540 panel, each row of 240 packed bytes
transfers in ~3µs. With 540 rows × 7 passes plus latch delays, a full
`paint()` completes in approximately **14ms at low quality** — roughly 70fps
equivalent, making this one of the fastest greyscale EPD drivers available.

---

## 10. clear() — Full Panel Erase

`clear()` ignores the framebuffer content and forces all pixels to white,
in several stages:

```
  Stage 1: Set packed_fastbuffer to all 0x00 (all-white target)

  Stage 2: ink_dual() on all rows (all work lands in the light plane)
           → marks all currently inked pixels as needing to clear
           → screen buffer reset to all 0x00

  Stage 3: 7 lighter waveform passes
           → moves inked pixels toward white, discharging DC bias

  Stage 4: Hard erase — 2 phases of 8 passes each:
           0b01010101 (full positive drive, all pixels)
           0b10101010 (full negative drive, all pixels)
           → aggressively resets any stubborn particles

  Stage 5: Neutralise with all-zero frame
```

The alternating hard drive in Stage 4 forces all particles fully to one
extreme and then the other, ensuring the panel is in a known clean state.

Note that `clear()` does not clear the Adafruit GFX canvas buffer, only the
physical screen. The next time `paint()` is called, the canvas contents will
be redrawn on a fresh white background.

---

## 11. Memory Map

```
  ┌──────────────────────────────────────────────────────────┐
  │  PSRAM (external, slower)                                │
  │                                                          │
  │  buffer              1200×825 bytes (GFX canvas, 8bpp)   │
  │  packed_screenbuffer 250,800 bytes  (screen state, 2bpp) │
  │  packed_paintbuffer  250,800 bytes  (desired frame)      │
  │  packed_lightbuffer  250,800 bytes  (light plane)        │
  └──────────────────────────────────────────────────────────┘

  ┌──────────────────────────────────────────────────────────┐
  │  Internal RAM (fast, CPU + DMA accessible)               │
  │                                                          │
  │  packed_fastbuffer   250,800 bytes  (dark plane; PSRAM   │
  │                                      if it would leave   │
  │                                      < 96 KB free)       │
  │  bitmask[] ×2        825 × 4 bytes  (chunk flags/row)    │
  │  dma_buffer1/2       308 bytes each (304 row + 4 pad)    │
  └──────────────────────────────────────────────────────────┘

  packed_row_bytes = packedRowBytesFor(1200) = 304 bytes per row
                     (300 rounded up to a 16-byte SIMD chunk)
  packed_size      = 304 × 825          = 250,800 bytes
```

`packed_fastbuffer` is preferred in internal RAM because it is read once per
row per pass. At 1200×825 it is most of the S3's internal heap, so `begin()`
only takes it there when 96 KB would remain for sweep lists, the paint task
and the sketch's own needs (WiFi). At the default 4 MHz pixel clock a row's
DMA takes ~77 µs, which hides PSRAM reads entirely, so the fallback costs
nothing measurable at that clock.

---

## 12. File Overview

```
  EPD_Painter.h              Class definition, config struct, buffer pointers
  EPD_Painter.cpp            C++ driver: init, DMA setup, paint(), clear(), sendRow()
  EPD_Painter.S              Xtensa assembly: pixel packing, waveform conversion,
                             ink_dual
  EPD_Painter_presets.h      The Inkplate 10 preset: pins, PMIC, clock, timing
  Inkplate10_Trains.h        Waveform tables (seed, untuned on this panel)
  epd_painter_powerctl       TPS65186 + PCAL6416A power sequencing, GMOD / OE
  epd_pin_driver.h           Direct-register GPIO pin driver
  epd_painter_shutdown.h/.cpp  Screen-buffer reconciliation across power cycles,
                               reset-to-shutdown trigger, shutdown image
                               storage, idle timer, pre-shutdown callbacks
```

### Key Data Flow Summary

```
  User code
     │
     │  drawPixel(), fillRect(), etc.   (Adafruit GFX API)
     ▼
  buffer[]               8bpp GFX canvas in PSRAM

  paint() called
     │
     │  compact_pixels()
     ▼
  packed_fastbuffer[]    2bpp packed, internal RAM

     │  ink_dual()  per row
     │  (delta detection vs packed_screenbuffer)
     │  (bitmask[] tracks darkening vs lightening chunks)
     ▼
  packed_fastbuffer[]    only changed pixels remain

     │  convert_packed_fb_to_ink() × 7 passes
     │  (darker waveform for bitmask=1 chunks,
     │   lighter waveform for bitmask=0 chunks)
     ▼
  dma_buffer1/2[]        ink drive bytes for one row

     │  sendRow() → GDMA reset → LCD_CAM 80MHz → GPIO pins
     ▼
  EPD panel              physical pixels update
```

---

*For questions about the assembly internals, see the comments in `EPD_Painter.S`
which document every register and operation in detail.*

---

## 13. The Inkplate 10 Port

The Inkplate 10's ED097TC2 is the same kind of panel as the boards the engine
grew up on: 8-bit parallel, 2 bits per pixel (01 darken, 10 whiten, 00
float), SPV/CKV gate scan, SPH/CL/LE source chain. What changed is size,
power, timing and orientation.

**Width that is not a multiple of 64.** 1200 px is 300 packed bytes. Every
SIMD routine processes 16-byte (64-pixel) chunks, loads with aligned
`ee.vld.128`, and a row's chunk mask is one 32-bit word. Rows are therefore
stored at `packedRowBytes()` = 304 bytes (19 chunks), and `_pack()` writes
the four padding bytes as zero. Padding pixels are white in both the paint
and screen buffers, so discovery never gives them work. They are clocked out
after the 1200 visible columns, past the end of the source chain, where
they act as extra row padding. Every buffer size goes through `packedSize()`.
The 8bpp canvas stays exactly 1200 wide; 1200 is a multiple of 16, so each
canvas row starts 16-byte aligned for the assembly packer.

**Rotation.** The Inkplate library walks its framebuffer from the last byte
to the first, so its (0,0) is the last pixel the panel receives. The preset
defaults to `ROTATION_180` to match. `_pack()` implements 0/180/CW for both
2bpp and 4bpp, which also lifts the old "16-grey is ROTATION_0 only"
restriction.

**Pixel clock.** `Config::pclk_hz` (default 4 MHz) sets LCD_CAM to the
160 MHz PLL / div_num / prescale, always prescale 2 for an even duty cycle.
The bit-banged Inkplate driver needed ≥125 ns CL-high on the breakout, but it
also changed D7 on the same store that raised CL, so it gave that line zero
setup time. LCD_CAM holds data and clock in a fixed half-period relationship,
the same configuration the earlier panels ran at 80 MHz, so faster clocks
are probably fine here too. They are untested, though, and every waveform
table is only valid at the row timing it was tuned at.

At 4 MHz a row is 308 bytes = 77 µs, a pass ~64 ms, so a 13-pass 4-level
NORMAL frame takes roughly 0.9 s. On the 960×540 boards the same frame took
~0.13 s. The 16-grey pass period is 75 ms for the same reason.

**Gate lead-in.** `vscan_skip_lines = 3` reproduces where the Inkplate
driver's `vscan_start()` puts the first data row. Rows are latched one send
late (see §8), so row 0 lands on gate output 5 in both drivers.

**Power.** TPS65186 behind a PCAL6416A at 0x20:

```
  powerOn:  GMOD high → WAKEUP high, 5 ms → ENABLE=0x20 (V3P3 on)
            → UPSEQ0=0xE4, DWNSEQ0=0x1B → PWRUP high
            → poll PG (0x0F) == 0xFA, ≤250 ms → VCOM high → OE high
  powerOff: LE/CKV/SPH/SPV low (EPD_Painter) → OE, GMOD low
            → VCOM, PWRUP low → poll PG == 0 → ENABLE=0x00 → WAKEUP low
```

V3P3 powers the panel's logic, so every panel line goes low before it
switches off. PWR_GOOD and INT get the expander's internal pull-ups. VCOM is
whatever is factory-programmed in the PMIC's EEPROM. `readVcomMv()` and
`readTemperature()` expose it and the panel thermistor.

**Pins.** Data lines 39–42 power up as the S3 JTAG pads, so `begin()` sets
output enable, the matrix route and the IO_MUX GPIO function for every bus
pin, in esp_lcd's i80 order. The previous code passed an IO_MUX register
address where `esp_rom_gpio_pad_select_gpio()` expects a GPIO number.

**SPH.** A per-row start pulse, low for the first byte only, as in the
Inkplate driver's `hscan_start()`. The earlier boards held SPH low for the
panel's whole powered life. On the ED097TC2 that fails: its two cascaded
600-column source drivers restart chip 1 when it fills, so the left half
shows the right half of the image and the right half is never driven.
LCD_CAM generates the pulse in hardware. SPH is routed to the i80 DC
line, and each row is a 1-cycle CMD phase (`lcd_cmd_value` = the row's
byte 0, DC low) followed by DOUT from byte 1 (DC high). DC idles high, so
SPH is handed back to a low GPIO in `powerOff()` and stays there until
`powerOn()`.

**Row window.** A frame only scans the rows that carry work. Rows above the
first changed row are stepped past with bare CKV clocks (`_skipRow`), with
float latched by `_vscanStart()` while the gates are off. Each pass and the
closing sweep stop after the last changed row. Because the gate driver is a
shift register, an early stop then latches float and clocks the token off
the bottom (`_flushGate`), so the next SPV doesn't run two tokens at once.
Pass length is the dose, so a short window is padded to
`Config::min_pass_us` (default 6 ms, 4-level only). Grey levels on a small
window therefore differ from the same levels on a full-screen paint, unless
`min_pass_us` is set to a full pass. `skip_idle_rows = false` restores full
scans.
