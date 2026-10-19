# EPD_Painter internals: what happens when `epd.paint()` is called

This is a code-level trace of one frame in `QUALITY_BW` mode on the Inkplate 10 (1200 × 825). It follows the call from the sketch through every function, task, lock, buffer, register write and pin edge, in the order they happen, and documents the byte layout of every data structure on the way.

All references are to `src/`. Numbers assume the stream sketch's configuration unless stated:
- `pclk_hz` 10 MHz, `bw_passes` 4, `min_pass_us` 4000;
- `skip_idle_rows` on, `skip_idle_cols` off;
- `ROTATION_180`, Arduino core (so `EPD_DELAY_US` = `delayMicroseconds`, a busy wait, and `EPD_DELAY_MS` = `delay`, a `vTaskDelay`);
- FreeRTOS tick 1 kHz.

---

## Contents

1. [State after `begin()`](#1-state-after-begin)
2. [Call trace overview](#2-call-trace-overview)
3. [Step 1, app task: `paint()` and `_pack()`](#3-step-1-app-task-paint-and-_pack)
4. [Step 2, paint task wakes up](#4-step-2-paint-task-wakes-up)
5. [Step 3, discovery: BW snap, `ink_dual`, sweep lists](#5-step-3-discovery)
6. [Step 4, `paintStage` decrement: `paint()` returns](#6-step-4-paintstage-decrement)
7. [Step 5, panel power-on (`PanelPowerGuard`)](#7-step-5-panel-power-on)
8. [Step 6, train binding and the per-pass broadcast table](#8-step-6-train-binding)
9. [Step 7, row window and column cut](#9-step-7-row-window-and-column-cut)
10. [Step 8, the pass loop](#10-step-8-the-pass-loop)
11. [Step 9, inside `sendRow()`: registers and pins](#11-step-9-inside-sendrow)
12. [Step 10, start and end of a pass: vscan, skips, `_endPass()`, `_flushGate()`, padding](#12-step-10-start-and-end-of-a-pass)
13. [Step 11, closing sweep](#13-step-11-closing-sweep)
14. [Step 12, second (mop-up) cycle and going idle](#14-step-12-second-cycle-and-idle)
15. [Step 13, power-off after the idle timeout](#15-step-13-power-off)
16. [Pin timeline of a whole frame](#16-pin-timeline-of-a-whole-frame)
17. [Data structure reference](#17-data-structure-reference)
18. [Kernel reference: assembly, line by line](#18-kernel-reference-assembly)
19. [Kernel reference: C versions](#19-kernel-reference-c)
20. [Worked example: 8 pixels from canvas to bus](#20-worked-example)
21. [Other entry points and how they differ](#21-other-entry-points)
22. [Timing summary](#22-timing-summary)

---

## 1. State after `begin()`

`paint()` relies on what `EPD_Painter::begin()` (`EPD_Painter.cpp`) set up.

### 1.1 Tasks

| Task | Created by | Priority / core | Stack | Body |
|---|---|---|---|---|
| `epd_paint` | `begin()`: `xTaskCreatePinnedToCore(_paint_task_entry, "epd_paint", 8000, this, 10, &_paint_task_h, 0)` | 10 / core 0 | 8000 B | `_paint_task_body()`: an endless loop, sections 4–14 |
| `panel_idle_off` | first `PanelPowerGuard` construction (`EPD_Painter.h`, `initOnce`) | 1 / any | 2048 B | Once a second: count `state` down, `powerOff()` at 0 |

The sketch's `loop()` runs in Arduino's `loopTask` (priority 1, core 1).

### 1.2 Synchronisation objects

| Object | Type | Meaning |
|---|---|---|
| `paintStage` | `std::atomic<int>` | 0 = idle; 2 = a new frame is in the paintbuffer and not yet discovered; 1 = one more (mop-up) cycle owed |
| `_paint_buffer_sem` | binary semaphore, starts given | Guards the paintbuffer and screenbuffer while one side reads or writes them |
| `_paint_active_sem` | binary semaphore, starts given | Held by the paint task while it works; `clear()` takes it to get the bus to itself |
| `PanelPowerGuard::power_mtx` | mutex | Guards the power state counter |

### 1.3 Memory

| Name | Type, size | Location | Contents |
|---|---|---|---|
| `packed_paintbuffer` | 304 × 825 = 250,800 B | PSRAM | Frame to show, packed 2bpp |
| `packed_screenbuffer` | 250,800 B | PSRAM | Model of the glass |
| `packed_fastbuffer` | 250,800 B | internal RAM if ≥ 96 KB would remain, else PSRAM | Dark plane |
| `packed_lightbuffer` | 250,800 B | PSRAM | Light plane |
| `bitmask[825]`, `bitmask_light[825]` | `uint32_t` | internal | Chunk masks per row |
| `dec_sweeps[825 × 2]` | `LineSweep` (12 B) | internal | Per-row sweep lists |
| `dec_nsweeps[825]` | `uint8_t` | internal | Sweeps per row (0–2) |
| `dma_buffer1`, `dma_buffer2` | 308 B, 16-aligned | internal, DMA-capable | Ping-pong bus rows |
| `dma_buffer0` | 308 B | internal, DMA-capable | Permanent all-zero row |
| `dma_desc0/1/2` | `dma_descriptor_t`, `alignas(8)` | inside the object | One descriptor per row buffer |
| `dma_buffer` | `uint8_t*` | | Points at whichever of buffer1/2 the CPU fills next |

Layouts are in section 17.

### 1.4 Peripheral state

**LCD_CAM** (IDF 6 path, `lcd_ll_*` calls):
- clock source PLL_F160M; group divider `div_num` and pixel prescale `pre` chosen by `epd_pick_pclk()`. For 10 MHz: div 8, pre 2, giving 160 / 8 / 2 = 10 MHz exactly.
- i8080 mode (`enable_rgb_mode(false)`), 8 data wires, no colour conversion, no auto next frame, no "always out".
- DC line: idle **high**, **low** during the CMD phase, high during data (`set_dc_level(lcd, true, false, true, true)`).
- phases: 1 CMD cycle, 0 dummy, 307 DOUT cycles (`set_phase_cycles(lcd, 1, 0, dma_row_bytes - 1)`).
- CL idles low.

**GPIO matrix:**
- D0..D7 → `LCD_DATA_OUTn`;
- CL → `LCD_PCLK`;
- SPH → `SIG_GPIO_OUT_IDX` (a plain GPIO, low) while unpowered. It is switched to `LCD_DC` in `powerOn()`.
- Data and CL pads at drive strength 3.

**GDMA:** one TX channel connected to LCD_CAM, `owner_check = false`, `auto_update_desc = false`.

**Control pins:**
- SPV, CKV, SPH, LE and CL are outputs, all low. SPV/CKV/LE/SPH are wrapped in `EPD_GpioPin` (`epd_pin_driver.h`): `set(true)` is one write of `1 << pin` to `GPIO_OUT_W1TS_REG` (`GPIO_OUT1_W1TS_REG` for pins ≥ 32), and `set(false)` writes the matching `W1TC` register. One store, no read-modify-write.
- GMOD and OE are outputs, low, owned by the power driver (`digitalWrite`).

**PMIC:** TPS65186 asleep (WAKEUP low), up/down sequences programmed; PCAL6416A A0–A2 outputs low, A3/A4 inputs with pull-ups.

**Model:** `packed_screenbuffer` and `packed_paintbuffer` are zeroed (all white). `paintStage` = 0, so the paint task is parked.

---

## 2. Call trace overview

```
loopTask (app)                          epd_paint task                          hardware
──────────────                          ──────────────                          ────────
epd.paint(fb)
  take _paint_buffer_sem
  _pack(fb → paintbuffer)   ~1-2 ms
  _pr_lo,_pr_hi = 0, 824
  paintStage = 2
  give _paint_buffer_sem
  while paintStage==2: vTaskDelay(1)
                                        (polling paintStage every 1 ms tick)
                                        take _paint_active_sem
                                        take _paint_buffer_sem
                                        _sb_guard_check()
                                        BW snap rows 0..824
                                        ink_dual × 825 rows
                                        _decision_batch_compat()
                                        give _paint_buffer_sem
                                        paintStage = 1
  ◄── returns ────────────────────────
                                        PanelPowerGuard → powerOn() if off ───► I²C, GMOD, OE, rails
                                        bind trains
                                        window r0..r1
                                        pass 0..3:
                                          build bcast[]
                                          vscan + skip rows < r0      ────────► SPV, CKV pulses
                                          rows r0..r1: convert + sendRow ─────► LE, CKV, CL+D0-7+SPH
                                          _endPass, _flushGate       ─────────► LE, CKV pulses
                                          pad to min_pass_us
                                        vTaskDelay(1)
                                        closing sweep r0..r1         ─────────► float rows
                                        paintStage == 1 → discovery again:
                                          nothing to do, paintStage = 0
                                        give _paint_active_sem, park
                                                                         (idle task, 1 Hz)
                                                                         timeout: powerOff()
```

---

## 3. Step 1, app task: `paint()` and `_pack()`

With the Adafruit wrapper, `gfx.paint()` is `_painter.paint(buffer)` (`EPD_Painter_Adafruit.h`). The canvas `buffer` is the `GFXcanvas8` buffer, 1200 × 825 bytes in PSRAM, one byte per pixel, values 0 (white) to 3 (black). Only the low two bits are used.

`EPD_Painter::paint(uint8_t *framebuffer)`:

```cpp
xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);   // blocks only while the paint task is in discovery
_pr_lo = 0; _pr_hi = _config.height - 1;            // every row may have changed
_pack(framebuffer, packed_paintbuffer, false);      // 8bpp canvas → packed 2bpp, physical order
paintStage = 2;
xSemaphoreGive(_paint_buffer_sem);
while (paintStage == 2) vTaskDelay(1);              // until the paint task has finished discovery
```

The semaphore is free while the paint task is driving passes (the pass loop reads the planes and sweep lists, not the paintbuffer). So `paint()` can pack a new frame while the previous one is still on the bus. It then waits in the `while` until the paint task has finished that drive *and* run discovery on the new frame.

### `_pack()` for `ROTATION_180`

```
for r in 0..824:                                  # physical row r
    src = fb + (824 - r) * 1200                   # canvas row, mirrored vertically
    epd_painter_compact_pixels(src, tmp, 1200)    # 1200 bytes → 300 packed bytes, forward
    d = paintbuffer + r * 304
    for b in 0..299:
        d[b] = rev2[ tmp[299 - b] ]               # reverse byte order AND the 4 pixels inside
    memset(d + 300, 0, 4)                         # stride padding stays zero
```

`rev2` is a static 256-byte table: `rev2[x] = (x&3)<<6 | ((x>>2)&3)<<4 | ((x>>4)&3)<<2 | (x>>6)&3`. It turns `[p0 p1 p2 p3]` into `[p3 p2 p1 p0]`; with the reversed byte order, that mirrors the row horizontally. `tmp` is a 512-byte, 16-aligned stack array.

The other rotations:
- `ROTATION_0` runs `epd_painter_compact_pixels` straight into the paintbuffer (one call for the whole frame if width/4 equals the stride, else row by row).
- `ROTATION_CW` uses `compact_pixels_rotated_cw()`, which transposes in 16-row blocks.

`epd_painter_compact_pixels` is the PIE kernel on the S3 (section 18.1) and C elsewhere (section 19). The whole frame costs roughly 1–2 ms; reading the 990 KB canvas out of PSRAM dominates.

---

## 4. Step 2, paint task wakes up

`_paint_task_body()`, top of the loop:

```cpp
if (paintStage == 0) {
    xSemaphoreGive(_paint_active_sem);       // "not working"
    while (paintStage == 0) vTaskDelay(1);   // poll once per 1 ms tick
    xSemaphoreTake(_paint_active_sem, portMAX_DELAY);
}
xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
_sb_guard_check();                           // no-op unless setStateGuard(true)
// template overlay: skipped (no template set)
```

The wake-up latency is up to one tick (≤ 1 ms) after `paintStage = 2`.

---

## 5. Step 3, discovery

All of this runs with `_paint_buffer_sem` held.

### 5.1 BW snap

```cpp
uint32_t *p = (uint32_t *)(packed_paintbuffer + pr_lo * 304);
for (i over (pr_hi - pr_lo + 1) * 304 / 4 words) {
    hi   = p[i] & 0xAAAAAAAA;     // bit 1 of every pixel
    p[i] = hi | (hi >> 1);        // 00,01 → 00 ; 10,11 → 11
}
```

That is 62,700 words, read-modify-write in PSRAM. Afterwards every pixel in the paintbuffer is 0 or 3.

### 5.2 `ink_dual` per row

```cpp
for (row = 0; row < pr_lo; row++)          bitmask[row] = bitmask_light[row] = 0;
for (row = pr_hi + 1; row < 825; row++)    bitmask[row] = bitmask_light[row] = 0;
for (row = pr_lo; row <= pr_hi; row++) {
    bitmask[row] = epd_painter_ink_dual(
        packed_paintbuffer  + row * 304,   // v: new frame
        packed_fastbuffer   + row * 304,   // out: dark plane
        packed_lightbuffer  + row * 304,   // out: light plane
        packed_screenbuffer + row * 304,   // s: model, updated in place
        304, &bitmask_light[row]);
    any_work |= bitmask[row] | bitmask_light[row];
}
```

Per pixel (2 bits), with `spread()` = "both bits set if any bit is set":

| Output | Formula | When written |
|---|---|---|
| dark plane | `v & ~spread(s)` | always, every chunk |
| light plane | `s & spread(v ^ s)` | only chunks whose light data is non-zero |
| screen model | `(s & ~spread(v ^ s)) \| dark` | only chunks that drive something |
| `bitmask` bit (31 − chunk) | dark chunk non-zero | |
| `bitmask_light` bit (31 − chunk) | light chunk non-zero | |

Each row is 19 chunks of 16 bytes (64 pixels). Chunk *k* is mask bit `31 − k`, so bits 31..13 are used and bits 12..0 stay 0.

In BW, after the snap, there are only three cases:

| `s` → `v` | dark | light | `s'` |
|---|---|---|---|
| 0 → 3 | 3 | 0 | 3 |
| 3 → 0 | 0 | 3 | 0 |
| unchanged | 0 | 0 | same |

Chunks without light work are *not* written in `packed_lightbuffer` and keep stale data. That is harmless: their mask bit is 0, and every consumer skips unmasked chunks.

### 5.3 `_decision_batch_compat()` (`epd_painter_decision.cpp`)

For every row (all 825, not just the window):

```cpp
n = 0;
if (bitmask[row])       dec_sweeps[row*2 + n++] = { fastbuffer  + row*304, bitmask[row],       {2, 4, 6} };
if (bitmask_light[row]) dec_sweeps[row*2 + n++] = { lightbuffer + row*304, bitmask_light[row], {3, 5, 7} };
dec_nsweeps[row] = n;
```

`dec[0..2]` are the decision ids for pixel values 1, 2 and 3. An id is `(level << 1) | dir`: 2/4/6 = apply (darken to) level 1/2/3, and 3/5/7 = remove (whiten from) level 1/2/3.

Then `xSemaphoreGive(_paint_buffer_sem)`.

---

## 6. Step 4, `paintStage` decrement

```cpp
paintStage -= 1;            // 2 → 1
if (!any_work) { vTaskDelay(1); continue; }
```

The app task's `while (paintStage == 2)` now exits and `paint()` returns. The app can draw the next frame into the canvas while this one is driven. If nothing changed (`any_work == 0`), the task loops back without touching the hardware.

---

## 7. Step 5, panel power-on

```cpp
PanelPowerGuard guard(*this);
```

The constructor:
1. On first use, creates `power_mtx` and the `panel_idle_off` task.
2. Takes `power_mtx`.
3. If `state == 0` (panel off), calls `EPD_Painter::powerOn()`.
4. Sets `state = _idle_timeout_s`: 5 s by default, 30 s in the stream sketch.
5. Gives the mutex.

The guard has no destructor: the panel stays on until the idle task counts `state` down to 0.

### `EPD_Painter::powerOn()`: only when the panel was off

| # | Action | Pins / bus |
|---|---|---|
| 1 | `_pin_le->set(false)`, `_pin_spv->set(false)`, `_pin_sph->set(false)` | LE, SPV, SPH low |
| 2 | `_powerDriver->powerOn()` → `powerOnLocked()`: | |
| 2a | `EPD_PIN_HIGH(pin_gmod)` | **GMOD high** |
| 2b | `expWrite(WAKEUP, 1)`, then `delay(5)` | I²C to 0x20: **WAKEUP high** |
| 2c | if `vcom_mv` is set: write VCOM1/VCOM2 | I²C to 0x48 |
| 2d | `tpsWrite(ENABLE, 0x20)` | V3P3 on (panel logic supply) |
| 2e | `tpsWrite(UPSEQ0, 0xE4)`, `tpsWrite(DWNSEQ0, 0x1B)` | rail order |
| 2f | `expWrite(PWRUP, 1)` | **PWRUP high**: the sequencer brings up the rails |
| 2g | poll `TPS_PG` every 1 ms until `0xFA` (250 ms timeout) | I²C reads |
| 2h | `expWrite(VCOM, 1)` | **VCOM enable** |
| 2i | `EPD_PIN_HIGH(pin_oe)` | **OE high** |
| 3 | `delay(5)` | rail settle |
| 4 | `memset(dma_buffer1/2, 0, 304)` | |
| 5 | `_sphToLcd(true)`: `esp_rom_gpio_connect_out_signal(SPH, LCD_DC)` | **SPH goes high** (DC idles high) |
| 6 | `_pushRow()` × 2: transmit 308 zero bytes, **no LE, no CKV** | 2 × (SPH low 1 CL, 308 CL clocks) |
| 7 | `SPV low`, `CKV low`, `delayMicroseconds(1)`, `CKV high`, `SPV high` | one gate clock with the start pulse |

The I²C runs at 100 kHz, so each register write is about 0.3–0.4 ms. With the 5 ms waits and the rail ramp, a cold power-on costs tens of milliseconds. It happens once per idle period, not per frame.

`_pushRow()` does:
1. Wait for `lcd_start` to clear (bounded spin).
2. Swap `dma_buffer`; optionally reset the FIFO.
3. `EPD_LCD_SET_CMD(buf[0])`, `gdma_start(desc)`, `EPD_LCD_START()`.
4. Wait for `lcd_start` to clear again.

---

## 8. Step 6, train binding

```cpp
wf_len = clamp(bw_passes, 1, 32);          // 4
uint8_t bw_dk[96], bw_lt[96];              // on the paint task's stack
memset(bw_dk, 1, 3 * wf_len);              // 3 levels × wf_len codes, all "darken"
memset(bw_lt, 2, 3 * wf_len);              // all "whiten"
lt_wf = bw_lt; dk_wf = bw_dk;

for (g = 1; g <= 3; g++) {
    dec_train[(g << 1) | 0] = dk_wf + (g - 1) * wf_len;   // ids 2, 4, 6
    dec_train[(g << 1) | 1] = lt_wf + (g - 1) * wf_len;   // ids 3, 5, 7
}
for (id = 8; id < 32; id++) dec_train[id] = nullptr;
for (id = 0; id < 32; id++) dec_len[id] = wf_len;
```

`dec_train` is an array of 32 pointers, one per decision id, each pointing at `wf_len` drive codes.

At the start of **every pass**, the broadcast table is rebuilt:

```cpp
uint8_t bcast[32];
bcast[0] = bcast[1] = 0;
for (id = 2; id < 32; id++) {
    code = (dec_train[id] && pass < dec_len[id]) ? dec_train[id][pass] : 0;
    if (code == 3) code = 0;              // "both" is never sent
    bcast[id] = code * 0x55;              // copy the 2-bit code into all 4 pixel fields
}
```

For BW, on every pass: `bcast[2] = bcast[4] = bcast[6] = 0x55` (darken), `bcast[3] = bcast[5] = bcast[7] = 0xAA` (whiten), and every other entry is 0.

---

## 9. Step 7, row window and column cut

```cpp
r0 = 0; r1 = 824;
if (skip_idle_rows) {
    while (r0 < r1 && !dec_nsweeps[r0]) r0++;    // first row with work
    while (r1 > r0 && !dec_nsweeps[r1]) r1--;    // last row with work
}
row_bytes = 304 + row_pad_bytes;                 // 308
if (skip_idle_cols) {                            // off in the stream sketch
    cols = OR of every sweep mask in r0..r1;
    row_bytes = (32 - ctz(cols)) * 16 + row_pad_bytes;   // up to the last chunk with work
    if (row_bytes < 308) _setBusRowBytes(row_bytes);
}
```

`_setBusRowBytes(n)`:
1. Waits for the bus to go idle.
2. Sets `dma_desc0/1/2.dw0.size = .length = n - 1` and `lcd_dout_cyclelen = n - 2`.
3. Applies them with `EPD_LCD_UPDATE()`.

---

## 10. Step 8, the pass loop

```cpp
for (pass = 0; pass < wf_len; pass++) {               // 4 passes
    pass_t0 = esp_timer_get_time();
    build bcast[]                                     // section 8
    for (row = 0; row < r0; row++) _skipRow(row == 0);
    for (row = r0; row <= r1; row++) {
        memset(dma_buffer, 0x00, 304);                // all float
        ls = &dec_sweeps[row * 2];
        for (s = 0; s < dec_nsweeps[row]; s++) {
            tbl = { bcast[ls[s].dec[2]],              // for pixel value 3
                    bcast[ls[s].dec[1]],              // value 2
                    bcast[ls[s].dec[0]] };            // value 1
            if (s == 0) epd_painter_convert_packed_fb_to_ink   (ls[s].plane_row, dma_buffer, 304, tbl, ls[s].mask);
            else        epd_painter_convert_packed_fb_to_ink_or(ls[s].plane_row, dma_buffer, 304, tbl, ls[s].mask);
        }
        [bw_hold_black: convert_or(screenbuffer row, hold_tbl = {0x55,0,0}, every visible chunk)]
        [discharge_rows: maybe _dischargeColumns()]
        sendRow(row == 0, false, false);              // section 11
    }
    _endPass();                                       // section 12
    if (r1 < 824) _flushGate(r1);
    epd_wait_until_us(pass_t0 + min_pass_us);         // pad short passes
}
```

What each row's bus data is:
- **Row with no sweep** (inside the window but unchanged): 304 zero bytes, all float.
- **Darken-only row:** `convert` writes `01` at pixels whose dark-plane value is 3, and `00` elsewhere in active chunks. Inactive chunks keep the `memset` zeros.
- **Whiten-only row:** the same with `10`.
- **Mixed row:** the first sweep writes and the second ORs. The planes are pixel-disjoint, so each pixel ends up with exactly one code.

The `memset` and the conversion run on the CPU **while the previous row is still being clocked out** by DMA, because `dma_buffer` always points at the idle buffer (section 11).

`epd_wait_until_us(t)` sleeps with `delay()` while more than 1.5 ms remain, then spins on `esp_timer_get_time()`.

---

## 11. Step 9, inside `sendRow()`

`sendRow(firstLine, lastLine, noAdvance)`, in order. This is the BW path, so there is no `le_hold_ns` and no `row_extra_us` (both are grey-16 only).

```
1  while (LCD_CAM.lcd_user.lcd_start) {}          spin: previous row still being clocked
2  if (dma_buffer == dma_buffer1) { desc = &dma_desc1; dma_buffer = dma_buffer2; }
   else                           { desc = &dma_desc2; dma_buffer = dma_buffer1; }
                                                  the buffer just filled goes out; the CPU
                                                  fills the other one next
3  if (afifo_reset_per_row) LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
4  EPD_LCD_SET_CMD(buffer[0])                     lcd_cmd_val (S3) / lcd_first_cmd_val (S31):
                                                  byte 0 goes out in the CMD phase
5  gdma_start(dma_chan, desc)                     DMA starts fetching buffer[1..307] into the LCD FIFO
6  dma_t0 = cycle count
7  if firstLine:          _vscanStart()                         (section 12.1)
   else if noAdvance:     LE high, LE low                       (not used by BW)
   else:
       CKV low                                    W1TC: every gate output off
       [delayMicroseconds(gate_off_us)]           0 by default: skipped
       LE high, LE low                            two register writes = the latch pulse.
                                                  The source chain holds the PREVIOUS row
                                                  (fully shifted in by step 1) → column outputs
       delayMicroseconds(gate_on_us)              1 µs
       CKV high                                   gate token moves one line; that line now
                                                  takes the data just latched
8  _waitPrefill(dma_t0)                           no-op unless dma_prefill_ns
9  EPD_LCD_START()                                lcd_start = 1 (S31 also sets lcd_update_reg):
                                                  LCD_CAM clocks this row out
10 if lastLine: spin until done; CKV low; LE high; LE low; CKV high
                                                  (latch this row now; used by the
                                                   closing sweep's last row)
```

### What the bus does after step 9 (one row)

```
                ┌ CMD (1 CL) ┐┌──────────────── DOUT phase (307 CL) ─────────────────┐
CL   ___________|‾‾|__|‾‾|__|‾‾|__|‾‾|__ ... __|‾‾|__________________________________
SPH  ‾‾‾‾‾‾‾‾‾‾‾|________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾ ... ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  (LCD DC line)
D7-0 ===========X  b0   X  b1  X  b2  X ... X b307 X===================================
                ← 308 × 100 ns = 30.8 µs at 10 MHz →
```

- Byte 0 comes from the CMD register; bytes 1..307 come from the descriptor (`buffer + 1`, length 307).
- Bytes 304..307 are the zero pad (`row_pad_bytes = 4`). They flush the far end of the source chain before the next latch.

### One row slot, all pins

```
            sendRow(n) called
            │ spin (row n-1 clocking)  │ pins      │ LCD_CAM clocks row n (30.8 µs) ...
CKV  ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_______‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
LE   _________________________________|‾|________________________________________________
                                       ↑ latch row n-1   ↑ CKV high after 1 µs: gate steps
CL   ~~~~~ (row n-1) ~~~~~~~~~~~~~~~~~~_______________~~~~~~~ (row n) ~~~~~~~~~~~~~~~~~~~~
CPU  [convert row n] [spin]           [pins]          [memset + convert row n+1] [spin]...
```

- The LE pulse is two consecutive register writes, tens of ns wide.
- The pin section takes about 1.2 µs: the `gate_on_us` delay plus the register writes.
- A row slot is therefore about 32 µs, with the bus busy for 30.8 µs of it.

### Pipeline across rows

```
 sendRow call:     r0          r0+1         r0+2       ...   r1         _endPass
 bytes clocked:    row r0      row r0+1     row r0+2         row r1     float row
 LE latches:       (vscan)     row r0       row r0+1         row r1-1   row r1
```

Each latch publishes the row clocked out during the previous slot. That is why `_endPass()` exists.

---

## 12. Step 10, start and end of a pass

### 12.1 `_vscanStart()`: from `sendRow()` or `_skipRow()` for row 0

```
SPV low
delayMicroseconds(vscan_hold_us = 8)
CKV low
LE high, LE low                     latch: the chain holds the float row the last
                                    pass or sweep ended with → outputs float
delayMicroseconds(1)
CKV high                            rising edge with SPV low: token enters the gate register
delayMicroseconds(8)
SPV high
repeat vscan_skip_lines (3) times:
    CKV low, delayMicroseconds(1), CKV high, delayMicroseconds(8)
```

This takes about 44 µs. With `r0 > 0`, `_skipRow(true)` makes this call on the first skipped row instead.

### 12.2 `_skipRow()` / `_gateStep()`: rows 1..r0−1 above the window

```
spin until lcd_start == 0
CKV low, delayMicroseconds(1), CKV high, delayMicroseconds(gate_step_us = 3)
```

That is about 4 µs per row, with no data and no latch.

### 12.3 `_endPass()`

```cpp
memset(dma_buffer, 0x00, 304);
sendRow(false);        // latches row r1, steps the gate onto its line, clocks a float row
```

### 12.4 `_flushGate(r1)`: only if `r1 < 824`

```
spin until lcd_start == 0       the float row from _endPass is fully in the chain
CKV low, LE high, LE low        gates off, outputs ← float
delayMicroseconds(1), CKV high, delayMicroseconds(3)
for i in r1 .. 825:  _gateStep()                (826 − r1 steps of ~4 µs)
```

This clocks the gate token off the bottom of the panel, so the next pass's SPV doesn't start a second token. Every line it crosses is driven with float.

### 12.5 Pass padding

`epd_wait_until_us(pass_t0 + 4000)`. Rows above the window are skipped and rows below are flushed, about 4 µs each. So every pass costs about 825 × 4 µs ≈ 3.3 ms of gate steps, plus 32 µs per sent row. The 4 ms floor therefore only binds for windows of about 20 rows or fewer.

---

## 13. Step 11, closing sweep

After the last pass:

```cpp
vTaskDelay(1);                                       // yield once per frame (watchdog, other tasks)
if (row_bytes < full_row_bytes) _setBusRowBytes(full_row_bytes);
memset(dma_buffer1, 0, 304); memset(dma_buffer2, 0, 304);
for (row = 0; row < r0; row++) _skipRow(row == 0);
for (row = r0; row <= r1; row++) sendRow(row == 0, row == r1);   // last one latches itself
if (r1 < 824) _flushGate(r1);
```

Every row in the window is latched with all-float data, so no line is left holding a drive code between frames. Afterwards:
- the profiling line, if `setPaintProfile(true)`;
- `_sb_guard_update()` (no-op by default);
- `_paints_done++`.

---

## 14. Step 12, second cycle and idle

Back at the loop top, `paintStage` is 1, so the task does not park. It runs discovery again (section 5) on the same paintbuffer:
- the BW snap changes nothing;
- `ink_dual` finds `v == s` everywhere, because the model was updated in cycle 1;
- `any_work = 0`.

So `paintStage` becomes 0, `vTaskDelay(1)`, and the loop continues. At the top it gives `_paint_active_sem` and parks in `while (paintStage == 0) vTaskDelay(1)`.

If the app submitted a new frame during cycle 1, `paintStage` was set back to 2. The second cycle then *is* the new frame's cycle: while streaming, there is one drive cycle per frame.

---

## 15. Step 13, power-off

`panel_idle_off` wakes every second, takes `power_mtx` and decrements `state`. When `state` reaches 0 it calls `EPD_Painter::powerOff()`:

```
LE low, CKV low
_sphToLcd(false)              SPH back to a plain GPIO (DC would idle it high)
SPH low, SPV low
powerOffLocked():
    OE low, GMOD low
    expWrite(VCOM, 0), expWrite(PWRUP, 0)
    poll TPS_PG until 0 (250 ms timeout)        rails collapsed
    tpsWrite(ENABLE, 0x00)                      V3P3 off
    expWrite(WAKEUP, 0)                         PMIC sleeps
```

Every panel line is low before V3P3 drops.

---

## 16. Pin timeline of a whole frame

A frame whose changes span rows 100..199, with the panel already powered and `bw_passes = 4`:

```
time →
 pass 0                                                                pass 1 ... closing sweep
 ├vscan┤├─ 99 skips ─┤├──── rows 100..199 ────┤├end┤├─ flush 627 steps ─┤ ...     (same shape,
 44 µs   ~0.4 ms      100 × ~32 µs = 3.2 ms   32µs  ~2.5 ms                          float data)

SPV  _|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  low once per pass
CKV  |_|‾|_|‾|_|‾|_|‾ ×99 ... |_‾‾ ×100 rows ... |_‾ |_|‾|_|‾ ×627 ...      one pulse per line
LE   _|‾|___________________|‾|‾|‾ ×100 ... ‾|_|‾|____________________       one per sent row
CL   ___________________▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒____________________       308 clocks per sent row
SPH  ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾|_‾|_‾|_‾ ×101 ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾       low for byte 0 of each row
GMOD ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  high while powered
OE   ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  high while powered
```

Each pass is about 6.2 ms here: 44 µs + ~0.4 ms + 3.2 ms + ~2.5 ms. Four passes plus a closing sweep make about 31 ms for the frame.

---

## 17. Data structure reference

### 17.1 8bpp canvas

`uint8_t[1200 × 825]`, row-major, `canvas[y * 1200 + x]`, value & 3: 0 = white, 3 = black. Values 1 and 2 are greys; BW snaps them to white and black.

### 17.2 Packed 2bpp row (paint, screen, fast and light buffers)

```
 byte:   0            1            2              299            300..303
        [p0 p1 p2 p3] [p4 p5 p6 p7] [p8 ...       ] [p1196..p1199] [0 0 0 0]   stride 304
 bits:   7-6 5-4 3-2 1-0    (p0 in bits 7-6)
 chunks: bytes 0-15 = chunk 0 (pixels 0-63), ..., bytes 288-303 = chunk 18
         (pixels 1152-1215, of which 1200-1215 are padding)
 pixel (x, physical row r): buf + r*304 + x/4, field (3 - x%4)*2
```

Rows are in **physical** order: row 0 is the first line the gate scan drives, and pixel 0 the first one clocked. With `ROTATION_180` that is canvas pixel (1199, 824).

| Buffer | Values it holds |
|---|---|
| paintbuffer | target levels; only 0/3 after the BW snap |
| screenbuffer | model of the glass; 0/3 in BW |
| fastbuffer (dark plane) | target level where the glass is white and the pixel changes, else 0 |
| lightbuffer (light plane) | old level where a non-white pixel changes, else 0; stale in unmasked chunks |

### 17.3 Chunk mask (`bitmask[row]`, `bitmask_light[row]`, `LineSweep::mask`)

```
 bit 31 = chunk 0 (pixels 0-63), bit 30 = chunk 1, ..., bit 13 = chunk 18; bits 12..0 unused (0)
```

The kernels consume it MSB first: `extui a9, a6, 31, 1; slli a6, a6, 1` in assembly, `chunk_flags & 0x80000000; chunk_flags <<= 1` in C.

### 17.4 `LineSweep` (`EPD_Painter.h`)

```cpp
struct LineSweep {            // 12 bytes on 32-bit
    const uint8_t *plane_row; // fastbuffer or lightbuffer row
    uint32_t       mask;      // chunk mask
    uint8_t        dec[3];    // decision id for pixel value 1, 2, 3
};                            // + 1 byte padding
```

Indexed as `dec_sweeps[row * DEC_MAX_SWEEPS + s]`, with `DEC_MAX_SWEEPS = 2`.

### 17.5 Decision ids and trains

| id | Meaning | `dec_train[id]` in BW |
|---|---|---|
| 0, 1 | level 0 (white): nothing | — (`bcast` 0) |
| 2 / 3 | apply / remove level 1 | `bw_dk + 0` / `bw_lt + 0` |
| 4 / 5 | apply / remove level 2 | `bw_dk + wf_len` / `bw_lt + wf_len` |
| 6 / 7 | apply / remove level 3 | `bw_dk + 2·wf_len` / `bw_lt + 2·wf_len` |
| 8..31 | grey-16 / direct engines | `nullptr` |

A train is `wf_len` bytes, one drive code (0..3) per pass.

### 17.6 Drive codes and `bcast`

| Code | Bits | `bcast` byte | On the panel |
|---|---|---|---|
| 0 | 00 | 0x00 | float |
| 1 | 01 | 0x55 | darken |
| 2 | 10 | 0xAA | whiten |
| 3 | 11 | sent as 0x00 | (both; never sent) |

The `tbl[3]` passed to `convert` is `{ byte for value 3, value 2, value 1 }`.

### 17.7 Bus row buffer (`dma_buffer1/2`)

```
 [0]        → CMD phase (SPH low), via lcd_cmd_val / lcd_first_cmd_val
 [1..303]   → DOUT phase, by DMA
 [304..307] → DOUT, always 0 (pad); zeroed in begin() and never written again
 each byte: 4 pixels' drive codes, pixel 0 in bits 7-6 → D7:D6
```

### 17.8 `dma_descriptor_t` (IDF `hal/dma_types.h`)

```
 dw0: size[11:0]=307  length[11:0]=307  ...  err_eof=0  suc_eof=1  owner=0
 buffer → dma_bufferN + 1
 next   → NULL            one descriptor per transfer; GDMA stops after it
```

`owner_check` is disabled, so the owner bit is never consulted and the descriptors are reused unchanged every row.

### 17.9 LCD_CAM fields touched per row

| Field | Written | Value |
|---|---|---|
| `lcd_user.lcd_start` | `EPD_LCD_START()`; polled | 1 = start; hardware clears it when the transfer is done |
| `lcd_user.lcd_update` / `lcd_update_reg` | S31 on every start; after `_setBusRowBytes` | apply parameters |
| `lcd_cmd_val` / `lcd_first_cmd_val` | every row | byte 0 |
| `lcd_misc.lcd_afifo_reset` | every row if `afifo_reset_per_row` | 1 |
| `lcd_user.lcd_dout_cyclelen` | `_setBusRowBytes` | bytes − 2 |

---

## 18. Kernel reference: assembly

`src/EPD_Painter.S` builds on Xtensa (S3) only. The calling convention is windowed: `entry a1, 32`, arguments in `a2..a7`, return value in `a2`, `retw.n`. The PIE registers `q0..q7` are 128 bits each.

### 18.1 `epd_painter_compact_pixels(a2 = in, a3 = out, a4 = bytes)`

```asm
ee.zero.q   q7                    ; q7 = 0
movi a10, 3 ; s32i a10, a1, 0
ee.vldbc.8  q5, a1                ; q5 = 0x03 × 16
movi a10, 1 ; wur.sar_byte a10    ; ee.src.q shifts by 1 byte
movi a10, 2 ; wsr.sar a10         ; ee.vsl.32 shifts by 2 bits
mov a11, a4
loop:
  ee.vld.128.ip q0, a2, 16        ; 16 pixels
  ee.andq   q0, q0, q5            ; keep 2 bits each
  ee.src.q  q1, q0, q7            ; q1 = q0 shifted down 1 byte
  ee.vsl.32 q0, q0                ; q0 <<= 2 within each 32-bit lane
  ee.orq    q0, q0, q1            ; repeated 3 times (q1 shifted again each time)
  ...
  ee.vunzip.8 q0, q1 ; ee.vunzip.8 q0, q1   ; gather the packed bytes into lane 0
  ee.stxq.32 q0, q7, a3, 0, 0     ; store 4 bytes
  addi a3, a3, 4 ; addi a11, a11, -16 ; bnez a11, loop
```

16 input bytes become 4 output bytes per iteration, first pixel in the top bits.

### 18.2 `epd_painter_convert_packed_fb_to_ink(a2 = in, a3 = out, a4 = len, a5 = tbl, a6 = mask)`

| Register | Holds |
|---|---|
| `q7` | 0x55 × 16 |
| `q4`, `q5`, `q6` | `tbl[0]`, `tbl[1]`, `tbl[2]` broadcast (bytes for values 3, 2, 1) |
| `SAR` | 1 |

```asm
loop:
  extui a9, a6, 31, 1 ; slli a6, a6, 1    ; next mask bit
  beqz  a9, skip
  ee.vld.128.ip q0, a2, 16                ; 64 pixels
  ee.andq   q3, q0, q7                    ; lo bits
  ee.vsr.32 q2, q0 ; ee.andq q2, q2, q7   ; hi bits
  ee.andq   q1, q3, q2                    ; value 3
  ee.notq   q0, q3 ; ee.andq q0, q2, q0   ; value 2 = hi & ~lo
  ee.notq   q2, q2 ; ee.andq q2, q3, q2   ; value 1 = lo & ~hi
  ee.vsl.32 q3, q1 ; ee.orq q3, q3, q1    ; widen m3 to both bits
  ee.vsl.32 q1, q0 ; ee.orq q0, q0, q1    ; widen m2
  ee.vsl.32 q1, q2 ; ee.orq q2, q2, q1    ; widen m1
  ee.andq q3, q4, q3 ; ee.andq q0, q5, q0 ; ee.orq q3, q3, q0
  ee.andq q2, q6, q2 ; ee.orq q3, q3, q2  ; out = w3&m3 | w2&m2 | w1&m1
  ee.vst.128.ip q3, a3, 16
  j next
skip:
  addi a2, a2, 16 ; addi a3, a3, 16       ; output untouched (stays at the memset float)
next:
  addi a4, a4, -16 ; bnez a4, loop
```

The `_or` variant inserts `ee.vld.128.ip q1, a3, 0 ; ee.orq q3, q3, q1` before the store.

### 18.3 `epd_painter_ink_dual(a2 = new, a3 = dark, a4 = light, a5 = screen, a6 = len, a7 = &maskL)`

| Register | Holds |
|---|---|
| `q6` | 0x55 × 16 |
| `q5` | 0xAA × 16 (`q6 << 1`) |
| `a12` | current chunk's mask bit, starts at `1 << 31` |
| `a11` | dark mask accumulator |
| `a9` | light mask accumulator |

```asm
loop:
  ee.vld.128.ip q2, a2, 16                ; v
  ee.vld.128.ip q3, a5, 16                ; s   (a5 now points past this chunk)
  ; q0 = ~spread(s)
  ee.vsl.32 q1, q3 ; ee.andq q1, q1, q5 ; ee.orq q0, q3, q1
  ee.vsr.32 q1, q0 ; ee.andq q1, q1, q6 ; ee.orq q0, q0, q1
  ee.notq q0, q0
  ee.andq q4, q2, q0                      ; dark = v & white
  ; q0 = spread(v ^ s)
  ee.xorq q0, q2, q3
  ee.vsl.32 q1, q0 ; ee.andq q1, q1, q5 ; ee.orq q0, q0, q1
  ee.vsr.32 q1, q0 ; ee.andq q1, q1, q6 ; ee.orq q0, q0, q1
  ee.andq q1, q3, q0                      ; light = s & changed
  ee.notq q0, q0 ; ee.andq q3, q3, q0 ; ee.orq q3, q3, q4   ; s' = (s & ~changed) | dark
  ee.vst.128.ip q4, a3, 16                ; dark plane: always stored
  ee.movi.32.a q4, a10, 0..3 ; or ...     ; a10 = OR of the 4 lanes
  beqz a10, 1f ; or a11, a11, a12         ; dark mask bit
1: mov a8, a10
  ee.movi.32.a q1, ... ; or ...           ; a10 = OR of the light lanes
  beqz a10, 2f
  or a9, a9, a12 ; ee.vst.128.ip q1, a4, 16 ; j 3f    ; light plane stored only if non-zero
2: addi a4, a4, 16
3: or a8, a8, a10
  beqz a8, next
  addi a10, a5, -16 ; ee.vst.128.ip q3, a10, 0        ; screen chunk written back
next:
  srli a12, a12, 1 ; addi a6, a6, -16 ; bnez a6, loop
s32i a9, a7, 0 ; mov a2, a11 ; retw.n
```

---

## 19. Kernel reference: C

`src/epd_painter_kernels_c.c` builds when `__XTENSA__` is not defined. It has the same symbols and gives the same output bit for bit, working on 32-bit words (16 pixels) instead of 128-bit vectors.

```c
// compact: one 32-bit little-endian word = canvas pixels p0..p3 in bytes 0..3
v = in[k] & 0x03030303;
b = (v << 6 | v >> 4 | v >> 14 | v >> 24) & 0xFF;     // p0→7:6  p1→5:4  p2→3:2  p3→1:0

// convert (per word)
lo = x & 0x55555555;  hi = (x >> 1) & 0x55555555;
m3 = lo & hi;  m2 = hi & ~lo;  m1 = lo & ~hi;
m3 |= m3 << 1;  m2 |= m2 << 1;  m1 |= m1 << 1;
out = (w3 & m3) | (w2 & m2) | (w1 & m1);              // w? = tbl byte × 0x01010101

// spread (used by ink_dual)
x |= (x << 1) & 0xAAAAAAAA;  x |= (x >> 1) & 0x55555555;
```

A chunk is 4 words. `ink_dual` computes the 4 words, ORs them to decide the mask bits, and stores exactly as the assembly does: dark always, light if non-zero, screen if anything is driven.

---

## 20. Worked example

Physical row 10, physical x = 0..7, new pixels `0 3 3 0 | 0 0 3 3`. The glass shows all white.

| Stage | Bytes |
|---|---|
| packed paintbuffer `[0..1]` | `00 11 11 00` = **0x3C**, `00 00 11 11` = **0x0F** |
| BW snap | unchanged (already 0/3) |
| screenbuffer before | 0x00, 0x00 |
| dark plane | 0x3C, 0x0F; `bitmask[10]` bit 31 set |
| light plane | 0x00, 0x00; chunk 0 not stored; `bitmask_light[10]` bit 31 clear |
| screenbuffer after | 0x3C, 0x0F |
| sweep list for row 10 | 1 sweep: `{fastbuffer + 10·304, 0x80000000, {2,4,6}}` |
| tbl (every pass) | `{bcast[6], bcast[4], bcast[2]}` = `{0x55, 0x55, 0x55}` |
| `convert` byte 0 (0x3C) | m3 = `00 11 11 00` → out = `00 01 01 00` = **0x14** |
| `convert` byte 1 (0x0F) | m3 = `00 00 11 11` → out = `00 00 01 01` = **0x05** |
| `dma_buffer[0..1]` | 0x14 (CMD phase, SPH low), 0x05 (first DMA byte) |
| bus | first CL: D7..D0 = `0 0 0 1 0 1 0 0`; second CL: `0 0 0 0 0 1 0 1` |
| passes | the same bytes on all 4 passes (BW trains are constant) |
| closing sweep | 0x00, 0x00 |

When the same pixels go back to white in the next frame:
- the light plane gets 0x3C / 0x0F;
- tbl = `{bcast[7], bcast[5], bcast[3]}` = `{0xAA, 0xAA, 0xAA}`;
- the bus bytes become `00 10 10 00` = 0x28 and `00 00 10 10` = 0x0A.

---

## 21. Other entry points

| Call | Difference from `paint()` |
|---|---|
| `paintLater(fb)` | Same, but returns right after `paintStage = 2` (no wait) |
| `paintRows(fb, y0, y1)` | Waits for the **previous** submission to be picked up *first*, then packs only rows y0..y1 (mapped to physical rows; with `ROTATION_180` that is 824−y1..824−y0) and sets `_pr_lo/_pr_hi`, so the snap and discovery touch only those rows. Returns without waiting |
| `paintPacked(packed, line_repeat)` | No `_pack`: `memcpy` (or a line-repeated copy) of a ready packed frame, then waits like `paint()` |
| `paintPackedRows(packed, r0, r1)` | `paintRows` for packed physical rows: waits for the previous pickup, copies rows r0..r1 only, returns. Used by the GIF player and the screen stream |
| `unpaintPacked(packed)` | Sets the screen model to `packed` and the paintbuffer to white, then waits: drives a whiten for everything in `packed` |
| `clear()` | Whitens the paintbuffer and lets one discovery cycle run (`paintStage = 1`). Then takes `_paint_active_sem` and drives 4 fixed-pattern phases itself from the caller's task: 0x55 × 6, 0xAA × 2, 0x55 × 4, 0xAA × 8 passes, each followed by `_endPass()` and `delay(5)`. Ends with a float sweep |

---

## 22. Timing summary

At 10 MHz, `bw_passes = 4`, panel already on:

| Item | Cost |
|---|---|
| One sent row | ~32 µs (30.8 µs bus + ~1.2 µs pins) |
| One skipped row / gate step | ~4 µs |
| `_vscanStart` | ~44 µs |
| `_flushGate(r1)` | ~4 µs × (826 − r1) |
| Any pass, fixed part (skips + flush) | ~3.3 ms |
| Full-window pass | 825 × 32 µs ≈ 26.5 ms |
| Full-screen BW frame | 4 × 26.5 + 26.5 (closing) + discovery ≈ 135 ms |
| 100-row window frame | ≈ 5 × 6.2 ms ≈ 31 ms |
| `_pack` (canvas path) | ~1–2 ms |
| BW snap + `ink_dual`, full frame | a few ms (PSRAM-bound) |
| Cold power-on | tens of ms (I²C at 100 kHz, 5 ms waits, rail ramp) |
| Paint task wake-up latency | ≤ 1 ms (1 kHz tick polling) |
