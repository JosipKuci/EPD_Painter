#ifdef ARDUINO
#include "esp32-hal.h"
#endif
#include <string.h>
#include <cstring>
#include "build_opt.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include <driver/periph_ctrl.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <hal/gpio_hal.h>
#include <soc/lcd_cam_struct.h>
#include <soc/gdma_struct.h>
#include "EPD_Painter.h"
#include "EPD_Painter_tuned.h"
#include <epd_painter_powerctl.h>
#include "epd_painter_bootctl.h"
#include "epd_pin_driver.h"

#ifndef EPD_PAINTER_NO_NVS
#include <nvs.h>
#endif

#ifdef ARDUINO
  #include "Wire.h"
#else
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
#endif

// Define EPD_ASM_TIMING to enable assembly-function timing output.
// Prints compact_pixels, epd_painter_ink_dual, and convert_packed_fb_to_ink
// durations to serial each frame so you can compare old vs new .S builds.
//#define EPD_ASM_TIMING

// LCD_CAM signal indices for the 8 parallel data lines
static const uint8_t kDataSignals[8] = {
  LCD_DATA_OUT0_IDX,
  LCD_DATA_OUT1_IDX,
  LCD_DATA_OUT2_IDX,
  LCD_DATA_OUT3_IDX,
  LCD_DATA_OUT4_IDX,
  LCD_DATA_OUT5_IDX,
  LCD_DATA_OUT6_IDX,
  LCD_DATA_OUT7_IDX,
};


// Assembly routines — see EPD_Painter.S for full documentation
extern "C" void epd_painter_compact_pixels(
  const uint8_t *input, uint8_t *output, uint32_t size);

// =============================================================================
// compact_pixels_rotated_cw
//
// Combines 90° clockwise rotation and 8bpp→2bpp compaction in one pass.
//
// The portrait canvas is src_w wide × src_h tall (e.g. 540×960).
// The physical panel is src_h wide × src_w tall (e.g. 960×540).
//
// Rotation mapping (clockwise):
//   canvas pixel (col=x, row=y)  →  panel pixel (col=y, row=src_w-1-x)
//
// Processed in blocks of 16 portrait rows so the working set (16 × src_w bytes)
// stays warm in cache across all src_w column iterations within each block,
// avoiding repeated PSRAM fetches for the strided column access pattern.
//
// src_h must be a multiple of 16. out_stride is the packed row stride
// (>= src_h / 4); bytes past src_h / 4 in each output row are left alone.
// =============================================================================
static IRAM_ATTR void compact_pixels_rotated_cw(
    const uint8_t* src, uint8_t* dst, int src_w, int src_h, int out_stride)
{

    for (int rb = 0; rb < src_h; rb += 16) {
        for (int cx = 0; cx < src_w; cx++) {
            // CW: canvas column cx → output row (src_w - 1 - cx)
            uint8_t* out       = dst + (src_w - 1 - cx) * out_stride + rb / 4;
            const uint8_t* col = src + rb * src_w + cx;

            // 16 portrait rows → 4 packed output bytes, 4 pixels per byte.
            // Fully unrolled; all 16 loads are to addresses within the current
            // 16-row block (rb * src_w .. (rb+15) * src_w), which is warm in cache.
            out[0] = ((col[ 0        ] & 3) << 6) | ((col[  src_w] & 3) << 4)
                   | ((col[ 2*src_w  ] & 3) << 2) |  (col[ 3*src_w] & 3);
            out[1] = ((col[ 4*src_w  ] & 3) << 6) | ((col[ 5*src_w] & 3) << 4)
                   | ((col[ 6*src_w  ] & 3) << 2) |  (col[ 7*src_w] & 3);
            out[2] = ((col[ 8*src_w  ] & 3) << 6) | ((col[ 9*src_w] & 3) << 4)
                   | ((col[10*src_w  ] & 3) << 2) |  (col[11*src_w] & 3);
            out[3] = ((col[12*src_w  ] & 3) << 6) | ((col[13*src_w] & 3) << 4)
                   | ((col[14*src_w  ] & 3) << 2) |  (col[15*src_w] & 3);
        }
    }

}

// 16-grey pack of one unrotated run: 8bpp level codes (0..15) → 4bpp, first
// pixel in the high nibble. _pack() handles rows, stride and rotation.
static void compact_pixels16(const uint8_t *src, uint8_t *dst, uint32_t n) {
  for (uint32_t i = 0; i < n; i += 2)
    *dst++ = (uint8_t)(((src[i] & 15) << 4) | (src[i + 1] & 15));
}

extern "C" void epd_painter_convert_packed_fb_to_ink(
  const uint8_t *packed_fb, uint8_t *output, uint32_t length,
  const uint8_t *waveform, uint32_t chunk_flags);

extern "C" void epd_painter_convert_packed_fb_to_ink_or(
  const uint8_t *packed_fb, uint8_t *output, uint32_t length,
  const uint8_t *waveform, uint32_t chunk_flags);

// Wait until an absolute microsecond deadline.
//
// Every inter-pass wait in this driver is part of the retention dose the
// waveform tables are calibrated against, so "about 4 ms" is not good
// enough. vTaskDelay quantises to the FreeRTOS tick, unblocks on a tick
// boundary rather than when the interval expires, and then still has to be
// scheduled behind WiFi and everything else - several percent of a 15 ms
// period, where a 15% dose error is enough to band a ramp visibly.
//
// esp_timer_get_time() is a 64-bit microsecond counter off the S3's
// systimer: monotonic, no wrap to worry about, ~100 ns to read. Sleep while
// there is more than ~1.5 ms left (so the CPU is yielded and the watchdog
// fed), then spin out the remainder. The spin is bounded and the paint task
// is dedicated, so it costs nothing that matters, and the deadline lands
// within interrupt latency instead of within a tick.
static inline void epd_wait_until_us(int64_t target_us) {
  for (;;) {
    const int64_t left = target_us - esp_timer_get_time();
    if (left <= 1500) break;
    EPD_DELAY_MS((uint32_t)((left - 1200) / 1000));
  }
  while (esp_timer_get_time() < target_us) { /* fine wait */ }
}

// Sub-microsecond hold, off the CPU cycle counter.
//
// esp_timer_get_time() has ~100 ns of read overhead and 1 us resolution, so
// it cannot express the widths that matter for a latch pulse. CCOUNT ticks
// once per CPU clock — 4.17 ns at 240 MHz — and reads in a few cycles.
//
// The loop itself sets the resolution floor: a read, a subtract and a
// compare is order 10-15 cycles, so anything under ~60 ns is simply "one
// pass of the loop" rather than the width asked for. Values are therefore
// a lower bound on the true hold, never an upper one.
// The CPU clock rate, read once at begin() and held.
//
// epd_hold_ns() counts CPU CYCLES, so a cycle is 4.17 ns at 240 MHz and
// 12.5 ns at 80. Nothing in the driver changes the clock, so one reading is
// correct — but if frequency scaling is ever added, THIS MUST BE UPDATED
// WITH IT. A stale rate scales every LE pulse silently, and a wrong latch
// width does not announce itself; it just quietly stops reproducing the
// tables it was calibrated against.
static uint32_t s_epd_cpu_mhz = 240;

static inline uint32_t epd_cpu_mhz() { return s_epd_cpu_mhz; }

// The CPU cycle-counter read was renamed between ESP-IDF majors:
// esp_cpu_get_ccount() (IDF 4.x / Arduino core 2.x) became
// esp_cpu_get_cycle_count() (IDF 5.x / Arduino core 3.x).
#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define EPD_CCOUNT() esp_cpu_get_cycle_count()
#else
  #define EPD_CCOUNT() esp_cpu_get_ccount()
#endif

static inline void IRAM_ATTR epd_hold_ns(uint32_t ns) {
  if (!ns) return;
  const uint32_t cycles = (uint32_t)(((uint64_t)ns * epd_cpu_mhz()) / 1000ULL);
  const uint32_t t0 = EPD_CCOUNT();
  while ((EPD_CCOUNT() - t0) < cycles) { /* spin */ }
}

extern "C" uint32_t epd_painter_ink_dual(const uint8_t *packed_paintbuffer, uint8_t *packed_fastbuffer, uint8_t *packed_lightbuffer, uint8_t *packed_screenbuffer, uint32_t length, uint32_t *maskL_out);

static inline void epd_gpio_func_sel(int pin) {
  esp_rom_gpio_pad_select_gpio((gpio_num_t)pin);
}


#define PASS_COUNT 13

// LCD_CAM pixel clock from the 160 MHz PLL: pclk = 160 MHz / div_num / pre,
// div_num 2..255 (integer group divider, fractional part unused), pre 2..64
// (clkcnt_n + 1). Picks the pair closest to the request; ties keep the
// smallest prescale, i.e. the lowest lcd_clk (4 MHz -> 160 / 20 / 2).
static uint32_t epd_pick_pclk(uint32_t hz, int &div_num, int &pre) {
  const uint32_t src = 160000000u;
  if (hz == 0) hz = 1;
  uint32_t best_err = UINT32_MAX, best_hz = 0;
  div_num = 2; pre = 2;
  for (int b = 2; b <= 64; b++) {
    uint32_t a = (src + (hz * (uint32_t)b) / 2) / (hz * (uint32_t)b);
    if (a < 2) a = 2;
    if (a > 255) a = 255;
    const uint32_t got = src / (a * (uint32_t)b);
    const uint32_t err = got > hz ? got - hz : hz - got;
    if (err < best_err) { best_err = err; best_hz = got; div_num = (int)a; pre = b; }
  }
  return best_hz;
}


EPD_Painter::EPD_Painter(const Config &config, bool portrait) {
  _config = config;
  _preset = &config;
  if (portrait) _config.rotation = Rotation::ROTATION_CW;
  packed_row_bytes = packedRowBytesFor(_config.width);
}


void EPD_Painter::setQuality(Quality quality) {
  if (_grey16 && (quality == Quality::QUALITY_FAST || quality == Quality::QUALITY_BW)) {
    printf("[EPD_Painter] QUALITY_FAST/BW cannot express 16 greys — call setGreyLevels(4) first\n");
    return;
  }
  const bool changed = (_config.quality != quality);
  _config.quality = quality;
  // Direct trains are per-quality: once the direct engine has been
  // enabled, keep the loaded set matched to the quality.
  if (changed && dec_spill_dir) _load_preset_direct_trains();
  // So are the 16-grey trains, and by more than a table: NORMAL runs 13
  // passes at one period, HIGH 20 at another. Leaving the library alone
  // across a quality change silently drove one quality's trains at the
  // other's dose, which is the single reason HIGH never looked right.
  if (changed && _grey16) _grey16_build_trains();
}

// Load the preset's tuned direct tables for the current quality. Boards
// with no tuned direct tables at all are left alone (fully manual); a
// tuned board with no set for this quality unloads every pair so the
// engine falls back to the DC-correct two-step.
void EPD_Painter::_load_preset_direct_trains() {
  if (!_config.trains.dir_normal && !_config.trains.dir_fast) return;
  const uint8_t (*tbl)[4][DEC_WF_LEN_DIR] = nullptr;
  switch (_config.quality) {
    case Quality::QUALITY_NORMAL: tbl = _config.trains.dir_normal; break;
    case Quality::QUALITY_FAST:   tbl = _config.trains.dir_fast;   break;
    default: break;  // HIGH: no tuned set
  }
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++) {
      if (f == t) continue;
      const uint8_t *tr = tbl ? tbl[f][t] : nullptr;
      // An all-float row is an ABSENT pair, not a train that does nothing.
      // A table may be partly filled — the FAST tuner publishes only the
      // pairs it actually converged on — and loading a row of zeros would
      // engage the direct engine for that pair and then drive no passes at
      // all, leaving the pixel stuck at its old level. Unloaded pairs must
      // fall back to the DC-correct two-step.
      if (tr) {
        bool any = false;
        for (int p = 0; p < DEC_WF_LEN_DIR && !any; p++) any = (tr[p] != 0);
        if (!any) tr = nullptr;
      }
      setDirectTrain((uint8_t)f, (uint8_t)t, tr, DEC_WF_LEN_DIR);
    }
}

// =============================================================================
// setGreyLevels() — 16-grey mode switch (decision engine phase C)
// =============================================================================
bool EPD_Painter::setGreyLevels(int levels) {
  if (levels != 4 && levels != 16) return false;
  const bool on = (levels == 16);
  if (on == _grey16) return true;

  if (_has_template) {
    printf("[EPD_Painter] setGreyLevels refused while a template is active"
           " — releaseTemplate() first\n");
    return false;
  }

  if (on && (_config.quality == Quality::QUALITY_FAST ||
             _config.quality == Quality::QUALITY_BW)) {
    printf("[EPD_Painter] 16-grey needs QUALITY_NORMAL or QUALITY_HIGH\n");
    return false;
  }

  if (on && !packed4_screenbuffer) {
    const size_t p4_bytes    = packedSize() * 2;
    const size_t plane_bytes = packedSize();
    packed4_paintbuffer  = (uint8_t *)heap_caps_aligned_alloc(16, p4_bytes, MALLOC_CAP_SPIRAM);
    packed4_screenbuffer = (uint8_t *)heap_caps_aligned_alloc(16, p4_bytes, MALLOC_CAP_SPIRAM);
    dec_spill = (uint8_t *)heap_caps_aligned_alloc(
        16, plane_bytes * (DEC_MAX_SWEEPS16 - 2), MALLOC_CAP_SPIRAM);
    dec_sweeps16 = (LineSweep *)heap_caps_aligned_alloc(
        4, sizeof(LineSweep) * _config.height * DEC_MAX_SWEEPS16, MALLOC_CAP_SPIRAM);
    if (!(packed4_paintbuffer && packed4_screenbuffer && dec_spill && dec_sweeps16)) {
      printf("[EPD_Painter] 16-grey buffer allocation failed\n");
      if (packed4_paintbuffer)  heap_caps_free(packed4_paintbuffer);
      if (packed4_screenbuffer) heap_caps_free(packed4_screenbuffer);
      if (dec_spill)            heap_caps_free(dec_spill);
      if (dec_sweeps16)         heap_caps_free(dec_sweeps16);
      packed4_paintbuffer = packed4_screenbuffer = dec_spill = nullptr;
      dec_sweeps16 = nullptr;
      return false;
    }
  }
  // Rebuild the train library on EVERY 16-grey enable, not just the first:
  // setQuality() skips its rebuild while 4-level mode is live (correctly —
  // 16-grey is off), so a FAST/NORMAL detour taken in 4-level mode leaves
  // dec_trains16 holding the previous quality's tables. Re-entering 16-grey
  // then drove NORMAL with HIGH's 20-pass trains truncated to 13 — a
  // posterised, scrambled grey ladder (M5PaperS3, 29 Jul).
  if (on) _grey16_build_trains();

  // Carry the physical screen state across the switch so ghost tracking
  // survives: 2bpp levels 0..3 <-> 4bpp levels {0, 5, 10, 15}.
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  const size_t packed_bytes = packedSize();
  if (on) {
    static const uint8_t lv[4] = { 0, 5, 10, 15 };
    for (size_t i = 0; i < packed_bytes; i++) {
      const uint8_t b = packed_screenbuffer[i];
      packed4_screenbuffer[i * 2]     = (uint8_t)((lv[(b >> 6) & 3] << 4) | lv[(b >> 4) & 3]);
      packed4_screenbuffer[i * 2 + 1] = (uint8_t)((lv[(b >> 2) & 3] << 4) | lv[b & 3]);
    }
  } else {
    for (size_t i = 0; i < packed_bytes; i++) {
      const uint8_t b0 = packed4_screenbuffer[i * 2];
      const uint8_t b1 = packed4_screenbuffer[i * 2 + 1];
      // nearest 4-level code: (v + 2) / 5 maps 0..15 -> 0..3
      packed_screenbuffer[i] = (uint8_t)(
          ((((b0 >> 4) & 15) + 2) / 5) << 6 | (((b0 & 15) + 2) / 5) << 4 |
          ((((b1 >> 4) & 15) + 2) / 5) << 2 | (((b1 & 15) + 2) / 5));
    }
  }
  _grey16 = on;
  _sb_guard_update();          // guard follows the active-mode buffer
  xSemaphoreGive(_paint_buffer_sem);
  return true;
}

// =============================================================================
// setDirectTransitions() — 4-level direct grey-to-grey engine
// (see DECISION_ENGINE.md "direct grey-to-grey transitions")
// =============================================================================
void EPD_Painter::releaseUnusedBuffers() {
  // 4-level mode: the 16-grey planes and its sweep list are unreachable.
  // setGreyLevels(16) reallocates them, guarded on packed4_screenbuffer, so
  // dropping them here is a cost-on-next-enable rather than a one-way door.
  if (!_grey16 && packed4_screenbuffer) {
    if (packed4_paintbuffer)  heap_caps_free(packed4_paintbuffer);
    if (packed4_screenbuffer) heap_caps_free(packed4_screenbuffer);
    if (dec_spill)            heap_caps_free(dec_spill);
    if (dec_sweeps16)         heap_caps_free(dec_sweeps16);
    packed4_paintbuffer  = nullptr;
    packed4_screenbuffer = nullptr;
    dec_spill            = nullptr;
    dec_sweeps16         = nullptr;
  }
  // Direct engine off: same argument, and this is the 25.9 KB of internal
  // RAM the 16-grey sweep list was denying it.
  if (!_decision_direct && dec_spill_dir) {
    heap_caps_free(dec_spill_dir);
    if (dec_sweeps_dir) heap_caps_free(dec_sweeps_dir);
    dec_spill_dir  = nullptr;
    dec_sweeps_dir = nullptr;
  }
}

bool EPD_Painter::setDirectTransitions(bool on) {
  if (!on) { _decision_direct = false; return true; }
  if (!dec_spill_dir) {
    const size_t plane_bytes = packedSize();
    dec_spill_dir = (uint8_t *)heap_caps_aligned_alloc(
        16, plane_bytes * (DEC_MAX_SWEEPS_DIR - 2), MALLOC_CAP_SPIRAM);
    const size_t sweep_bytes =
        sizeof(LineSweep) * _config.height * DEC_MAX_SWEEPS_DIR;
    dec_sweeps_dir = (LineSweep *)heap_caps_aligned_alloc(
        4, sweep_bytes, MALLOC_CAP_INTERNAL);
    // Internal RAM is preferred — this list is walked per row, per pass — but
    // it is not worth refusing the whole engine over. A sketch can legitimately
    // have no internal heap left and still want direct transitions: the tuner
    // is the case in point, with ~125 KB of its own statics, the 130 KB
    // fastbuffer and a WiFi stack it cannot drop because it needs the network
    // to reach the scanner. It measured 18 KB free, fragmented into blocks no
    // larger than 10 KB, against the 26 KB contiguous wanted here — so the
    // grey-to-grey phase could not run at all on a board with megabytes of
    // PSRAM idle.
    //
    // PSRAM costs read latency on a sequential walk of 12-byte structs, which
    // the cache absorbs reasonably well. Slower directs beat no directs.
    if (!dec_sweeps_dir) {
      dec_sweeps_dir = (LineSweep *)heap_caps_aligned_alloc(
          4, sweep_bytes, MALLOC_CAP_SPIRAM);
      if (dec_sweeps_dir)
        printf("[EPD_Painter] direct sweeps in PSRAM (internal heap full: "
               "%u free, %u largest)\n",
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    }
    if (!(dec_spill_dir && dec_sweeps_dir)) {
      printf("[EPD_Painter] direct-transition allocation failed\n");
      if (dec_spill_dir)  heap_caps_free(dec_spill_dir);
      if (dec_sweeps_dir) heap_caps_free(dec_sweeps_dir);
      dec_spill_dir = nullptr;
      dec_sweeps_dir = nullptr;
      return false;
    }
    // First enable: load the preset's tuned tables for the current
    // quality. Later enables keep whatever is loaded, so the engine can
    // be A/B-toggled without losing manually uploaded trains.
    _load_preset_direct_trains();
  }
  _decision_direct = true;
  return true;
}

// =============================================================================
// Template layer — a protected high-quality static layer under animation
// (see the header). The whole feature is one overlay: protected pixels are
// forced back to their template values in the packed paintbuffer before
// discovery, so no engine path can ever see them change.
// =============================================================================

bool EPD_Painter::setTemplate(const uint8_t *fb, Quality quality) {
  if (!fb) return false;
  if (_grey16 && (quality == Quality::QUALITY_FAST || quality == Quality::QUALITY_BW)) {
    printf("[EPD_Painter] template: FAST/BW cannot express 16 greys\n");
    return false;
  }
  if (_has_template) releaseTemplate();

  // Paint the template at ITS quality, then restore the app's.
  const Quality qsave = _config.quality;
  _config.quality = quality;
  paint((uint8_t *)fb);
  while (!paintIdle()) EPD_DELAY_MS(2);
  _config.quality = qsave;

  // Capture the template by packing the CALLER'S fb — exactly as paint()
  // packs it, rotation included — never by copying the screenbuffer. The
  // screenbuffer is shared mutable state: another task's paintLater can
  // land between the template paint and a screenbuffer copy, baking its
  // dynamic content into the protected layer as un-erasable ghosts (seen
  // in the field: magitrac's tracker glyphs stamped into its GUI chrome).
  const size_t bytes = packedSize() * (_grey16 ? 2 : 1);
  tpl_data = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  tpl_mask = (uint8_t *)heap_caps_aligned_alloc(16, bytes, MALLOC_CAP_SPIRAM);
  if (!(tpl_data && tpl_mask)) {
    printf("[EPD_Painter] template allocation failed\n");
    if (tpl_data) heap_caps_free(tpl_data);
    if (tpl_mask) heap_caps_free(tpl_mask);
    tpl_data = tpl_mask = nullptr;
    return false;
  }
  _pack(fb, tpl_data, _grey16);
  for (size_t i = 0; i < bytes; i++) {
    const uint8_t b = tpl_data[i];
    uint8_t m = 0;
    if (_grey16) {
      if (b & 0xF0) m |= 0xF0;
      if (b & 0x0F) m |= 0x0F;
    } else {
      if (b & 0xC0) m |= 0xC0;
      if (b & 0x30) m |= 0x30;
      if (b & 0x0C) m |= 0x0C;
      if (b & 0x03) m |= 0x03;
    }
    tpl_mask[i] = m;
  }
  _tpl_grey16   = _grey16;
  _tpl_quality  = quality;
  _has_template = true;
  return true;
}

void EPD_Painter::releaseTemplate() {
  if (!_has_template) return;

  // Undo in the quality that painted: erase ONLY the template pixels,
  // with the template's trains. Everything else on screen is untouched.
  while (!paintIdle()) EPD_DELAY_MS(2);
  const Quality qsave = _config.quality;
  _config.quality = _tpl_quality;
  _has_template = false;              // stop the overlay for this paint

  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  const size_t bytes = packedSize() * (_tpl_grey16 ? 2 : 1);
  uint8_t *pb = _tpl_grey16 ? packed4_paintbuffer : packed_paintbuffer;
  for (size_t i = 0; i < bytes; i++) pb[i] &= (uint8_t)~tpl_mask[i];
  _pr_lo = 0; _pr_hi = _config.height - 1;
  paintStage = 1;                     // clear()'s submission pattern
  xSemaphoreGive(_paint_buffer_sem);
  while (!paintIdle()) EPD_DELAY_MS(2);

  _config.quality = qsave;
  heap_caps_free(tpl_data);
  heap_caps_free(tpl_mask);
  tpl_data = tpl_mask = nullptr;
}

// =============================================================================
// sendRow()
// =============================================================================
#ifdef EPD_ROW_PHASE_TIMING
// Per-paint accumulators: where does a row's time go? Reset at each
// drive's pass loop, printed after it.
int64_t g_rp_wait_us = 0, g_rp_pins_us = 0;
uint32_t g_rp_rows = 0;
#endif

void EPD_Painter::sendRow(bool firstLine, bool lastLine, bool noAdvance) {
#ifdef EPD_ROW_PHASE_TIMING
  const int64_t _rp_t0 = esp_timer_get_time();
#endif
  // Wait for LCD peripheral to finish consuming the previous row.
  // This also guarantees the previously-started DMA transfer is complete,
  // since the LCD FIFO cannot drain faster than DMA fills it.
  //long count=0;
  while (LCD_CAM.lcd_user.lcd_start) {}
#ifdef EPD_ROW_PHASE_TIMING
  g_rp_wait_us += esp_timer_get_time() - _rp_t0;
#endif
  //printf("yielded %d \n",count);
  //delayMicroseconds(4);

  // dma_buffer points at the buffer the CPU just finished writing.
  // Start DMA on its matching descriptor, then swap so the next
  // convert_packed_fb_to_ink() writes into the now-idle buffer.
  //delayMicroseconds(10);
  dma_descriptor_t *desc;
  if (dma_buffer == dma_buffer1) {
    desc = &dma_desc1;
    dma_buffer = dma_buffer2;
  } else {
    desc = &dma_desc2;
    dma_buffer = dma_buffer1;
  }

  // Reset the AFIFO while the bus is idle, then start GDMA *before* the
  // CKV/LE pin sequence below, so the DMA has time to fetch the descriptor
  // and prime the LCD AFIFO before lcd_start (esp-idf's i80 driver inserts
  // a delay here for the same reason).  The pin sequence takes ≥1us, so it
  // doubles as the prefill window at zero added row time.
  //
  // Note: the right-edge "image repeated ~16px apart" artifact once blamed
  // on this prefill race was actually the panel's source-driver shift chain
  // needing extra flush clocks — fixed by Config::row_pad_bytes (see
  // How_It_Works.md §7).  GDMA underflow flags confirmed the FIFO never
  // starved with this ordering.
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  LCD_CAM.lcd_cmd_val.lcd_cmd_value = static_cast<const uint8_t *>(desc->buffer)[-1];
  gdma_start(dma_chan, (intptr_t)desc);

#ifdef EPD_ROW_PHASE_TIMING
  const int64_t _rp_t2 = esp_timer_get_time();
#endif
  if (firstLine) {
    _vscanStart();
  } else if (noAdvance) {
    // Latch the previously transmitted data onto the source outputs without
    // advancing the gate: the current row drives this data for one more slot.
    _pin_le->set(true);
    if (_grey16) epd_hold_ns(_config.le_hold_ns);
    _pin_le->set(false);
  } else {
    // Gate OFF before the latch, as Inkplate's vscan_end()/hscan_start().
    // A gate line's pixels keep the LAST source voltage they saw while it
    // was on, so latching with CKV still high let the previous line's
    // pixels pick up this row's drive codes before the gate dropped: a
    // one-row vertical bleed (white seeping into the row above).
    _pin_ckv->set(false);
    if (_config.gate_off_us) EPD_DELAY_US(_config.gate_off_us);
    _pin_le->set(true);
    if (_grey16) epd_hold_ns(_config.le_hold_ns);
    _pin_le->set(false);
    if (_config.gate_on_us) EPD_DELAY_US(_config.gate_on_us);
    _pin_ckv->set(true);
  }
  // Optional extra charging time per row (see Config::row_extra_us). Held
  // after the gate advances and before the next row's transfer is started,
  // so it lengthens the window during which this row's pixels are being
  // charged rather than merely idling the bus.
  //
  // 16-GREY ONLY, and the distinction is frame rate rather than quality.
  // This costs height x row_extra_us per pass — 2.2 ms at 540 rows and 4 us.
  // A 16-grey pass is already padded to a constant period with ~8 ms spare,
  // so the cost is absorbed entirely: same 195 ms frame at NORMAL, more
  // charge delivered, nothing paid. The 4-level path has no such padding
  // (just a fixed inter-pass delay), so the same 2.2 ms lands straight on
  // the frame — about -27% at 4-level NORMAL and far worse at FAST. And
  // four coarse levels barely notice the positional variation this exists
  // to correct.
  if (_grey16) {
    const int extra = _config.rowExtraUs();
    if (extra > 0) EPD_DELAY_US(extra);
  }
#ifdef EPD_ROW_PHASE_TIMING
  g_rp_pins_us += esp_timer_get_time() - _rp_t2;
  g_rp_rows++;
#endif

  LCD_CAM.lcd_user.lcd_start = 1;
  if (lastLine) {
    while (LCD_CAM.lcd_user.lcd_start) {}

    _pin_ckv->set(false);
    _pin_le->set(true);
    _pin_le->set(false);
    _pin_ckv->set(true);
  }
}

// =============================================================================
// _vscanStart() / _skipRow()
// =============================================================================
// Vertical start: SPV low across one CKV rising edge puts the start token on
// the first gate line, then vscan_skip_lines more clocks step it past the
// panel's invisible lead-in lines — the Inkplate driver's vscan_start().
//
// While CKV is low (every gate output off) it also latches whatever is in
// the source chain. Every pass, sweep and flush ends by transmitting an
// unlatched float row (_endPass(), the closing sweep, _pushRow()), so this
// puts float on the source outputs before the token starts moving. Without
// it the outputs would still carry the previous pass's last row, which a
// skipped row (_skipRow) would then be driven with. Latching with the gates
// off leaves the previous pass's last line holding its full drive.
// The bus must be idle: callers wait on lcd_start first.
void EPD_Painter::_vscanStart() {
  const int hold = _config.vscan_hold_us;
  _pin_spv->set(false);
  EPD_DELAY_US(hold);
  _pin_ckv->set(false);
  _pin_le->set(true);
  _pin_le->set(false);
  EPD_DELAY_US(1);
  _pin_ckv->set(true);
  EPD_DELAY_US(hold);
  _pin_spv->set(true);
  for (int i = 0; i < _config.vscan_skip_lines; i++) {
    _pin_ckv->set(false);
    EPD_DELAY_US(1);
    _pin_ckv->set(true);
    EPD_DELAY_US(hold);
  }
}

// Step the gate past a row that has nothing to drive: CKV only — no data
// transfer and no latch, so the row sees the float data _vscanStart()
// latched. ~1 us against a full row transfer (31 us at 10 MHz). Only valid
// for rows BEFORE the first sendRow() of a pass; after that the latch holds
// real data.
void EPD_Painter::_skipRow(bool firstLine) {
  while (LCD_CAM.lcd_user.lcd_start) {}
  if (firstLine) {
    _vscanStart();
  } else {
    _gateStep();
  }
}

// One gate clock with no row behind it: CKV low 1 us, then high for
// gate_step_us so the next falling edge is never closer than that.
void EPD_Painter::_gateStep() {
  _pin_ckv->set(false);
  EPD_DELAY_US(1);
  _pin_ckv->set(true);
  if (_config.gate_step_us) EPD_DELAY_US(_config.gate_step_us);
}

// The gate driver is a shift register: a pass that stops at row lastRow
// leaves its token on that row's line, and the next SPV would start a second
// one — both would then walk down together, the stale one driving a copy of
// every row onto a line further down. A full pass never has this problem
// (its token leaves the bottom on the next pass's lead-in clocks). So after
// a pass or sweep that stops early, take the gates off, latch the float row
// that was just transmitted, and clock the token off the bottom of the panel.
// Lines it crosses are driven with float, which is what they already hold.
// ~2 us per remaining row, against a full transfer each.
void EPD_Painter::_flushGate(int lastRow) {
  while (LCD_CAM.lcd_user.lcd_start) {}
  _pin_ckv->set(false);
  _pin_le->set(true);
  _pin_le->set(false);
  EPD_DELAY_US(1);
  _pin_ckv->set(true);
  if (_config.gate_step_us) EPD_DELAY_US(_config.gate_step_us);
  for (int i = lastRow; i <= _config.height; i++) _gateStep();   // + 1 spare clock
}

// =============================================================================
// begin()
// =============================================================================
bool EPD_Painter::begin() {

  // Every SIMD path works in 16-pixel input groups and 64-pixel chunks, and
  // a row's chunk mask is one 32-bit word.
  packed_row_bytes = packedRowBytesFor(_config.width);
  if (_config.width % 16 != 0 || packed_row_bytes / 16 > 32) {
    printf("[EPD_Painter] width %u unsupported: must be a multiple of 16, at most 2048\n",
           (unsigned)_config.width);
    return false;
  }

  // -- Start I2C. The global Wire unless the sketch supplied a bus: a second
  // TwoWire on controller 0 would fight Wire for the same peripheral.
#ifdef ARDUINO
  if (_config.i2c.wire == nullptr) _config.i2c.wire = &Wire;
  _config.i2c.wire->begin(_config.i2c.sda, _config.i2c.scl, _config.i2c.freq);
  EPD_DELAY_MS(10);
#else
  // I2C not used in ESP-IDF builds
#endif

  // ---- Configure EPD control pins (all low: the panel is unpowered) ----
  // GMOD and OE belong to the power driver.
  const int16_t ctl_pins[] = { _config.pin_spv, _config.pin_ckv, _config.pin_sph,
                               _config.pin_le, _config.pin_cl };
  for (int16_t p : ctl_pins) { EPD_PIN_OUTPUT(p); EPD_PIN_LOW(p); }

  // Each row is sent with row_pad_bytes of trailing zeros: the panel's
  // source-driver shift chain has more stages than visible columns, so the
  // real pixel data needs extra CL clocks to reach the far (right-hand) end
  // before LE latches. Without the pad the last ~16 columns repeat content
  // from further left. Zero bytes are neutral drive — electrically harmless —
  // and cost ~50ns per row. See How_It_Works.md §7.
  const int dma_row_bytes = packed_row_bytes + _config.row_pad_bytes;

  // ---- Enable and reset LCD_CAM peripheral ----
  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  periph_module_reset(PERIPH_LCD_CAM_MODULE);
  LCD_CAM.lcd_user.lcd_reset = 1;
  EPD_DELAY_US(100);

  // ---- Configure LCD_CAM pixel clock (Config::pclk_hz) ----
  int clk_div = 2, clk_pre = 2;
  const uint32_t pclk = epd_pick_pclk(_config.pclk_hz, clk_div, clk_pre);
  LCD_CAM.lcd_clock.clk_en = 1;
  LCD_CAM.lcd_clock.lcd_clk_sel = 3;              // PLL_F160M
  LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
  LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
  LCD_CAM.lcd_clock.lcd_clk_equ_sysclk = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_num = clk_div;
  LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
  LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
  LCD_CAM.lcd_clock.lcd_clkcnt_n = clk_pre - 1;
  printf("[EPD_Painter] CL %lu Hz (asked %lu): %u x %u px, %d bus bytes/row, ~%lu us/pass\n",
         (unsigned long)pclk, (unsigned long)_config.pclk_hz,
         (unsigned)_config.width, (unsigned)_config.height, dma_row_bytes,
         (unsigned long)((uint64_t)dma_row_bytes * _config.height * 1000000ULL / pclk));

  // ---- Configure LCD_CAM for i8080 8-bit parallel mode ----
  LCD_CAM.lcd_ctrl.lcd_rgb_mode_en = 0;
  LCD_CAM.lcd_rgb_yuv.lcd_conv_bypass = 0;
  LCD_CAM.lcd_misc.lcd_next_frame_en = 0;
  LCD_CAM.lcd_data_dout_mode.val = 0;
  LCD_CAM.lcd_user.lcd_always_out_en = 0;
  LCD_CAM.lcd_user.lcd_8bits_order = 0;
  LCD_CAM.lcd_user.lcd_bit_order = 0;
  LCD_CAM.lcd_user.lcd_2byte_en = 0;
  LCD_CAM.lcd_user.lcd_dummy = 0;
  LCD_CAM.lcd_user.lcd_dummy_cyclelen = 0;
  // SPH is a per-row start pulse: low for the row's FIRST byte only, then
  // high. The source drivers are two cascaded 600-column chips; held low
  // for the whole row (the old scheme), chip 1 restarts when it fills and
  // swallows the right half of the row, and chip 2 never starts — the left
  // half of the panel shows the right half of the image.
  //
  // LCD_CAM produces the pulse in hardware: SPH is routed to the i80 DC
  // line, each row is a 1-cycle CMD phase carrying the row's first byte
  // (DC low) followed by a DOUT phase DMA'd from byte 1 on (DC high), and
  // DC idles high between rows. Exactly the Inkplate driver's
  // hscan_start(), at bus speed.
  LCD_CAM.lcd_misc.lcd_cd_idle_edge = 1;   // SPH idles high
  LCD_CAM.lcd_misc.lcd_cd_cmd_set   = 1;   // low during CMD (first byte)
  LCD_CAM.lcd_misc.lcd_cd_dummy_set = 0;
  LCD_CAM.lcd_misc.lcd_cd_data_set  = 0;   // high for the rest of the row
  LCD_CAM.lcd_user.lcd_cmd = 1;
  LCD_CAM.lcd_user.lcd_cmd_2_cycle_en = 0;
  LCD_CAM.lcd_user.lcd_dout_cyclelen = dma_row_bytes - 2;
  LCD_CAM.lcd_user.lcd_dout = 1;
  LCD_CAM.lcd_user.lcd_update = 1;

  // ---- Route 8-bit data bus pins using config ----
  for (int i = 0; i < 8; i++) {
    int8_t pin = _config.data_pins[i];
    // Output enable + matrix route + IO_MUX to GPIO function, the esp_lcd
    // i80 order. GPIO39-42 power up as the JTAG pads, so the IO_MUX select
    // is not optional on this wiring.
    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(pin, kDataSignals[i], false, false);
    epd_gpio_func_sel(pin);
    gpio_set_drive_capability((gpio_num_t)pin, (gpio_drive_cap_t)3);
  }

  // ---- Route pixel clock to CL pin ----
  gpio_set_direction((gpio_num_t)_config.pin_cl, GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(_config.pin_cl, LCD_PCLK_IDX, false, false);
  epd_gpio_func_sel(_config.pin_cl);
  gpio_set_drive_capability((gpio_num_t)_config.pin_cl, (gpio_drive_cap_t)3);

  // ---- SPH: LCD_CAM's DC line while powered (see lcd_cd_* above). The
  // panel is off here and DC idles high, so it stays a low GPIO until
  // powerOn() hands it over. ----
  _sphToLcd(false);
  gpio_set_drive_capability((gpio_num_t)_config.pin_sph, (gpio_drive_cap_t)3);

  // ---- Allocate GDMA channel ----
  gdma_channel_alloc_config_t dma_chan_config = {
    .sibling_chan = NULL,
    .direction = GDMA_CHANNEL_DIRECTION_TX,
    .flags = { .reserve_sibling = 0 },
  };
  gdma_new_channel(&dma_chan_config, &dma_chan);
  gdma_connect(dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

  // Cache the GDMA channel index for direct register access in sendRow().
  // LCD_CAM is peripheral 5 in the GDMA peri_sel register.
  _dma_channel_id = 0;
  for (int i = 0; i < 5; i++) {
    if (GDMA.channel[i].out.peri_sel.sel == 5) { _dma_channel_id = i; break; }
  }

  gdma_strategy_config_t strategy_config = {
    .owner_check = false,
    .auto_update_desc = false,
  };
  gdma_apply_strategy(dma_chan, &strategy_config);

  // ---- Allocate DMA row buffers ----
  dma_buffer1 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer2 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  dma_buffer0 = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, dma_row_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (dma_buffer0) memset(dma_buffer0, 0x00, dma_row_bytes);

  // Zero both buffers once so the pad region stays neutral forever —
  // pixel converts only ever write the first packed_row_bytes.
  memset(dma_buffer1, 0x00, dma_row_bytes);
  memset(dma_buffer2, 0x00, dma_row_bytes);

  dma_buffer = dma_buffer1;

  // ---- Set up DMA descriptors (one per buffer, stopped after each row) ----
  // Byte 0 of each row is sent by the CMD phase (lcd_cmd_value, with SPH
  // low), so the DMA starts at byte 1. GDMA TX from internal RAM has no
  // alignment requirement on the buffer address.
  dma_desc2.dw0.suc_eof = 1;
  dma_desc2.dw0.size = dma_row_bytes - 1;
  dma_desc2.dw0.length = dma_row_bytes - 1;
  dma_desc2.buffer = const_cast<uint8_t *>(dma_buffer2 + 1);
  dma_desc2.next = nullptr;

  dma_desc1.dw0.suc_eof = 1;
  dma_desc1.dw0.size = dma_row_bytes - 1;
  dma_desc1.dw0.length = dma_row_bytes - 1;
  dma_desc1.buffer = const_cast<uint8_t *>(dma_buffer1 + 1);
  dma_desc1.next = nullptr;

  dma_desc0.dw0.suc_eof = 1;
  dma_desc0.dw0.size = dma_row_bytes - 1;
  dma_desc0.dw0.length = dma_row_bytes - 1;
  dma_desc0.buffer = const_cast<uint8_t *>(dma_buffer0 + 1);
  dma_desc0.next = nullptr;

  // ---- Allocate packed 2bpp framebuffers ----
  const size_t packed_size = packedSize();

  // Prefer internal RAM for speed, fall back to PSRAM if the contiguous
  // internal-heap block isn't available (e.g. after WiFi has been brought
  // up first — leaves the heap fragmented).  PSRAM-backed fastbuffer is
  // slower for the per-pixel ops in epd_painter_ink_dual() but boots cleanly.
  //
  // At 1200x825 the plane is 245 KB — most of the S3's internal heap. Only
  // take it there if that still leaves room for the sweep lists below, the
  // paint task's stack and whatever the sketch brings up afterwards (WiFi
  // alone wants ~50 KB).
  static constexpr size_t kInternalHeadroom = 96 * 1024;
  if (heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) >= packed_size + kInternalHeadroom)
    packed_fastbuffer = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_INTERNAL));
  if (!packed_fastbuffer) {
    printf("[EPD_Painter] packed_fastbuffer in PSRAM (internal heap: %u largest block)\n",
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
    packed_fastbuffer = static_cast<uint8_t *>(
      heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
  }

  packed_screenbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));
  packed_paintbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  // Light plane lives in PSRAM: ink writes and the pass loop reads only the
  // chunks flagged in bitmask_light, so access is sparse (boundary chunks),
  // not a full per-pass sweep.
  packed_lightbuffer = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(16, packed_size, MALLOC_CAP_SPIRAM));

  bitmask = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));
  bitmask_light = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));

  // Decision engine bookkeeping (small, internal): per-line todo words and
  // sweep lists — see DECISION_ENGINE.md.
  dec_todo = static_cast<uint32_t *>(
    heap_caps_aligned_alloc(4, _config.height * 4, MALLOC_CAP_INTERNAL));
  dec_sweeps = static_cast<LineSweep *>(
    heap_caps_aligned_alloc(4, _config.height * DEC_MAX_SWEEPS * sizeof(LineSweep),
                            MALLOC_CAP_INTERNAL));
  dec_nsweeps = static_cast<uint8_t *>(
    heap_caps_aligned_alloc(4, _config.height, MALLOC_CAP_INTERNAL));
  memset(dec_train, 0, sizeof(dec_train));
  if (!(dec_todo && dec_sweeps && dec_nsweeps)) return false;

  if (!(dma_buffer1 && dma_buffer2 && dma_buffer0 && packed_fastbuffer && packed_screenbuffer &&
        packed_paintbuffer && packed_lightbuffer && bitmask && bitmask_light)) {
    printf("[EPD_Painter] buffer allocation failed\n");
    return false;
  }
  memset(packed_screenbuffer, 0, packed_size);
  memset(packed_paintbuffer,  0, packed_size);

  // ── TPS65186 + PCAL6416A power driver ──
  _powerDriver = new epd_painter_powerctl();
  if (!_powerDriver->begin(_config)) {
    printf("[EPD_Painter] power driver init failed — check I2C SDA=%d SCL=%d\n",
           _config.i2c.sda, _config.i2c.scl);
    return false;
  }

  _pin_spv = new EPD_GpioPin(uint8_t(_config.pin_spv));
  _pin_ckv = new EPD_GpioPin(uint8_t(_config.pin_ckv));
  _pin_le  = new EPD_GpioPin(uint8_t(_config.pin_le));
  _pin_sph = new EPD_GpioPin(uint8_t(_config.pin_sph));

  _paint_active_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_active_sem); 
  _paint_buffer_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(_paint_buffer_sem); 

  // Seed the clock rate from the hardware rather than trusting the declared
  // default — epd_hold_ns() sizes the LE pulse in CPU cycles, so a sketch
  // running at anything other than 240 MHz would get the wrong width.
  {
    const uint32_t now_mhz = (uint32_t)getCpuFrequencyMhz();
    if (now_mhz) s_epd_cpu_mhz = now_mhz;
  }

  xTaskCreatePinnedToCore(
    _paint_task_entry, "epd_paint", 8000, this, 10, &_paint_task_h, 0);

  // A board that has tuned itself carries its own measurements; they beat
  // the per-board train files, which beat the formula fallback. Board
  // detection has run by now, so the blobs' board keys check against the
  // real hardware.
  loadTunedFromNVS();

  if (_autoShutdown) {
    EPD_BootCtl boot(*this);
    if (boot.shutdownPending()) boot.shutdown();  // [[noreturn]] on shutdown path
  }

  return true;
}

// =============================================================================
// loadTunedFromNVS()
// =============================================================================
// One blob per quality under the "epd_painter" namespace, written by the
// tuneup example. install() validates magic, CRC and the board key, so a
// missing namespace, a corrupt entry or another board's blob all fall
// through to the compiled-in tables without complaint — an untuned board
// must pay nothing for this beyond one failed nvs_open().
int EPD_Painter::loadTunedFromNVS() {
#ifndef EPD_PAINTER_NO_NVS
  nvs_handle_t h;
  // READONLY open fails cleanly both when the namespace has never been
  // written and when NVS itself is not initialised (plain ESP-IDF apps
  // that never called nvs_flash_init) — either way there is nothing here.
  if (nvs_open("epd_painter", NVS_READONLY, &h) != ESP_OK) return 0;
  static const char *keys[] = { "fast", "normal", "high" };
  // Static: BLOB_SIZE is ~1.5 KB, and begin() runs on task stacks we do
  // not size.
  static uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  int n = 0;
  for (int i = 0; i < 3; i++) {
    size_t len = sizeof(buf);
    if (nvs_get_blob(h, keys[i], buf, &len) != ESP_OK) continue;
    if (EPD_PainterTuned::install(*this, buf, len)) {
      printf("[EPD_Painter] NVS tuned tables installed: %s\n", keys[i]);
      n++;
    } else {
      printf("[EPD_Painter] NVS blob '%s' rejected (wrong board or corrupt)\n",
             keys[i]);
    }
  }
  nvs_close(h);
  return n;
#else
  return 0;
#endif
}

// =============================================================================
// end()
// =============================================================================
bool EPD_Painter::end() {
  if (_paint_task_h) {
    vTaskDelete(_paint_task_h);
    _paint_task_h = nullptr;
  }

  vSemaphoreDelete(_paint_active_sem);
  vSemaphoreDelete(_paint_buffer_sem);

  if (dma_chan) {
    gdma_disconnect(dma_chan);
    gdma_del_channel(dma_chan);
    dma_chan = nullptr;
  }
  periph_module_disable(PERIPH_LCD_CAM_MODULE);
  return true;
}

// =============================================================================
// Power control
// =============================================================================
// Transmit one row of the current dma_buffer through the source shift
// chain with NO latch and NO gate clock — data passes through and is
// never driven. Used to flush the chain at power-on.
void EPD_Painter::_pushRow() {
  uint32_t spin = 0;
  while (LCD_CAM.lcd_user.lcd_start && ++spin < 200000) {}
  if (LCD_CAM.lcd_user.lcd_start) return;
  dma_descriptor_t *desc;
  if (dma_buffer == dma_buffer1) { desc = &dma_desc1; dma_buffer = dma_buffer2; }
  else                           { desc = &dma_desc2; dma_buffer = dma_buffer1; }
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  LCD_CAM.lcd_cmd_val.lcd_cmd_value = static_cast<const uint8_t *>(desc->buffer)[-1];
  gdma_start(dma_chan, (intptr_t)desc);
  LCD_CAM.lcd_user.lcd_start = 1;
  spin = 0;
  while (LCD_CAM.lcd_user.lcd_start && ++spin < 200000) {}
}

void EPD_Painter::debugPinBench() {
  if (!_pin_le || !_pin_ckv) { printf("[bench] pins not initialised\n"); return; }
  const int64_t t0 = esp_timer_get_time();
  for (int i = 0; i < 1000; i++) { _pin_le->set(true); _pin_le->set(false); }
  const int64_t t1 = esp_timer_get_time();
  for (int i = 0; i < 1000; i++) { _pin_ckv->set(true); _pin_ckv->set(false); }
  const int64_t t2 = esp_timer_get_time();
  printf("[bench] LE pulse %.3f us  |  CKV pulse %.3f us  (1000 each)\n",
         (t1 - t0) / 1000.0, (t2 - t1) / 1000.0);
}

void EPD_Painter::powerOn() {
#ifdef EPD_DEBUG_REGISTERS
  printf("[EPD] panel power ON\n");
#endif
  _pin_le->set(false);
  _pin_spv->set(false);
  _pin_sph->set(false);

  _powerDriver->powerOn();

  // Flush the source shift chain. After a rail cycle the chain can hold
  // stale drive data at a variable offset — the first frame's rows then
  // land shifted a few pixels left/right (seen as thin ghost lines on a
  // square's edge, and magitrac's misaligned note ghosts). Two rows of
  // zeros pushed through with no latch leave it clean, aligned, and
  // full of float codes — exactly the state v1.1.0's per-paint closing
  // scan used to guarantee.
  if (dma_buffer) {
    // Rails need real settle time before the chain clocks reliably; the
    // pin driver only guarantees 100us.
    EPD_DELAY_MS(5);
    memset(dma_buffer1, 0x00, packed_row_bytes);
    memset(dma_buffer2, 0x00, packed_row_bytes);
    // SPH back to LCD_CAM: every pushed row now carries its own start
    // pulse, so the flush also re-synchronises the source drivers.
    _sphToLcd(true);
    _pushRow();
    _pushRow();
  }

  _pin_spv->set(false);
  _pin_ckv->set(false);
  EPD_DELAY_US(1);
  _pin_ckv->set(true);
  _pin_spv->set(true);
}

void EPD_Painter::powerOff() {
#ifdef EPD_DEBUG_REGISTERS
  printf("[EPD] panel power OFF\n");
#endif
  // Every scan line low before V3P3 goes: a line left high back-powers the
  // unpowered source and gate drivers through their protection diodes. The
  // data bus and CL already idle low (row pads are zero, CL idles low).
  _pin_le->set(false);
  _pin_ckv->set(false);
  _sphToLcd(false);       // LCD_CAM's DC idles HIGH — take SPH back first
  _pin_sph->set(false);
  _pin_spv->set(false);
  _powerDriver->powerOff();
}

// SPH between LCD_CAM's DC output (panel powered: per-row start pulse) and
// a plain GPIO (panel off: must sit low, DC would idle it high).
void EPD_Painter::_sphToLcd(bool lcd) {
  const int pin = _config.pin_sph;
  if (lcd) {
    esp_rom_gpio_connect_out_signal(pin, LCD_DC_IDX, false, false);
  } else {
    esp_rom_gpio_connect_out_signal(pin, SIG_GPIO_OUT_IDX, false, false);
  }
}

int8_t EPD_Painter::readTemperature() {
  return _powerDriver ? _powerDriver->readTemperature() : 0;
}

int EPD_Painter::readVcomMv() {
  return _powerDriver ? _powerDriver->readVcomMv() : 0;
}

bool EPD_Painter::setVcomMv(int mv, bool persist) {
  return _powerDriver ? _powerDriver->setVcomMv(mv, persist) : false;
}

// =============================================================================
// _endPass()
// =============================================================================
// sendRow() latches the row transmitted BEFORE it, so the last row of a pass
// is still sitting in the source chain when the pass loop ends — and the
// next pass's first row overwrites it unlatched. One more (float) row send
// latches it and clocks the gate onto its line, where it holds through the
// inter-pass settle like every other row holds through the pass.
void EPD_Painter::_endPass() {
  memset(dma_buffer, 0x00, packed_row_bytes);
  sendRow(false);
}

// =============================================================================
// _dischargeColumns() — Config::discharge_rows
// =============================================================================
// Called just before sendRow(n). The source outputs carry row n-2's drive
// on its gate line, the chain holds row n-1 (the buffer dma_buffer is NOT
// pointing at), and dma_buffer holds row n. Gates off, shift and latch a
// float row so every column is pulled back to neutral with no line open,
// then shift row n-1 back in; that transfer is the discharge time.
// sendRow(n) then latches row n-1 and opens its line as usual.
void EPD_Painter::_txRow(dma_descriptor_t *desc) {
  while (LCD_CAM.lcd_user.lcd_start) {}
  LCD_CAM.lcd_misc.lcd_afifo_reset = 1;
  LCD_CAM.lcd_cmd_val.lcd_cmd_value = static_cast<const uint8_t *>(desc->buffer)[-1];
  gdma_start(dma_chan, (intptr_t)desc);
  EPD_DELAY_US(1);                       // let GDMA prime the FIFO
  LCD_CAM.lcd_user.lcd_start = 1;
  while (LCD_CAM.lcd_user.lcd_start) {}
}

void EPD_Painter::_dischargeColumns() {
  dma_descriptor_t *reload = (dma_buffer == dma_buffer1) ? &dma_desc2 : &dma_desc1;
  while (LCD_CAM.lcd_user.lcd_start) {}
  _pin_ckv->set(false);
  _txRow(&dma_desc0);
  _pin_le->set(true);
  _pin_le->set(false);
  _txRow(reload);
}

// =============================================================================
// _setBusRowBytes()
// =============================================================================
// Bytes clocked per row: byte 0 rides the CMD phase, the rest the DMA'd
// DOUT phase. begin() sets the full row; skip_idle_cols shortens it for a
// frame's passes and the pass loop restores it before the closing sweep.
void EPD_Painter::_setBusRowBytes(int n) {
  const int full = packed_row_bytes + _config.row_pad_bytes;
  if (n > full) n = full;
  if (n < 2) n = 2;
  while (LCD_CAM.lcd_user.lcd_start) {}
  dma_desc1.dw0.size = dma_desc1.dw0.length = n - 1;
  dma_desc2.dw0.size = dma_desc2.dw0.length = n - 1;
  dma_desc0.dw0.size = dma_desc0.dw0.length = n - 1;
  LCD_CAM.lcd_user.lcd_dout_cyclelen = n - 2;
  LCD_CAM.lcd_user.lcd_update = 1;
}

// =============================================================================
// _pack()
// =============================================================================
void EPD_Painter::_pack(const uint8_t *fb, uint8_t *dst, bool grey16, int r0, int r1) const {
  const int W = _config.width, H = _config.height;
  const int prb = packed_row_bytes;
  if (r1 < 0 || r1 >= H) r1 = H - 1;
  if (r0 < 0) r0 = 0;
  if (_config.rotation == Rotation::ROTATION_CW) { r0 = 0; r1 = H - 1; }

  if (!grey16) {
    const int used = W / 4;
    if (_config.rotation == Rotation::ROTATION_CW) {
      compact_pixels_rotated_cw(fb, dst, H, W, prb);
    } else if (_config.rotation == Rotation::ROTATION_180) {
      // Pack the source row forward with the SIMD packer, then reverse it a
      // byte at a time through a table that also reverses the four 2-bit
      // fields inside each byte. Replaces a per-pixel C walk that read the
      // PSRAM canvas backwards one byte at a time.
      static const struct Rev2 {
        uint8_t t[256];
        Rev2() {
          for (int x = 0; x < 256; x++)
            t[x] = (uint8_t)(((x & 3) << 6) | (((x >> 2) & 3) << 4) |
                             (((x >> 4) & 3) << 2) | ((x >> 6) & 3));
        }
      } rev2;
      alignas(16) uint8_t tmp[512];          // >= packed_row_bytes (begin() caps at 512)
      for (int r = r0; r <= r1; r++) {
        epd_painter_compact_pixels(fb + (size_t)(H - 1 - r) * W, tmp, W);
        uint8_t *d = dst + (size_t)r * prb;
        for (int b = 0; b < used; b++) d[b] = rev2.t[tmp[used - 1 - b]];
      }
    } else if (used == prb) {
      epd_painter_compact_pixels(fb + (size_t)r0 * W, dst + (size_t)r0 * prb,
                                 (uint32_t)W * (r1 - r0 + 1));
    } else {
      for (int r = r0; r <= r1; r++)
        epd_painter_compact_pixels(fb + (size_t)r * W, dst + (size_t)r * prb, W);
    }
    if (used != prb)
      for (int r = r0; r <= r1; r++) memset(dst + (size_t)r * prb + used, 0, prb - used);
    return;
  }

  // 16-grey: 8bpp level codes (0..15) -> 4bpp, first pixel in the high nibble.
  const int prb4 = prb * 2, used4 = W / 2;
  for (int r = r0; r <= r1; r++) {
    uint8_t *d = dst + (size_t)r * prb4;
    if (_config.rotation == Rotation::ROTATION_CW) {
      // panel (col c, row r) = canvas (x = H-1-r, y = c); canvas is H wide.
      const uint8_t *s = fb + (H - 1 - r);
      for (int c = 0; c < W; c += 2)
        d[c >> 1] = (uint8_t)(((s[(size_t)c * H] & 15) << 4) | (s[(size_t)(c + 1) * H] & 15));
    } else if (_config.rotation == Rotation::ROTATION_180) {
      const uint8_t *s = fb + (size_t)(H - r) * W - 1;
      for (int b = 0; b < used4; b++, s -= 2)
        d[b] = (uint8_t)(((s[0] & 15) << 4) | (s[-1] & 15));
    } else {
      compact_pixels16(fb + (size_t)r * W, d, W);
    }
    memset(d + used4, 0, prb4 - used4);
  }
}


// Waveform tables are defined per-device in EPD_Painter_presets.h
// and stored in _config.waveforms.


// =============================================================================
// paint()
// =============================================================================

void EPD_Painter::paint(uint8_t *framebuffer) {
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);

#ifdef EPD_ASM_TIMING
  const int64_t _cp_t0 = esp_timer_get_time();
#endif
  _pr_lo = 0; _pr_hi = _config.height - 1;
  _pack(framebuffer, _grey16 ? packed4_paintbuffer : packed_paintbuffer, _grey16);
#ifdef EPD_ASM_TIMING
  printf("[paint] compact_pixels: %lld us\n", esp_timer_get_time() - _cp_t0);
#endif

  paintStage=2;
  xSemaphoreGive(_paint_buffer_sem); 

  // wait until this buffer has been picked up by the paint loop.
  while(paintStage==2){
      vTaskDelay(1);
  }
}

// =============================================================================
// paintRows() — paint() limited to the canvas rows that changed
// =============================================================================
void EPD_Painter::paintRows(uint8_t *framebuffer, int y0, int y1) {
  const int H = _config.height;
  if (y0 > y1) { const int t = y0; y0 = y1; y1 = t; }
  if (y0 < 0) y0 = 0;
  if (y1 > H - 1) y1 = H - 1;
  // Canvas rows -> physical rows. CW turns canvas rows into panel columns,
  // so it (and the engines without a row-limited discovery) takes it all.
  int lo = y0, hi = y1;
  if (_config.rotation == Rotation::ROTATION_180) { lo = H - 1 - y1; hi = H - 1 - y0; }
  if (_config.rotation == Rotation::ROTATION_CW || _grey16 ||
      _decision_engine || _decision_direct || _has_template) { lo = 0; hi = H - 1; }

  // Wait for the PREVIOUS submission to be picked up, not this one: the
  // frame is packed out of the canvas before returning, so the caller can
  // draw the next one while the paint task compares and drives this one.
  // paint() waits after instead, which puts the compare on the caller's
  // critical path.
  while (paintStage == 2) {
    vTaskDelay(1);
  }
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  _pr_lo = lo; _pr_hi = hi;
  _pack(framebuffer, _grey16 ? packed4_paintbuffer : packed_paintbuffer, _grey16, lo, hi);
  paintStage = 2;
  xSemaphoreGive(_paint_buffer_sem);
}

// =============================================================================
// paintPacked() — like paint() but skips compaction; buffer is already 2bpp.
// line_repeat > 1: `packed` holds height/line_repeat rows, each driven
// line_repeat times (vertical pixel doubling for reduced-bandwidth streams).
// =============================================================================
void EPD_Painter::paintPacked(const uint8_t* packed, int line_repeat) {
  if (_grey16) {
    printf("[EPD_Painter] paintPacked() is a 4-level path — call setGreyLevels(4) first\n");
    return;
  }
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  _pr_lo = 0; _pr_hi = _config.height - 1;
  if (line_repeat <= 1) {
    memcpy(packed_paintbuffer, packed, packedSize());
  } else {
    uint8_t* dst = packed_paintbuffer;
    const int src_rows = _config.height / line_repeat;
    for (int r = 0; r < src_rows; ++r) {
      for (int k = 0; k < line_repeat; ++k) {
        memcpy(dst, packed + r * packed_row_bytes, packed_row_bytes);
        dst += packed_row_bytes;
      }
    }
  }
  paintStage = 2;
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == 2) {
    vTaskDelay(1);
  }
}

// =============================================================================
// unpaintPacked() — DC-balance pass: tells the driver the screen currently
// shows 'packed', then drives a blank (all-zero) frame so every pixel that
// was darkened gets a matching lightening pulse.
// =============================================================================
void EPD_Painter::unpaintPacked(const uint8_t* packed) {
  if (_grey16) {
    printf("[EPD_Painter] unpaintPacked() is a 4-level path — call setGreyLevels(4) first\n");
    return;
  }
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  _pr_lo = 0; _pr_hi = _config.height - 1;
  memcpy(packed_screenbuffer, packed, packed_row_bytes * _config.height);
  memset(packed_paintbuffer,  0x00,  packed_row_bytes * _config.height);
  paintStage = 2;
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == 2) {
    vTaskDelay(1);
  }
}

// =============================================================================
// paintLater
// =============================================================================
void EPD_Painter::paintLater(uint8_t *framebuffer) {
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
#ifdef EPD_ASM_TIMING
    const int64_t _cpl_t0 = esp_timer_get_time();
#endif
    _pr_lo = 0; _pr_hi = _config.height - 1;
    _pack(framebuffer, _grey16 ? packed4_paintbuffer : packed_paintbuffer, _grey16);
#ifdef EPD_ASM_TIMING
    printf("[paintLater] compact_pixels: %lld us\n", esp_timer_get_time() - _cpl_t0);
#endif

    
    paintStage=2;
    xSemaphoreGive(_paint_buffer_sem); 
}

// =============================================================================
// _paint_task_entry() / _paint_task_body()
// =============================================================================
void EPD_Painter::_paint_task_entry(void *arg) {
  static_cast<EPD_Painter *>(arg)->_paint_task_body();
}

void EPD_Painter::_paint_task_body() {
  int64_t prof_last_t0 = 0;
  for (;;) {
    const int64_t prof_wait_t0 = esp_timer_get_time();
    if (paintStage==0){
      xSemaphoreGive(_paint_active_sem);
      while(paintStage==0){
         vTaskDelay(1);
      }
      xSemaphoreTake(_paint_active_sem, portMAX_DELAY);
    }

    // Dual-plane delta pass: reads the new frame straight out of the (PSRAM)
    // paintbuffer and emits both drive directions — dark plane into the
    // internal fastbuffer, light plane into PSRAM — with a chunk mask each.
    // There is no direction priority and no deferral: chunks flagged in both
    // masks are resolved in the pass loop by driving their row twice.
    // The buffer semaphore is held for the whole sweep because the
    // paintbuffer is being read directly; paint()/paintPacked() block on it
    // rather than on paintStage if they arrive mid-sweep.
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
    _sb_guard_check();
    // Template layer: force protected pixels back to their template
    // values before discovery — every engine path (SIMD, C, direct,
    // 16-grey) then simply sees them as unchanged.
    // Tripwire: an app that never calls setTemplate() must never see
    // this fire — if it does, something scribbled on the driver object
    // (heap overflow elsewhere) and flipped the flag with garbage
    // pointers. Announce loudly instead of stamping junk on the glass.
    if (_has_template && (!tpl_data || !tpl_mask)) {
      _tpl_trip_hits++;
      printf("[EPD_Painter] CORRUPTION: template flag set with null "
             "planes (tpl_data=%p tpl_mask=%p) — disabling overlay\n",
             tpl_data, tpl_mask);
      _has_template = false;
    }
    if (_has_template && _tpl_grey16 == _grey16) {
      static bool announced = false;
      if (!announced) {
        announced = true;
        _tpl_trip_hits++;
        printf("[EPD_Painter] template overlay ACTIVE (data=%p mask=%p)\n",
               tpl_data, tpl_mask);
      }
      const size_t words = packedSize() * (_grey16 ? 2 : 1) / 4;
      uint32_t *p = (uint32_t *)(_grey16 ? packed4_paintbuffer
                                         : packed_paintbuffer);
      const uint32_t *t = (const uint32_t *)tpl_data;
      const uint32_t *m = (const uint32_t *)tpl_mask;
      for (size_t i = 0; i < words; i++) p[i] = (p[i] & ~m[i]) | t[i];
    }

#ifdef EPD_ASM_TIMING
    const int64_t _ink_t0 = esp_timer_get_time();
#endif
    int64_t prof_idle_us = 0, prof_disc_us = 0;
    if (_profile) {
      _prof_rows = 0; _prof_rowloop_us = 0; _prof_cycle_t0 = esp_timer_get_time();
      _prof_merged = 0;
      prof_idle_us = _prof_cycle_t0 - prof_wait_t0;
    }
    // Black and white mode: snap the new frame to levels 0 and 3 by copying
    // each pixel's high bit over its low bit (0,1 -> 0; 2,3 -> 3). Done here
    // so paint(), paintLater() and paintPacked() all get it, and so the
    // screen state only ever records what the glass actually shows.
    // Rows this submission may have changed (paintRows(); else all). The
    // rest are known identical to the screen and skipped by the BW snap
    // and the 4-level discovery below.
    const int pr_lo = _pr_lo < 0 ? 0 : _pr_lo;
    const int pr_hi = _pr_hi >= _config.height ? _config.height - 1 : _pr_hi;
    if (!_grey16 && _config.quality == Quality::QUALITY_BW) {
      uint32_t *p = (uint32_t *)(packed_paintbuffer + (size_t)pr_lo * packed_row_bytes);
      const size_t words = (size_t)(pr_hi - pr_lo + 1) * packed_row_bytes / 4;
      for (size_t i = 0; i < words; i++) {
        const uint32_t hi = p[i] & 0xAAAAAAAAu;
        p[i] = hi | (hi >> 1);
      }
    }

    uint32_t any_work = 0;
    const LineSweep *sweep_list = dec_sweeps;
    int sweep_stride = DEC_MAX_SWEEPS;
    if (_grey16) {
      // 16-grey discovery batches decisions into sweep lists directly —
      // up to 10 sweeps per line, greedy first-appearance order.
      any_work = _decision_discover16();
      sweep_list = dec_sweeps16;
      sweep_stride = DEC_MAX_SWEEPS16;
    } else if (_decision_direct) {
      // Direct-transition engine: greedy-sweep discovery over the 2bpp
      // buffers, grey-to-grey pixels driven in one paint where a pair
      // train is loaded (see DECISION_ENGINE.md).
      any_work = _decision_discover_direct();
      sweep_list = dec_sweeps_dir;
      sweep_stride = DEC_MAX_SWEEPS_DIR;
    } else if (_decision_engine) {
      // Decision engine: C discovery — planes, masks, per-line todo words.
      any_work = _decision_discover();
      _decision_batch_compat();
    } else {
      for (int row = 0; row < pr_lo; row++) bitmask[row] = bitmask_light[row] = 0;
      for (int row = pr_hi + 1; row < _config.height; row++) bitmask[row] = bitmask_light[row] = 0;
      for (int row = pr_lo; row <= pr_hi; row++) {
        const uint8_t *pb_row = packed_paintbuffer + row * packed_row_bytes;
        uint8_t *fbD_row = packed_fastbuffer  + row * packed_row_bytes;
        uint8_t *fbL_row = packed_lightbuffer + row * packed_row_bytes;
        uint8_t *sb_row  = packed_screenbuffer + row * packed_row_bytes;

        bitmask[row] = epd_painter_ink_dual(pb_row, fbD_row, fbL_row, sb_row,
                                            packed_row_bytes, &bitmask_light[row]);
        any_work |= bitmask[row] | bitmask_light[row];
      }
      // Both 4-level engines feed the same per-line sweep lists (the fixed
      // compatibility mapping — the July dual-plane layout, exactly).
      _decision_batch_compat();
    }
#ifdef EPD_ASM_TIMING
    printf("[paint_task] ink discovery (all rows): %lld us\n", esp_timer_get_time() - _ink_t0);
#endif
    xSemaphoreGive(_paint_buffer_sem);
    if (_profile) prof_disc_us = esp_timer_get_time() - _prof_cycle_t0;

    paintStage-=1;

    if (!any_work) {
      // Nothing changed anywhere — skip the drive passes entirely. Under
      // continuous streaming this makes the mop-up second cycle nearly free;
      // for stills it makes repainting an unchanged frame a no-op.
      vTaskDelay(1);
      continue;
    }

    PanelPowerGuard guard(*this);

    const uint8_t *lt_wf;
    const uint8_t *dk_wf;
    int wf_len;

    const Waveforms &wf = _config.waveforms;
    // QUALITY_BW trains: a plain darken (1) or whiten (2) run per level.
    // Levels 1 and 2 only reach the glass from a frame painted in another
    // quality (or unpaintPacked()); full runs settle those to black/white.
    uint8_t bw_dk[3 * 32], bw_lt[3 * 32];

    if (_config.quality == Quality::QUALITY_BW) {
      wf_len = _config.bw_passes < 1 ? 1 : _config.bw_passes > 32 ? 32 : _config.bw_passes;
      memset(bw_dk, 1, 3 * wf_len);
      memset(bw_lt, 2, 3 * wf_len);
      lt_wf = bw_lt;
      dk_wf = bw_dk;
    } else if (_config.quality == Quality::QUALITY_FAST) {
      lt_wf = &wf.fast_lighter[0][0];
      dk_wf = &wf.fast_darker[0][0];
      wf_len = 7;
    } else if(_config.quality == Quality::QUALITY_NORMAL) {
      lt_wf = &wf.normal_lighter[0][0];
      dk_wf = &wf.normal_darker[0][0];
      wf_len = 13;
    } else {
      lt_wf = &wf.high_lighter[0][0];
      dk_wf = &wf.high_darker[0][0];
      wf_len = 13;
    }

#ifdef EPD_ASM_TIMING
    int64_t _conv_total = 0;
    int _dbl_rows = 0;
#endif
    // Decision -> train bindings for this paint. 16-grey mode binds the
    // formula train library (30 trains, ids 2..31; placeholder until phase
    // D calibrates them). 4-level mode binds the calibrated tables —
    // apply(g) is the darker table's row g-1, remove(g) the lighter table's
    // (id = (level << 1) | dir) — and clears the higher ids so a stale
    // 16-grey binding can never leak into a 4-level paint.
    if (_grey16) {
      for (int id = 2; id < DEC_IDS; id++) dec_train[id] = dec_trains16[id];
    } else {
      for (int g = 1; g <= 3; g++) {
        dec_train[(g << 1) | 0] = dk_wf + (g - 1) * wf_len;
        dec_train[(g << 1) | 1] = lt_wf + (g - 1) * wf_len;
      }
      for (int id = 8; id < DEC_IDS; id++) dec_train[id] = nullptr;
      if (_decision_direct) {
        // Direct grey-to-grey trains: loaded pairs only — discovery never
        // emits an unloaded pair's id, but keep binding and gate in step.
        for (int f = 1; f <= 3; f++)
          for (int t = 1; t <= 3; t++)
            if (f != t && (_dir_loaded & (1u << ((f << 2) | t))))
              dec_train[directId(f, t)] = dec_trains_dir[(f << 2) | t];
      }
    }

    // Pass count = length of the longest train in play (DECISION_ENGINE.md).
    // Passes beyond every active train's last non-float code drive nothing;
    // skipping them costs no dose fidelity because each executed pass keeps
    // its exact constant period, and every pixel's final drive still gets a
    // full-period window before the next latch. dec_todo holds the frame's
    // decision ids from discovery. 4-level mode keeps its calibrated fixed
    // pass count.
    // Per-id train lengths: base trains are exactly wf_len codes (the
    // FAST tables are 7-long rows — reading past them would overrun into
    // the next level's row), direct trains up to DEC_WF_LEN_DIR. The
    // bcast loop floats any id past its own length.
    uint8_t dec_len[DEC_IDS];
    for (int id = 0; id < DEC_IDS; id++) dec_len[id] = (uint8_t)wf_len;

    if (_grey16) {
      // 16-grey trains are the quality's own length, not the 4-level pass
      // count wf_len was seeded from: at HIGH they are 20 long, and a
      // stale 13 would float the last seven passes of every train.
      const int g16len = g16TrainLen();
      for (int id = 0; id < DEC_IDS; id++) dec_len[id] = (uint8_t)g16len;
      uint32_t inplay = 0;
      for (int row = 0; row < _config.height; row++) inplay |= dec_todo[row];

      // Temporal partition (DECISION_ENGINE.md "direct grey-to-grey"):
      // a grey-to-grey pixel took BOTH a remove and an apply this frame.
      // Its remove fully erases first, then its apply drives from erased
      // glass — so each apply id shifts right by the longest remove
      // among ITS OWN from-partners this frame (a deep erase elsewhere
      // never stalls a shallow transition: 15->3 costs 13+3 passes
      // without dragging 3->15 to 26). Disjoint pass ranges keep the
      // pixel's two sweeps OR-safe; DC composes -Q(from) + Q(to) from
      // the tuned tables. Frames without grey-to-grey shift nothing.
      for (int to = 1; to <= 15; to++) {
        const uint16_t fromset = dec_gg_from[to];
        if (!fromset) continue;
        const int aid = to << 1;
        if (!(inplay & (1u << aid)) || !dec_train[aid]) continue;
        int R = 0;
        for (int f = 1; f <= 15; f++) {
          if (!(fromset & (1u << f))) continue;
          const uint8_t *rt = dec_train[(f << 1) | 1];
          if (!rt) continue;
          for (int p = R; p < g16len; p++)
            if (rt[p]) R = p + 1;
        }
        if (R > 0) {
          uint8_t *sh = dec_shifted[aid];
          memset(sh, 0, R);
          memcpy(sh + R, dec_train[aid], g16len);
          dec_train[aid] = sh;
          dec_len[aid] = (uint8_t)(R + g16len);
        }
      }

      int need = 1;
      for (int id = 2; id < DEC_IDS; id++) {
        if (!(inplay & (1u << id)) || !dec_train[id]) continue;
        const int len = dec_len[id];
        for (int p = 0; p < len; p++)
          if (dec_train[id][p] && p >= need) need = p + 1;
      }
      wf_len = need;
    } else if (_decision_direct) {
      // Direct trains may be LONGER than the quality's pass count (deep
      // lightening transitions need erase + re-darken room). Extend the
      // frame to the longest direct train actually in play — content
      // without deep transitions pays nothing (DECISION_ENGINE.md).
      uint32_t inplay = 0;
      for (int row = 0; row < _config.height; row++) inplay |= dec_todo[row];
      for (int id = 16; id < DEC_IDS; id++) {
        if (!(inplay & (1u << id)) || !dec_train[id]) continue;
        dec_len[id] = DEC_WF_LEN_DIR;
        for (int p = wf_len; p < DEC_WF_LEN_DIR; p++)
          if (dec_train[id][p]) wf_len = p + 1;
      }
    }

    // The pass loop consumes per-line sweep lists: a sweep = one staged
    // 2bpp slot plane + a chunk mask + 3 decision ids naming the waveform
    // train each slot indexes. The engine ORs a line's sweeps into the row
    // accumulator before the single latch; the k-way OR is safe because a
    // pixel belongs to exactly one sweep, so the others write float for it
    // (see DECISION_ENGINE.md). Phase B lists are the fixed dual-plane
    // pair: apply-decisions in one sweep, remove-decisions in the other,
    // pixel-disjoint by construction.
#ifdef EPD_GREY16_PASS_TIMING
    int64_t _g16_rowloop_min = 0, _g16_rowloop_max = 0;
    int64_t _g16_period_min = 0, _g16_period_max = 0;
#endif
#ifdef EPD_ROW_PHASE_TIMING
    { extern int64_t g_rp_wait_us, g_rp_pins_us; extern uint32_t g_rp_rows;
      g_rp_wait_us = g_rp_pins_us = 0; g_rp_rows = 0; }
#endif
    // Row window: only rows r0..r1 carry work this frame. Rows above r0 are
    // stepped past with bare gate clocks, and the pass ends at r1 — the next
    // pass restarts the gate at the top anyway. Rows outside the window are
    // never latched, so they keep the float the last closing sweep left.
    int r0 = 0, r1 = _config.height - 1;
    if (_config.skip_idle_rows) {
      while (r0 < r1 && !dec_nsweeps[r0]) r0++;
      while (r1 > r0 && !dec_nsweeps[r1]) r1--;
    }

    // Column cut-off: the last chunk with work in any row of the window
    // (mask bit 31 = chunk 0). One length for the whole frame.
    const int full_row_bytes = packed_row_bytes + _config.row_pad_bytes;
    int row_bytes = full_row_bytes;
    if (_config.skip_idle_cols) {
      uint32_t cols = 0;
      for (int row = r0; row <= r1; row++) {
        const LineSweep *ls = &sweep_list[row * sweep_stride];
        for (int s = 0; s < dec_nsweeps[row]; s++) cols |= ls[s].mask;
      }
      if (cols) row_bytes = (32 - __builtin_ctz(cols)) * 16 + _config.row_pad_bytes;
      if (row_bytes < full_row_bytes) _setBusRowBytes(row_bytes);
    }

    // bw_hold_black: every visible chunk (mask bit 31 = chunk 0).
    const bool bw_hold = _config.bw_hold_black && !_grey16 &&
                         _config.quality == Quality::QUALITY_BW;
    const int hold_chunks = packed_row_bytes / 16;
    const uint32_t hold_mask = hold_chunks >= 32 ? 0xFFFFFFFFu : ~(0xFFFFFFFFu >> hold_chunks);

    for (uint8_t pass = 0; pass < wf_len; pass++) {
      const int64_t pass_t0 = esp_timer_get_time();
      // Per-decision broadcast bytes for this pass: code * 0x55 replicates
      // a 2-bit drive code across the 4 pixels of a byte. Ids 0/1 (level
      // 0 = white) have no trains and always float.
      //
      // CODE 3 ("drive both electrodes") IS EMITTED AS FLOAT. 11 turns on
      // both source transistors for the pixel at once: the two halves
      // fight, so the net field — and the net charge in the DC ledger —
      // is nominally nothing, but the contention current is real and is
      // drawn by every pixel carrying the code, on every pass that has
      // it. That current is the supply burden the panel is least able to
      // afford exactly when a photograph is on the glass and thousands of
      // pixels are switching, which is where the levels were measured
      // drifting up to 12 lum (29 Jul context probe: flat black moved the
      // staircase not at all, mixed dithered content moved it a lot). So
      // the pointless pulse is dropped rather than paid for.
      //
      // This is the single point where a waveform code becomes bus bits
      // for BOTH engines — the 4-level pairs are loaded into dec_train[]
      // alongside the 16-grey trains — so it also covers tuned NVS blobs
      // and any legacy table, without rewriting their data. Pair
      // evolution never mints a 3 (its charge sits outside the ledger the
      // DC-neutral operators are built on); every 3 still in the tree is
      // inherited from the hand-built and dithertune-era 4-level pairs.
      uint8_t bcast[DEC_IDS];
      bcast[0] = bcast[1] = 0;
      for (int id = 2; id < DEC_IDS; id++) {
        uint8_t code = (dec_train[id] && pass < dec_len[id])
                           ? dec_train[id][pass] : 0;
        if (code == 3) code = 0;
        bcast[id] = (uint8_t)(code * 0x55);
      }

      for (int row = 0; row < r0; row++) _skipRow(row == 0);
      // discharge_rows: does the data of row-1 / row-2 drive anything?
      bool drv1 = false, drv2 = false;
      for (int row = r0; row <= r1; row++) {
        if (_profile && pass == 0 && (bitmask[row] & bitmask_light[row])) _prof_merged++;
#ifdef EPD_ASM_TIMING
        const int64_t _conv_t0 = esp_timer_get_time();
        if (pass == 0 && (bitmask[row] & bitmask_light[row])) _dbl_rows++;
#endif
        // Chunks with no work in any sweep must be explicitly floated
        // (zeroed) — convert skips them and would otherwise leave the
        // previous row's drive data. The first sweep overwrites its active
        // chunks, later sweeps OR on top: all of a line's ink rides in ONE
        // transmission with full retention dose. (Driving the row once per
        // sweep instead does NOT work — a second latch cuts the first
        // data's retention from a full pass period to one ~30us row slot,
        // starving its dose to nothing. Verified optically with a
        // three-band stripe probe.)
        memset(dma_buffer, 0x00, packed_row_bytes);
        const LineSweep *ls = &sweep_list[row * sweep_stride];
        const int n = dec_nsweeps[row];
        for (int s = 0; s < n; s++) {
          // Table entry order is slot 3, 2, 1 (slot 0 = float).
          uint8_t tbl[3] = { bcast[ls[s].dec[2]], bcast[ls[s].dec[1]], bcast[ls[s].dec[0]] };
          if (s == 0)
            epd_painter_convert_packed_fb_to_ink(ls[s].plane_row, dma_buffer, packed_row_bytes, tbl, ls[s].mask);
          else
            epd_painter_convert_packed_fb_to_ink_or(ls[s].plane_row, dma_buffer, packed_row_bytes, tbl, ls[s].mask);
        }
        // bw_hold_black: unchanged blacks (screen 3; pixels whitening this
        // frame are already recorded as 0) get darken too. OR onto a
        // darkening pixel leaves it 01.
        if (bw_hold) {
          static const uint8_t hold_tbl[3] = { 0x55, 0x00, 0x00 };   // slot 3, 2, 1
          epd_painter_convert_packed_fb_to_ink_or(packed_screenbuffer + (size_t)row * packed_row_bytes,
                                                  dma_buffer, packed_row_bytes, hold_tbl, hold_mask);
        }
#ifdef EPD_ASM_TIMING
        _conv_total += esp_timer_get_time() - _conv_t0;
#endif
        bool drv0 = false;
        if (_config.discharge_rows) {
          const uint32_t *w = (const uint32_t *)dma_buffer;
          uint32_t acc = 0;
          if (_config.discharge_rows == 1) {
            for (int i = 0; i < packed_row_bytes / 4; i++)
              acc |= w[i] & 0xAAAAAAAAu & ~(w[i] << 1);    // code 10 = whiten
          } else {
            for (int i = 0; i < packed_row_bytes / 4; i++) acc |= w[i];
          }
          drv0 = acc != 0;
          if (drv2 && row != 0) _dischargeColumns();
        }
        sendRow(row == 0, false, false);
        drv2 = drv1; drv1 = drv0;
        if (_profile) _prof_rows++;
      }
      if (drv2) _dischargeColumns();
      _endPass();
      if (r1 < _config.height - 1) _flushGate(r1);
      if (_profile) _prof_rowloop_us += esp_timer_get_time() - pass_t0;

     if (_grey16) {
        // Constant pass period: dose = time between a row's latches = the
        // pass duration, and the row loop's length varies with content
        // (sweep count). Pad to a fixed period so dose depends only on
        // the trains — but never squeeze the inter-pass settle below the
        // quality's floor. Trains are calibrated at this period.
        const int64_t elapsed = esp_timer_get_time() - pass_t0;
#ifdef EPD_GREY16_PASS_TIMING
        if (pass == 0 || elapsed < _g16_rowloop_min) _g16_rowloop_min = elapsed;
        if (elapsed > _g16_rowloop_max) _g16_rowloop_max = elapsed;
#endif
        const int64_t period = (_config.quality == Quality::QUALITY_HIGH)
                                   ? _config.g16_pass_us_high
                                   : _config.g16_pass_us_normal;
        const int64_t settle_floor = _config.g16_settle_us;

        // Pad to an ABSOLUTE deadline, not a relative delay.
        //
        // The old form asked vTaskDelay for a whole number of milliseconds,
        // which stacked three errors on a quantity the trains are calibrated
        // against: the pad was rounded to the nearest ms (±500 us), the tick
        // granularity added up to another 1 ms, and the task then had to be
        // *scheduled* behind WiFi and everything else. On a 15 ms period that
        // is several percent of jitter, and a 15% dose error is enough to
        // band a ramp visibly - so "constant pass period" was only constant
        // to within the range that actually matters.
        //
        // Coarse then fine: sleep (yielding the CPU, feeding the watchdog)
        // while there is more than ~1.5 ms to go, then spin on the
        // microsecond counter for the remainder. The spin is bounded and the
        // paint task is dedicated, so the cost is nothing that matters, and
        // the deadline is hit to within interrupt latency rather than a tick.
        int64_t target = pass_t0 + period;
        const int64_t floor_target = esp_timer_get_time() + settle_floor;
        if (target < floor_target) target = floor_target;   // overrun: keep settle
        epd_wait_until_us(target);
#ifdef EPD_GREY16_PASS_TIMING
        {
          const int64_t actual = esp_timer_get_time() - pass_t0;
          if (pass == 0 || actual < _g16_period_min) _g16_period_min = actual;
          if (actual > _g16_period_max) _g16_period_max = actual;
        }
#endif
      } else {
        // A pass over a small row window is short, and pass length IS the
        // dose each drive code delivers. Pad to min_pass_us so a few changed
        // lines are not driven by a fraction of what the tables assume.
        if (_config.min_pass_us > 0)
          epd_wait_until_us(pass_t0 + _config.min_pass_us);

        // 4-level keeps its "fixed wait after the row loop" semantics - it
        // has no constant-period model and its tables are calibrated to
        // that - but the wait itself is now honest to the microsecond
        // rather than to a tick. QUALITY_FAST and QUALITY_BW = no wait.
        if (_config.quality == Quality::QUALITY_HIGH)
          epd_wait_until_us(esp_timer_get_time() + _config.pass_gap_us_high);
        else if (_config.quality == Quality::QUALITY_NORMAL)
          epd_wait_until_us(esp_timer_get_time() + _config.pass_gap_us_normal);
      }
    }
#ifdef EPD_GREY16_PASS_TIMING
    if (_grey16) {
      const int want = (int)(_config.quality == Quality::QUALITY_HIGH
                                 ? _config.g16_pass_us_high
                                 : _config.g16_pass_us_normal);
      printf("[grey16] pass row-loop %lld..%lld us | ACTUAL period %lld..%lld us "
             "(want %d, jitter %lld us)\n",
             (long long)_g16_rowloop_min, (long long)_g16_rowloop_max,
             (long long)_g16_period_min, (long long)_g16_period_max, want,
             (long long)(_g16_period_max - _g16_period_min));
    }
#endif
#ifdef EPD_ASM_TIMING
    printf("[paint_task] convert_packed_fb_to_ink (all passes, all rows, darker+lighter): %lld us\n", _conv_total);
    printf("[paint_task] merged (mixed-chunk) rows: %d of %d\n", _dbl_rows, _config.height);
#endif
#ifdef EPD_ROW_PHASE_TIMING
    { extern int64_t g_rp_wait_us, g_rp_pins_us; extern uint32_t g_rp_rows;
      if (g_rp_rows)
        printf("[rowprof] rows=%u  dma-wait=%lld us (%.1f/row)  pins=%lld us (%.1f/row)\n",
               (unsigned)g_rp_rows, (long long)g_rp_wait_us,
               (double)g_rp_wait_us / g_rp_rows,
               (long long)g_rp_pins_us, (double)g_rp_pins_us / g_rp_rows); }
#endif

    vTaskDelay(1);  // yield once per frame: feeds WDT and lets application task run
    const int64_t _prof_close_t0 = esp_timer_get_time();

    // Closing sweep: float every row the passes drove, so none is left
    // holding its last drive code. Only the window needs it. Always full
    // width: it also zeroes the source positions a column cut-off skipped.
    if (row_bytes < full_row_bytes) _setBusRowBytes(full_row_bytes);
    memset(dma_buffer1, 0x00, packed_row_bytes);
    memset(dma_buffer2, 0x00, packed_row_bytes);
    for (int row = 0; row < r0; ++row) _skipRow(row == 0);
    for (int row = r0; row <= r1; ++row) {
      sendRow(row == 0, row == r1);
    }
    if (r1 < _config.height - 1) _flushGate(r1);

    if (_profile) {
      // One line per DRIVE CYCLE, from picking the frame up to the end of
      // the closing sweep. rows = row sends in the passes (passes x window).
      // wait = idle before this frame arrived; compare = discovery (pack is
      // on the caller's side); passes = row loops incl. skips and flushes;
      // gaps = pass padding and settle; close = closing sweep. period = start
      // to start of consecutive cycles, i.e. the real frame interval here.
      const int64_t now = esp_timer_get_time();
      const int64_t total = now - _prof_cycle_t0;
      const int64_t close = now - _prof_close_t0;
      const int64_t gaps = total - prof_disc_us - _prof_rowloop_us - close;
      printf("[paintprof] %d passes, %lu rows, %d/%d B/row | wait %lld  compare %lld  "
             "passes %lld  gaps %lld  close %lld  = %lld ms | period %lld ms\n",
             (int)wf_len, (unsigned long)_prof_rows, row_bytes, full_row_bytes,
             (long long)(prof_idle_us / 1000), (long long)(prof_disc_us / 1000),
             (long long)(_prof_rowloop_us / 1000), (long long)(gaps / 1000),
             (long long)(close / 1000), (long long)(total / 1000),
             (long long)(prof_last_t0 ? (_prof_cycle_t0 - prof_last_t0) / 1000 : 0));
      prof_last_t0 = _prof_cycle_t0;
    }

    _sb_guard_update();
    _paints_done.fetch_add(1);
  }
}

// GPIO set/clear via direct register write — pins always <32
#define GPIO_SET(mask) (GPIO.out_w1ts = (mask))
#define GPIO_CLR(mask) (GPIO.out_w1tc = (mask))

// Clock one bit: set/clear data, then pulse clk
#define SHIFT_BIT(data_set, data_clr, clk_mask) \
  data_set; clk_mask; clk_mask

// =============================================================================
// clearBuffers()
// Zero all packed pixel buffers so the DC-balance baseline is reset.
// Call this after painting a shutdown image and before cutting power, so that
// on the next boot unpaintPacked() works from a clean white baseline rather
// than the painted state that was left in PSRAM.
// =============================================================================
// ---- screenbuffer state guard (see header) --------------------------------
// Full shadow copy: on mismatch we fingerprint the foreign write — where
// it landed, how much, and what the bytes look like (ASCII = a string
// overflow, structured data = someone's buffer).

void EPD_Painter::_sb_guard_update() {
  if (!_sb_guard_on) return;
  const uint8_t *sb = _grey16 ? packed4_screenbuffer : packed_screenbuffer;
  if (!sb) return;
  const size_t bytes = packedSize() * (_grey16 ? 2 : 1);
  if (!_sb_shadow)
    _sb_shadow = (uint8_t *)heap_caps_malloc(packedSize() * 2, MALLOC_CAP_SPIRAM);
  if (!_sb_shadow) { _sb_guard_on = false; return; }
  memcpy(_sb_shadow, sb, bytes);
  _sb_guard_valid = true;
}

void EPD_Painter::_sb_guard_check() {
  if (!_sb_guard_on || !_sb_guard_valid || !_sb_shadow) return;
  const uint8_t *sb = _grey16 ? packed4_screenbuffer : packed_screenbuffer;
  if (!sb) return;
  const size_t bytes = packedSize() * (_grey16 ? 2 : 1);
  if (memcmp(_sb_shadow, sb, bytes) == 0) return;

  _sb_guard_hits++;
  size_t first = 0, last = 0, count = 0;
  for (size_t i = 0; i < bytes; i++) {
    if (_sb_shadow[i] != sb[i]) {
      if (!count) first = i;
      last = i;
      count++;
    }
  }
  printf("[EPD_Painter] CORRUPTION #%lu: foreign write into screenbuffer "
         "(%p): offsets %u..%u, %u bytes changed. int free=%u largest=%u\n",
         (unsigned long)_sb_guard_hits, sb, (unsigned)first, (unsigned)last,
         (unsigned)count,
         (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
         (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  const size_t dump = (bytes - first < 32) ? bytes - first : 32;
  printf("[EPD_Painter]   was:");
  for (size_t i = 0; i < dump; i++) printf(" %02x", _sb_shadow[first + i]);
  printf("\n[EPD_Painter]   now:");
  for (size_t i = 0; i < dump; i++) printf(" %02x", sb[first + i]);
  printf("\n[EPD_Painter]   txt: ");
  for (size_t i = 0; i < dump; i++) {
    const uint8_t c = sb[first + i];
    printf("%c", (c >= 32 && c < 127) ? c : '.');
  }
  printf("\n");
}

void EPD_Painter::clearBuffers() {
  const size_t packed_bytes = packedSize();
  if (packed_fastbuffer)   memset(packed_fastbuffer,   0, packed_bytes);
  if (packed_screenbuffer) memset(packed_screenbuffer, 0, packed_bytes);
  if (packed_paintbuffer)  memset(packed_paintbuffer,  0, packed_bytes);
  if (packed4_screenbuffer) memset(packed4_screenbuffer, 0, packed_bytes * 2);
  if (packed4_paintbuffer)  memset(packed4_paintbuffer,  0, packed_bytes * 2);
  _sb_guard_update();
}

// =============================================================================
// computeDirtyRects()
// Scans packed_screenbuffer vs packed_paintbuffer row by row.  Each row is
// checked byte-by-byte (1 byte = 4 pixels at 2bpp) to find the leftmost and
// rightmost changed byte.
//
// Rows are grouped into rectangles.  When a clean row is encountered inside an
// open rectangle the "wasted" pixel count grows by (rect_width × 4).  Once
// wasted exceeds tolerance the rectangle is closed at the last dirty row and a
// new one will be opened at the next dirty row.
//
// tolerance = 0  → only consecutive dirty rows are merged (tight rects)
// tolerance = large → everything collapses toward a single full-screen rect
// =============================================================================
int EPD_Painter::computeDirtyRects(Rect* out_rects, int max_rects, int tolerance) const {
    if (!packed_screenbuffer || !packed_paintbuffer || max_rects <= 0) return 0;

    const int prb = packed_row_bytes;
    const int H   = _config.height;

    int  count   = 0;
    bool open    = false;
    int  rect_y0 = 0, rect_y1 = 0;
    int  rect_bx0 = 0, rect_bx1 = 0;
    int  wasted  = 0;

    for (int row = 0; row < H && count < max_rects; row++) {
        const uint8_t* s = packed_screenbuffer + row * prb;
        const uint8_t* p = packed_paintbuffer  + row * prb;

        // Find changed byte span for this row
        int bx0 = prb, bx1 = 0;
        for (int bx = 0; bx < prb; bx++) {
            if (s[bx] != p[bx]) {
                if (bx < bx0) bx0 = bx;
                bx1 = bx + 1;
            }
        }
        const bool dirty = (bx0 < bx1);

        if (dirty) {
            if (!open) {
                open     = true;
                rect_y0  = row;
                rect_y1  = row + 1;
                rect_bx0 = bx0;
                rect_bx1 = bx1;
                wasted   = 0;
            } else {
                if (bx0 < rect_bx0) rect_bx0 = bx0;
                if (bx1 > rect_bx1) rect_bx1 = bx1;
                rect_y1 = row + 1;
                wasted  = 0;
            }
        } else if (open) {
            wasted += (rect_bx1 - rect_bx0) * 4;
            if (wasted > tolerance) {
                out_rects[count++] = {
                    (int16_t)(rect_bx0 * 4),
                    (int16_t) rect_y0,
                    (int16_t)((rect_bx1 - rect_bx0) * 4),
                    (int16_t)(rect_y1 - rect_y0)
                };
                open = false;
            }
        }
    }

    if (open && count < max_rects) {
        out_rects[count++] = {
            (int16_t)(rect_bx0 * 4),
            (int16_t) rect_y0,
            (int16_t)((rect_bx1 - rect_bx0) * 4),
            (int16_t)(rect_y1 - rect_y0)
        };
    }

    return count;
}

// =============================================================================
// clearDirtyAreas()
// =============================================================================
void EPD_Painter::clearDirtyAreas(uint8_t* framebuffer, int tolerance, ClearMode mode) {
    // Compact 8bpp framebuffer into packed_paintbuffer (same as paint()).
    xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
    _pack(framebuffer, packed_paintbuffer, false);
    xSemaphoreGive(_paint_buffer_sem);

    Rect rects[32];
    int n = computeDirtyRects(rects, 32, tolerance);
    if (n > 0)
        clear(rects, n, mode);
}

// =============================================================================
// clear()
// Optional rects array restricts clearing to those regions only.
// =============================================================================
void EPD_Painter::clear(const Rect* rects, int num_rects, ClearMode mode) {

  const bool partial = (rects != nullptr && num_rects > 0);

  // NOTE (Tony, ghost hunt): clear() no longer auto-releases an active
  // template — a template now SURVIVES a full clear (the overlay simply
  // re-drives it on the next paint). Apps release explicitly.

  PanelPowerGuard guard(*this);

  const int prb = packed_row_bytes;

  // Paint white into the affected area of the paint buffer. In 16-grey
  // mode the paint task reads the 4bpp paintbuffer, so whiten that too.
  xSemaphoreTake(_paint_buffer_sem, portMAX_DELAY);
  if (!partial) {
    memset(packed_paintbuffer, 0x00, prb * _config.height);
    if (_grey16) memset(packed4_paintbuffer, 0x00, (size_t)prb * 2 * _config.height);
  } else {
    for (int r = 0; r < num_rects; r++) {
      int bx0 = rects[r].x / 4;
      int bx1 = (rects[r].x + rects[r].w + 3) / 4;
      if (bx1 > prb) bx1 = prb;
      int y0  = rects[r].y;
      int y1  = rects[r].y + rects[r].h;
      if (y1 > _config.height) y1 = _config.height;
      for (int row = y0; row < y1; row++) {
        memset(packed_paintbuffer + row * prb + bx0, 0x00, bx1 - bx0);
        if (_grey16)
          memset(packed4_paintbuffer + (size_t)row * prb * 2 + bx0 * 2, 0x00, (bx1 - bx0) * 2);
      }
    }
  }
  _pr_lo = 0; _pr_hi = _config.height - 1;
  paintStage = 1;
  xSemaphoreGive(_paint_buffer_sem);

  while (paintStage == 1) {
    vTaskDelay(1);
  }

  xSemaphoreTake(_paint_active_sem, portMAX_DELAY);

  dma_buffer = dma_buffer1;

  // Hardware clear phases.
  int num_phases;
  int totpass[4];
  if (mode == ClearMode::SOFT) {
    num_phases = 2;
    totpass[0] = 1; totpass[1] = 1;
  } else {
    num_phases = 4;
    totpass[0] = 6; totpass[1] = 2; totpass[2] = 4; totpass[3] = 8;
  }
  for (int phase = 0; phase < num_phases; phase++) {

    uint8_t pattern = (phase % 2 == 0) ? 0b01010101 : 0b10101010;

    for (int passes = 0; passes < totpass[phase]; passes++) {
      for (int row = 0; row < _config.height; ++row) {
        if (!partial) {
          memset(dma_buffer, pattern, prb);
        } else {
          memset(dma_buffer, 0x00, prb);
          for (int r = 0; r < num_rects; r++) {
            if (row >= rects[r].y && row < rects[r].y + rects[r].h) {
              int bx0 = rects[r].x / 4;
              int bx1 = (rects[r].x + rects[r].w + 3) / 4;
              if (bx1 > prb) bx1 = prb;
              memset(dma_buffer + bx0, pattern, bx1 - bx0);
            }
          }
          // The hardware phases must not scrub protected template pixels
          // (the delta unpaint above them is already overlay-protected).
          if (_has_template) {
            if (!_tpl_grey16) {
              const uint8_t *m = tpl_mask + (size_t)row * prb;
              for (int bx = 0; bx < prb; bx++)
                dma_buffer[bx] &= (uint8_t)~m[bx];
            } else {
              const uint8_t *m = tpl_mask + (size_t)row * prb * 2;
              for (int bx = 0; bx < prb; bx++) {
                const uint8_t a = m[bx * 2], b = m[bx * 2 + 1];
                uint8_t dm = 0;
                if (a & 0xF0) dm |= 0xC0;
                if (a & 0x0F) dm |= 0x30;
                if (b & 0xF0) dm |= 0x0C;
                if (b & 0x0F) dm |= 0x03;
                dma_buffer[bx] &= (uint8_t)~dm;
              }
            }
          }
        }
        sendRow(row == 0);
      }
      _endPass();
      EPD_DELAY_MS(5);
    }
  }

  // Send neutral to close out all rows.
  memset(dma_buffer1, 0x00, prb);
  memset(dma_buffer2, 0x00, prb);
  for (int row = 0; row < _config.height; ++row) {
    sendRow(row == 0, row == _config.height - 1);
  }

  xSemaphoreGive(_paint_active_sem);
}
// =============================================================================
// fxClear() — sweeping bar clear effect
//
// A thick bar travels left-to-right across the panel. Within the bar, each
// pixel is independently and randomly driven black or white on every pass,
// so adjacent pixels may be heading in opposite directions — the "fuzz".
// Left of the bar, pixels are settled white; right of the bar they are driven
// black and wait for the bar to sweep over them.
//
// Tuning constants (pixels, dimensionless):
//   BAR_W   — width of the active bar
//   STEP    — columns advanced per step
//   NPASSES — render passes per step (more = slower but better clearing)
// =============================================================================
void EPD_Painter::fxClear() {
  const int prb = packed_row_bytes;

  // Wait for any in-progress paint to finish, then take exclusive control.
  // Unlike clear(), we do NOT issue a white frame first — the screenbuffer
  // must still reflect the current display state for the pixel mask to work.
  xSemaphoreTake(_paint_active_sem, portMAX_DELAY);

  PanelPowerGuard guard(*this);

  const int H = _config.height;

  dma_buffer = dma_buffer1;

  // All white (0b00) screenbuffer pixels are driven fully black in the dark
  // zone and fully white in the light zone — no partial selection. This gives
  // maximum contrast: a solid black leading edge and a solid white trailing
  // edge. DC balance is maintained because dark and light zones are equal size.
  const int BAR_DARK  = 180;   // rows of black voltage (leading edge)
  const int BAR_LIGHT = 180;   // rows of white voltage (trailing edge)
  const int BAR_H     = BAR_DARK + BAR_LIGHT;
  const int STEP      = 15;    // 1 row per step → 60 dark + 60 light pulses per pixel

  for (int bar_top = -BAR_H; bar_top <= H; bar_top += STEP) {
    const int dark_top = bar_top + BAR_LIGHT;   // dark zone: lower (leading) half
    const int bar_bot  = bar_top + BAR_H;

    for (int row = 0; row < H; row++) {
      uint32_t* buf32 = reinterpret_cast<uint32_t*>(dma_buffer);

      if (row >= bar_top && row < dark_top) {
        // Light phase (upper/trailing): all white pixels → white voltage
        for (int i = 0; i < prb / 4; i++) {
          const uint32_t* sb32 = reinterpret_cast<const uint32_t*>(packed_screenbuffer + row * prb) + i;
          uint32_t either     = (*sb32 | (*sb32 >> 1)) & 0x55555555u;
          uint32_t pixel_mask = ~(either | (either << 1));
          buf32[i] = 0xAAAAAAAAu & pixel_mask;
        }
      } else if (row >= dark_top && row < bar_bot) {
        // Dark phase (lower/leading): all white pixels → black voltage
        for (int i = 0; i < prb / 4; i++) {
          const uint32_t* sb32 = reinterpret_cast<const uint32_t*>(packed_screenbuffer + row * prb) + i;
          uint32_t either     = (*sb32 | (*sb32 >> 1)) & 0x55555555u;
          uint32_t pixel_mask = ~(either | (either << 1));
          buf32[i] = 0x55555555u & pixel_mask;
        }
      } else {
        memset(dma_buffer, 0x00, prb);
      }
      sendRow(row == 0, false);
    }
    _endPass();
  }


  // Final neutral flush — de-energise all pixels
  memset(dma_buffer1, 0x00, prb);
  memset(dma_buffer2, 0x00, prb);
  for (int row = 0; row < H; ++row) {
    sendRow(row == 0, row == H - 1);
  }

  // Screenbuffer is unchanged — non-white pixels were never driven, white pixels
  // were already 0x00. No update needed.

  xSemaphoreGive(_paint_active_sem);
}


// =============================================================================
// GrayDither — streaming Floyd-Steinberg against the measured level curve
//
// Error distribution:
//          [x]  →  right:        7/16
//   below-left:  3/16   below:  5/16   below-right:  1/16
// =============================================================================
bool EPD_Painter::GrayDither::begin(const EPD_Painter &p,
                                    uint8_t *dst, int dstW, int dstH,
                                    int srcW, int srcH,
                                    int dx, int dy, int dw, int dh) {
    end();
    if (!dst || srcW <= 0 || srcH <= 0 || dstW <= 0 || dstH <= 0) return false;
    if (dw <= 0) dw = srcW;
    if (dh <= 0) dh = srcH;

    _dst = dst; _dstW = dstW; _dstH = dstH;
    _srcW = srcW; _srcH = srcH;
    _dx = dx; _dy = dy; _dw = dw; _dh = dh;
    _levels = p._grey16 ? 16 : 4;
    _nextY = 0;

    // What each level renders as: the panel's measured curve when it has been
    // tuned, an even ramp otherwise. The even ramp is the only honest guess
    // available, and is also exactly the assumption that produces banding.
    if (const uint8_t *lum = p._config.levelLum(_levels))
        for (int i = 0; i < _levels; i++) _lum[i] = lum[i];
    else
        for (int i = 0; i < _levels; i++)
            _lum[i] = (uint8_t)(255 - i * 255 / (_levels - 1));

    // Error rows follow the DESTINATION width - diffusion happens in output
    // space, so a shrunk image spreads error across the pixels it will really
    // occupy rather than across source pixels that get thrown away.
    const size_t erow = (size_t)(_dw + 2) * sizeof(int16_t);
    _cur  = (int16_t *)heap_caps_malloc(erow, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    _next = (int16_t *)heap_caps_malloc(erow, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!_cur || !_next) { end(); return false; }
    memset(_cur,  0, erow);
    memset(_next, 0, erow);
    return true;
}

void EPD_Painter::GrayDither::emit(const uint8_t *src, int y) {
    const size_t erow = (size_t)(_dw + 2) * sizeof(int16_t);
    memset(_next, 0, erow);
    const int ty = _dy + y;

    // SERPENTINE: alternate scan direction per row. A fixed direction gives
    // diffusion a standing bias, and with a wide gap in the level curve the
    // bias organises into column-aligned worms — measured on the glass as
    // full-height dark streaks through a photograph.
    const bool rtl = (y & 1);
    for (int i = 0; i < _dw; i++) {
        const int x = rtl ? (_dw - 1 - i) : i;
        const int fwd = rtl ? -1 : 1;             // "ahead" in scan order
        int v = (int)src[(int)((int64_t)x * _srcW / _dw)] + _cur[x + 1];
        if (v < 0) v = 0; else if (v > 255) v = 255;

        // Nearest level BY MEASURED LUMINANCE. Linear over at most 16 entries
        // — nothing beside the paint — and the curve is not guaranteed
        // monotonic on a poorly tuned panel, so a binary search is not safe.
        int best = 0, bestd = 1000;
        for (int i2 = 0; i2 < _levels; i2++) {
            const int d = abs((int)_lum[i2] - v);
            if (d < bestd) { bestd = d; best = i2; }
        }
        const int e = v - (int)_lum[best];        // what could not be shown

        const int tx = _dx + x;
        if (tx >= 0 && tx < _dstW && ty >= 0 && ty < _dstH)
            _dst[(size_t)ty * _dstW + tx] = (uint8_t)best;

        // Diffuse even where clipped: an error dropped at the edge bands
        // along it. Shares divide toward zero — the old arithmetic shifts
        // floored toward -inf, which turned a -1 error into FOUR -1 shares:
        // a x4 amplifier for small dark errors, self-reinforcing down the
        // image. The remainder goes to the ahead-neighbour so the total
        // diffused is EXACTLY e: error is conserved, never invented.
        const int e3 = (e * 3) / 16;
        const int e5 = (e * 5) / 16;
        const int e1 = (e * 1) / 16;
        const int e7 = e - e3 - e5 - e1;          // ahead: share + remainder
        _cur [x + 1 + fwd] += (int16_t)e7;        // ahead
        _next[x + 1 - fwd] += (int16_t)e3;        // behind, next row
        _next[x + 1      ] += (int16_t)e5;        // below
        _next[x + 1 + fwd] += (int16_t)e1;        // ahead, next row
    }
    int16_t *t = _cur; _cur = _next; _next = t;
}

void EPD_Painter::GrayDither::row(const uint8_t *src, int srcY) {
    if (!_cur || !src) return;
    // Emit every destination row this source row is responsible for. Shrinking
    // skips source rows; enlarging repeats one. Driving the loop from the
    // DESTINATION side means the scale factor never has to divide evenly.
    while (_nextY < _dh && (int)((int64_t)_nextY * _srcH / _dh) <= srcY) {
        emit(src, _nextY);
        _nextY++;
    }
}

void EPD_Painter::GrayDither::end() {
    if (_cur)  { heap_caps_free(_cur);  _cur  = nullptr; }
    if (_next) { heap_caps_free(_next); _next = nullptr; }
    _dst = nullptr;
}

// =============================================================================
// drawGray8() — the whole-image convenience over GrayDither
// =============================================================================
void EPD_Painter::drawGray8(uint8_t *dst, int dstW, int dstH,
                            const uint8_t *src, int srcW, int srcH,
                            int dx, int dy, int dw, int dh) const {
    GrayDither d;
    if (!d.begin(*this, dst, dstW, dstH, srcW, srcH, dx, dy, dw, dh)) return;
    for (int y = 0; y < srcH; y++) d.row(src + (size_t)y * srcW, y);
    d.end();
}

// =============================================================================
// packBuffer()
// =============================================================================
uint8_t* EPD_Painter::packBuffer(const uint8_t* fb) const {
    uint8_t* buf = static_cast<uint8_t*>(
        heap_caps_aligned_alloc(16, packedSize(), MALLOC_CAP_SPIRAM));
    if (!buf) return nullptr;
    _pack(fb, buf, false);
    return buf;
}
