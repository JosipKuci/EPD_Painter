#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "EPD_Painter.h"
#include "EPD_Painter_trains.h"

#if defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716) || defined(EPD_PAINTER_PRESET_AUTO)
// -----------------------------------------------------------------------
// LilyGo T5 4.7" EPD47 H716 (ESP32-S3 variant, capacitive touch)
//   74HCT4094D shift register with H716-specific bit layout:
//     QP0=EP_LE, QP1=PWR_DIS, QP2=POS_PWR, QP3=NEG_PWR,
//     QP4=EP_STV, QP5=SCAN_DIR, QP6=EP_MODE, QP7=EP_OE
//   CKV, STH, CKH and the 8-bit data bus remain direct GPIOs.
// -----------------------------------------------------------------------
    inline EPD_Painter::Config EPD_LILYGO_EPD47_H716_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,             // shift register (power managed by H716 power driver)
        .pin_sph  = 40,             // STH  — direct GPIO
        .pin_oe   = -1,             // shift register QP7
        .pin_cl   = 41,             // CKH  — direct GPIO
        .pin_spv  = EPD_SR_PIN(4),  // EP_STV — shift register QP4
        .pin_ckv  = 38,             // CKV   — direct GPIO
        .pin_le   = EPD_SR_PIN(0),  // EP_LE — shift register QP0
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 8, 1, 2, 3, 4, 5, 6, 7 },  // D0–D7
        .i2c = { .sda = 18, .scl = 17, .freq = 100000 },
        .power = { .pca_addr = -1, .tps_addr = -1 },
        .waveforms = EPD_WF_H716,        // LilyGo_H716_Trains.h
        .shift = { .data = 13, .clk = 12, .strobe = 0, .le_time = 0,
                   .driver = EPD_Painter::Shift::H716 },
        // Row-loop cost here is conversion-bound like every board (the
        // shift register adds only ~1us/row via the fast-GPIO path;
        // measured: LE pulse 1.17us). Typical worst passes ~16 ms; the
        // pathological full-screen-inversion case overruns and clamps to
        // the settle floor, same as the M5PaperS3 at its 15 ms period.
        // Trains are calibrated at THESE periods.
        //
        // HIGH 24000 -> 20000, measured 28 July 2026 (dose ladder, `D`).
        // The longer period was inherited from an assumption that a
        // shift-register board must have slow rows. It does not - the
        // shifting is bit-banged from assembly and completes in under a
        // microsecond, and the measured row loops on this board (8.4-13.1 ms)
        // sit in the same band as the LilyGo GPS's direct-GPIO ones.
        //
        // The ladder settles it: 24 / 20 / 18 ms all bottom out at the same
        // depth (103 of 109 units) with the knee at pass 7, while the first
        // pass - the finest increment the panel can be asked for - goes
        // 2.70 / 2.26 / 1.89 level steps. So the longer period bought no
        // depth and cost 20% in quantisation and 80 ms a frame. 18 ms is
        // finer still, but its allowance (16.5 ms) is under the worst row
        // loop photo content has been seen to take here (19.2 ms), where
        // 20 ms leaves 18.5 ms. Matching NORMAL also lets a HIGH train
        // inherit a NORMAL one slot-for-slot, since a slot then means the
        // same dose in both.
        .g16_pass_us_normal = 20000,
        .g16_pass_us_high   = 20000,
        .trains = {
            .g16_apply       = TUNED16_H716_NORMAL,
            .g16_remove      = TUNED16_H716_NORMAL_REMOVE,
            .g16_apply_high  = TUNED16_H716_HIGH,
            .g16_remove_high = TUNED16_H716_HIGH_REMOVE,
            .dir_normal      = DIRECT_H716_NORMAL,
            // No dir_fast: the July FAST directs died with the old FAST
            // waveform pair — see LilyGo_H716_Trains.h.
        },
        .level_lum      = LEVEL_LUM_H716_NORMAL,
        .level_lum_high = LEVEL_LUM_H716_HIGH,
        .level_lum4     = LEVEL_LUM4_H716_FAST,
    };
#endif

#if defined(EPD_PAINTER_PRESET_M5PAPER_S3) || defined(EPD_PAINTER_PRESET_AUTO)
    inline EPD_Painter::Config EPD_M5PAPER_S3_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr    = 46,
        .pin_syspwr = 44,
        .pin_sph  = 13,
        .pin_oe   = 45,
        .pin_cl   = 16,
        .pin_spv  = 17,
        .pin_ckv  = 18,
        .pin_le   = 15,
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 6, 14, 7, 12, 9, 11, 8, 10 },
        .i2c = {
            .sda = 41,
            .scl = 42,
            .freq = 100000
        },
        .waveforms = EPD_WF_M5PAPERS3,   // M5PaperS3_Trains.h
        .trains = {
            .g16_apply  = TUNED16_M5PAPERS3_NORMAL,
            .g16_remove = TUNED16_M5PAPERS3_NORMAL_REMOVE,
            .dir_normal = DIRECT_M5PAPERS3_NORMAL,
            .dir_fast   = DIRECT_M5PAPERS3_FAST,
        },
    };
#endif
#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS) || defined(EPD_PAINTER_PRESET_AUTO)
    inline EPD_Painter::Config EPD_LILYGO_T5_S3_GPS_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,  // managed via TPS65185 over I2C (powerctl)
        .pin_sph  = 41,
        .pin_oe   = -1,  // managed via PCA9555 over I2C (powerctl)
        .pin_cl   = 4,
        .pin_spv  = 45,
        .pin_ckv  = 48,
        .pin_le   = 42,
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 5,6,7,15,16,17,18,8 },
        .i2c = {
            .sda = 39,
            .scl = 40,
            .freq = 100000
        },
        .power = {
            .pca_addr = 0x20,
            .tps_addr = 0x68,
        },
        .waveforms = EPD_WF_LILYGO_T5S3_GPS,  // LilyGo_T5S3_GPS_Trains.h
        // MEASURED 24 Jul 2026: this panel's re-darkens are weaker than
        // the M5PaperS3's — at the default 15 ms period its 16-grey
        // black floors at ~67 while the glass reaches ~51 at 20 ms (the
        // H716's cure). The tuned tables below are 15 ms tables, so the
        // period stays 15 ms until a 20 ms retune session lands: raising
        // it without retuning scrambles the ladder (verified). The 20 ms
        // retune must use the match-card method — staircase-vs-anchor
        // tuning cannot converge here (frame-composition shifts move
        // landings by 3-6 levels between frames; also a ~5-unit
        // top-to-bottom panel gradient, calibrate via the L15 pair).
        .trains = {
            .g16_apply  = TUNED16_LILYGO_T5S3_NORMAL,
            .g16_remove = TUNED16_LILYGO_T5S3_NORMAL_REMOVE,
            .dir_normal = DIRECT_LILYGO_T5S3_NORMAL,
            .dir_fast   = DIRECT_LILYGO_T5S3_FAST,
        },
    };
#endif
// -----------------------------------------------------------------------
// LilyGo T5 S3 H752 (older variant with 74HCT4094D shift register)
//   74HCT4094D outputs used by the panel driver:
//     QP0 -> EP_LE, QP4 -> EP_STV, QP5 -> PWR_EN, QP6 -> EP_MODE, QP7 -> EP_OE
//   CKV/CKH/STH and the 8-bit data bus remain direct GPIOs.
// -----------------------------------------------------------------------
#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752) || defined(EPD_PAINTER_PRESET_AUTO)
    inline EPD_Painter::Config EPD_LILYGO_T5_S3_H752_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,            // shift register (power managed by shiftctl->powerOn/Off)
        .pin_sph  = 9,             // STH  — direct GPIO
        .pin_oe   = -1,            // shift register (power managed by shiftctl->powerOn/Off)
        .pin_cl   = 10,            // CKH  — direct GPIO
        .pin_spv  = EPD_SR_PIN(4), // EP_STV / SPV — shift register QP4
        .pin_ckv  = 39,            // CKV  — direct GPIO
        .pin_le   = EPD_SR_PIN(0), // EP_LE — shift register QP0
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 11, 12, 13, 14, 21, 47, 45, 38 },
        .i2c = { .sda = 6, .scl = 5, .freq = 100000 },   // I2C bus exposed for peripherals on H752
        .power = { .pca_addr = -1, .tps_addr = -1 },
        .waveforms = EPD_WF_H752,        // LilyGo_H752_Trains.h (hand-built)
        .shift = { .data = 2, .clk = 42, .strobe = 1, .le_time = 0 },
        // No .trains: this variant has not had a scanner-tuning session.
        // 16-grey runs on the formula library and direct transitions stay
        // on the two-step path until a match-card session tunes it.
    };

#endif

#if defined(EPD_PAINTER_PRESET_M5PAPER_S3)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_M5PAPER_S3_PRESET;
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_LILYGO_T5_S3_GPS_PRESET;
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_LILYGO_T5_S3_H752_PRESET;
#elif defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716)
    inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_LILYGO_EPD47_H716_PRESET;
#elif defined(EPD_PAINTER_PRESET_AUTO)
// Auto-select preset based on target board (via preprocessor macros defined by the build system)
    inline EPD_Painter::Config EPD_PAINTER_PRESET = {
    .width    = 960,
    .height   = 540,
    .pin_pwr  = -1,
    .pin_sph  = -1,
    .pin_oe   = -1,
    .pin_cl   = -1,
    .pin_spv  = -1,
    .pin_ckv  = -1,
    .pin_le   = -1,
    .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
    .data_pins = { -1, -1, -1, -1, -1, -1, -1, -1 },
    .waveforms = { },
};

inline EPD_Painter::ProbeSettings Probe[] = {
    // Boards with unique I2C addresses go first to minimise failed-probe noise.
    { &EPD_LILYGO_T5_S3_GPS_PRESET,      39, 40, 0x20, false }, // PCA9555  — unique addr
    { &EPD_LILYGO_EPD47_H716_PRESET,     18, 17, 0x5D, false }, // GT911    — unique addr
    // The GT911's address depends on its INT/RST strapping at power-up and
    // genuinely moves between 0x5D and 0x14 across warm resets — a board
    // that probed fine all day can vanish after a reflash. Probe both, and
    // the BQ25896 charger as a strap-independent backstop.
    { &EPD_LILYGO_EPD47_H716_PRESET,     18, 17, 0x14, false }, // GT911 alt addr
    { &EPD_LILYGO_EPD47_H716_PRESET,     18, 17, 0x6B, false }, // BQ25896 charger
    // Boards sharing 0x51 (BM8563 RTC) are distinguished by their SCL/SDA pins.
    { &EPD_LILYGO_T5_S3_H752_PRESET,      6,  5, 0x51, false }, // BM8563 on bus 6/5
    { &EPD_M5PAPER_S3_PRESET,            41, 42, 0x51, false }, // BM8563 on bus 41/42
    // Add new board probes here, ensuring they have unique I2C addresses and pins
};
#else
    #error "No EPD_PAINTER_PRESET defined; please define one of the presets or set EPD_PAINTER_PRESET_AUTO"
#endif

#endif
