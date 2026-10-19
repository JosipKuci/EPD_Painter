#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "sdkconfig.h"
#include "EPD_Painter.h"
#include "EPD_Painter_trains.h"

// -----------------------------------------------------------------------
// Soldered Inkplate 10 panel (ED097TC2, 9.7", 1200x825) on an ESP32-S3.
//
// Wiring is the inkplate10-s3-test breakout (InkplateLibrary
// INKPLATE10_S3_WIRING.md). The data bus is split across GPIO21 and
// GPIO38-48 — which cost the bit-banged Inkplate driver four register
// stores per byte — but LCD_CAM reaches every pin through the GPIO
// matrix, so here the split costs nothing.
//
//   D0..D7  38 39 40 41 42 47 48 21
//   CL 4   SPV 5   GMOD 6   OE 7   CKV 10   LE 11   SPH 12
//   I2C SDA 8 / SCL 9: PCAL6416A @0x20, TPS65186 @0x48
//
// GPIO48 and GPIO38 drive the onboard WS2812 on most S3 devkits: a load on
// a data line. Cut the LED jumper if the board has one.
// -----------------------------------------------------------------------
inline EPD_Painter::Config EPD_INKPLATE10_PRESET = {
    .width    = 1200,
    .height   = 825,
    .pin_sph  = 12,
    .pin_oe   = 7,
    .pin_gmod = 6,
    .pin_cl   = 4,
    .pin_spv  = 5,
    .pin_ckv  = 10,
    .pin_le   = 11,
    .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
    // The Inkplate library scans its framebuffer from the last byte to the
    // first, so its (0,0) is the last pixel the panel receives. 180 here
    // keeps images the same way up as they are under the Inkplate library.
    .rotation = EPD_Painter::Rotation::ROTATION_180,
    // 4 MHz = 125 ns CL high, the shortest pulse the bit-banged Inkplate
    // driver found working on the breakout. LCD_CAM gives data half a clock
    // of setup where the bit-banged driver gave D7 none, so this is likely
    // pessimistic: raise it once the wiring has been proven, and retune.
    .pclk_hz  = 4000000,
    .vscan_skip_lines = 3,
    .vscan_hold_us    = 8,
    .data_pins = { 38, 39, 40, 41, 42, 47, 48, 21 },
    .i2c = {
        .wire = nullptr,  // begin() uses the global Wire on these pins
        .sda  = 8,
        .scl  = 9,
        .freq = 100000,
    },
    .power = {
        .expander_addr = 0x20,
        .tps_addr      = 0x48,
        .pin_wakeup    = 0,
        .pin_pwrup     = 1,
        .pin_vcom      = 2,
        .pin_int       = 3,
        .pin_pwrgood   = 4,
    },
    .waveforms = EPD_WF_INKPLATE10,  // Inkplate10_Trains.h (seed, untuned)
    // A 16-grey pass must outlast the row loop: 825 rows x 308 bus bytes at
    // 4 MHz is 63.5 ms before conversion, row_extra_us and LE holds. 75 ms
    // leaves the settle floor intact. Scale it with pclk_hz.
    .g16_pass_us_normal = 75000,
    .g16_pass_us_high   = 75000,
    // No .trains: never scanner-tuned. 16-grey runs on the formula library
    // and direct transitions stay on the two-step path.
};

// -----------------------------------------------------------------------
// The same Inkplate 10 panel and power board on an ESP32-S31.
//
//   D0..D7  8 9 10 11 12 13 14 15   (the S31's IO MUX LCD_DATA0..7 pads)
//   CL 40 (IO MUX LCD_PCLK pad)   SPH 43   CKV 16   LE 17   SPV 18
//   OE 19   GMOD 44
//   I2C SDA 60 / SCL 61: PCAL6416A @0x20, TPS65186 @0x48
//
// GPIO60/61 are boot-mode strapping pins with weak pull-ups; the I2C
// pull-ups hold them at the same (normal boot) level. GPIO33/34 are left
// for USB Serial/JTAG. Everything is routed through the GPIO matrix, so any
// free pins work; these just keep the bus on its native pads.
//
// Untuned and unproven on hardware: same seed trains and timing as the S3.
// -----------------------------------------------------------------------
inline EPD_Painter::Config EPD_INKPLATE10_S31_PRESET = {
    .width    = 1200,
    .height   = 825,
    .pin_sph  = 43,
    .pin_oe   = 19,
    .pin_gmod = 44,
    .pin_cl   = 40,
    .pin_spv  = 18,
    .pin_ckv  = 16,
    .pin_le   = 17,
    .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
    .rotation = EPD_Painter::Rotation::ROTATION_180,
    .pclk_hz  = 4000000,
    .vscan_skip_lines = 3,
    .vscan_hold_us    = 8,
    .afifo_reset_per_row = false,
    .data_pins = { 8, 9, 10, 11, 12, 13, 14, 15 },
    .i2c = {
        .wire = nullptr,
        .sda  = 60,
        .scl  = 61,
        .freq = 100000,
    },
    .power = {
        .expander_addr = 0x20,
        .tps_addr      = 0x48,
        .pin_wakeup    = 0,
        .pin_pwrup     = 1,
        .pin_vcom      = 2,
        .pin_int       = 3,
        .pin_pwrgood   = 4,
    },
    .waveforms = EPD_WF_INKPLATE10,
    .g16_pass_us_normal = 75000,
    .g16_pass_us_high   = 75000,
};

// The preset for the chip being built for.
#if CONFIG_IDF_TARGET_ESP32S31
inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_INKPLATE10_S31_PRESET;
#else
inline EPD_Painter::Config& EPD_PAINTER_PRESET = EPD_INKPLATE10_PRESET;
#endif

#endif
