#ifndef EPD_PAINTER_DEVICES_H
#define EPD_PAINTER_DEVICES_H

#include "EPD_Painter.h"

#if defined(EPD_PAINTER_PRESET_M5PAPER_S3)
    static EPD_Painter::Config EPD_PAINTER_PRESET = {
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
        .waveforms = {
            .fast_lighter   = { { 1, 2, 2, 2, 2, 2, 3 },
                                { 3, 2, 2, 2, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 1, 1, 3, 3, 1, 3, 1 },
                                { 3, 1, 1, 1, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 3, 1, 1, 2, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 1, 1, 3, 1, 1, 1, 1, 2, 1, 2, 2, 2 },
                                { 1, 2, 2, 1, 1, 2, 3, 3, 1, 1, 1, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 3, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 2, 2, 2, 1, 2, 1, 1, 2, 1 },
                                { 3, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 2 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
    };

#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS)
    static EPD_Painter::Config EPD_PAINTER_PRESET = {
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
        .waveforms = {
            .fast_lighter   = { { 1, 2, 2, 2, 2, 2, 3 },
                                { 3, 2, 2, 2, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 1, 1, 3, 3, 1, 3, 1 },
                                { 1, 3, 1, 1, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 1, 1, 1, 2, 2, 3, 2, 2, 2, 2, 2, 2 },
                                { 1, 2, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 2, 1, 1, 1, 3, 1, 2, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 2, 2, 3, 1, 1, 3, 1, 3, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                                { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
    };
// -----------------------------------------------------------------------
// LilyGo T5 S3 H752 (older variant with 74HCT4094D shift register)
//   74HCT4094D outputs used by the panel driver:
//     QP0 -> EP_LE, QP4 -> EP_STV, QP5 -> PWR_EN, QP6 -> EP_MODE, QP7 -> EP_OE
//   CKV/CKH/STH and the 8-bit data bus remain direct GPIOs.
// -----------------------------------------------------------------------
#elif defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_H752)
    static EPD_Painter::Config EPD_PAINTER_PRESET = {
        .width    = 960,
        .height   = 540,
        .pin_pwr  = -1,   // shift register
        .pin_sph  = 9,    // STH  — direct GPIO
        .pin_oe   = -1,   // shift register
        .pin_cl   = 10,   // CKH  — direct GPIO
        .pin_spv  = -1,   // EP_STV / SPV via shift register Q4
        .pin_ckv  = 39,   // CKV  — direct GPIO
        .pin_le   = -1,   // EP_LE via shift register Q0
        .quality  = EPD_Painter::Quality::QUALITY_NORMAL,
        .data_pins = { 11, 12, 13, 14, 21, 47, 45, 38 },
        .i2c = { .sda = 6, .scl = 5, .freq = 100000 },   // I2C bus exposed for peripherals on H752
        .power = { .pca_addr = -1, .tps_addr = -1 },
        .waveforms = {
            .fast_lighter   = { { 1, 3, 2, 3, 2, 2, 3 },
                                { 3, 2, 3, 2, 2, 3, 2 },
                                { 2, 2, 2, 2, 2, 2, 2 } },
            .fast_darker    = { { 3, 1, 3, 2, 1, 1, 3 },
                                { 1, 3, 1, 3, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1 } },
            .normal_lighter = { { 1, 1, 1, 1, 2, 3, 3, 2, 2, 2, 2, 2, 2 },
                                { 2, 1, 2, 2, 1, 2, 2, 2, 0, 0, 2, 2, 3 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .normal_darker  = { { 1, 2, 1, 3, 1, 3, 1, 2, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 2, 2, 3, 1, 1, 3, 2, 1, 1, 3 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                                { 1, 1, 2, 2, 1, 2, 2, 2, 2, 1, 2, 2, 2 },
                                { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
            .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                                { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                                { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
        },
        .shift = { .data = 2, .clk = 42, .strobe = 1 },
    };
    
#else
#error "No EPD_Painter device selected."
#endif


#endif
