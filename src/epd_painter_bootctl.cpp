#include "epd_painter_bootctl.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_sleep.h"
#include "esp_heap_caps.h"
#include <stdio.h>

EPD_BootCtl::EPD_BootCtl(EPD_Painter& epd) : _epd(epd) {
    if (_isUsbConnected()) return;

    if (!_readFlag()) {
        _writeFlag(true);
        _paintFractal();
        delay(500);    // allow the panel to finish driving before touching buffers
        _epd.clearBuffers();
        _powerOff();   // [[noreturn]]
    }

    _writeFlag(false);
    _unpaintFractal();
}

void EPD_BootCtl::shutdown() {
    _writeFlag(true);
    _paintFractal();
    delay(500);    // allow the panel to finish driving before touching buffers
    _epd.clearBuffers();
    _powerOff();   // [[noreturn]]
}

// =============================================================================
// XOR fractal
//
// pixel(x, y) = (x ^ y) & 3
//
// This is a self-similar (fractal) bit-pattern that tiles perfectly.  It is
// generated directly into packed 2bpp format (MSB = leftmost pixel) without
// an 8bpp intermediate, so the whole 960×540 buffer is filled in < 1 ms.
//
// DC-balance property: each of the four grey levels appears exactly 25 % of
// the time, making unpaintPacked() as efficient as possible on the next boot.
// =============================================================================

uint8_t* EPD_BootCtl::_allocFractal() const {
    const EPD_Painter::Config& cfg = _epd.getConfig();
    const uint16_t W = cfg.width;
    const uint16_t H = cfg.height;
    const size_t   packed_size = (size_t)W * H / 4;

    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_SPIRAM);
    if (!packed) {
        printf("[BOOT] fractal: PSRAM alloc failed (%u bytes)\n", (unsigned)packed_size);
        return nullptr;
    }

    // Multi-scale XOR fractal — combines 4-pixel, 16-pixel and 64-pixel octaves.
    // Each octave is the low 2 bits of (px_scaled XOR py_scaled).  Summing them
    // creates self-similar structure visible at all zoom levels on the display,
    // avoiding the sub-pixel averaging that makes a single-scale XOR look grey.
    //
    // DC balance: each grey level appears ~25 % of pixels (W=960 is 15×64).
    for (int py = 0; py < H; py++) {
        uint8_t* row   = packed + py * (W / 4);
        // Hoist the y-only terms out of the inner loop.
        uint8_t  yterm = (uint8_t)((py >> 2) ^ (py >> 4) ^ (py >> 6));
        for (int px = 0; px < W; px += 4) {
            auto pixel = [yterm](int x) -> uint8_t {
                return ((x >> 2) ^ (x >> 4) ^ (x >> 6) ^ yterm) & 3;
            };
            row[px >> 2] = (uint8_t)(
                pixel(px    ) << 6 |
                pixel(px + 1) << 4 |
                pixel(px + 2) << 2 |
                pixel(px + 3)
            );
        }
    }
    return packed;
}

void EPD_BootCtl::_paintFractal() {
    uint8_t* packed = _allocFractal();
    if (!packed) return;
    _epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    _epd.clear();
    _epd.paintPacked(packed);
    heap_caps_free(packed);
}

void EPD_BootCtl::_unpaintFractal() {
    uint8_t* packed = _allocFractal();
    if (!packed) return;
    _epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    _epd.unpaintPacked(packed);
    _epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
    heap_caps_free(packed);
}

// =============================================================================
// Power off
// =============================================================================

void EPD_BootCtl::_powerOff() {
    const EPD_Painter::Config& cfg = _epd.getConfig();

    if (cfg.pin_syspwr >= 0) {
        // M5PaperS3: a HIGH→LOW pulse on pin_syspwr cuts the system rail.
        pinMode(cfg.pin_syspwr, OUTPUT);
        digitalWrite(cfg.pin_syspwr, HIGH);
        delay(100);
        digitalWrite(cfg.pin_syspwr, LOW);
        printf("[BOOT] pin_syspwr pulsed. If still running, USB is keeping device alive.\n");
    } else {
#ifdef ARDUINO
        // LilyGo T5 S3 GPS: disable the BQ25896 battery FET (reg 0x09 bit 5)
        // to disconnect the battery and cut power to the whole board.
        TwoWire* wire = cfg.i2c.wire;
        if (wire) {
            const uint8_t BQ_ADDR = 0x6B;
            wire->beginTransmission(BQ_ADDR);
            wire->write(0x09);
            wire->endTransmission();
            wire->requestFrom(BQ_ADDR, (uint8_t)1);
            uint8_t reg = wire->available() ? wire->read() : 0;
            wire->beginTransmission(BQ_ADDR);
            wire->write(0x09);
            wire->write(reg | (1 << 5));  // BATFET_DIS
            wire->endTransmission();
            printf("[BOOT] BQ25896 BATFET disabled. If still running, USB is keeping device alive.\n");
        }
#endif
    }

    // Fall through to deep sleep — covers the USB-alive case and ensures
    // the [[noreturn]] contract is met.
    esp_deep_sleep_start();
    while (true) {}
}

// =============================================================================
// USB detection — BQ25896 reg 0x0B (LilyGo only; returns false on M5PaperS3)
// =============================================================================

bool EPD_BootCtl::_isUsbConnected() const {
#ifdef ARDUINO
    const EPD_Painter::Config& cfg = _epd.getConfig();
    TwoWire* wire = cfg.i2c.wire;
    if (!wire) return false;
    const uint8_t BQ_ADDR = 0x6B;
    wire->beginTransmission(BQ_ADDR);
    wire->write(0x0B);
    if (wire->endTransmission(false) != 0) return false;
    if (wire->requestFrom(BQ_ADDR, (uint8_t)1) == 0) return false;
    uint8_t reg = wire->read();
    return (reg & 0xF8) != 0;  // VBUS_GD (bit 7) or VBUS_STAT (bits 6:3)
#else
    return false;
#endif
}

// =============================================================================
// NVS flag
// =============================================================================

static void _nvsInit() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
}

bool EPD_BootCtl::_readFlag() const {
    _nvsInit();
    nvs_handle_t h;
    uint8_t val = 0;  // default false → shutdown on very first boot
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u8(h, NVS_KEY, &val);
        nvs_close(h);
    }
    return val != 0;
}

void EPD_BootCtl::_writeFlag(bool v) const {
    _nvsInit();
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_u8(h, NVS_KEY, v ? 1 : 0);
        nvs_commit(h);
        nvs_close(h);
    }
}
