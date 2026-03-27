#pragma once

#include "build_opt.h"
#include "EPD_Painter.h"

// =============================================================================
// EPD_BootCtl — reset-toggle power management with fractal shutdown image.
//
// Every reset acts as a power toggle:
//   flag=false (device was running)  → paint fractal, power off, deep sleep.
//   flag=true  (device was sleeping) → DC-balance unpaint, continue startup.
//
// On the very first boot after flashing (NVS key absent, defaults false) the
// device shuts down immediately. The next reset wakes it into normal operation.
// From that point the reset button cleanly toggles on/off.
//
// Call AFTER epd.begin() so the panel and DMA are ready to paint.
//
// Usage:
//   epd.begin();
//   EPD_BootCtl boot(epd);   // never returns on the shutdown path
//   // normal startup code here
//
// For an explicit programmatic shutdown (low battery, long press, etc.):
//   boot.shutdown();          // never returns
//
// Shutdown image: XOR fractal — (x ^ y) & 3, generated in ~1 ms.
// Stored in NVS namespace "epdboot", key "flag".
//
// Board-specific power-off:
//   pin_syspwr >= 0  (M5PaperS3)       — HIGH→LOW pulse on that GPIO.
//   pin_syspwr == -1 (LilyGo T5 S3)    — BQ25896 BATFET disable over I2C.
// =============================================================================
class EPD_BootCtl {
public:
    explicit EPD_BootCtl(EPD_Painter& epd);

    // Explicit clean shutdown regardless of flag state.  Never returns.
    [[noreturn]] void shutdown();

private:
    EPD_Painter& _epd;

    static constexpr const char* NVS_NS  = "epdboot";
    static constexpr const char* NVS_KEY = "flag";

    // Allocate a PSRAM-backed packed 2bpp XOR-fractal buffer.
    // Caller must heap_caps_free() it.
    uint8_t* _allocFractal() const;

    void _paintFractal();
    void _unpaintFractal();

    bool _isUsbConnected() const;

    [[noreturn]] void _powerOff();

    bool _readFlag() const;
    void _writeFlag(bool val) const;
};
