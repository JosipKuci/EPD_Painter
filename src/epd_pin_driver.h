#pragma once
#include <stdint.h>
#include <soc/gpio_reg.h>

#ifndef IRAM_ATTR
  #define IRAM_ATTR
#endif

// =============================================================================
// EPD_PinDriver — abstract interface for a single EPD control pin.
// =============================================================================
class EPD_PinDriver {
public:
    virtual void set(bool high) = 0;
    virtual ~EPD_PinDriver() = default;
};

// =============================================================================
// EPD_GpioPin — direct GPIO register write, IRAM-safe.
// =============================================================================
class EPD_GpioPin : public EPD_PinDriver {
public:
    EPD_GpioPin() : _pin(0) {}
    explicit EPD_GpioPin(uint8_t pin) : _pin(pin) {}
    EPD_GpioPin& operator=(const EPD_GpioPin&) = default;

    void set(bool high) override {
        if (high) {
            if (_pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG,  1UL << _pin);
            else           REG_WRITE(GPIO_OUT1_W1TS_REG, 1UL << (_pin - 32));
        } else {
            if (_pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG,  1UL << _pin);
            else           REG_WRITE(GPIO_OUT1_W1TC_REG, 1UL << (_pin - 32));
        }
    }

private:
    uint8_t _pin;
};

// =============================================================================
// EPD_PowerDriver — abstract interface for board-level power sequencing.
// =============================================================================
class EPD_PowerDriver {
public:
    virtual bool powerOn() = 0;
    virtual void powerOff() = 0;
    virtual ~EPD_PowerDriver() = default;
};
