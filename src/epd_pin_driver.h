#pragma once
#include <stdint.h>
#include <soc/gpio_reg.h>

// =============================================================================
// EPD_PinDriver — abstract interface for a single EPD control pin.
//
// Implementations route set() to either a direct GPIO register write
// (EPD_GpioPin) or through the 74HCT4094D shift register (EPD_SRPin).
// The driver core never needs to know which hardware a pin lives on.
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
    explicit EPD_GpioPin(uint8_t pin) : _pin(pin) {}

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
// EPD_ISRController — minimal interface for shift-register bit manipulation.
// Both board-specific SR power drivers implement this so EPD_SRPin can work
// with any SR-based board without knowing the concrete power driver type.
// =============================================================================
class EPD_ISRController {
public:
    virtual void sr_set_bit(uint8_t index, bool val) = 0;
    virtual ~EPD_ISRController() = default;
};

// =============================================================================
// EPD_SRPin — shift-register output via any EPD_ISRController.
// =============================================================================
class EPD_SRPin : public EPD_PinDriver {
public:
    EPD_SRPin(EPD_ISRController* sr, uint8_t index)
        : _sr(sr), _index(index) {}

    void set(bool high) override {
        _sr->sr_set_bit(_index, high);
    }

private:
    EPD_ISRController* _sr;
    uint8_t _index;
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
