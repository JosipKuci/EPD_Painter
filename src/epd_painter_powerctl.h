#pragma once

#include "build_opt.h"
#include <EPD_Painter.h>
#include "epd_pin_driver.h"

class epd_painter_powerctl : public EPD_PowerDriver {
public:
  epd_painter_powerctl();

  bool begin(EPD_Painter::Config config);

  bool powerOn();
  void powerOff();

  bool isPwrGood();
  uint8_t readTpsPg();
  uint8_t readPcaPort(uint8_t port);

  void setVcomMv(int vcom_mv);

private:
  EPD_Painter::Config config;

  // ---- PCA9535 cached state ----
  uint8_t _pca_out[2];
  uint8_t _pca_cfg[2];

  // ---- PCA9535 logical pin mapping ----
  static constexpr uint8_t PIN_OE      = 8;   // port1 bit0
  static constexpr uint8_t PIN_MODE    = 9;   // port1 bit1
  static constexpr uint8_t PIN_PWRUP   = 11;  // port1 bit3
  static constexpr uint8_t PIN_VCOM    = 12;  // port1 bit4
  static constexpr uint8_t PIN_WAKEUP  = 13;  // port1 bit5
  static constexpr uint8_t PIN_PWRGOOD = 14;  // port1 bit6 input
  static constexpr uint8_t PIN_INT     = 15;  // port1 bit7 input

  // ---- PCA pin direction constants (I2C expander, not real GPIO) ----
  static constexpr uint8_t PCA_OUTPUT = 0;
  static constexpr uint8_t PCA_INPUT  = 1;

  // ---- TPS65185 registers ----
  static constexpr uint8_t TPS_ENABLE = 0x01;
  static constexpr uint8_t TPS_VCOM1  = 0x03;
  static constexpr uint8_t TPS_VCOM2  = 0x04;
  static constexpr uint8_t TPS_UPSEQ0 = 0x09;
  static constexpr uint8_t TPS_UPSEQ1 = 0x0A;
  static constexpr uint8_t TPS_PG     = 0x0F;

  // ---- PCA low-level ----
  bool pcaWriteReg(uint8_t reg, uint8_t val);
  bool pcaReadReg(uint8_t reg, uint8_t& val);
  bool pcaPinMode(uint8_t pin, uint8_t mode);
  bool pcaWrite(uint8_t pin, bool val);
  bool pcaRead(uint8_t pin, bool& val);

  // ---- TPS low-level ----
  bool tpsWrite(uint8_t reg, uint8_t val);
  bool tpsWrite16(uint8_t reg, uint8_t lo, uint8_t hi);
  bool tpsRead(uint8_t reg, uint8_t& val);
};

// =============================================================================
// EPD_GpioPowerDriver — direct GPIO power control (e.g. M5PaperS3).
// Used when there is no PMIC or shift register managing the panel rails.
// =============================================================================
class EPD_GpioPowerDriver : public EPD_PowerDriver {
public:
    EPD_GpioPowerDriver(int8_t pin_oe, int8_t pin_pwr)
        : _pin_oe(pin_oe), _pin_pwr(pin_pwr) {}

    bool powerOn() override {
        EPD_PIN_HIGH(_pin_oe);
        EPD_DELAY_US(100);
        EPD_PIN_HIGH(_pin_pwr);
        EPD_DELAY_US(100);
        return true;
    }

    void powerOff() override {
        EPD_PIN_LOW(_pin_oe);
        EPD_DELAY_US(100);
        EPD_PIN_LOW(_pin_pwr);
        EPD_DELAY_US(100);
    }

private:
    int8_t _pin_oe;
    int8_t _pin_pwr;
};

class epd_painter_powerctl_74HCT4094D : public EPD_PowerDriver {
public:
  epd_painter_powerctl_74HCT4094D();

  bool begin(EPD_Painter::Config& config);

  bool powerOn();
  void powerOff();

  void IRAM_ATTR sr_set_bit(uint8_t index, bool val);

private:
  EPD_Painter::Config* config;

  struct ShiftState {
    bool ep_latch_enable   = false; // QP0 -> EP_LE
    bool q1_unused         = false; // QP1 -> not used by the driver
    bool q2_unused         = false; // QP2 -> not used by the driver
    bool q3_unused         = false; // QP3 -> not used by the driver
    bool ep_stv            = false; // QP4 -> EP_STV / SPV
    bool power_enable      = false; // QP5 -> PWR_EN
    bool ep_mode           = false; // QP6 -> EP_MODE
    bool ep_output_enable  = false; // QP7 -> EP_OE
  } _sr;

  void sr_push_bits();
};
