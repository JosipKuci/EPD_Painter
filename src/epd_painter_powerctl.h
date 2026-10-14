#pragma once

#include "build_opt.h"
#include <EPD_Painter.h>
#include "epd_pin_driver.h"

// =============================================================================
// epd_painter_powerctl — Inkplate 10 panel power: TPS65186 PMIC behind a
// PCAL6416A I/O expander, with the panel's GMOD and OE lines on native GPIOs.
//
//   expander pin   signal     direction
//   A0 (0)         WAKEUP     output   TPS65186 only answers on I2C while high
//   A1 (1)         PWRUP      output   starts the hardware power-up sequencer
//   A2 (2)         VCOM       output   asserted only once every rail is good
//   A3 (3)         TPS INT    input    open drain, needs the expander pull-up
//   A4 (4)         PWR_GOOD   input    open drain, needs the expander pull-up
//
// Pin numbers and both I2C addresses come from Config::power, so a board
// spin that moves them is a preset change, not a driver change.
//
// The TPS65186 V3P3 switch (ENABLE bit 5) is what powers the panel's logic,
// so every panel control line must be low before it goes off — anything
// left high back-powers the source drivers through their ESD diodes.
// EPD_Painter::powerOff() drives the scan lines low before calling down
// here; this driver owns GMOD and OE.
// =============================================================================
class epd_painter_powerctl : public EPD_PowerDriver {
public:
  bool begin(const EPD_Painter::Config &config);

  bool powerOn() override;
  void powerOff() override;

  // PMIC status register 0x0F; 0xFA = every rail good.
  uint8_t readPowerGood();
  // Hardware PWR_GOOD line as seen on the expander.
  bool isPwrGoodPin();
  // Panel temperature from the TPS65186 thermistor input, in degrees C.
  int8_t readTemperature();
  // VCOM currently held by the PMIC (factory programmed into its EEPROM),
  // in millivolts, negative.
  int readVcomMv();
  // See EPD_Painter::setVcomMv().
  bool setVcomMv(int mv, bool persist);

private:
  EPD_Painter::Config _cfg;

  // Serializes powerOn/powerOff. The paint task powers the panel on from its
  // own core while the idle task may concurrently power it down;
  // unserialized, a power-off can put the TPS back to sleep mid-power-on and
  // the rails never come up.
  SemaphoreHandle_t _mtx = nullptr;
  bool _on = false;

  uint8_t _exp_out[2] = { 0x00, 0x00 };
  uint8_t _exp_cfg[2] = { 0xFF, 0xFF };

  // ---- PCAL6416A registers ----
  static constexpr uint8_t EXP_INPUT0   = 0x00;
  static constexpr uint8_t EXP_OUTPUT0  = 0x02;
  static constexpr uint8_t EXP_CONFIG0  = 0x06;
  static constexpr uint8_t EXP_PUPDEN0  = 0x46;
  static constexpr uint8_t EXP_PUPDSEL0 = 0x48;

  // ---- TPS65186 registers ----
  static constexpr uint8_t TPS_TEMP    = 0x00;
  static constexpr uint8_t TPS_ENABLE  = 0x01;
  static constexpr uint8_t TPS_VCOM1   = 0x03;
  static constexpr uint8_t TPS_VCOM2   = 0x04;
  static constexpr uint8_t TPS_INT1    = 0x07;
  static constexpr uint8_t TPS_UPSEQ0  = 0x09;
  static constexpr uint8_t TPS_DWNSEQ0 = 0x0B;
  static constexpr uint8_t TPS_TMST1   = 0x0D;
  static constexpr uint8_t TPS_PG      = 0x0F;
  static constexpr uint8_t TPS_PG_OK   = 0xFA;

  int16_t _vcom_mv = 0;   // 0 = PMIC EEPROM value
  bool writeVcomLocked(int mv, bool program);

  bool powerOnLocked();
  void powerOffLocked();

  bool regWrite(int addr, uint8_t reg, const uint8_t *data, int n);
  bool regRead(int addr, uint8_t reg, uint8_t &val);

  bool expWrite(int8_t pin, bool val);
  bool expRead(int8_t pin, bool &val);
  bool expPinMode(int8_t pin, bool input, bool pullup);

  bool tpsWrite(uint8_t reg, uint8_t val) { return regWrite(_cfg.power.tps_addr, reg, &val, 1); }
  bool tpsRead(uint8_t reg, uint8_t &val) { return regRead(_cfg.power.tps_addr, reg, val); }
};
