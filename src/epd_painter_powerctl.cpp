#include "epd_painter_powerctl.h"
#include <stdio.h>

#ifndef ARDUINO
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
#endif

// Routine power-cycle trace. With idle power-off (default 5s) powerOn/powerOff
// run constantly, so the per-cycle messages are compiled out unless
// EPD_DEBUG_REGISTERS is defined. Errors always print.
//#define EPD_DEBUG_REGISTERS
#ifdef EPD_DEBUG_REGISTERS
#define EPD_DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define EPD_DEBUG_PRINT(...)
#endif

bool epd_painter_powerctl::begin(const EPD_Painter::Config &cfg) {
  _cfg = cfg;
  _vcom_mv = cfg.power.vcom_mv;
  if (!_mtx) _mtx = xSemaphoreCreateMutex();

  // Panel GMOD / OE start low: the rails are off.
  if (_cfg.pin_gmod >= 0) { EPD_PIN_OUTPUT(_cfg.pin_gmod); EPD_PIN_LOW(_cfg.pin_gmod); }
  if (_cfg.pin_oe   >= 0) { EPD_PIN_OUTPUT(_cfg.pin_oe);   EPD_PIN_LOW(_cfg.pin_oe);   }

  // Outputs low before they are turned into outputs — the expander's output
  // register resets to 0xFF, so flipping direction first would pulse WAKEUP,
  // PWRUP and VCOM high for a moment.
  const auto &p = _cfg.power;
  bool ok = expWrite(p.pin_wakeup, false) && expWrite(p.pin_pwrup, false) &&
            expWrite(p.pin_vcom, false) &&
            expPinMode(p.pin_wakeup, false, false) &&
            expPinMode(p.pin_pwrup,  false, false) &&
            expPinMode(p.pin_vcom,   false, false);
  // PWR_GOOD and INT are open-drain outputs of the TPS65186: without the
  // expander pull-up they read 0 forever.
  if (ok && p.pin_pwrgood >= 0) ok = expPinMode(p.pin_pwrgood, true, true);
  if (ok && p.pin_int >= 0)     ok = expPinMode(p.pin_int, true, true);
  if (!ok) {
    printf("[PWRCTL] PCAL6416A not answering at 0x%02X\n", p.expander_addr);
    return false;
  }

  // Program the sequencers once, the way the Inkplate driver does: WAKEUP
  // is held only for the write, the chip sleeps until the first powerOn().
  expWrite(p.pin_wakeup, true);
  EPD_DELAY_MS(2);
  const uint8_t seq[4] = { 0x1B, 0x00, 0x1B, 0x00 };  // UPSEQ0/1, DWNSEQ0/1
  ok = regWrite(p.tps_addr, TPS_UPSEQ0, seq, 4);
  uint8_t v1 = 0, v2 = 0;
  tpsRead(TPS_VCOM1, v1);
  tpsRead(TPS_VCOM2, v2);
  expWrite(p.pin_wakeup, false);
  if (!ok) {
    printf("[PWRCTL] TPS65186 not answering at 0x%02X\n", p.tps_addr);
    return false;
  }
  printf("[PWRCTL] TPS65186 OK, VCOM = %d mV\n",
         -((((int)(v2 & 0x01) << 8) | v1) * 10));
  if (_vcom_mv) printf("[PWRCTL] VCOM override %d mV (applied at every power-up)\n", _vcom_mv);
  return true;
}

bool epd_painter_powerctl::powerOn() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  bool ok = _on || powerOnLocked();
  if (_mtx) xSemaphoreGive(_mtx);
  return ok;
}

bool epd_painter_powerctl::powerOnLocked() {
  EPD_DEBUG_PRINT("[PWRCTL] Power-on sequence...\n");
  const auto &p = _cfg.power;

  // Gate driver mode on before the rails, OE only once they are good —
  // the Inkplate einkOn() order.
  if (_cfg.pin_gmod >= 0) EPD_PIN_HIGH(_cfg.pin_gmod);

  if (!expWrite(p.pin_wakeup, true)) return false;
  EPD_DELAY_MS(5);

  // Sleep reloads VCOM from the PMIC EEPROM, so an override goes back in on
  // every wake, before the rails come up.
  if (_vcom_mv && !writeVcomLocked(_vcom_mv, false)) return false;

  // V3P3 switch on (panel logic supply); the rails themselves are brought up
  // by the PWRUP pin through the sequencer.
  if (!tpsWrite(TPS_ENABLE, 0x20))   return false;
  if (!tpsWrite(TPS_UPSEQ0, 0xE4))   return false;
  if (!tpsWrite(TPS_DWNSEQ0, 0x1B))  return false;
  if (!expWrite(p.pin_pwrup, true))  return false;

  int timeout = 0;
  uint8_t pg = 0;
  while (timeout < 250) {
    EPD_DELAY_MS(1);
    ++timeout;
    if (tpsRead(TPS_PG, pg) && pg == TPS_PG_OK) break;
  }
  if (pg != TPS_PG_OK) {
    bool pin = false;
    expRead(p.pin_pwrgood, pin);
    printf("[PWRCTL] TPS65186 power-up failed: PG=0x%02X (want 0x%02X), PWR_GOOD pin=%d\n",
           pg, TPS_PG_OK, (int)pin);
    powerOffLocked();
    return false;
  }
  EPD_DEBUG_PRINT("[PWRCTL] rails good after %d ms\n", timeout);

  if (!expWrite(p.pin_vcom, true)) return false;
  if (_cfg.pin_oe >= 0) EPD_PIN_HIGH(_cfg.pin_oe);
  _on = true;
  return true;
}

void epd_painter_powerctl::powerOff() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  powerOffLocked();
  if (_mtx) xSemaphoreGive(_mtx);
}

void epd_painter_powerctl::powerOffLocked() {
  EPD_DEBUG_PRINT("[PWRCTL] Power-off...\n");
  const auto &p = _cfg.power;

  if (_cfg.pin_oe   >= 0) EPD_PIN_LOW(_cfg.pin_oe);
  if (_cfg.pin_gmod >= 0) EPD_PIN_LOW(_cfg.pin_gmod);

  expWrite(p.pin_vcom, false);
  expWrite(p.pin_pwrup, false);

  // Let the down-sequencer collapse every rail before the chip sleeps.
  uint8_t pg = 0xFF;
  for (int t = 0; t < 250; t++) {
    if (tpsRead(TPS_PG, pg) && pg == 0) break;
    EPD_DELAY_MS(1);
  }
  // V3P3 off while the chip is still awake to hear it.
  tpsWrite(TPS_ENABLE, 0x00);
  expWrite(p.pin_wakeup, false);
  _on = false;
}

uint8_t epd_painter_powerctl::readPowerGood() {
  uint8_t v = 0;
  tpsRead(TPS_PG, v);
  return v;
}

bool epd_painter_powerctl::isPwrGoodPin() {
  bool v = false;
  expRead(_cfg.power.pin_pwrgood, v);
  return v;
}

int8_t epd_painter_powerctl::readTemperature() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  const auto &p = _cfg.power;
  const bool wake = !_on;
  if (wake) { expWrite(p.pin_wakeup, true); EPD_DELAY_MS(5); }
  tpsWrite(TPS_TMST1, 0x80);  // start a conversion
  EPD_DELAY_MS(5);
  uint8_t t = 0;
  tpsRead(TPS_TEMP, t);
  if (wake) expWrite(p.pin_wakeup, false);
  if (_mtx) xSemaphoreGive(_mtx);
  return (int8_t)t;
}

int epd_painter_powerctl::readVcomMv() {
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  const bool wake = !_on;
  if (wake) { expWrite(_cfg.power.pin_wakeup, true); EPD_DELAY_MS(5); }
  uint8_t v1 = 0, v2 = 0;
  tpsRead(TPS_VCOM1, v1);
  tpsRead(TPS_VCOM2, v2);
  if (wake) expWrite(_cfg.power.pin_wakeup, false);
  if (_mtx) xSemaphoreGive(_mtx);
  return -((((int)(v2 & 0x01) << 8) | v1) * 10);
}

// VCOM1 (0x03) holds bits 7..0 of |VCOM| / 10 mV, VCOM2 (0x04) bit 0 holds
// bit 8 and bit 6 (PROG) burns both into the EEPROM. Chip must be awake.
bool epd_painter_powerctl::writeVcomLocked(int mv, bool program) {
  const int raw = ((-mv) + 5) / 10 & 0x1FF;
  uint8_t r4 = 0;
  if (!tpsWrite(TPS_VCOM1, (uint8_t)(raw & 0xFF))) return false;
  if (!tpsRead(TPS_VCOM2, r4)) return false;
  r4 = (uint8_t)((r4 & ~((1u << 0) | (1u << 6))) | ((raw >> 8) & 1));
  if (!tpsWrite(TPS_VCOM2, r4)) return false;
  if (program) {
    EPD_DELAY_MS(1);
    if (!tpsWrite(TPS_VCOM2, (uint8_t)(r4 | (1u << 6)))) return false;
    // INT (open drain, active low) falls when programming completes; with
    // no INT line, the datasheet's programming time is well under 100 ms.
    bool intHigh = true;
    if (_cfg.power.pin_int >= 0) {
      for (int t = 0; t < 1000 && expRead(_cfg.power.pin_int, intHigh) && intHigh; t++)
        EPD_DELAY_MS(1);
    } else {
      EPD_DELAY_MS(100);
    }
    uint8_t int1 = 0;
    tpsRead(TPS_INT1, int1);   // reading clears the flag
  }
  uint8_t v1 = 0, v2 = 0;
  if (!tpsRead(TPS_VCOM1, v1) || !tpsRead(TPS_VCOM2, v2)) return false;
  return ((((int)(v2 & 1)) << 8) | v1) == raw;
}

bool epd_painter_powerctl::setVcomMv(int mv, bool persist) {
  if (mv > -10 || mv < -5110) {
    printf("[PWRCTL] VCOM %d mV out of range (-5110..-10)\n", mv);
    return false;
  }
  if (_mtx) xSemaphoreTake(_mtx, portMAX_DELAY);
  bool ok;
  if (persist) {
    // Program with the rails up, as Inkplate's writeVCOMToPanelEEPROM().
    const bool was_on = _on;
    _vcom_mv = (int16_t)mv;
    ok = was_on || powerOnLocked();
    if (ok) ok = writeVcomLocked(mv, true);
    if (!was_on) powerOffLocked();
  } else {
    const bool wake = !_on;
    if (wake) { expWrite(_cfg.power.pin_wakeup, true); EPD_DELAY_MS(5); }
    ok = writeVcomLocked(mv, false);
    if (wake) expWrite(_cfg.power.pin_wakeup, false);
    if (ok) _vcom_mv = (int16_t)mv;
  }
  if (_mtx) xSemaphoreGive(_mtx);
  printf("[PWRCTL] VCOM %d mV %s%s\n", mv, persist ? "programmed to EEPROM" : "set (RAM)",
         ok ? "" : " FAILED");
  return ok;
}

// ---- PCAL6416A ----

bool epd_painter_powerctl::expWrite(int8_t pin, bool val) {
  if (pin < 0 || pin > 15) return pin < 0;
  const int port = pin >> 3, bit = pin & 7;
  if (val) _exp_out[port] |=  (uint8_t)(1u << bit);
  else     _exp_out[port] &= (uint8_t)~(1u << bit);
  return regWrite(_cfg.power.expander_addr, EXP_OUTPUT0 + port, &_exp_out[port], 1);
}

bool epd_painter_powerctl::expRead(int8_t pin, bool &val) {
  if (pin < 0 || pin > 15) return false;
  uint8_t r = 0;
  if (!regRead(_cfg.power.expander_addr, EXP_INPUT0 + (pin >> 3), r)) return false;
  val = (r >> (pin & 7)) & 1;
  return true;
}

bool epd_painter_powerctl::expPinMode(int8_t pin, bool input, bool pullup) {
  if (pin < 0 || pin > 15) return pin < 0;
  const int port = pin >> 3;
  const uint8_t m = (uint8_t)(1u << (pin & 7));
  const int a = _cfg.power.expander_addr;
  if (pullup) {
    uint8_t en = 0, sel = 0;
    if (!regRead(a, EXP_PUPDEN0 + port, en) || !regRead(a, EXP_PUPDSEL0 + port, sel))
      return false;
    sel |= m;  // pull-up, not pull-down
    en  |= m;
    if (!regWrite(a, EXP_PUPDSEL0 + port, &sel, 1) || !regWrite(a, EXP_PUPDEN0 + port, &en, 1))
      return false;
  }
  if (input) _exp_cfg[port] |= m;
  else       _exp_cfg[port] &= (uint8_t)~m;
  return regWrite(a, EXP_CONFIG0 + port, &_exp_cfg[port], 1);
}

// ---- I2C ----

bool epd_painter_powerctl::regWrite(int addr, uint8_t reg, const uint8_t *data, int n) {
#ifdef ARDUINO
  TwoWire *w = _cfg.i2c.wire;
  if (!w) return false;
  w->beginTransmission(addr);
  w->write(reg);
  for (int i = 0; i < n; i++) w->write(data[i]);
  return w->endTransmission() == 0;
#else
  return false;  // I2C not used in ESP-IDF builds
#endif
}

bool epd_painter_powerctl::regRead(int addr, uint8_t reg, uint8_t &val) {
#ifdef ARDUINO
  TwoWire *w = _cfg.i2c.wire;
  if (!w) return false;
  w->beginTransmission(addr);
  w->write(reg);
  if (w->endTransmission(false) != 0) return false;
  if (w->requestFrom(addr, 1) != 1 || !w->available()) return false;
  val = w->read();
  return true;
#else
  return false;  // I2C not used in ESP-IDF builds
#endif
}
