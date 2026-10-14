// Inkplate 10 fast counter.
//
// A counter in the middle of the screen, repainted as fast as the panel
// allows. Only the rows under the counter change, so the row window
// (Config::skip_idle_rows) scans ~120 of the 825 rows per pass, and FAST
// quality drives 7 passes with no settle gap.
//
// Build:
//   arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app,USBMode=hwcdc,CDCOnBoot=cdc
//
// Serial prints the time per update. Set PROFILE to 1 for the per-pass
// breakdown (rows scanned, row-loop time, settle time).
//
// FAST is black/white quality: each update leaves a faint ghost of the old
// digits. CLEAR_EVERY does a full-screen clear now and then to wipe it.

#include <EPD_Painter_Adafruit.h>
#include <Fonts/FreeSansBold24pt7b.h>

#define PROFILE      0
#define CLEAR_EVERY  200   // full clear after this many updates (0 = never)

// Everything tuned for speed. Start from the board preset and change only
// what matters here.
static EPD_Painter::Config fastConfig() {
  EPD_Painter::Config c = EPD_PAINTER_PRESET;
  c.pclk_hz        = 10000000;  // proven on the breakout; try 20000000 if clean
  c.skip_idle_rows = true;      // scan only the rows that change (default)
  c.skip_idle_cols = true;      // experimental: set false if stray marks appear
  // Pass floor for small windows. Pass length is the dose: lower = faster
  // but the ink may not fully reach black/white. 4 ms is aggressive; raise
  // it toward 6000 if digits come out grey or leave strong ghosts.
  c.min_pass_us    = 4000;
  c.quality        = EPD_Painter::Quality::QUALITY_FAST;  // 7 passes, no gap
  return c;
}

EPD_PainterAdafruit epd(fastConfig());

// Counter box, centred. Big enough for 6 digits at text size 3.
static const int BOX_W = 700, BOX_H = 200;
static int boxX, boxY;

static void drawCounter(uint32_t n) {
  epd.fillRect(boxX, boxY, BOX_W, BOX_H, 0);       // white box

  char buf[12];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)n);
  int16_t bx, by;
  uint16_t bw, bh;
  epd.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
  epd.setCursor(boxX + (BOX_W - bw) / 2 - bx, boxY + (BOX_H - bh) / 2 - by);
  epd.print(buf);
}

static void fullRedraw(uint32_t n) {
  epd.clear();                                     // wipe ghosts on the glass
  epd.fillScreen(0);
  epd.drawRect(boxX - 4, boxY - 4, BOX_W + 8, BOX_H + 8, 3);
  drawCounter(n);
  epd.paint();
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!epd.begin()) {
    Serial.println("epd.begin() failed - see the log above");
    while (true) delay(1000);
  }
  // Keep the PMIC up between updates: a power-up costs tens of ms.
  epd.driver().setIdleTimeout(30);
  epd.driver().setPaintProfile(PROFILE);

  boxX = (epd.width() - BOX_W) / 2;
  boxY = (epd.height() - BOX_H) / 2;

  epd.setFont(&FreeSansBold24pt7b);
  epd.setTextSize(3);
  epd.setTextColor(3);

  fullRedraw(0);
}

void loop() {
  static uint32_t n = 0;
  n++;

  if (CLEAR_EVERY && n % CLEAR_EVERY == 0) {
    fullRedraw(n);
    return;
  }

  drawCounter(n);

  const int64_t t0 = esp_timer_get_time();
  epd.paint();                                     // returns once picked up
  while (!epd.driver().paintIdle()) delay(1);      // wait for the passes
  const int64_t ms = (esp_timer_get_time() - t0) / 1000;

  static int64_t sum = 0;
  sum += ms;
  if (n % 10 == 0) {
    Serial.printf("update %lu: %lld ms (avg %lld ms over last 10)\n",
                  (unsigned long)n, ms, sum / 10);
    sum = 0;
  }
}
