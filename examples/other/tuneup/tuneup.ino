// ============================================================================
// EPD_Painter TUNEUP — the self-tuning rig.
//
// Any user with a network scanner can lay their board face-down on the
// glass and let it tune its own waveform trains — the same closed optical
// loop used to tune the shipped presets, with the BOARD driving the
// scanner instead of a PC.
//
//   1. WiFi   — credentials live in this file (see WIFI_SSID below). If
//               they're still blank the panel tells you to set them and
//               reflash.
//   2. Scanner — discovers eSCL/AirScan scanners via mDNS (_uscan._tcp)
//               and lists them on the panel; tap one to pick it (the
//               choice is remembered).
//   3. Demo scan — requests a grayscale eSCL scan and shows it on the
//               panel in 16 native greys.
//   4. SELF-TUNE — put the board face-down on the glass and it tunes its
//               own 16-grey trains against dithered optical references,
//               saves them to flash (LittleFS), and loads them on every
//               boot from then on. See tuner.h for the method.
//
// eSCL ("AirScan") is the driverless scanning protocol spoken by nearly
// every network scanner made since ~2015, and by sane-airscan bridges for
// older ones. No drivers, just HTTP + XML — well within an ESP32.
//
// Touch: GT911 via https://github.com/tonywestonuk/gt911-arduino
// (same pattern as the maze_3d example). While the board is face-down the
// touchscreen is against the glass, so tuning is driven over serial or by
// the TUNE button's countdown.
//
// Serial commands (115200): s demo scan · T self-tune · q abort tune ·
// P <us> tune-period override · L tuned-blob status · X erase tuned blob ·
// n re-pick scanner · r reboot
//
// Libraries: Adafruit GFX (required), gt911-arduino (required),
// JPEGDEC (required for tuning and the scan preview).
// ============================================================================

// ---------------------------------------------------------------------------
// >>> YOUR NETWORK GOES HERE <<<
// ---------------------------------------------------------------------------
#define WIFI_SSID "REDACTED"
#define WIFI_PASS "REDACTED"

// Choose your board (or leave all commented for auto-probe).
//#define EPD_PAINTER_PRESET_M5PAPER_S3
//#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

// Set to 0 if you don't want to install the JPEGDEC library — the demo scan
// still runs (reporting the size only) but SELF-TUNE needs it.
// (An explicit switch, not __has_include: the Arduino build only pulls a
// library into the include path when it sees an unconditional include.)
#define TUNEUP_HAVE_JPEGDEC 1

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include "EPD_Painter_Adafruit.h"
#include "tuned_storage.h"
#include "tuneup_splash.h"
#include <gt911_lite.h>

#if TUNEUP_HAVE_JPEGDEC
#include <JPEGDEC.h>
#endif

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);
Preferences prefs;

// A boxed, centred, tappable label (defined up here so the Arduino
// preprocessor's hoisted prototypes can see the type).
struct Btn { int x, y, w, h; };

// ---- demo scan parameters --------------------------------------------------
// Region measured from the glass origin (the corner arrow on the scanner
// bed). 125 x 72 mm comfortably covers a 960x540 panel laid in the corner.
static const int   SCAN_DPI   = 150;
static const float SCAN_W_MM  = 125.0f;
static const float SCAN_H_MM  = 72.0f;
static const size_t SCAN_BUF_MAX = 2u * 1024u * 1024u;  // PSRAM, plenty for a JPEG

// ---- state -----------------------------------------------------------------
static String   scHost;            // scanner host (IP as text) …
static uint16_t scPort = 0;        // … and port
static String   scName = "?";      // MakeAndModel from its capabilities
static uint8_t *scanBuf = nullptr; // shared download buffer (PSRAM)
static bool     tunedLoaded = false;

// ============================================================================
// EPD console — a dumb teletype on the panel. Every line also goes to serial.
// ============================================================================
static int uiRow = 48;

static void uiClear(const char *title) {
  epd.fillScreen(0);
  epd.fillRect(0, 0, epd.width(), 36, 15);
  epd.setTextWrap(false);
  epd.setTextSize(3);
  epd.setTextColor(0);
  epd.setCursor(12, 7);
  epd.print(title);
  epd.setTextSize(2);
  epd.setTextColor(15);
  uiRow = 48;
  epd.paint();
  Serial.printf("\n===== %s =====\n", title);
}

static void uiLine(const char *fmt, ...) {
  char b[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  Serial.println(b);
  b[78] = 0;  // one panel row at text size 2
  if (uiRow > epd.height() - 18) {           // body full — wipe and restart
    epd.fillRect(0, 40, epd.width(), epd.height() - 40, 0);
    uiRow = 48;
  }
  epd.setCursor(12, uiRow);
  epd.print(b);
  uiRow += 18;
  epd.paint();
}

// ============================================================================
// Touch — GT911, same probing pattern as maze_3d: the display driver owns
// the I2C bus; the coordinate range comes from the controller's own config
// registers (0x8146/8148), swapped axes detected from portrait ranges.
// ============================================================================
static GT911_Lite touch;
static uint16_t tp_xmax = 960, tp_ymax = 540;
static bool     tp_swap = false, touch_ok = false;

static void touchInit() {
  TwoWire *bus = epd.getConfig().i2c.wire;
  if (!bus) return;
  // Never let a wedged bus hang the UI: a stalled I2C read would block the
  // wait loops forever, taking the serial console down with it and leaving
  // the board looking dead.
  bus->setTimeOut(50);
  touch_ok = false;
  touch.begin(bus);
  for (uint8_t addr : { (uint8_t)0x5D, (uint8_t)0x14 }) {
    bus->beginTransmission(addr);
    bus->write(0x81); bus->write(0x46);
    if (bus->endTransmission(false) != 0) continue;
    if (bus->requestFrom(addr, (uint8_t)4) != 4) continue;
    uint16_t xm = bus->read(); xm |= bus->read() << 8;
    uint16_t ym = bus->read(); ym |= bus->read() << 8;
    if (xm == 0 || ym == 0 || xm == 0xFFFF) break;
    tp_xmax = xm; tp_ymax = ym;
    tp_swap = (xm < ym);
    touch_ok = true;
    Serial.printf("[tuneup] touch 0x%02x, range %ux%u%s\n", addr, xm, ym,
                  tp_swap ? " (swapped)" : "");
    return;
  }
  Serial.println("[tuneup] no touch controller found!");
}

// Re-open the touch controller and forget any stuck press state.
//
// After a long tuning run — a quarter of an hour of continuous panel
// driving, with the power controller sharing this same I2C bus — the
// GT911 stops reporting, so every interactive screen that follows looks
// dead to the user. Re-running begin() revives it; clearing the edge flag
// covers the other half of the problem, a press left latched "down" by
// the board resting face-first on the scanner glass, which would stop any
// later tap ever registering as a fresh press.
static bool touch_was_down = false;

// Called before a screen that waits for a tap. It only forgets a stale
// press edge — the board resting face-down on the glass can leave one
// latched, and then no later tap counts as a fresh press.
//
// It deliberately does NOT re-initialise the controller. The dead buttons
// this was written to cure turned out to be a polling-rate problem (see
// the delay() note in tuner.h's wait loops), and re-applying the GT911's
// config mid-session is a risk for no proven gain.
static void touchRewake() {
  touch_was_down = false;
}

// One rising-edge tap, mapped to panel coordinates. Non-blocking.
static bool touchTapped(int &px, int &py) {
  if (!touch_ok) return false;
  bool &was_down = touch_was_down;
  touch.read();
  bool down = touch.isTouched;
  bool tap = down && !was_down;
  was_down = down;
  if (!tap) return false;
  uint16_t rx = touch.x, ry = touch.y;
  if (tp_swap) {
    px = (int)ry * epd.width() / tp_ymax;
    py = (int)(tp_xmax - rx) * epd.height() / tp_xmax;
  } else {
    px = (int)rx * epd.width() / tp_xmax;
    py = (int)ry * epd.height() / tp_ymax;
  }
  return true;
}

static void drawBtn(const Btn &b, const char *l1, const char *l2 = nullptr) {
  epd.drawRect(b.x, b.y, b.w, b.h, 15);
  epd.drawRect(b.x + 1, b.y + 1, b.w - 2, b.h - 2, 15);
  epd.setTextColor(15);
  epd.setTextSize(3);
  int tw = strlen(l1) * 18;
  epd.setCursor(b.x + (b.w - tw) / 2, b.y + (l2 ? 10 : (b.h - 21) / 2));
  epd.print(l1);
  if (l2) {
    epd.setTextSize(2);
    tw = strlen(l2) * 12;
    epd.setCursor(b.x + (b.w - tw) / 2, b.y + b.h - 26);
    epd.print(l2);
  }
  epd.setTextSize(2);
}
static bool hitBtn(const Btn &b, int px, int py) {
  return px >= b.x && px < b.x + b.w && py >= b.y && py < b.y + b.h;
}

// ============================================================================
// WiFi — credentials are compile-time. No keyboard needed anywhere.
// ============================================================================
static void wifiConnect() {
  if (!strlen(WIFI_SSID)) {
    uiClear("WIFI NOT SET");
    uiLine("This board has no keyboard, so the WiFi details live in the");
    uiLine("sketch itself.");
    uiLine("");
    uiLine("Open   examples/other/tuneup/tuneup.ino");
    uiLine("Set    #define WIFI_SSID \"your network\"");
    uiLine("       #define WIFI_PASS \"your password\"");
    uiLine("near the top of the file, then flash it again.");
    for (;;) delay(1000);
  }
  uiClear("WIFI");
  WiFi.mode(WIFI_STA);
  for (int attempt = 1;; attempt++) {
    uiLine("Connecting to \"%s\" (attempt %d) ...", WIFI_SSID, attempt);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (millis() - t0 < 20000) {
      if (WiFi.status() == WL_CONNECTED) {
        uiLine("Connected, IP %s", WiFi.localIP().toString().c_str());
        delay(800);
        return;
      }
      delay(200);
    }
    WiFi.disconnect(true);
    uiLine("No luck - check WIFI_SSID / WIFI_PASS in the sketch. Retrying.");
    delay(2000);
  }
}

// ============================================================================
// Minimal HTTP/1.0 client. HTTP/1.0 on purpose: the server can't chunk the
// reply and closes the connection at the end of the body — so "read until
// close" is the whole framing story, and a blocking scanner (one that scans
// DURING the GET) just looks like a slow body.
// ============================================================================
static bool httpTxn(const char *method, const String &path, const String &body,
                    uint8_t *buf, size_t bufMax, size_t *outLen,
                    int *outStatus, String *outLocation,
                    uint32_t timeoutMs = 120000) {
  *outLen = 0;
  *outStatus = 0;
  WiFiClient c;
  if (!c.connect(scHost.c_str(), scPort, 8000)) return false;
  c.setTimeout(timeoutMs);

  c.printf("%s %s HTTP/1.0\r\nHost: %s:%u\r\nConnection: close\r\nUser-Agent: EPD_Painter-tuneup\r\n",
           method, path.c_str(), scHost.c_str(), scPort);
  if (body.length())
    c.printf("Content-Type: text/xml\r\nContent-Length: %u\r\n", (unsigned)body.length());
  c.print("\r\n");
  if (body.length()) c.print(body);

  String line = c.readStringUntil('\n');           // "HTTP/1.x 200 OK"
  int sp = line.indexOf(' ');
  if (sp < 0) { c.stop(); return false; }
  *outStatus = atoi(line.c_str() + sp + 1);

  for (;;) {                                        // headers
    line = c.readStringUntil('\n');
    line.trim();
    if (!line.length()) break;
    if (outLocation && line.startsWith("Location:")) {
      *outLocation = line.substring(9);
      outLocation->trim();
    }
  }

  size_t n = 0;                                     // body until close
  uint32_t deadline = millis() + timeoutMs;
  while ((c.connected() || c.available()) && millis() < deadline && n < bufMax) {
    int av = c.available();
    if (!av) { delay(5); continue; }
    int r = c.read(buf + n, min((size_t)av, bufMax - n));
    if (r > 0) n += r;
  }
  c.stop();
  *outLen = n;
  return true;
}

// Pull "<anyprefix:Tag>value<" out of an XML blob — good enough for eSCL.
static String xmlTag(const char *xml, const char *tag) {
  String needle = String(":") + tag + ">";
  const char *p = strstr(xml, needle.c_str());
  if (!p) return "";
  p += needle.length();
  const char *e = strchr(p, '<');
  if (!e) return "";
  return String(p).substring(0, e - p);
}

// ============================================================================
// eSCL scan — POST a job for a region (mm from the glass origin), download
// the JPEG into scanBuf, DELETE the job. Returns the byte count (0 = fail).
// ============================================================================
static size_t esclScan(float x0mm, float y0mm, float wmm, float hmm, int dpi) {
  const int xo = (int)(x0mm / 25.4f * 300.0f);       // eSCL units: 1/300 inch
  const int yo = (int)(y0mm / 25.4f * 300.0f);
  const int w300 = (int)(wmm / 25.4f * 300.0f);
  const int h300 = (int)(hmm / 25.4f * 300.0f);
  String xml =
      "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
      "<scan:ScanSettings xmlns:scan=\"http://schemas.hp.com/imaging/escl/2011/05/03\" "
      "xmlns:pwg=\"http://www.pwg.org/schemas/2010/12/sm\">"
      "<pwg:Version>2.0</pwg:Version>"
      "<scan:Intent>Photo</scan:Intent>"
      "<pwg:ScanRegions>"
      "<pwg:ScanRegion>"
      "<pwg:ContentRegionUnits>escl:ThreeHundredthsOfInches</pwg:ContentRegionUnits>"
      "<pwg:XOffset>" + String(xo) + "</pwg:XOffset>"
      "<pwg:YOffset>" + String(yo) + "</pwg:YOffset>"
      "<pwg:Width>" + String(w300) + "</pwg:Width>"
      "<pwg:Height>" + String(h300) + "</pwg:Height>"
      "</pwg:ScanRegion>"
      "</pwg:ScanRegions>"
      "<scan:InputSource>Platen</scan:InputSource>"
      "<scan:ColorMode>Grayscale8</scan:ColorMode>"
      "<scan:XResolution>" + String(dpi) + "</scan:XResolution>"
      "<scan:YResolution>" + String(dpi) + "</scan:YResolution>"
      "<pwg:DocumentFormat>image/jpeg</pwg:DocumentFormat>"
      "</scan:ScanSettings>";

  size_t len; int status; String location;
  if (!httpTxn("POST", "/eSCL/ScanJobs", xml, scanBuf, 4096, &len, &status, &location, 15000))
    return 0;
  if ((status != 201 && status != 200) || !location.length()) {
    Serial.printf("[escl] job refused (HTTP %d)\n", status);
    return 0;
  }
  if (location.startsWith("http")) {              // absolute URL -> path only
    int slash = location.indexOf('/', 8);
    if (slash > 0) location = location.substring(slash);
  }

  // Some scanners block the GET until the page is ready; others answer 503
  // until then. Handle both.
  //
  // Do NOT retry a 404 here. On the MG5200's NAS bridge the GET performs the
  // whole scan inline, and when scanimage finally gives up the bridge marks
  // the job done and answers 503 — so the 503 retry below immediately draws
  // a 404 from the now-dead job. That 404 means "this job is finished",
  // never "not yet", and retrying it can only burn the timeout. Failing fast
  // is correct: the rig's outer retry POSTs a FRESH job, which is the only
  // thing that can succeed once the Canon's BJNP stack has wedged.
  uint32_t t0 = millis();
  for (;;) {
    if (!httpTxn("GET", location + "/NextDocument", "", scanBuf, SCAN_BUF_MAX, &len, &status, nullptr, 120000))
      return 0;
    if (status == 200 && len) break;
    if (status == 503 && millis() - t0 < 120000) { delay(1500); continue; }
    Serial.printf("[escl] no document (HTTP %d)\n", status);
    return 0;
  }
  { // tell the scanner we're done with the job (best effort)
    size_t dl; int ds;
    httpTxn("DELETE", location, "", scanBuf + SCAN_BUF_MAX - 512, 512, &dl, &ds, nullptr, 5000);
  }
  const bool isJpeg = len > 2 && scanBuf[0] == 0xFF && scanBuf[1] == 0xD8;
  Serial.printf("[escl] %u bytes in %lu s (%s)\n", (unsigned)len,
                (millis() - t0) / 1000, isJpeg ? "JPEG" : "not JPEG?");
  return isJpeg ? len : 0;
}

// ============================================================================
// JPEG -> 8-bit grayscale (grayImg/grayW/grayH)
// ============================================================================
#if TUNEUP_HAVE_JPEGDEC
static JPEGDEC  jpegDec;
static uint8_t *grayImg = nullptr;
static int      grayW = 0, grayH = 0;

static int jpegDrawCb(JPEGDRAW *d) {
  const uint8_t *src = (const uint8_t *)d->pPixels;
  for (int y = 0; y < d->iHeight; y++) {
    int dy = d->y + y;
    if (dy >= grayH) break;
    int w = min(d->iWidth, grayW - d->x);
    if (w > 0) memcpy(grayImg + (size_t)dy * grayW + d->x, src + (size_t)y * d->iWidth, w);
  }
  return 1;
}

static bool decodeGrayFromScan(size_t len) {
  if (!jpegDec.openRAM(scanBuf, (int)len, jpegDrawCb)) {
    Serial.printf("[jpeg] open failed (err %d)\n", jpegDec.getLastError());
    return false;
  }
  jpegDec.setPixelType(EIGHT_BIT_GRAYSCALE);
  grayW = jpegDec.getWidth();
  grayH = jpegDec.getHeight();

  // If the scanner ignored our region/resolution and sent something huge,
  // decode at 1/2, 1/4 or 1/8 scale rather than running out of PSRAM.
  if (grayImg) { heap_caps_free(grayImg); grayImg = nullptr; }
  int opt = 0;
  for (int shift = 0; shift <= 3; shift++) {
    const size_t need = ((size_t)grayW >> shift) * ((size_t)grayH >> shift);
    if (need + 65536 < heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)) {
      static const int opts[] = { 0, JPEG_SCALE_HALF, JPEG_SCALE_QUARTER, JPEG_SCALE_EIGHTH };
      opt = opts[shift];
      grayW >>= shift;
      grayH >>= shift;
      break;
    }
    if (shift == 3) {
      Serial.println("[jpeg] out of PSRAM even at 1/8 scale");
      jpegDec.close();
      return false;
    }
  }
  grayImg = (uint8_t *)heap_caps_malloc((size_t)grayW * grayH, MALLOC_CAP_SPIRAM);
  if (!grayImg) { jpegDec.close(); return false; }
  int ok = jpegDec.decode(0, 0, opt);
  jpegDec.close();
  if (!ok) Serial.printf("[jpeg] decode failed (err %d)\n", jpegDec.getLastError());
  return ok;
}

// ---- the "Tune up Complete" artwork ---------------------------------------
// Decoded straight out of flash into the canvas. An ordered dither carries
// the photograph's gradients: at 16 levels a face quantised flat shows
// obvious contour bands across the skin, while the dither's texture is
// finer than the eye resolves at reading distance.
static uint8_t *splashFb = nullptr;
static int      splashFbW = 0, splashFbH = 0;

static const uint8_t splashBayer8[8][8] = {
  {  0, 32,  8, 40,  2, 34, 10, 42 }, { 48, 16, 56, 24, 50, 18, 58, 26 },
  { 12, 44,  4, 36, 14, 46,  6, 38 }, { 60, 28, 52, 20, 62, 30, 54, 22 },
  {  3, 35, 11, 43,  1, 33,  9, 41 }, { 51, 19, 59, 27, 49, 17, 57, 25 },
  { 15, 47,  7, 39, 13, 45,  5, 37 }, { 63, 31, 55, 23, 61, 29, 53, 21 }
};

// The decoder hands back blocks, so gather the whole greyscale image first
// (into the scan buffer, which is idle here) and dither it in one pass.
static uint8_t *splashGray = nullptr;

static int splashDrawCb(JPEGDRAW *d) {
  const uint8_t *src = (const uint8_t *)d->pPixels;
  for (int y = 0; y < d->iHeight; y++) {
    const int dy = d->y + y;
    if (dy < 0 || dy >= splashFbH) continue;
    uint8_t *drow = splashGray + (size_t)dy * splashFbW;
    for (int x = 0; x < d->iWidth; x++) {
      const int dx = d->x + x;
      if (dx >= 0 && dx < splashFbW) drow[dx] = src[(size_t)y * d->iWidth + x];
    }
  }
  return 1;
}

// Floyd-Steinberg to the panel's 16 levels.
//
// A photograph needs error diffusion, not an ordered matrix: at 16 levels
// a Bayer dither leaves its own cross-hatch in smooth areas and still
// contours across skin, whereas diffusion pushes each pixel's rounding
// error into its neighbours and the banding disappears into fine grain.
static void splashDiffuse(uint8_t *fb, int W, int H) {
  // What each level really renders as, measured on the glass by the tuner
  // and kept with the tables. The levels are NOT evenly spaced — on this
  // panel the light end bunches together — so quantising against an
  // assumed even ramp diffuses error into levels that cannot express it,
  // which is precisely what banding looks like. Falls back to even steps
  // on a board that has never been measured.
  const uint8_t *lum = EPD_PainterTuned::levelLuminance();
  uint8_t even[16];
  if (!lum) {
    for (int i = 0; i < 16; i++) even[i] = (uint8_t)(255 - i * 255 / 15);
    lum = even;
  }

  static int16_t errCur[TUNEUP_SPLASH_W];
  static int16_t errNext[TUNEUP_SPLASH_W];
  memset(errCur, 0, sizeof(errCur));
  for (int y = 0; y < H; y++) {
    memset(errNext, 0, sizeof(errNext));
    const uint8_t *srow = splashGray + (size_t)y * W;
    uint8_t *drow = fb + (size_t)y * W;
    for (int x = 0; x < W; x++) {
      int v = (int)srow[x] + errCur[x];
      if (v < 0) v = 0; else if (v > 255) v = 255;
      // nearest level by MEASURED luminance
      int lv = 0, bestd = 1000;
      for (int i = 0; i < 16; i++) {
        const int d = abs((int)lum[i] - v);
        if (d < bestd) { bestd = d; lv = i; }
      }
      const int err = v - (int)lum[lv];            // what we could not show
      drow[x] = (uint8_t)lv;                       // lum[] is already level-indexed
      if (x + 1 < W) errCur[x + 1]  += (int16_t)(err * 7 / 16);
      if (x > 0)     errNext[x - 1] += (int16_t)(err * 3 / 16);
      errNext[x]                    += (int16_t)(err * 5 / 16);
      if (x + 1 < W) errNext[x + 1] += (int16_t)(err * 1 / 16);
    }
    memcpy(errCur, errNext, sizeof(errCur));
  }
}

static void tuneupSplashDraw(uint8_t *fb, int fbW, int fbH) {
  splashFb = fb; splashFbW = fbW; splashFbH = fbH;
  memset(fb, 0, (size_t)fbW * fbH);

  // Borrow the scan buffer for the greyscale image — it is only in use
  // while a scan is in flight, and never during this screen.
  splashGray = scanBuf;
  if (!splashGray) { Serial.println("[splash] no buffer"); return; }
  memset(splashGray, 255, (size_t)fbW * fbH);

  // Reuse the file-scope decoder: a JPEGDEC holds several kilobytes of
  // internal buffers, so declaring one on the stack overflows the loop
  // task (stack canary panic).
  if (!jpegDec.openRAM((uint8_t *)TUNEUP_SPLASH_JPEG, (int)TUNEUP_SPLASH_JPEG_LEN,
                       splashDrawCb)) {
    Serial.printf("[splash] open failed (%d)\n", jpegDec.getLastError());
    return;
  }
  jpegDec.setPixelType(EIGHT_BIT_GRAYSCALE);
  const bool ok = jpegDec.decode(0, 0, 0);
  jpegDec.close();
  if (!ok) {
    Serial.printf("[splash] decode failed (%d)\n", jpegDec.getLastError());
    return;
  }
  splashDiffuse(fb, fbW, fbH);
}

// Show the decoded scan letterboxed on the panel in 16 native greys, with
// a histogram/stats line on serial.
static void showScan() {
  uint32_t hist[16] = {0};
  uint64_t sum = 0;
  const size_t np = (size_t)grayW * grayH;
  for (size_t i = 0; i < np; i++) {
    hist[grayImg[i] >> 4]++;
    sum += grayImg[i];
  }
  Serial.printf("[analysis] %dx%d mean %.1f\n", grayW, grayH, (double)sum / np);
  Serial.print("[analysis] histogram (dark..light): ");
  for (int i = 15; i >= 0; i--) Serial.printf("%lu ", (unsigned long)hist[i]);
  Serial.println();

  const int W = epd.width(), H = epd.height();
  float s = min((float)W / grayW, (float)H / grayH);
  int dw = (int)(grayW * s), dh = (int)(grayH * s);
  int x0 = (W - dw) / 2, y0 = (H - dh) / 2;
  uint8_t *fb = epd.getBuffer();
  memset(fb, 0, (size_t)W * H);
  for (int y = 0; y < dh; y++) {
    const uint8_t *srow = grayImg + (size_t)(int)(y / s) * grayW;
    uint8_t *drow = fb + (size_t)(y0 + y) * W + x0;
    for (int x = 0; x < dw; x++)
      drow[x] = 15 - (srow[(int)(x / s)] >> 4);   // 255 (paper white) -> level 0
  }
  epd.paint();
}
#endif

// ============================================================================
// Scanner discovery + selection (touch, with serial fallback), remembered
// in NVS so a face-down boot needs no interaction.
// ============================================================================
static bool esclHello(const String &host, uint16_t port, String *makeModel) {
  String saveH = scHost; uint16_t saveP = scPort;
  scHost = host; scPort = port;
  size_t len; int status;
  bool ok = httpTxn("GET", "/eSCL/ScannerCapabilities", "", scanBuf, 32768, &len, &status, nullptr, 10000);
  scHost = saveH; scPort = saveP;
  if (!ok || status != 200 || !len) return false;
  scanBuf[min(len, (size_t)32767)] = 0;
  String mm = xmlTag((const char *)scanBuf, "MakeAndModel");
  *makeModel = mm.length() ? mm : String("eSCL scanner");
  return true;
}

static bool scannerTrySaved() {
  String host = prefs.getString("schost", "");
  uint16_t port = (uint16_t)prefs.getUInt("scport", 0);
  if (!host.length() || !port) return false;
  uiLine("Scanner: trying saved %s:%u ...", host.c_str(), port);
  String mm;
  if (esclHello(host, port, &mm)) {
    scHost = host; scPort = port; scName = mm;
    uiLine("Scanner ready: %s", mm.c_str());
    return true;
  }
  uiLine("Saved scanner not answering.");
  return false;
}

// Draw the discovered scanners as buttons; tap picks one (or type its
// number over serial), RESCAN repeats the search.
static void pickScanner() {
  struct Hit { String host; uint16_t port; String label; };
  static Hit hits[5];

  for (;;) {
    uiClear("FIND SCANNER");
    uiLine("Searching for eSCL/AirScan scanners (_uscan._tcp) ...");

    int nh = 0;
    for (int round = 0; round < 3 && nh == 0; round++) {
      int n = MDNS.queryService("uscan", "tcp");
      for (int i = 0; i < n && nh < 5; i++) {
        String ip = MDNS.address(i).toString();
        uint16_t port = MDNS.port(i);
        if (ip == "0.0.0.0" || !port) continue;
        bool dup = false;
        for (int j = 0; j < nh; j++)
          if (hits[j].host == ip && hits[j].port == port) dup = true;
        if (dup) continue;
        hits[nh].host = ip;
        hits[nh].port = port;
        hits[nh].label = MDNS.hostname(i);
        nh++;
      }
    }

    // ---- the pick screen ----
    const int W = epd.width(), H = epd.height();
    epd.fillScreen(0);
    epd.fillRect(0, 0, W, 36, 15);
    epd.setTextSize(3);
    epd.setTextColor(0);
    epd.setCursor(12, 7);
    epd.print(nh ? "TAP YOUR SCANNER" : "NO SCANNERS FOUND");
    epd.setTextColor(15);

    Btn rows[5];
    const int top = 52, gap = 10;
    const int rh = 74;
    for (int i = 0; i < nh; i++) {
      rows[i] = { 20, top + i * (rh + gap), W - 40, rh };
      char l2[64];
      snprintf(l2, sizeof(l2), "%s:%u", hits[i].host.c_str(), hits[i].port);
      drawBtn(rows[i], hits[i].label.c_str(), l2);
      Serial.printf("[tuneup] %d: %s at %s\n", i + 1, hits[i].label.c_str(), l2);
    }
    if (!nh) {
      epd.setTextSize(2);
      epd.setCursor(20, 70);
      epd.print("Is the scanner awake and on this network?");
    }
    Btn rescan = { 20, H - 84, W - 40, 64 };
    drawBtn(rescan, "RESCAN");
    epd.paint();
    Serial.println("[tuneup] tap a scanner, or type its number (r = rescan)");

    int sel = -1;
    bool rescanNow = false;
    while (sel < 0 && !rescanNow) {
      int px, py;
      if (touchTapped(px, py)) {
        if (hitBtn(rescan, px, py)) rescanNow = true;
        for (int i = 0; i < nh; i++)
          if (hitBtn(rows[i], px, py)) sel = i;
      }
      if (Serial.available()) {
        const int ch = Serial.read();
        if (ch == 'r') rescanNow = true;
        else if (ch >= '1' && ch <= '0' + nh) sel = ch - '1';
      }
      delay(20);
    }
    if (rescanNow) continue;

    uiClear("CHECKING");
    uiLine("Asking %s:%u for its capabilities ...", hits[sel].host.c_str(), hits[sel].port);
    String mm;
    if (esclHello(hits[sel].host, hits[sel].port, &mm)) {
      scHost = hits[sel].host;
      scPort = hits[sel].port;
      scName = mm;
      prefs.putString("schost", scHost);
      prefs.putUInt("scport", scPort);
      uiLine("Scanner ready: %s  (saved)", mm.c_str());
      delay(800);
      return;
    }
    uiLine("No eSCL answer from %s:%u.", hits[sel].host.c_str(), hits[sel].port);
    delay(2000);
  }
}

// ============================================================================
// The demo scan
// ============================================================================
static bool demoScan() {
  uiClear("SCANNING");
  uiLine("Scanner: %s  (%s:%u)", scName.c_str(), scHost.c_str(), scPort);
  uiLine("Job: %.0f x %.0f mm, %d dpi, grayscale. Lamp should be moving ...",
         SCAN_W_MM, SCAN_H_MM, SCAN_DPI);
  const size_t len = esclScan(0, 0, SCAN_W_MM, SCAN_H_MM, SCAN_DPI);
  if (!len) {
    uiLine("Scan failed - see serial log.");
    return false;
  }
  uiLine("Received %u bytes.", (unsigned)len);
#if TUNEUP_HAVE_JPEGDEC
  if (decodeGrayFromScan(len)) showScan();
#else
  uiLine("Set TUNEUP_HAVE_JPEGDEC to 1 (and install JPEGDEC) to see it here.");
#endif
  return true;
}

// ============================================================================
// The self-tuner (stage 2) — see tuner.h (16-grey) and tuner_fast.h (FAST).
// ============================================================================
#include "tuner.h"
#include "tuner_fast.h"
#include "tuner_evo.h"

// One entry point for all three. They share the rig — registration, the
// scanner, the results screen — and differ in what they are measuring.
// How many generations the evolutionary tuner runs. It measures 60
// candidates per scan, and on the first run reached worst 3.3 units by
// generation 13 - matching the best result this board has ever produced,
// in a third of the scans. 30 leaves room to refine without feeding the
// printer more acquisitions than it tolerates.
static int evoGenerations = 30;

static bool runSelfTune(Set which) {
#if TUNEUP_HAVE_JPEGDEC
  switch (which) {
    case SET_FAST: return runSelfTuneFast();
    case SET_HIGH: return runEvoTune(EPD_Painter::Quality::QUALITY_HIGH, evoGenerations);
    default:       return runEvoTune(EPD_Painter::Quality::QUALITY_NORMAL, evoGenerations);
  }
#else
  (void)which;
  Serial.println("[tune] JPEGDEC required - set TUNEUP_HAVE_JPEGDEC to 1");
  return false;
#endif
}

// ============================================================================
// Which quality to tune.
//
// The three are separate tunes with separate results, kept in separate
// blobs, so this is a real choice rather than a preference: tuning one
// leaves the others exactly as they were. The screen says what each one
// costs, because they differ by a factor of three in time and the user is
// about to leave the board face-down on a scanner.
// ============================================================================
static Set tunePickQuality(bool *cancelled) {
  const int W = epd.width(), H = epd.height();
  epd.fillScreen(0);
  epd.fillRect(0, 0, W, 40, 15);
  epd.setTextColor(0);
  epd.setTextSize(3);
  epd.setCursor(12, 8);
  epd.print("WHAT SHALL I TUNE?");
  epd.setTextColor(15);
  epd.setTextSize(2);
  epd.setCursor(12, 52);
  epd.print("Each quality is tuned and stored on its own. Tuning one");
  epd.setCursor(12, 72);
  epd.print("leaves the other two exactly as they are.");

  const int bw = (W - 80) / 3, by = 110, bh = 210;
  Btn bFast   = { 20,                by, bw, bh };
  Btn bNormal = { 40 + bw,           by, bw, bh };
  Btn bHigh   = { 60 + bw * 2,       by, bw, bh };
  drawBtn(bFast,   "FAST",   "4 levels + motion");
  drawBtn(bNormal, "NORMAL", "16 greys, 13 passes");
  drawBtn(bHigh,   "HIGH",   "16 greys, 20 passes");

  epd.setTextSize(1);
  struct { Btn b; const char *l1, *l2, *l3; } notes[3] = {
    { bFast,   "7 passes, no settle delay.",  "Four levels and the six",     "grey-to-grey transitions." },
    { bNormal, "13 passes at 15 ms.",         "Evolutionary search over",    "raw waveforms. ~15 min." },
    { bHigh,   "20 passes at 15 ms.",         "Same, with 20 slots to",      "shape. ~15 min." },
  };
  for (int i = 0; i < 3; i++) {
    epd.setCursor(notes[i].b.x + 10, by + bh - 52);
    epd.print(notes[i].l1);
    epd.setCursor(notes[i].b.x + 10, by + bh - 40);
    epd.print(notes[i].l2);
    epd.setCursor(notes[i].b.x + 10, by + bh - 28);
    epd.print(notes[i].l3);
  }

  // What is already on this board, so a re-tune is a decision rather than
  // a guess.
  epd.setTextSize(2);
  epd.setCursor(12, by + bh + 24);
  char have[96];
  snprintf(have, sizeof(have), "Already tuned:  FAST %s   NORMAL %s   HIGH %s",
           tunedPresent(epd.driver(), SET_FAST)   ? "yes" : "no ",
           tunedPresent(epd.driver(), SET_NORMAL) ? "yes" : "no ",
           tunedPresent(epd.driver(), SET_HIGH)   ? "yes" : "no ");
  epd.print(have);

  Btn back = { 20, H - 90, W - 40, 70 };
  drawBtn(back, "BACK");
  epd.paint();
  Serial.println("[tuneup] tune which? tap, or serial: f = FAST, n = NORMAL, h = HIGH, q = back");

  // The finger that tapped TUNE ME is still on the glass, and these
  // buttons have just appeared underneath it: the menu's TUNE ME spans
  // x 490..940 and HIGH lands at 646..939, NORMAL at 333..626. Without a
  // settle and a drain, the press that OPENS this page also answers it,
  // and the tune starts on whichever option happened to be under the
  // finger. Same trap tuneConfirmScreen() documents - a full-screen paint
  // also throws transients into the GT911, so polling straight away
  // catches those as well as the real press.
  while (!epd.driver().paintIdle()) delay(10);
  delay(700);
  while (Serial.available()) Serial.read();
  touchRewake();
  int drainX, drainY;
  touchTapped(drainX, drainY);        // consume anything already latched

  for (;;) {
    if (Serial.available()) {
      const int c = Serial.read();
      if (c == 'f' || c == 'F') { *cancelled = false; return SET_FAST; }
      if (c == 'n' || c == 'N') { *cancelled = false; return SET_NORMAL; }
      if (c == 'h' || c == 'H') { *cancelled = false; return SET_HIGH; }
      if (c == 'q') { *cancelled = true; return SET_NORMAL; }
    }
    int px, py;
    if (touchTapped(px, py)) {
      if (hitBtn(bFast, px, py))   { *cancelled = false; return SET_FAST; }
      if (hitBtn(bNormal, px, py)) { *cancelled = false; return SET_NORMAL; }
      if (hitBtn(bHigh, px, py))   { *cancelled = false; return SET_HIGH; }
      if (hitBtn(back, px, py))    { *cancelled = true;  return SET_NORMAL; }
    }
    delay(8);
  }
}

// Touch entry: instructions + countdown so the user can close the lid.
static void tuneCountdownAndRun() {
  bool cancelled = false;
  const Set which = tunePickQuality(&cancelled);
  if (cancelled) return;

  uiClear("SELF-TUNE");
  uiLine("I am going to tune my own %s trains against the scanner.",
         tunedSetName(which));
  uiLine("");
  uiLine("1. Open the scanner lid.");
  uiLine("2. Lay me FACE DOWN on the glass, screen flat against it.");
  uiLine("3. Keep my USB lead connected (watch progress in the serial log).");
  uiLine("4. Close the lid gently as far as the cable allows.");
  uiLine("");
  uiLine(which == SET_FAST
             ? "The FAST tune takes about 10 minutes and flashes many patterns."
             : which == SET_HIGH
             ? "The HIGH tune takes about 25 minutes and flashes many patterns."
             : "The NORMAL tune takes 10-15 minutes and flashes many patterns.");
  for (int s = 30; s > 0; s--) {
    char b[40];
    snprintf(b, sizeof(b), "Starting in %2d s ...", s);
    epd.fillRect(0, epd.height() - 60, epd.width(), 40, 0);
    epd.setCursor(12, epd.height() - 52);
    epd.setTextSize(3);
    epd.print(b);
    epd.setTextSize(2);
    epd.paint();
    Serial.printf("[tune] %s\n", b);
    delay(1000);
    if (Serial.available() && Serial.peek() == 'q') { Serial.read(); return; }
  }
  runSelfTune(which);
}

// ============================================================================
// Splash view — the artwork rendered in 16 native greys, held until tapped.
//
// Held rather than flashed past, because the menu would otherwise repaint
// over it instantly. Painted at HIGH only if this board actually has a HIGH
// table: rendering 20 passes through the formula library flatters nothing
// and would misrepresent the panel.
// ============================================================================
static void showSplash() {
  const bool haveHigh = tunedPresent(epd.driver(), SET_HIGH);
  epd.setQuality(haveHigh ? EPD_Painter::Quality::QUALITY_HIGH
                          : EPD_Painter::Quality::QUALITY_NORMAL);
  epd.driver().setGreyLevels(16);
  epd.clear();
  while (!epd.driver().paintIdle()) delay(10);
  tuneupSplashDraw(epd.getBuffer(), epd.width(), epd.height());
  epd.paint();
  while (!epd.driver().paintIdle()) delay(10);
  Serial.printf("[tuneup] splash at %s (%d passes, %d us, row charge %d us) "
                "- tap to return\n",
                haveHigh ? "HIGH" : "NORMAL", epd.driver().g16TrainLen(),
                haveHigh ? epd.driver()._config.g16_pass_us_high
                         : epd.driver()._config.g16_pass_us_normal,
                epd.driver()._config.rowExtraUs());

  // Settle before listening: a full-screen 16-grey paint throws transients
  // into the GT911, and the tap that opened this page is still latched.
  delay(700);
  while (Serial.available()) Serial.read();
  touchRewake();
  int dx, dy;
  touchTapped(dx, dy);
  for (;;) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); return; }
    if (touchTapped(dx, dy)) return;
    delay(8);
  }
}

// ============================================================================
// Menu
// ============================================================================
static Btn btnScan, btnTune, btnSplash, btnScanner;

static void showMenuCard() {
  const int W = epd.width(), H = epd.height();
  epd.fillScreen(0);
  epd.fillRect(0, 0, W, 36, 15);
  epd.setTextSize(3);
  epd.setTextColor(0);
  epd.setCursor(12, 7);
  epd.print("EPD TUNEUP");
  epd.setTextColor(15);
  epd.setTextSize(2);
  epd.setCursor(12, 48);
  char b[100];
  snprintf(b, sizeof(b), "WiFi %s   Scanner %s (%s:%u)", WiFi.localIP().toString().c_str(),
           scName.c_str(), scHost.c_str(), scPort);
  b[78] = 0;
  epd.print(b);
  epd.setCursor(12, 68);
  epd.print(tunedLoaded ? "Running flash-tuned trains."
                        : "Running preset trains (not yet self-tuned).");

  const int bw3 = (W - 80) / 3;
  btnScan    = { 20,                100, bw3, 150 };
  btnTune    = { 40 + bw3,          100, bw3, 150 };
  btnSplash  = { 60 + bw3 * 2,      100, bw3, 150 };
  btnScanner = { 20, 270, W - 40, 70 };
  drawBtn(btnScan, "DEMO SCAN", "scan the bed, show it here");
  drawBtn(btnTune, "TUNE ME", "pick FAST / NORMAL / HIGH");
  drawBtn(btnSplash, "SHOW PHOTO", "see the greys on real art");
  drawBtn(btnScanner, "CHANGE SCANNER");

  epd.setCursor(12, H - 130);
  epd.print("Serial: s scan  T tune (pick quality)  P <us> period  L blobs");
  epd.setCursor(12, H - 110);
  epd.print("        X erase all tuned blobs  n scanner  r reboot");
  epd.paint();
  Serial.println("[tuneup] menu: s/T/P/L/X/n/r (or tap)");
}

// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Calibration rig: never shutdown-on-reset. The flasher's hard reset
  // would otherwise power the board off after every upload — and a tune
  // can run unattended for a quarter of an hour, so it must survive one.
  epd.setAutoShutdown(false);

  if (!epd.begin()) {
    Serial.println("EPD begin() failed - check board/preset.");
    for (;;) delay(1000);
  }

  if (!LittleFS.begin(true)) Serial.println("[tuneup] LittleFS mount failed");
  {
    const int n = tunedLoadAll(epd.driver());
    tunedLoaded = (n > 0);
    Serial.printf("[tuneup] tuned blobs: %d installed (FAST %s, NORMAL %s, HIGH %s)\n",
                  n,
                  tunedPresent(epd.driver(), SET_FAST)   ? "yes" : "no",
                  tunedPresent(epd.driver(), SET_NORMAL) ? "yes" : "no",
                  tunedPresent(epd.driver(), SET_HIGH)   ? "yes" : "no");
  }

  epd.driver().setGreyLevels(16);   // builds trains: preset or flash-tuned

  // The panel still physically holds whatever was on it before the reset,
  // but the driver comes up assuming a blank screen — so a delta paint
  // would leave the old image ghosting underneath. Start from a real
  // hard clear.
  epd.clear();
  while (!epd.driver().paintIdle()) delay(10);

  touchInit();

  scanBuf = (uint8_t *)heap_caps_malloc(SCAN_BUF_MAX, MALLOC_CAP_SPIRAM);
  if (!scanBuf) {
    Serial.println("No PSRAM for the scan buffer.");
    for (;;) delay(1000);
  }

  prefs.begin("tuneup", false);

  uiClear("EPD TUNEUP");
  uiLine("The self-tuning rig.");
  if (tunedLoaded)
    uiLine("Flash-tuned trains loaded (period %d us).",
           epd.driver()._config.g16_pass_us_normal);
  uiLine("");

  wifiConnect();
  MDNS.begin("epd-tuneup");
  if (!scannerTrySaved()) pickScanner();
  showMenuCard();
}

void loop() {
  int px, py;
  if (touchTapped(px, py)) {
    if (hitBtn(btnScan, px, py)) {
      while (demoScan()) {
        // after-scan bar: left = again, right = back
        const int W = epd.width(), H = epd.height();
        epd.fillRect(0, H - 30, W, 30, 15);
        epd.setTextColor(0);
        epd.setCursor(12, H - 23);
        epd.print("TAP LEFT: scan again");
        epd.setCursor(W / 2 + 12, H - 23);
        epd.print("TAP RIGHT: menu");
        epd.setTextColor(15);
        epd.paint();
        int qx, qy;
        for (;;) {
          if (touchTapped(qx, qy)) break;
          if (Serial.available() && Serial.peek() == 's') { Serial.read(); qx = 0; break; }
          if (Serial.available()) { Serial.read(); qx = W; break; }
          delay(20);
        }
        if (qx >= W / 2) break;
      }
      showMenuCard();
    } else if (hitBtn(btnTune, px, py)) {
      tuneCountdownAndRun();
      showMenuCard();
    } else if (hitBtn(btnSplash, px, py)) {
      showSplash();
      showMenuCard();
    } else if (hitBtn(btnScanner, px, py)) {
      pickScanner();
      showMenuCard();
    }
    return;
  }

  if (!Serial.available()) { delay(20); return; }
  const int c = Serial.read();
  switch (c) {
    case 's': demoScan(); Serial.println("[tuneup] (any key for menu)"); break;
    case 'T': {
      bool cancelled = false;
      const Set which = tunePickQuality(&cancelled);
      if (!cancelled) runSelfTune(which);
      showMenuCard();
    } break;
    case 'P': {
      tunePeriodUs = (uint32_t)Serial.parseInt();
      Serial.printf("[tuneup] tune period override: %lu us%s\n",
                    (unsigned long)tunePeriodUs,
                    tunePeriodUs ? "" : " (preset)");
    } break;
    case 'L': {
      Serial.printf("[tuneup] blobs: FAST %s  NORMAL %s  HIGH %s\n",
                    tunedPresent(epd.driver(), SET_FAST)   ? "present" : "absent",
                    tunedPresent(epd.driver(), SET_NORMAL) ? "present" : "absent",
                    tunedPresent(epd.driver(), SET_HIGH)   ? "present" : "absent");
      Serial.printf("[tuneup] 16-grey periods: NORMAL %d us x %d passes, "
                    "HIGH %d us x %d passes\n",
                    epd.driver()._config.g16_pass_us_normal, EPD_Painter::DEC_WF_LEN16,
                    epd.driver()._config.g16_pass_us_high, EPD_Painter::DEC_WF_LEN16_HIGH);
    } break;
    case 'X':
      tunedEraseAll();
      Serial.println("[tuneup] all tuned blobs erased - reboot ('r') for preset tables");
      break;
    case 'E': {
      // E [gens]  evolutionary tune (HIGH). Mutation-only search over the
      // raw drive sequence, 60 candidates per scan.
      int gens = Serial.parseInt();
      if (gens < 1) gens = evoGenerations;
      if (gens > 200) gens = 200;
      runEvoTune(EPD_Painter::Quality::QUALITY_HIGH, gens);
      showMenuCard();
    } break;
    case 'A': {
      // A 0|1 - alternate-slot mode: even slots pinned float.
      evoSettleMask = (Serial.parseInt() != 0);
      Serial.printf("[tuneup] settle mask: %s\n",
                    evoSettleMask ? "ON" : "off");
    } break;
    case 'R': {
      // R 0|1 - patch position randomisation off/on.
      evoRandomPos = (Serial.parseInt() != 0);
      Serial.printf("[tuneup] patch positions: %s\n",
                    evoRandomPos ? "shuffled" : "fixed");
    } break;
    case 'B': {
      // Stability sweep: which (row charge, pass period) holds a grey
      // steadiest across position and surrounding content.
      runStabilitySweep();
      showMenuCard();
    } break;
    case 'Q': {
      // Paint the registration quadrant mark and hold, so it can be scanned
      // independently when registration fails.
      epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
      epd.driver().setGreyLevels(16);
      tunePaintQuadrant();
      tuneClear();
      tunePaint();
      Serial.println("[tuneup] quadrant mark painted and held");
    } break;
    case 'g': {
      // Same, at NORMAL - for comparing the two qualities on identical
      // content. 13 slots against HIGH's 20.
      epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
      epd.driver().setGreyLevels(16);
      epd.clear();
      while (!epd.driver().paintIdle()) delay(10);
      tuneupSplashDraw(epd.getBuffer(), epd.width(), epd.height());
      epd.paint();
      while (!epd.driver().paintIdle()) delay(10);
      Serial.printf("[tuneup] splash painted at NORMAL (%d passes, %d us, "
                    "row_extra %d us)\n",
                    epd.driver().g16TrainLen(),
                    epd.driver()._config.g16_pass_us_normal,
                    epd.driver()._config.rowExtraUs());
    } break;
    case 'G': {
      // Paint the splash artwork at HIGH, 16 greys. A visual check of the
      // tuned trains on real photographic content rather than on a test
      // card - which is the content that exposed the pass-budget overrun,
      // so it is the one worth looking at.
      epd.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
      epd.driver().setGreyLevels(16);
      epd.clear();
      while (!epd.driver().paintIdle()) delay(10);
      tuneupSplashDraw(epd.getBuffer(), epd.width(), epd.height());
      epd.paint();
      while (!epd.driver().paintIdle()) delay(10);
      Serial.printf("[tuneup] splash painted at HIGH (%d passes, %d us, "
                    "row_extra %d us)\n",
                    epd.driver().g16TrainLen(),
                    epd.driver()._config.g16_pass_us_high,
                    epd.driver()._config.rowExtraUs());
    } break;
    case 'W': {
      // W <us> - extra per-row charging time. Tests whether the horizontal
      // gradient is transient charging (more time helps) or steady-state
      // rail droop (it does not).
      const int us = (int)Serial.parseInt();
      const int v = constrain(us, 0, 40);
      epd.driver()._config.row_extra_us_normal = v;
      epd.driver()._config.row_extra_us_high   = v;
      Serial.printf("[tuneup] row charge = %d us both qualities (adds %.1f ms "
                    "per pass over %d rows)\n", v,
                    v * epd.height() / 1000.0f, epd.height());
    } break;
    case 'U': {
      // Positional uniformity: driven vs dithered at the same density.
      static const int LV[] = { 4, 8, 12 };
      runUniformityProbe(EPD_Painter::Quality::QUALITY_HIGH, LV, 3);
      showMenuCard();
    } break;
    case 'S': {
      // Staircase check on the INSTALLED tables. Independent of the tuner:
      // full-height solid bars, which is the worst case for sweeps per row
      // and therefore the content most likely to expose a dose problem.
      runStaircaseCheck(EPD_Painter::Quality::QUALITY_HIGH);
      showMenuCard();
    } break;
    case 'D': {
      // Dose-ladder sweep: pick the pass period from measurement.
      static const int PERIODS[] = { 19000, 15000, 13000 };
      runDoseSweep(PERIODS, (int)(sizeof(PERIODS) / sizeof(PERIODS[0])));
      showMenuCard();
    } break;
    case 'K': {
      // Compensation sweep: cells fixed, whole-panel background varied,
      // period swept. Yields lum-per-ms and the ms that cancels a
      // background shift — the coefficient a renderer would need.
      runCompSweep(COMP_KNOB_PERIOD);
      showMenuCard();
    } break;
    case 'k': {
      // Same sweep, but varying row charge time instead of pass period.
      runCompSweep(COMP_KNOB_ROW);
      showMenuCard();
    } break;
    case 'l': {
      // LE pulse width, with row charge forced to 0: does a properly held
      // latch remove the content dependence on its own?
      runCompSweep(COMP_KNOB_LE);
      showMenuCard();
    } break;
    case 'F': {
      // F 0|1 - reference-ladder correction (see evoRefLadderFix).
      evoRefLadderFix = (Serial.parseInt() != 0);
      Serial.printf("[tuneup] reference ladder correction: %s\n",
                    evoRefLadderFix ? "ON" : "off (plain density)");
    } break;
    case 'J': benchLumaScan(); benchLeHold(); break;
    case 'V': tunePreviewResults(); showMenuCard(); break;
    case 'C': {
      // Paint the tuner's own match card and leave it up, so the levels
      // it claims to have matched can be measured independently of the
      // tuner's own geometry and arithmetic.
      tuneClear();
      tunePaintMatchCard();
      epd.paint();
      while (!epd.driver().paintIdle()) delay(10);
      Serial.println("[tuneup] match card painted (top = dither refs, "
                     "bottom = native levels)");
    } break;
    case 'n': pickScanner(); showMenuCard(); break;
    case 'm': showMenuCard(); break;
    case 'r': Serial.println("rebooting"); delay(300); ESP.restart(); break;
    default: break;
  }
}
