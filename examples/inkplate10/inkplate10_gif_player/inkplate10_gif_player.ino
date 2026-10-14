// Inkplate 10 GIF player.
//
// Plays an animated GIF stored in flash, scaled up to fill the panel.
// Each line is quantised to the nearest of 4 greys as it decodes (no
// dithering); only the rectangle the frame touched is scaled and handed to
// paintRows(). The delta engine then drives only the pixels that changed, and
// the row window scans only the rows that changed. Small moving parts are
// fast; full-screen motion is limited by the panel.
//
// Your own GIF:
//   python3 gif2h.py my.gif      (rewrites gif_data.h, then rebuild)
// The GIF must be at most 480 px wide (AnimatedGIF's limit on ESP32). Small
// is better anyway: it is scaled up on the board.
//
// Needs the AnimatedGIF library (bitbank2) from the Library Manager.
//
// Build:
//   arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app,USBMode=hwcdc,CDCOnBoot=cdc
//
// No dithering: smooth gradients band into 4 flat greys, but unchanged parts
// of the picture stay identical between frames and cost nothing to paint.

#include <EPD_Painter_Adafruit.h>
#include <AnimatedGIF.h>
#include "gif_data.h"

#define SCALE             0   // 0 = fit the panel, else a fixed integer scale
#define CLEAR_EVERY_LOOPS 5   // full clear after this many loops (0 = never)
// 1 = play at the GIF's own frame rate (Bad Apple: 20 fps, 50 ms a frame).
// 0 = as fast as the board can go, to see the real limit.
#define FOLLOW_GIF_TIMING 1
#define TIMING            1   // print where each frame's time goes
#define PAINT_PROFILE     1   // driver's per-cycle breakdown ([paintprof] lines)

static EPD_Painter::Config fastConfig() {
  EPD_Painter::Config c = EPD_PAINTER_PRESET;
  c.pclk_hz        = 10000000;
  c.skip_idle_rows = true;
  c.skip_idle_cols = true;    // experimental: set false if stray marks appear
  c.min_pass_us    = 4000;
  // Black and white only: greys snap to the nearer of the two, and each
  // frame is bw_passes passes. Use QUALITY_FAST for 4 greys (7 passes).
  c.quality        = EPD_Painter::Quality::QUALITY_BW;
  c.bw_passes      = 4;   // raise if blacks look grey, lower for speed
  // Experimental fixes for white smearing down into black (try one at a
  // time): 1 = discharge the columns after rows that whiten, 2 = after any
  // driven row. hold_black = keep unchanged blacks driven black.
  c.discharge_rows = 0;
  c.bw_hold_black  = false;
  return c;
}

EPD_PainterAdafruit epd(fastConfig());
AnimatedGIF gif;

// The GIF canvas as grey levels (0 = white ... 3 = black), quantised as
// each line decodes. Frames draw on top of each other here, which is how GIF
// composes them.
static uint8_t *lvl;
static int gw, gh;

// The last frame's rectangle and disposal method, for disposal 2.
static int fx, fy, fw, fh, fdisp;

// Destination rectangle on the panel and the integer scale.
static int dx, dy, dw, dh, scale;

// Source rectangle touched since the last render (x1 < x0 = nothing).
static int dirtyX0, dirtyY0, dirtyX1 = -1, dirtyY1 = -1;
static void markDirty(int x0, int y0, int x1, int y1) {
  if (dirtyX1 < dirtyX0) { dirtyX0 = x0; dirtyY0 = y0; dirtyX1 = x1; dirtyY1 = y1; return; }
  dirtyX0 = min(dirtyX0, x0); dirtyY0 = min(dirtyY0, y0);
  dirtyX1 = max(dirtyX1, x1); dirtyY1 = max(dirtyY1, y1);
}

// Palette index -> grey level, rebuilt on the first line of every frame
// (frames may carry their own palette).
static uint8_t palLvl[256];

// One expanded panel row, in internal RAM: built once per source row, then
// copied to the scale rows it covers in the PSRAM canvas.
static uint8_t *rowBuf;

// One decoded line. Palette to grey level, skipping transparent pixels so
// the previous frame shows through.
static void onLine(GIFDRAW *d) {
  const int y = d->iY + d->y;
  if (y >= gh) return;
  const int w = min(d->iWidth, gw - d->iX);
  if (w <= 0) return;

  if (d->y == 0) {
    fx = d->iX; fy = d->iY; fw = d->iWidth; fh = d->iHeight;
    fdisp = d->ucDisposalMethod;
    const uint8_t *p = d->pPalette24;
    for (int i = 0; i < 256; i++, p += 3) {
      const int l = (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8;  // 0..255
      palLvl[i] = ((255 - l) * 3 + 127) / 255;
    }
  }
  markDirty(d->iX, y, d->iX + w - 1, y);

  const uint8_t *s = d->pPixels;
  uint8_t *out = lvl + y * gw + d->iX;
  if (d->ucHasTransparency) {
    const uint8_t t = d->ucTransparent;
    for (int x = 0; x < w; x++)
      if (s[x] != t) out[x] = palLvl[s[x]];
  } else {
    for (int x = 0; x < w; x++) out[x] = palLvl[s[x]];
  }
}

// Scale the dirty part of lvl[] onto the panel canvas. Returns the panel
// rows written as [*py0, *py1], or false if nothing changed.
static bool render(int *py0, int *py1) {
  if (dirtyX1 < dirtyX0) return false;
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width();

  const int cx0 = dirtyX0 * scale;                       // panel columns,
  const int cx1 = min(dw, (dirtyX1 + 1) * scale);        // relative to dx
  for (int sy = dirtyY0; sy <= dirtyY1; sy++) {
    const int ry0 = sy * scale;
    if (ry0 >= dh) break;
    const int ry1 = min(dh, ry0 + scale);

    // Expand one source row into rowBuf[cx0 .. cx1).
    const uint8_t *src = lvl + sy * gw;
    uint8_t *o = rowBuf + cx0;
    for (int sx = dirtyX0; o < rowBuf + cx1; sx++) {
      const uint8_t v = src[sx];
      for (int k = 0; k < scale; k++) *o++ = v;
    }
    for (int ry = ry0; ry < ry1; ry++)
      memcpy(fb + (dy + ry) * W + dx + cx0, rowBuf + cx0, cx1 - cx0);
  }
  *py0 = dy + dirtyY0 * scale;
  *py1 = dy + min(dh, (dirtyY1 + 1) * scale) - 1;
  dirtyX1 = dirtyY1 = -1;
  return true;
}

static bool openGif() {
  if (!gif.open((uint8_t *)gif_data, sizeof(gif_data), onLine)) {
    Serial.printf("gif.open failed: error %d\n", gif.getLastError());
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!epd.begin()) {
    Serial.println("epd.begin() failed - see the log above");
    while (true) delay(1000);
  }
  epd.driver().setIdleTimeout(30);   // keep the PMIC up between frames
  epd.driver().setPaintProfile(PAINT_PROFILE);

  gif.begin(GIF_PALETTE_RGB888);
  if (!openGif()) while (true) delay(1000);
  gw = gif.getCanvasWidth();
  gh = gif.getCanvasHeight();

  const int W = epd.width(), H = epd.height();
  scale = SCALE;
  if (scale <= 0) scale = max(1, min(W / gw, H / gh));   // largest whole scale
  dw = min(W, gw * scale);
  dh = min(H, gh * scale);
  dx = (W - dw) / 2;
  dy = (H - dh) / 2;
  Serial.printf("GIF %d x %d, scale %d -> %d x %d on the panel\n", gw, gh, scale, dw, dh);

  lvl    = (uint8_t *)heap_caps_malloc(gw * gh, MALLOC_CAP_SPIRAM);
  rowBuf = (uint8_t *)heap_caps_malloc(dw + scale, MALLOC_CAP_INTERNAL);
  if (!lvl || !rowBuf) {
    Serial.println("out of memory");
    while (true) delay(1000);
  }
  memset(lvl, 0, gw * gh);   // white

  epd.clear();
  epd.fillScreen(0);
}

void loop() {
  static int loops = 0, frames = 0;
  static int64_t fpsT0 = esp_timer_get_time();

  // Disposal 2: the last frame's area goes back to the background before the
  // next frame draws. Disposal 3 (restore previous) is rare and treated as 1.
  if (fdisp == 2) {
    const int w = max(0, min(fw, gw - fx));
    for (int y = fy; y < min(gh, fy + fh); y++)
      memset(lvl + y * gw + fx, 0, w);
    if (w > 0 && fh > 0) markDirty(fx, fy, fx + w - 1, min(gh, fy + fh) - 1);
    fdisp = 0;
  }

  static int64_t tDec = 0, tRen = 0, tPaint = 0, tWait = 0, tLast = 0, tFrame = 0;
  static int nT = 0;

  const int64_t t0 = esp_timer_get_time();
  int delayMs = 0;
  const int rc = gif.playFrame(false, &delayMs);
  const int64_t t1 = esp_timer_get_time();
  if (rc < 0) {
    Serial.printf("decode error %d\n", gif.getLastError());
    gif.close();
    openGif();
    return;
  }

  // Only the rows this frame touched are scaled, packed and compared.
  // paintRows() returns once the frame is packed; the panel works on it
  // while the next one decodes. The previous frame's rows are included
  // too, so its mop-up cycle is not cut short by a narrower window.
  static int prevY0 = -1, prevY1 = -1;
  int y0, y1;
  const bool changed = render(&y0, &y1);
  const int64_t t2 = esp_timer_get_time();
  if (changed) {
    if (prevY0 >= 0) { const int a = min(y0, prevY0), b = max(y1, prevY1);
                       prevY0 = y0; prevY1 = y1; y0 = a; y1 = b; }
    else { prevY0 = y0; prevY1 = y1; }
    epd.paintRows(y0, y1);
  }
  const int64_t t3 = esp_timer_get_time();
  frames++;

  // Keep to the GIF's timing when the panel is quicker than the frame delay.
  // While this is on, fewer passes or a faster clock cannot raise the fps
  // above the GIF's own rate.
  const int64_t spent = (t3 - t0) / 1000;
  if (FOLLOW_GIF_TIMING && delayMs > spent) delay(delayMs - spent);
  const int64_t t4 = esp_timer_get_time();

  if (TIMING) {
    tDec += t1 - t0; tRen += t2 - t1; tPaint += t3 - t2; tWait += t4 - t3;
    if (tLast) tFrame += t4 - tLast;
    tLast = t4;
    if (++nT == 50) {
      // paint() long = the panel is the limit (it waited for the last frame).
      // pace long = the GIF's frame delay is the limit.
      Serial.printf("avg ms/frame: decode %.1f  render %.1f  paint() %.1f  "
                    "pace %.1f  | frame %.1f (%.1f fps)\n",
                    tDec / 5e4, tRen / 5e4, tPaint / 5e4, tWait / 5e4,
                    tFrame / 5e4, 5e7 / (double)max<int64_t>(tFrame, 1));
      tDec = tRen = tPaint = tWait = tFrame = 0;
      nT = 0;
    }
  }

  if (rc == 0) {   // last frame: rewind
    gif.reset();
    const int64_t now = esp_timer_get_time();
    Serial.printf("loop %d: %d frames, %.1f fps\n", ++loops, frames,
                  frames * 1e6 / (now - fpsT0));
    frames = 0;
    fpsT0 = now;
    if (CLEAR_EVERY_LOOPS && loops % CLEAR_EVERY_LOOPS == 0) {
      while (!epd.driver().paintIdle()) delay(1);
      epd.clear();      // wipe the ghosts FAST leaves behind
      epd.paint();      // and put the current frame back
    }
  }
}
