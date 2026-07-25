// ============================================================================
// tuner.h — stage 2: the on-device self-tuning engine.
//
// Runs with the board FACE-DOWN on the scanner glass, so everything is
// driven over serial ('T') or by the menu's TUNE button (which gives you a
// countdown to close the lid). Progress goes to serial; the panel itself
// is busy being the test subject.
//
// The method is the dither-match card (see extras/calibration/dithertune.py
// and the tuning comments in src/EPD_Painter_trains.h): each of the 16
// columns pairs a Bayer-dithered black/white reference (the optical truth
// for level g = density g/15) with the same level painted as a native
// waveform grey. Scanner gamma, lamp falloff and session drift cancel
// because every comparison lives inside one frame of one scan.
//
// GEOMETRY (learned the hard way): thresholded bounding boxes are fooled
// by whatever else is dark near the panel — the USB cable, and the lid
// shadow wedge it props open. So after locating the panel roughly (black
// paint, wide scan) and learning the face-down mirroring (corner mark),
// the tuner paints a black RING 40 px inside the panel edges and walks
// outward from the scan centre to the ring's INNER edges. Those anchors
// give an exact panel-pixel-to-scan-pixel mapping that nothing outside
// the panel can disturb, and every later measurement uses the anchors —
// including scans of an all-white panel, which no threshold could find.
//
// Train parameterisation: (d, w, g, r) — d darkens, w whitens, g float
// gaps, r re-darkens. This grammar spans the shipped hand-tuned tables:
// coarse steps from d (saturating), big lifts from w (15-30 units), fine
// steps from r (re-darkens fire in small fresh-response steps), and the
// float gap as the fine-trim knob (a gap before the re-darkens weakens
// them — the hand-tuner's trick). How much it weakens them is measured,
// not assumed: see tuneProbeGap(). A float pass buys a whole pass period
// of field-free settle, so the old guess of "a few units" was never safe.
//
// Convergence policy (lessons from two failed shapes before this one):
// - The panel DRIFTS under heavy exercise (the deep end wanders tens of
//   units over a session) and edits couple globally through the frame
//   composition. So: at most TWO levels are edited per cycle, and the
//   UNEDITED levels' error changes form a drift baseline (median) that is
//   subtracted before an edit's observed effect updates its gain estimate.
// - Damped proportional edits: every candidate has an expected move in
//   scanner units (seeded from a measured pure-dose ladder, updated
//   online); an edit is only taken when its expected move ≤ 1.7x the
//   error, and overshoots that flip the sign revert and shrink the gain.
// - Two warm-up cycles run before any editing so the first attributions
//   aren't polluted by the panel settling into exercise equilibrium.
// - Deep levels are seeded by INVERTING the dose ladder against the
//   measured dither references — at a new pass period the old shapes can
//   be tens of units off, far outside the damped edits' reach.
//
// Charge-matched removes are derived analytically from the tuned applies
// (k darkens then k + net whitens; wr − k = net keeps the DC ledger
// exact), then verified optically: paint the card, repaint white, scan
// the residual against the L0 column.
//
// The result is packaged by EPD_Painter_tuned.h and stored by whatever
// tuned_storage.h implements (LittleFS here); it is reloaded on
// every boot.
// ============================================================================

#pragma once

#if TUNEUP_HAVE_JPEGDEC

// ---- knobs -----------------------------------------------------------------
// ---- tolerances are fractions of a LEVEL STEP, not scanner units ---------
//
// Every threshold below used to be an absolute number of scanner units,
// which silently hard-coded one panel and one scanner. Full scale here is
// ~97 units across 15 intervals, so a level step is ~6.5 - but a different
// panel, a different scan resolution or a different lamp changes that
// number, and every absolute tolerance then means something different.
// A tolerance of "4.5 units" is meaningless; "0.7 of a level step" is not.
//
// tuneStep() below measures the step from the panel's own dither reference
// ladder, so these fractions carry to any device. The values are the ratios
// the absolute constants happened to have on the LilyGo H716, so behaviour
// on that board is unchanged.
static const float TUNE_TOL_STEPS       = 0.70f;  // was 4.5 units
// Looser than the placement tolerance, and that is not a concession - it is
// arithmetic. A gap is the DIFFERENCE of two neighbouring errors, so it runs
// larger than either and can reach twice the worst placement error. Gating
// gaps tighter than placement therefore makes convergence unreachable by
// construction, which is exactly what happened: every run on 0.54 ended
// converged=0, stalling on gap while maxerr was already inside tolerance
// (attempt 1 finished at maxerr 5.6, gap 6.9 - a ratio of 1.2, which is
// typical). 1.1 still catches the cases that actually band, like the 39/2
// step pair one run produced, without demanding the impossible.
static const float TUNE_GAP_TOL_STEPS   = 1.10f;  // was 0.54, unreachable
static const float TUNE_AVG_BELOW_STEPS = 1.23f;  // was 8.0
static const float TUNE_AVG_BELOW3_STEPS= 0.77f;  // was 5.0
static const float TUNE_PROBE_COST_STEPS= 0.92f;  // was 6.0
static const float TUNE_MAX_RISK_STEPS  = 1.85f;  // was 12.0
static const float TUNE_GAP_NOISE_STEPS = 0.23f;  // was 1.5 units/pass
static const int   TUNE_MAX_CYCLES = 34;
// Whole-run retries when an attempt stalls instead of converging. Restarting
// from a fresh probe escapes a bad basin; three attempts at ~5 min is a
// better use of the time than a fourth parameter tweak.
static const int   TUNE_MAX_ATTEMPTS = 1;
// How many stuck levels one run may hand to the rescue. Each costs one extra
// scan and targets a different level, so this is "how many stubborn levels do
// we expect", not a retry count.
static const int   TUNE_MAX_RESCUES  = 6;
// Quiet time between attempts, for the SCANNER's benefit rather than the
// panel's. Flatbeds wedge on sustained scan RATE, not scan length, and
// back-to-back attempts turn a 6-minute run into 20 minutes of continuous
// scanning - which is exactly what stopped the Canon answering mid-run and
// cost a power cycle. A minute of silence is cheap against a five-minute
// wedge plus hands on the hardware.
static const uint32_t TUNE_ATTEMPT_COOLDOWN_MS = 60000;
static const int   TUNE_WARMUP     = 2;
// Levels the seeder reserves as whiten-only shapes (whitens, no re-darkens)
// so tuneCalibrateGains() always has something to solve the whiten gain
// from. Two, not one, so the figure is an average rather than a single read.
static const int   TUNE_WHITEN_PROBES = 2;
// Ceiling on what reserving a probe may cost that level's seed fit (see
// TUNE_PROBE_COST_STEPS). Beyond ~one level step the trade is bad: an
// inaccurate gw is recoverable by the online learner, a level seeded a whole
// step off target is a visible band.
//
// How wrong the measured gains are assumed to be. Three runs put this
// panel's whiten gain at 15.1 / 20.2 / 24.4, so +/-25% is if anything
// generous. This one is genuinely dimensionless and needs no scaling.
static const float TUNE_GAIN_ERR   = 0.25f;
// Banding is a function of the SPACING between adjacent levels, not of any
// one level's absolute placement, and until these the converger only ever
// looked at the latter. On a ~97-unit full scale a level step is ~6.5
// units, so a level could sit 4.5 out — 69% of a step — and still pass:
// two neighbours in tolerance with opposite signs halve or double a gap.
// TUNE_GAP_TOL bounds |err[i] - err[i+1]| directly; TUNE_SPACING_W is how
// hard the planner weighs it against absolute placement when choosing a
// shape.
//
// The tolerance is set from what it does to the ramp, not by feel. One
// scanner unit is ~2.62 lum units here and a level step is 6.49 units
// (17 lum), so a gap error G leaves steps of 17 +/- 2.62G:
//     G = 5.0 -> steps 4..30 lum   (run 2 measured G = 4.0 and banded)
//     G = 2.5 -> steps 10..24      usable
//     G = 1.5 -> steps 13..21      good
// 5.0 would have passed run 2 unchanged, which is the false pass that
// started this. 2.5 was then tried and proved unreachable - three runs sat
// at gapMax 4 against 1-2 units of scan noise, so convergence never fired
// and every result came from the best-shape fallback. A gate that cannot
// pass is not a gate, and the planner spends its edits chasing it. 0.54 of
// a step (3.5 units here) is the compromise: measurably tighter than the 4
// the panel keeps landing on, still above the noise floor.
static const float TUNE_SPACING_W  = 0.5f;   // per neighbour, in the plan score
// Must be a resolution the scanner natively offers — eSCL servers snap an
// unsupported value to the nearest supported one, silently returning a
// much smaller image than asked for (100 dpi became 75 on the Canon,
// which shrank the panel below the geometry checks and failed the run).
static const int   TUNE_DPI        = 150;
static const int   TUNE_SCAN_GAP_MS = 4000;   // breather between scans
static const float TUNE_REG_W_MM   = 210.0f;
static const float TUNE_REG_H_MM   = 148.0f;

static uint32_t tunePeriodUs = 0;  // 0 = tune at the preset period; 'P' overrides

// ---- geometry --------------------------------------------------------------
struct TuneGeo {
  float x0mm, y0mm, wmm, hmm;  // tight scan window (mm from the glass origin)
  bool  flipx, flipy;          // panel-to-scan axis flips (face-down mirror)
  int   ax0, ax1, ay0, ay1;    // scan px of the ring's inner edges
                               // (= panel px RING_T and W-RING_T / H-RING_T)
  bool  valid;
};
static TuneGeo tgeo = {};
static const int RING_T = 40;  // ring thickness in panel px

// ---- per-level tuner state -------------------------------------------------
struct TuneLv {
  int8_t d, w, g, r;           // train = d darkens, w whitens, g gaps, r re-darkens
  int8_t pd, pw, pg, pr;       // the shape before the last edit (for revert)
  float  err, lastErr;
  float  gd, gw, gr, gg;       // expected gain per step (scanner units)
  int8_t lastOp;               // 1=d 2=w 3=r 4=gap, 0 = not edited / multi-param
  int8_t lastSteps;
  float  predMove;             // the move the planner expected from that edit
  uint8_t nvalid;              // gain observations made (0 = model still a guess)
  bool   edited;               // an edit was applied last cycle
  bool   skip;                 // cool-down after a bad edit
  float  bestAbs;              // smallest |err| ever measured for this level …
  int8_t bd, bw, bg, br;       // … and the shape that achieved it
  // A level the rescue has taken over. Its train is stored literally rather
  // than as (d,w,g,r), because the rescue mutates the DRIVE SEQUENCE - it can
  // insert and delete passes, producing shapes the grammar cannot express
  // (leading floats, alternating d,w,d,w tails - exactly the shapes the
  // hand-tuned direct trains needed). Once a level goes raw the planner
  // leaves it alone; only a later rescue may change it again.
  bool    hasRaw;
  uint8_t raw[13];
  uint8_t bad;                 // edits that made this level worse
  bool   frozen;               // give up editing: keep the best-known shape
};
static TuneLv tl[16];
static float tuneDose[14];     // landing of {1 x n} (monotone-enforced), n=1..13
static float probeRef[16];     // dither reference means from the probe frame

// ---- the unit every tolerance is expressed in ------------------------------
// One level step, in scanner units, measured from this panel's own dither
// reference ladder: white to black over 15 intervals. This is what makes the
// tuner portable - a different panel, scan resolution or lamp changes the
// absolute numbers, and every threshold scales with it automatically.
//
// probeRef[] is filled by tuneProbe(), so these are only valid after it has
// run; every caller is downstream of that. The floor stops a failed or
// nonsense probe turning into absurdly tight tolerances.
static float tuneStep() {
  const float span = probeRef[0] - probeRef[15];
  return (span > 20.0f ? span : 20.0f) / 15.0f;
}
static float tuneTol()        { return TUNE_TOL_STEPS        * tuneStep(); }
static float tuneGapTol()     { return TUNE_GAP_TOL_STEPS    * tuneStep(); }
static float tuneAvgBelow()   { return TUNE_AVG_BELOW_STEPS  * tuneStep(); }
static float tuneAvgBelow3()  { return TUNE_AVG_BELOW3_STEPS * tuneStep(); }
static float tuneProbeMaxCost(){ return TUNE_PROBE_COST_STEPS * tuneStep(); }
static float tuneMaxRisk()    { return TUNE_MAX_RISK_STEPS   * tuneStep(); }
static float tuneGapNoise()   { return TUNE_GAP_NOISE_STEPS  * tuneStep(); }
static int   removeExtra[16];

// ============================================================================
// Panel painting (canvas is 8bpp level codes; we're in 16-grey mode)
// ============================================================================
static void tuneWaitIdle() {
  while (!epd.driver().paintIdle()) delay(10);
  delay(150);
}
static void tunePaint() { epd.paint(); tuneWaitIdle(); }

// Activation flash before every measurement.
//
// A pixel's response depends on where its particles have been sitting, so
// measuring straight after arbitrary content bakes that history into the
// result. Driving the panel rail-to-rail several times first agitates the
// ink and leaves every pixel in the same physical state, which is what
// makes one cycle comparable with the next.
//
// This is built from repeated HARD clears rather than a hand-rolled
// black/white paint loop, because a hard clear is DC balanced by
// construction: its four phases run {6,2,4,8} passes with alternating
// polarity, so ten darkening passes are matched by ten whitening ones.
// Flashing by painting level 15 then level 0 would instead rely on the
// removes being charge-matched to the applies — and during convergence
// they are not, since matched removes are only derived at the end. That
// route would inject charge on every cycle.
static const int TUNE_ACTIVATE_CLEARS = 5;      // rail-to-rail flashing before each measurement

// No settling anywhere in here: the flashes run back-to-back and the
// caller drives the panel the instant this returns. Any pause lets the
// agitated particles relax again, which is the whole thing this is trying
// to avoid — so the usual post-paint settle is deliberately not used.
static void tuneClear() {
  for (int i = 0; i < TUNE_ACTIVATE_CLEARS; i++) {
    epd.clear();
    while (!epd.driver().paintIdle()) delay(2);
  }
}

static void tuneFill(uint8_t g) {
  memset(epd.getBuffer(), g, (size_t)epd.width() * epd.height());
}

// Registration pattern: black square over the panel's top-left quadrant.
static void tunePaintQuadrant() {
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width(), H = epd.height();
  memset(fb, 0, (size_t)W * H);
  for (int y = 0; y < H / 2; y++) memset(fb + (size_t)y * W, 15, W / 2);
}

// Geometry anchor pattern: black ring hugging the panel edges, white core.
static void tunePaintRing() {
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width(), H = epd.height();
  memset(fb, 0, (size_t)W * H);
  for (int y = 0; y < H; y++) {
    if (y < RING_T || y >= H - RING_T) { memset(fb + (size_t)y * W, 15, W); continue; }
    memset(fb + (size_t)y * W, 15, RING_T);
    memset(fb + (size_t)y * W + W - RING_T, 15, RING_T);
  }
}

static const uint8_t tuneBayer8[8][8] = {
  {  0, 32,  8, 40,  2, 34, 10, 42 },
  { 48, 16, 56, 24, 50, 18, 58, 26 },
  { 12, 44,  4, 36, 14, 46,  6, 38 },
  { 60, 28, 52, 20, 62, 30, 54, 22 },
  {  3, 35, 11, 43,  1, 33,  9, 41 },
  { 51, 19, 59, 27, 49, 17, 57, 25 },
  { 15, 47,  7, 39, 13, 45,  5, 37 },
  { 63, 31, 55, 23, 61, 29, 53, 21 }
};

// The match card (same layout as grey16_testcard 'm'): top half = Bayer
// dithered references, thin black divider, bottom half = native levels.
static void tunePaintMatchCard() {
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width(), H = epd.height(), mid = H / 2;
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      const int g = (x * 16) / W;
      uint8_t v;
      if (y < mid - 3) {
        const int thr = (g * 64 + 7) / 15;
        v = (tuneBayer8[y & 7][x & 7] < thr) ? 15 : 0;
      } else if (y < mid + 3) {
        v = 15;
      } else {
        v = (uint8_t)g;
      }
      fb[(size_t)y * W + x] = v;
    }
}

// Each of these puts the panel back into the state a scan expects, so a
// measurement can be retried after the user has been away fixing things.
static void tuneRepaintCard()  { tunePaintMatchCard(); tuneClear(); tunePaint(); }
static void tuneRepaintRing()  { tunePaintRing();      tuneClear(); tunePaint(); }
static void tuneRepaintBlack() { tuneFill(15);         tuneClear(); tunePaint(); }
static void tuneRepaintQuad()  { tunePaintQuadrant();  tuneClear(); tunePaint(); }
static void tuneRepaintCardThenWhite() {
  tuneRepaintCard();
  tuneFill(0);
  tunePaint();
}

// ============================================================================
// Scan + geometry
// ============================================================================
static bool tuneRegister();               // defined below
static void (*tuneRepaint)() = nullptr;   // repaints the pattern being measured
static bool tuneRecovering = false;       // suppresses nested recovery

// Full-screen "come and fix me" card. The board is face-down on the glass
// while tuning, so this is what the user reads when they pick it up —
// which is the only moment it can possibly be read.
static void tuneAlertScreen(const char *l1, const char *l2, const char *l3) {
  const int W = epd.width(), H = epd.height();
  epd.fillScreen(15);                      // black field: unmissable
  epd.setTextColor(0);
  epd.setTextSize(4);
  epd.setCursor(40, 60);
  epd.print(l1);
  epd.setTextSize(3);
  epd.setCursor(40, 140);
  epd.print(l2);
  epd.setTextSize(2);
  epd.setCursor(40, 200);
  epd.print(l3);
  epd.setCursor(40, H - 60);
  epd.print("Then TAP THE SCREEN to carry on  (serial: any key, q = abort)");
  tunePaint();
}

// An outcome the user must actually see. E-paper takes a moment to settle,
// so this WAITS for a tap rather than flashing by on a timer — a message
// that vanishes before the screen has resolved reads as "nothing happened".
static void tuneConfirmScreen(const char *title, const char *l2, const char *l3) {
  const int W = epd.width(), H = epd.height();

  // On a panel-sized screen the artwork carries the message; the empty
  // rounded panel it leaves is exactly where the button and status go.
  Btn ok;
  if (W == TUNEUP_SPLASH_W && H == TUNEUP_SPLASH_H) {
    tuneupSplashDraw(epd.getBuffer(), W, H);
    ok = { TUNEUP_SPLASH_BTN_X, TUNEUP_SPLASH_BTN_Y,
           TUNEUP_SPLASH_BTN_W, TUNEUP_SPLASH_BTN_H };
    // Status card down the artwork's clear left field: the outcome, a
    // 16-level check, and the button. It sits on white so none of it has
    // to be read against the photograph.
    const int cardX = 14, cardY = 246, cardW = 282, cardH = 250;
    epd.fillRect(cardX, cardY, cardW, cardH, 0);
    epd.drawRect(cardX, cardY, cardW, cardH, 15);

    epd.setTextColor(15);
    epd.setTextSize(3);
    epd.setCursor(cardX + 12, cardY + 14);
    epd.print(title);

    // 16 patches, two rows of eight, each boxed and gapped so adjacent
    // levels can be compared directly against a white surround. Judging
    // level separation through a photograph confuses the panel's level
    // spacing with the dither; this shows the levels themselves.
    const int pw = 30, ph = 30, gap = 3, gx = cardX + 12, gy = cardY + 66;
    epd.setTextSize(1);
    epd.setCursor(gx, gy - 12);
    epd.print("16 LEVELS");
    for (int i = 0; i < 16; i++) {
      const int cx = gx + (i % 8) * (pw + gap);
      const int cy = gy + (i / 8) * (ph + gap);
      epd.fillRect(cx, cy, pw, ph, (uint8_t)i);
      epd.drawRect(cx, cy, pw, ph, 15);
    }

    epd.fillRect(ok.x, ok.y, ok.w, ok.h, 0);
    epd.drawRect(ok.x, ok.y, ok.w, ok.h, 15);
    epd.drawRect(ok.x + 1, ok.y + 1, ok.w - 2, ok.h - 2, 15);
    epd.setTextSize(3);
    epd.setCursor(ok.x + (ok.w - 36) / 2, ok.y + (ok.h - 24) / 2);
    epd.print("OK");
  } else {
    epd.fillScreen(0);
    epd.setTextColor(15);
    epd.setTextSize(5);
    epd.setCursor(40, 90);
    epd.print(title);
    epd.setTextSize(3);
    epd.setCursor(40, 190);
    epd.print(l2);
    epd.setTextSize(2);
    epd.setCursor(40, 250);
    epd.print(l3);
    ok = { W / 2 - 160, 360, 320, 110 };
    drawBtn(ok, "OK");
  }

  // Activate LAST, immediately before driving. The flash only helps if the
  // drive follows at once — leave a gap (a JPEG decode and a dither pass,
  // several seconds) and the particles settle back before the image is
  // painted, so the agitation is wasted. Tony spotted this from the long
  // pause between the flashing stopping and the picture appearing.
  tuneClear();
  epd.paint();
  while (!epd.driver().paintIdle()) delay(10);

  // This page has NO timeout — it waits for the user. Two things are needed
  // to make that hold in practice:
  //  * the tap must land ON the button. Accepting a tap anywhere let a
  //    single phantom dismiss the page, which looked exactly like a timeout.
  //  * touch is ignored until the panel has settled. A full-screen 16-grey
  //    paint throws transients into the GT911, and polling fast enough to
  //    catch real presses also catches those.
  delay(700);
  while (Serial.available()) Serial.read();
  touchRewake();
  int px, py;
  touchTapped(px, py);                    // consume anything already latched
  Serial.printf("[tune] %s - %s (tap OK, or any key)\n", title, l2);
  for (;;) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); return; }
    if (touchTapped(px, py)) {
      Serial.printf("[touch] tap at %d,%d (OK %d..%d/%d..%d)\n", px, py,
                    ok.x, ok.x + ok.w, ok.y, ok.y + ok.h);
      if (hitBtn(ok, px, py)) return;
    }
    delay(8);        // must out-pace GT_FRAME_GAP_MAX_MS (40ms) or every
                     // press is discarded as an EPD phantom
  }
}

// Park until the user acknowledges. Returns false if they abort.
static bool tuneAlertAndWait() {
  Serial.println("\n[tune] *** SCANNER NOT RESPONDING ***");
  Serial.println("[tune] *** power-cycle the scanner, then tap the panel ***");
  tuneAlertScreen("SCANNER ERROR", "Please reset the scanner",
                  "The scanner stopped answering. Switch it off and on again.");
  touchRewake();
  for (;;) {
    if (Serial.available()) return Serial.read() != 'q';
    int px, py;
    if (touchTapped(px, py)) return true;
    delay(8);        // fast enough for the touch confirmation window
  }
}

static bool tuneScan(float x0, float y0, float w, float h) {
  // Flatbeds wedge under sustained scanning (the Canon's BJNP stack stops
  // accepting connections for a minute or so), so be patient before
  // troubling anyone: several quiet retries, and only then ask for hands.
  for (int attempt = 0;; attempt++) {
    delay(TUNE_SCAN_GAP_MS);
    const size_t len = esclScan(x0, y0, w, h, TUNE_DPI);
    if (len && decodeGrayFromScan(len)) return true;
    if (attempt < 3) {
      Serial.printf("[tune] scan failed (attempt %d/4), waiting 20 s...\n", attempt + 1);
      delay(20000);
      continue;
    }
    if (tuneRecovering) return false;      // don't nest recovery inside recovery
    if (!tuneAlertAndWait()) return false;

    // They may have picked the board up to read that, so the panel's
    // position on the glass is no longer trustworthy: re-register, then
    // repaint whatever we were measuring and start the scan over.
    tuneRecovering = true;
    const bool ok = tuneRegister();
    tuneRecovering = false;
    if (!ok) { Serial.println("[tune] could not re-register after recovery"); return false; }
    if (tuneRepaint) tuneRepaint();
    attempt = -1;                          // fresh retry budget
  }
}

// Adaptive bounding box for the wide registration scan: cutoffs derive
// from the strongest row/column counts, not the window size.
// Pick the dark/light split from the scan itself rather than assuming one.
//
// The threshold here used to be a hardcoded 120: a pixel counted as "dark"
// only if it scanned below that. That silently encoded ONE panel's contrast.
// A second LilyGo, whose black renders lighter under the preset tables, put
// no pixels at all below 120 - so registration reported "no dark panel found"
// with the board sitting plainly on the glass.
//
// The scan is reliably bimodal at this point (bright scanner bed, dark panel
// painted full black), so the split can be measured. Percentiles rather than
// min/max, because a single hot pixel or a shadow at the platen edge would
// drag either extreme. Returns 0 if the image has no usable contrast, which
// is the honest answer when nothing is on the glass.
static uint8_t tuneDarkThreshold() {
  const int W = grayW, H = grayH;
  uint32_t hist[256] = {0};
  const size_t n = (size_t)W * H;
  for (size_t i = 0; i < n; i++) hist[grayImg[i]]++;

  const uint32_t lowCut = (uint32_t)(n * 0.02f), highCut = (uint32_t)(n * 0.98f);
  int lo = 0, hi = 255;
  uint32_t acc = 0;
  for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc >= lowCut)  { lo = v; break; } }
  acc = 0;
  for (int v = 0; v < 256; v++) { acc += hist[v]; if (acc >= highCut) { hi = v; break; } }

  // Too little spread means there is no panel/bed distinction to find.
  if (hi - lo < 25) return 0;
  // 45% of the way up from dark to light: comfortably above the panel's own
  // black, comfortably below the bed, and it tracks whatever contrast this
  // particular panel and scanner happen to produce.
  return (uint8_t)(lo + 0.45f * (hi - lo));
}

static bool tuneBBoxAdaptive(uint8_t thresh, int *bx0, int *by0,
                             int *bx1, int *by1) {
  const int W = grayW, H = grayH;
  if (W > 2048 || H > 2048) return false;
  static int rowsC[2048], colsC[2048];
  memset(rowsC, 0, sizeof(int) * H);
  memset(colsC, 0, sizeof(int) * W);
  for (int y = 0; y < H; y++) {
    const uint8_t *row = grayImg + (size_t)y * W;
    for (int x = 0; x < W; x++)
      if (row[x] < thresh) { rowsC[y]++; colsC[x]++; }
  }
  int rmax = 0, cmax = 0;
  for (int y = 0; y < H; y++) rmax = max(rmax, rowsC[y]);
  for (int x = 0; x < W; x++) cmax = max(cmax, colsC[x]);
  const int rcut = max(40, rmax / 2), ccut = max(40, cmax / 2);
  int x0 = W, x1 = -1, y0 = H, y1 = -1;
  for (int y = 0; y < H; y++)
    if (rowsC[y] > rcut) { if (y < y0) y0 = y; if (y > y1) y1 = y; }
  for (int x = 0; x < W; x++)
    if (colsC[x] > ccut) { if (x < x0) x0 = x; if (x > x1) x1 = x; }
  if (x1 < 0 || y1 < 0 || x1 - x0 < 60 || y1 - y0 < 40) return false;
  *bx0 = x0; *by0 = y0; *bx1 = x1; *by1 = y1;
  return true;
}

// Ring anchors: from the scan centre (always inside the white core), walk
// outward until the dark ring bars are hit. Cable shadows and lid wedges
// sit OUTSIDE the ring and can never move its inner edges.
static bool tuneFindRing() {
  const int W = grayW, H = grayH;
  const int cx = W / 2, cy = H / 2;
  // per-column / per-row dark counts
  auto colDark = [&](int x) {
    int n = 0;
    for (int y = 0; y < H; y++) n += (grayImg[(size_t)y * W + x] < 120);
    return n;
  };
  auto rowDark = [&](int y) {
    int n = 0;
    const uint8_t *row = grayImg + (size_t)y * W;
    for (int x = 0; x < W; x++) n += (row[x] < 120);
    return n;
  };
  const int ccut = (int)(H * 0.25f), rcut = (int)(W * 0.25f);
  int ax0 = -1, ax1 = -1, ay0 = -1, ay1 = -1;
  for (int x = cx; x >= 0; x--)   if (colDark(x) > ccut) { ax0 = x; break; }
  for (int x = cx; x < W; x++)    if (colDark(x) > ccut) { ax1 = x; break; }
  for (int y = cy; y >= 0; y--)   if (rowDark(y) > rcut) { ay0 = y; break; }
  for (int y = cy; y < H; y++)    if (rowDark(y) > rcut) { ay1 = y; break; }
  if (ax0 < 0 || ax1 < 0 || ay0 < 0 || ay1 < 0) return false;
  // Sanity, as a fraction of the frame so the check survives any dpi.
  if (ax1 - ax0 < grayW / 4 || ay1 - ay0 < grayH / 4) return false;
  tgeo.ax0 = ax0; tgeo.ax1 = ax1; tgeo.ay0 = ay0; tgeo.ay1 = ay1;
  Serial.printf("[tune] ring anchors x %d..%d  y %d..%d\n", ax0, ax1, ay0, ay1);
  return true;
}

// Map a panel-pixel rect (interior coordinates) to scan pixels via the
// ring anchors, honouring the face-down flips.
static void tunePanelRect(int px0, int px1, int py0, int py1,
                          int *sx0, int *sx1, int *sy0, int *sy1) {
  const float sx = (float)(tgeo.ax1 - tgeo.ax0) / (epd.width() - 2 * RING_T);
  const float sy = (float)(tgeo.ay1 - tgeo.ay0) / (epd.height() - 2 * RING_T);
  auto mapx = [&](int x) {
    const float o = (x - RING_T) * sx;
    return (int)(tgeo.flipx ? tgeo.ax1 - o : tgeo.ax0 + o);
  };
  auto mapy = [&](int y) {
    const float o = (y - RING_T) * sy;
    return (int)(tgeo.flipy ? tgeo.ay1 - o : tgeo.ay0 + o);
  };
  int a = mapx(px0), b = mapx(px1);
  *sx0 = min(a, b); *sx1 = max(a, b);
  a = mapy(py0); b = mapy(py1);
  *sy0 = min(a, b); *sy1 = max(a, b);
}

static float tuneRectMean(int cx0, int cx1, int ry0, int ry1) {
  cx0 = max(cx0, 0); ry0 = max(ry0, 0);
  cx1 = min(cx1, grayW); ry1 = min(ry1, grayH);
  uint64_t sum = 0;
  uint32_t n = 0;
  for (int y = ry0; y < ry1; y++) {
    const uint8_t *row = grayImg + (size_t)y * grayW;
    for (int x = cx0; x < cx1; x++) { sum += row[x]; n++; }
  }
  return n ? (float)sum / n : 0.0f;
}

// Column i's interior x-range in panel px (25% margins inside the 60 px bar).
static void tuneColSpan(int i, int *px0, int *px1) {
  const int bw = epd.width() / 16;
  *px0 = i * bw + bw / 4;
  *px1 = (i + 1) * bw - bw / 4;
}

// Measure the match card via the anchors: per-column means of the ref and
// native bands. No thresholding — geometry is fixed.
static bool tuneMeasureCardOnce(float ref[16], float nat[16]) {
  tuneRepaint = tuneRepaintCard;
  if (!tuneScan(tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm)) return false;
  const int H = epd.height();
  for (int i = 0; i < 16; i++) {
    int px0, px1, sx0, sx1, sy0, sy1;
    tuneColSpan(i, &px0, &px1);
    tunePanelRect(px0, px1, (int)(H * 0.06f), (int)(H * 0.42f), &sx0, &sx1, &sy0, &sy1);
    ref[i] = tuneRectMean(sx0, sx1, sy0, sy1);
    tunePanelRect(px0, px1, (int)(H * 0.58f), (int)(H * 0.94f), &sx0, &sx1, &sy0, &sy1);
    nat[i] = tuneRectMean(sx0, sx1, sy0, sy1);
  }
  return true;
}

// Gradient-corrected errors: the L0 and L15 pairs are identical drives top
// and bottom; interpolate their deltas by level.
// Average several scans of the SAME painted card.
//
// Once the errors are down near the tolerance, a scan's own noise — a
// unit or two — is a large fraction of what is being corrected, and the
// converger starts chasing that noise instead of the panel. Averaging two
// reads cuts it by root two. The card is still on the glass after a scan,
// so this costs only the extra scan: no repaint, no extra drive, and the
// panel is not disturbed between the two reads.
static int tuneScansPerMeasure = 1;

static bool tuneMeasureCard(float ref[16], float nat[16]) {
  if (!tuneMeasureCardOnce(ref, nat)) return false;
  for (int k = 1; k < tuneScansPerMeasure; k++) {
    float r2[16], n2[16];
    if (!tuneMeasureCardOnce(r2, n2)) break;      // keep the reads we have
    for (int i = 0; i < 16; i++) {
      ref[i] = (ref[i] * k + r2[i]) / (k + 1);
      nat[i] = (nat[i] * k + n2[i]) / (k + 1);
    }
  }
  return true;
}

static void tuneErrors(const float ref[16], const float nat[16], float err[16]) {
  const float g0 = nat[0] - ref[0], g15 = nat[15] - ref[15];
  for (int i = 0; i < 16; i++)
    err[i] = (nat[i] - ref[i]) - (g0 + (g15 - g0) * (i / 15.0f));
}

static void tunePrintBands(const char *tag, const float ref[16], const float nat[16]) {
  Serial.printf("[tune] %s ref:", tag);
  for (int i = 0; i < 16; i++) Serial.printf(" %3.0f", ref[i]);
  Serial.println();
  Serial.printf("[tune] %s nat:", tag);
  for (int i = 0; i < 16; i++) Serial.printf(" %3.0f", nat[i]);
  Serial.println();
}

// ============================================================================
// Registration: rough locate -> orientation -> ring anchors.
// ============================================================================
static bool tuneRegister() {
  Serial.println("[tune] registration: full black, wide scan...");
  tuneFill(15);
  tuneClear();
  tunePaint();
  tuneRepaint = tuneRepaintBlack;
  if (!tuneScan(0, 0, TUNE_REG_W_MM, TUNE_REG_H_MM)) return false;
  int x0, y0, x1, y1;
  const uint8_t dth = tuneDarkThreshold();
  Serial.printf("[tune] dark threshold %d (measured from the scan)\n", dth);
  if (!dth || !tuneBBoxAdaptive(dth, &x0, &y0, &x1, &y1)) {
    Serial.println("[tune] no dark panel found - is the board face-down on the glass?");
    return false;
  }
  // Generous margins: the window only has to CONTAIN the panel, and the
  // ring anchors (found by walking out from the centre) are indifferent
  // to anything beyond it. Clipping the panel, by contrast, loses a ring
  // bar and kills the run.
  const float mmpp = 25.4f / TUNE_DPI;
  tgeo.x0mm = max(0.0f, x0 * mmpp - 5.0f);
  tgeo.y0mm = max(0.0f, y0 * mmpp - 5.0f);
  tgeo.wmm  = (x1 - x0) * mmpp + 12.0f;
  tgeo.hmm  = (y1 - y0) * mmpp + 12.0f;
  Serial.printf("[tune] panel near %.0f,%.0f  %.0fx%.0f mm\n",
                tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm);

  Serial.println("[tune] orientation: quadrant mark...");
  tunePaintQuadrant();
  tuneClear();
  tunePaint();
  tuneRepaint = tuneRepaintQuad;
  if (!tuneScan(tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm)) return false;
  const uint8_t qth = tuneDarkThreshold();
  if (!qth || !tuneBBoxAdaptive(qth, &x0, &y0, &x1, &y1)) {
    Serial.println("[tune] quadrant mark not found");
    return false;
  }
  tgeo.flipx = ((x0 + x1) / 2) > grayW / 2;
  tgeo.flipy = ((y0 + y1) / 2) > grayH / 2;
  Serial.printf("[tune] orientation: flipx=%d flipy=%d\n", tgeo.flipx, tgeo.flipy);

  Serial.println("[tune] geometry: anchor ring...");
  tunePaintRing();
  tuneClear();
  tunePaint();
  tuneRepaint = tuneRepaintRing;
  if (!tuneScan(tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm)) return false;
  if (!tuneFindRing()) {
    Serial.println("[tune] anchor ring not found");
    return false;
  }
  tgeo.valid = true;
  return true;
}

// ============================================================================
// Trains: (d, w, g, r) grammar
// ============================================================================
static void tuneTrainOfShape(int d, int w, int g, int r, uint8_t out[13]) {
  memset(out, 0, 13);
  int p = 0;
  for (int i = 0; i < d && p < 13; i++) out[p++] = 1;
  for (int i = 0; i < w && p < 13; i++) out[p++] = 2;
  p += g;                              // float gap
  if (p > 13) p = 13;
  for (int i = 0; i < r && p < 13; i++) out[p++] = 1;
}

static void tuneTrainOf(int lv, uint8_t out[13]) {
  if (tl[lv].hasRaw) { memcpy(out, tl[lv].raw, 13); return; }
  tuneTrainOfShape(tl[lv].d, tl[lv].w, tl[lv].g, tl[lv].r, out);
}

// Net charge of whatever train a level ACTUALLY has: darkens minus whitens,
// counted from the drive sequence rather than inferred from (d,w,g,r).
// Inferring it breaks the moment a level goes raw - the remove would be
// charge-matched to a shape the level no longer has, and a DC imbalance is
// ghosting.
static int tuneNetOf(int lv) {
  uint8_t t[13];
  tuneTrainOf(lv, t);
  int net = 0;
  for (int i = 0; i < 13; i++) { if (t[i] == 1) net++; else if (t[i] == 2) net--; }
  return net;
}

static void tuneUploadApplies() {
  uint8_t t[13];
  for (int lv = 1; lv <= 15; lv++) {
    tuneTrainOf(lv, t);
    epd.driver().setDecisionTrain((uint8_t)(lv << 1), t);
  }
}

// Parse a raw train into the grammar (d 1s, w 2s, g 0s, r 1s).
static void tuneParse(const uint8_t t[13], TuneLv *L) {
  int p = 0;
  L->d = L->w = L->g = L->r = 0;
  while (p < 13 && t[p] == 1) { L->d++; p++; }
  while (p < 13 && t[p] == 2) { L->w++; p++; }
  while (p < 13 && t[p] == 0) { L->g++; p++; }
  while (p < 13 && t[p] == 1) { L->r++; p++; }
  if (!L->r) L->g = 0;                 // trailing floats aren't a gap
}

static void tuneSeedShapes() {
  const uint8_t (*ap)[13] = epd.driver()._config.trains.g16_apply;
  for (int lv = 1; lv <= 15; lv++) {
    uint8_t t[13];
    if (ap) {
      memcpy(t, ap[lv], 13);
    } else {
      memset(t, 0, 13);
      const int dose = (lv * 2 * 13 + 7) / 15;
      const int full = dose / 2;
      for (int p = 0; p < full; p++) t[p] = 1;
      if ((dose & 1) && full + 1 < 13) { t[full] = 1; t[full + 1] = 2; }
    }
    tl[lv] = {};
    tuneParse(t, &tl[lv]);
    tl[lv].gd = 15.0f; tl[lv].gw = 22.0f; tl[lv].gr = 12.0f; tl[lv].gg = 5.0f;
    tl[lv].bestAbs = 1e9f;
    tl[lv].bd = tl[lv].d; tl[lv].bw = tl[lv].w;
    tl[lv].bg = tl[lv].g; tl[lv].br = tl[lv].r;
  }
  tl[15].d = 13; tl[15].w = 0; tl[15].g = 0; tl[15].r = 0;   // black anchor
  memset(removeExtra, 0, sizeof(removeExtra));
}

// Predicted landing of a shape, in scanner units: the measured pure-dose
// curve for its darken run, plus each whiten's lift, minus each
// re-darken's bite, plus the float gap's weakening of those re-darkens.
// Gains start as rough constants and are learned per level as the
// converger observes what its edits actually do.
static float tunePredict(const TuneLv &L, int d, int w, int g, int r) {
  const int dd = constrain(d, 1, 13);
  return tuneDose[dd] + w * L.gw - r * L.gr + g * L.gg;
}

static void tuneSeedModel(bool reserveProbes);  // defined below, used by the probe

// ============================================================================
// Float-gap gain probe.
//
// The gap was the one knob whose gain was never measured. tuneCalibrateGains()
// solves the whiten and re-darken gains from the first card, but gg kept its
// seeded guess of 5, and the online learner can only correct it AFTER a gap
// edit has been taken — which the planner will not do while it believes the
// move is worth a handful of units. Chicken-and-egg: the knob has to be
// selected once before its gain can be learned, and its gain is what decides
// whether it gets selected.
//
// That guess was never safe. A float pass is not a brief pause: at the
// 16-grey pass period it hands the pixel a WHOLE period of field-free settle
// on top of the inter-pass floor (4 ms of a 15-20 ms period), so the true
// effect can be several times the assumed 5. And the error runs in the
// direction that keeps the knob from ever being tried — the same failure
// tuneCalibrateGains() warns about for the whiten ("being 40% wrong about a
// 30-unit move is what produced the overshoot cascades").
//
// Method mirrors the dose ladder: hold (d, w, r) fixed per base, sweep g
// across the columns, read every column from one scan — so all cells share
// one panel state and drift is common-mode. Three bases, because one base is
// one panel region's opinion. The gap needs w > 0 and r > 0 to be legal, and
// every cell here fits d+w+g+r <= 13 at g = 3.
// ============================================================================
static bool tuneProbeGap() {
  Serial.println("[tune] float-gap ladder...");
  struct Base { uint8_t d, w, r; };
  static const Base bases[] = { { 4, 1, 1 }, { 6, 1, 2 }, { 3, 2, 1 } };
  const int NB = (int)(sizeof(bases) / sizeof(bases[0]));
  const int NG = 4;                       // g = 0..3, the planner's own range

  uint8_t t[13];
  int col[NB][NG];
  int c = 1;
  for (int b = 0; b < NB; b++)
    for (int g = 0; g < NG; g++, c++) {
      col[b][g] = (c <= 14) ? c : 0;
      if (!col[b][g]) continue;
      memset(t, 0, 13);
      int p = 0;                          // same emitter shape as tuneTrainOf()
      for (int i = 0; i < bases[b].d && p < 13; i++) t[p++] = 1;
      for (int i = 0; i < bases[b].w && p < 13; i++) t[p++] = 2;
      p += g;
      if (p > 13) p = 13;
      for (int i = 0; i < bases[b].r && p < 13; i++) t[p++] = 1;
      epd.driver().setDecisionTrain((uint8_t)(col[b][g] << 1), t);
    }

  // Column 15 is the deep anchor the gradient correction needs; any columns
  // the bases did not claim get the same drive so they cannot mislead a scan.
  memset(t, 0, 13);
  for (int i = 0; i < 13; i++) t[i] = 1;
  epd.driver().setDecisionTrain((uint8_t)(15 << 1), t);
  for (int lv = c; lv <= 14; lv++)
    epd.driver().setDecisionTrain((uint8_t)(lv << 1), t);

  tunePaintMatchCard();
  tuneClear();
  tunePaint();
  // A slope over four columns is more noise-sensitive than an absolute
  // landing, and this one number seeds every shape the converger starts
  // from — so take the averaged read here regardless of how far out the
  // run currently is. Costs one extra scan, no repaint, no extra drive.
  const int savedScans = tuneScansPerMeasure;
  tuneScansPerMeasure = max(savedScans, 2);
  float ref[16], nat[16];
  const bool measured = tuneMeasureCard(ref, nat);
  tuneScansPerMeasure = savedScans;
  if (!measured) return false;
  tunePrintBands("gap", ref, nat);

  const float g0 = nat[0] - ref[0], g15 = nat[15] - ref[15];
  float land[16];
  for (int i = 0; i < 16; i++)
    land[i] = nat[i] - (g0 + (g15 - g0) * (i / 15.0f));

  // Least-squares slope of landing vs gap length, per base.
  float slope[NB];
  float sum = 0;
  int n = 0;
  for (int b = 0; b < NB; b++) {
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    int m = 0;
    for (int g = 0; g < NG; g++) {
      if (!col[b][g]) continue;
      const float y = land[col[b][g]];
      sx += g; sy += y; sxx += (float)g * g; sxy += g * y; m++;
    }
    if (m < 2) continue;
    const float den = m * sxx - sx * sx;
    if (fabsf(den) < 1e-3f) continue;
    slope[n] = (m * sxy - sx * sy) / den;
    Serial.printf("[tune] gap base (%d,%d,%d): %+.1f units/pass\n",
                  bases[b].d, bases[b].w, bases[b].r, slope[n]);
    sum += slope[n]; n++;
  }
  if (!n) return false;

  // A null result must set the gain to ZERO, not to some floor. Clamping a
  // measured nothing up to a minimum makes the gap the FINEST knob in the
  // set, and the seeder then reaches for it to mop up every last unit of
  // quantisation residual — buying predicted units the panel never delivers,
  // at one pass of train length each.
  //
  // Two tests, because a mean on its own hides disagreement. The effect must
  // clear scan noise AND the bases must agree on its sign. (First LilyGo run
  // at the 20 ms NORMAL period: -2.1, +0.2, +0.3 — mean -0.5 with the sign
  // split three ways, and the old floor turned that into a confident 1.0.)
  //
  // Zero is final for the run: tunePredict() then values a gap at nothing,
  // the length penalty makes it strictly negative-scoring, so tunePlanOne()
  // never takes one and the online learner never gets a lastOp==4 to revise.
  // That is the intent — the probe decides, not a guess that outlived it.
  const float mean = sum / n;
  bool agree = true;
  for (int i = 1; i < n; i++)
    if ((slope[i] < 0) != (slope[0] < 0)) agree = false;

  float gg = 0.0f;
  if (fabsf(mean) >= tuneGapNoise() && agree)
    gg = constrain(fabsf(mean), tuneGapNoise(), 40.0f);

  for (int lv = 1; lv <= 15; lv++) tl[lv].gg = gg;

  if (gg == 0.0f) {
    Serial.printf("[tune] gap gain: none detected (mean %+.1f, bases %s) — "
                  "treating float gaps as neutral\n",
                  mean, agree ? "agree" : "disagree on sign");
  } else {
    // tunePredict() carries the gap as +g*gg and the online learner stores
    // magnitudes, so a genuine DARKENING gap cannot be represented — say so
    // rather than bury the sign in the fabsf() below.
    if (mean < 0)
      Serial.println("[tune] WARNING: gap darkened the landing consistently — "
                     "tunePredict's +g*gg sign is wrong for this panel");
    Serial.printf("[tune] measured gap gain: %.1f units/pass (%d bases, "
                  "5.0 was assumed)\n", gg, n);
  }
  return true;
}

// ============================================================================
// Pure-dose ladder probe. Column n runs {1 x n}; measured against the same
// frame's dither refs. Yields the dose curve (monotone-enforced), the
// per-level d-gains, and dose-inversion seeds for the deep levels.
// ============================================================================
static bool tuneProbe() {
  Serial.println("[tune] pure-dose ladder...");
  uint8_t t[13];
  for (int lv = 1; lv <= 15; lv++) {
    memset(t, 0, 13);
    for (int i = 0; i < min(lv, 13); i++) t[i] = 1;
    epd.driver().setDecisionTrain((uint8_t)(lv << 1), t);
  }
  tunePaintMatchCard();
  tuneClear();
  tunePaint();
  float ref[16], nat[16];
  if (!tuneMeasureCard(ref, nat)) return false;
  tunePrintBands("probe", ref, nat);
  const float g0 = nat[0] - ref[0], g15 = nat[15] - ref[15];
  for (int n = 1; n <= 13; n++)
    tuneDose[n] = nat[n] - (g0 + (g15 - g0) * (n / 15.0f));
  for (int n = 2; n <= 13; n++)                     // saturation is monotone;
    if (tuneDose[n] > tuneDose[n - 1]) tuneDose[n] = tuneDose[n - 1];  // clamp noise
  for (int i = 0; i < 16; i++) probeRef[i] = ref[i];

  Serial.print("[tune] dose curve: ");
  for (int n = 1; n <= 13; n++) Serial.printf("%.0f ", tuneDose[n]);
  Serial.println();

  // d-gain per level from the local slope at its seed depth
  for (int lv = 1; lv <= 14; lv++) {
    const int d = constrain((int)tl[lv].d, 1, 12);
    tl[lv].gd = constrain(fabsf(tuneDose[d + 1] - tuneDose[d]), 2.0f, 30.0f);
  }

  // Model seeding for the mid/deep levels: at a new pass period the
  // shipped shapes can be tens of units off, so build each level from the
  // measured curve instead. Pure darkens alone cannot do it — the curve
  // saturates, and several levels fall inside one darken step — so the
  // search runs over the whole (d,w,g,r) grammar. Levels 1-5 keep their
  // shipped shapes: they encode ghost behaviour, and the converger can
  // now refine them with the same grammar.
  //
  // The gap gain must be real BEFORE seeding: tuneSeedModel() scores shapes
  // through tunePredict(), so a wrong gg biases which shapes it picks, not
  // just how the converger edits them later.
  if (!tuneProbeGap())
    Serial.println("[tune] gap probe failed - keeping the assumed gap gain");
  tuneSeedModel(true);   // pre-calibration: guarantee the whiten probes
  return true;
}

// Best shape for one level under the model. whitenOnly restricts the search
// to shapes carrying whitens and NO re-darkens — the structure
// tuneCalibrateGains() needs in order to solve the whiten gain at all.
// Returns the score of the shape it chose, so callers can compare what a
// restriction costs against the free choice.
static float tuneSolveShape(const TuneLv &L, float target, bool whitenOnly,
                            int *od, int *ow, int *og, int *orr) {
  const int w0 = whitenOnly ? 1 : 0;
  const int rmax = whitenOnly ? 0 : 5;
  int bd = 1, bw = w0, bg = 0, br = 0;
  float best = 1e9f;
  for (int d = 1; d <= 13; d++)
    for (int w = w0; w <= 4; w++)
      for (int g = 0; g <= 3; g++)
        for (int r = 0; r <= rmax; r++) {
          if (w == 0 && (r || g)) continue;   // re-darkens need a whiten first
          if (r == 0 && g) continue;          // a gap only weakens re-darkens
          if (d + w + g + r > 13) continue;
          const float score = fabsf(tunePredict(L, d, w, g, r) - target)
                            + 0.10f * (d + w + g + r);
          if (score < best) { best = score; bd = d; bw = w; bg = g; br = r; }
        }
  *od = bd; *ow = bw; *og = bg; *orr = br;
  return best;
}

// Build each level's shape from the model. Deliberately varies the
// structure across levels (some with re-darkens, some without) — that
// spread is what lets tuneCalibrateGains() solve for the real gains from
// the first measurement. reserveProbes ENFORCES that spread; pass it only
// for the seeding that precedes calibration, since afterwards the gains are
// already solved and the constraint would cost seed fit for nothing.
static void tuneSeedModel(bool reserveProbes) {
  int sd[15], sw[15], sg[15], sr[15];   // the shape each level will take
  int qd[15], qw[15];                   // its best whiten-only alternative
  float cost[15];                       // what taking that alternative costs

  for (int lv = 6; lv <= 14; lv++) {
    int d, w, g, r;
    const float free_score =
        tuneSolveShape(tl[lv], probeRef[lv], false, &d, &w, &g, &r);
    sd[lv] = d; sw[lv] = w; sg[lv] = g; sr[lv] = r;

    int ad, aw, ag, ar;
    const float probe_score =
        tuneSolveShape(tl[lv], probeRef[lv], true, &ad, &aw, &ag, &ar);
    qd[lv] = ad; qw[lv] = aw;
    cost[lv] = probe_score - free_score;
  }

  // tuneCalibrateGains() can solve the whiten gain ONLY from levels carrying
  // whitens and no re-darkens. The comment above always claimed that spread
  // was deliberate, but nothing enforced it — and on the first LilyGo run
  // with the gap knob switched off, every level 6-14 seeded with re-darkens.
  // n came out 0 and gw fell back to a constant measured on a DIFFERENT
  // board, roughly twice this panel's own figure, on the largest knob the
  // planner owns. Convert wherever the constraint costs least, and report
  // both the level and the price so a bad trade is visible rather than silent.
  if (reserveProbes) {
    int have = 0;
    for (int lv = 6; lv <= 14; lv++)
      if (sw[lv] > 0 && sr[lv] == 0) have++;

    while (have < TUNE_WHITEN_PROBES) {
      int pick = -1;
      for (int lv = 6; lv <= 14; lv++) {
        if (sw[lv] > 0 && sr[lv] == 0) continue;   // already serves as a probe
        if (qw[lv] <= 0) continue;                 // no legal whiten-only shape
        if (pick < 0 || cost[lv] < cost[pick]) pick = lv;
      }
      if (pick < 0) break;
      // A probe is only worth having if the constraint is CHEAP. Without this
      // ceiling the loop took the least-bad candidate no matter how bad it
      // was: tuneSolveShape(whitenOnly) always returns something, so a level
      // whose best whiten-only shape lands a whole level-step off target got
      // reserved anyway. That is how one run put L6 twenty-two units light and
      // came out coarser than the built-in tables - one wrecked level ruining
      // an otherwise tight set. Falling back to the constant gw costs accuracy
      // in one gain estimate; wrecking a level costs a visible band.
      if (cost[pick] > tuneProbeMaxCost()) {
        Serial.printf("[tune] no cheap whiten probe left (best L%d would cost "
                      "%.1f > %.1f) - not reserving\n",
                      pick, cost[pick], tuneProbeMaxCost());
        break;
      }
      sd[pick] = qd[pick]; sw[pick] = qw[pick]; sg[pick] = 0; sr[pick] = 0;
      Serial.printf("[tune] L%d reserved as a whiten probe (+%.1f seed error)\n",
                    pick, cost[pick]);
      have++;
    }
    if (have < TUNE_WHITEN_PROBES)
      Serial.printf("[tune] WARNING: only %d whiten probe(s) — gw may fall back "
                    "to the built-in constant\n", have);
  }

  for (int lv = 6; lv <= 14; lv++) {
    TuneLv &L = tl[lv];
    L.d = sd[lv]; L.w = sw[lv]; L.g = sg[lv]; L.r = sr[lv];
    L.hasRaw = false;                    // re-seeding returns it to the grammar
    L.bestAbs = 1e9f;                    // shapes changed: old bests are void
    L.bd = L.d; L.bw = L.w; L.bg = L.g; L.br = L.r;
    L.gd = constrain(fabsf(tuneDose[min((int)L.d + 1, 13)] - tuneDose[(int)L.d]),
                     2.0f, 30.0f);
    Serial.printf("[tune] seed L%-2d (%d,%d,%d,%d) -> %.0f  (ref %.0f)\n",
                  lv, L.d, L.w, L.g, L.r,
                  tunePredict(L, L.d, L.w, L.g, L.r), probeRef[lv]);
  }
}

// Solve for the whiten and re-darken gains from the first measurement of
// the seeded shapes. Every level's landing is known (ref + err) and its
// pure-darken base is known (the dose curve), so the structure terms fall
// out: levels carrying whitens but no re-darkens give the whiten's lift
// directly, and that in turn unlocks the re-darken's bite on the rest.
// Guessed constants are never good enough here — this panel's whiten runs
// ~31 units against an assumed 22, and being 40% wrong about a 30-unit
// move is what produced the overshoot cascades.
static void tuneCalibrateGains() {
  float sw = 0;
  int nw = 0;
  for (int lv = 6; lv <= 14; lv++) {
    const TuneLv &L = tl[lv];
    if (L.w > 0 && L.r == 0) {
      const float landing = probeRef[lv] + L.err;
      sw += (landing - tuneDose[constrain((int)L.d, 1, 13)]) / L.w;
      nw++;
    }
  }
  const float gw = nw ? constrain(sw / nw, 8.0f, 70.0f) : 31.0f;

  float sr = 0;
  int nr = 0;
  for (int lv = 6; lv <= 14; lv++) {
    const TuneLv &L = tl[lv];
    if (L.r > 0) {
      const float landing = probeRef[lv] + L.err;
      // L.gg is measured by tuneProbeGap() before this runs; using the old
      // hardcoded 5.0 here would contradict it and corrupt the re-darken solve.
      const float without_r = tuneDose[constrain((int)L.d, 1, 13)] + L.w * gw + L.g * L.gg;
      sr += (without_r - landing) / L.r;
      nr++;
    }
  }
  const float gr = nr ? constrain(sr / nr, 4.0f, 60.0f) : 22.0f;

  for (int lv = 1; lv <= 14; lv++) { tl[lv].gw = gw; tl[lv].gr = gr; }
  Serial.printf("[tune] measured gains: whiten %.1f (n=%d)  re-darken %.1f (n=%d)\n",
                gw, nw, gr, nr);
}

// ============================================================================
// The damped converger.
//
// The edit search is a NEIGHBOURHOOD ENUMERATION over the whole (d,w,g,r)
// grammar rather than a list of single-parameter nudges. That matters
// because the dose curve saturates: between 2 and 3 darkens this panel
// jumps ~28 units, and several target levels live inside that one step.
// No single-parameter move can land there — the reachable set only gets
// finer when a whiten is added and re-darkens climb back in ~12-unit
// steps (the '1,1,2,1,1' shape the hand-tuned tables use). Enumerating
// lets the planner propose that whole restructure as ONE damped move
// whose predicted effect is small, instead of refusing every available
// nudge as too coarse.
//
// Predicted landing of a shape, in scanner units: the measured pure-dose
// curve for its darken run, plus each whiten's lift, minus each
// re-darken's bite, plus the float gap's weakening of those re-darkens.
// ============================================================================
// ---- spacing ----------------------------------------------------------
// The reference ladder already defines the intended spacing, so the gap
// the eye sees between levels i and i+1 is
//     (ref[i] - ref[i+1]) + (err[i] - err[i+1]).
// Only the second term is ours. A uniform bias shifts the whole ramp and
// the image dither absorbs it; opposite-signed neighbours halve or double
// a step, and THAT is what reads as a band. So banding is governed by the
// DIFFERENCE of adjacent errors, which is what these measure.
//
// Levels 0 and 15 anchor the ends: tuneErrors() pins both to exactly zero
// by construction, so they are honest neighbours for L1 and L14.
static float tuneGapErr(int lv) {
  const float a = fabsf(tl[lv].err - tl[lv - 1].err);
  const float b = fabsf(tl[lv].err - tl[lv + 1].err);
  return max(a, b);
}

static float tuneGapMax() {
  float g = 0;
  for (int i = 0; i <= 14; i++) g = max(g, fabsf(tl[i].err - tl[i + 1].err));
  return g;
}

// How bad a level is overall: its own placement plus what it does to the
// two gaps it sits between.
static float tuneBadness(int lv) {
  return fabsf(tl[lv].err) + TUNE_SPACING_W * tuneGapErr(lv);
}

// Enumerate legal shapes near the current one and take the best damped
// move. Returns false when nothing helps (level is quantisation-limited).
static bool tunePlanOne(int lv) {
  TuneLv &L = tl[lv];
  const float e = L.err;
  const float here = tunePredict(L, L.d, L.w, L.g, L.r);

  int bd = L.d, bw = L.w, bg = L.g, br = L.r;
  float bestScore = 1e9f;
  bool found = false;

  const int d0 = max(1, L.d - 3), d1 = min(13, L.d + 3);
  for (int d = d0; d <= d1; d++)
    for (int w = 0; w <= 4; w++)
      for (int g = 0; g <= 3; g++)
        for (int r = 0; r <= 5; r++) {
          if (w == 0 && (r || g)) continue;   // re-darkens need a whiten first
          if (r == 0 && g) continue;          // a gap only weakens re-darkens
          // Gap measured inert (tuneProbeGap gated gg to zero): such an edit
          // predicts NO move, and on a quantisation-limited level where every
          // real move overshoots, |e| + costs beats |e + move| + costs — so the
          // planner takes the no-op, reports success, and burns a cycle while
          // padding the train. Run 2 left L7 as (1,1,3,2), three dead floats.
          // Candidates with g == 0 stay enumerated, so a level carrying a gap
          // from a loaded blob can still shed it.
          if (g && L.gg <= 0.0f) continue;
          if (d + w + g + r > 13) continue;
          if (d == L.d && w == L.w && g == L.g && r == L.r) continue;

          // predicted change in this level's error
          const float move = tunePredict(L, d, w, g, r) - here;
          // Damping, widened while the gains are still guesses: an
          // unvalidated model is routinely off by 2x, and an overshoot
          // costs two cycles (the bad measurement plus the revert).
          const float uncert = (L.nvalid == 0) ? 2.0f : (L.nvalid == 1 ? 1.4f : 1.0f);
          if (fabsf(move) * uncert > 1.7f * fabsf(e)) continue;

          // The line above bounds the move's EXPECTED size, which is not the
          // same as bounding its risk. A restructure that trades whitens for
          // re-darkens predicts ~0 because the terms cancel in the model - but
          // their ERRORS do not cancel. (w=1,r=1) -> (w=4,r=5) predicts about
          // -1 unit; at gains known to +/-25% it can land 18 out. That is how
          // a level sitting at err 0 got thrown to -52 in one cycle, with the
          // proportional damping raising no objection because 1.7*|e| was ~0.
          //
          // So bound the total DRIVE CHANGE an edit makes, scaled by how well
          // the gains are known. Single-parameter nudges pass; wholesale
          // restructures do not. TUNE_MAX_RISK is a judgement call, not a
          // measurement - it is set to admit a modest two-parameter edit and
          // reject the 147-unit restructures that were wrecking levels.
          const float risk =
                fabsf((float)(w - L.w)) * L.gw
              + fabsf((float)(r - L.r)) * L.gr
              + fabsf((float)(g - L.g)) * L.gg
              + fabsf(tuneDose[constrain(d, 1, 13)]
                    - tuneDose[constrain((int)L.d, 1, 13)]);
          if (risk * TUNE_GAIN_ERR > tuneMaxRisk()) continue;
          const int changed = (d != L.d) + (w != L.w) + (g != L.g) + (r != L.r);
          // primary: residual error, PLUS what this landing does to the two
          // gaps the level sits between — a move that fixes its own error
          // while opening a hole against a neighbour trades a band for a
          // number. Then prefer simpler edits and shorter trains (a shorter
          // train paints faster).
          const float eNew = e + move;
          const float gapPen = fabsf(eNew - tl[lv - 1].err)
                             + fabsf(eNew - tl[lv + 1].err);
          const float score = fabsf(eNew) + TUNE_SPACING_W * gapPen
                            + 0.35f * changed
                            + 0.10f * (d + w + g + r);
          if (score < bestScore) {
            bestScore = score;
            bd = d; bw = w; bg = g; br = r;
            found = true;
          }
        }
  if (!found) return false;

  L.pd = L.d; L.pw = L.w; L.pg = L.g; L.pr = L.r;
  L.predMove = tunePredict(L, bd, bw, bg, br) - here;
  const int changed = (bd != L.d) + (bw != L.w) + (bg != L.g) + (br != L.r);
  // Gains are only learnable from an unambiguous single-parameter move.
  L.lastOp = 0;
  L.lastSteps = 1;
  if (changed == 1) {
    if (bd != L.d) { L.lastOp = 1; L.lastSteps = bd - L.d; }
    if (bw != L.w) { L.lastOp = 2; L.lastSteps = bw - L.w; }
    if (br != L.r) { L.lastOp = 3; L.lastSteps = br - L.r; }
    if (bg != L.g) { L.lastOp = 4; L.lastSteps = bg - L.g; }
  }
  L.d = bd; L.w = bw; L.g = bg; L.r = br;
  L.edited = true;
  return true;
}

// ============================================================================
// Rescue a stuck level by spending the WHOLE card on it.
//
// The converger edits one level per cycle and measures one outcome, which is
// a poor deal for a level that will not settle: L10 spent eight cycles
// limit-cycling -10 -> +8 -> -10 -> +11 -> -11, one scan each.
//
// But the card's 16 columns take their train from their OWN level index, so
// columns can be borrowed as experiment slots - the same trick the gap ladder
// already uses. Paint fourteen DIFFERENT candidate trains for one level, read
// all fourteen landings from a single scan, keep the best. Fourteen
// candidates per ~4 s instead of one per cycle.
//
// Columns 0 and 15 stay as the gradient anchors, so slots are 1..14. The
// reference half of each column is a dither of ITS density and is therefore
// meaningless here - candidates are judged on absolute gradient-corrected
// landing against the target level's own reference, exactly as the dose
// ladder judges doses.
//
// Returns true if it improved on the incumbent.
static bool tuneRescueLevel(int lv) {
  const float before = fabsf(tl[lv].err);   // incumbent, as the card measures it

  // Candidates are RAW DRIVE SEQUENCES, not (d,w,g,r) tuples.
  //
  // The grammar can only stretch runs: more darkens, more whitens, a gap of a
  // given width. It cannot insert a float mid-train, cannot delete a pass, and
  // cannot produce the alternating d,w,d,w tails the hand-tuned direct trains
  // needed. A level the planner has failed to move is exactly where those
  // shapes are worth trying, so the rescue edits the sequence itself:
  // substitute a pass, INSERT one (shifting the rest right, dropping the
  // last), or DELETE one (shifting left, padding with a float).
  uint8_t cand[15][13];
  uint8_t cur[13];
  tuneTrainOf(lv, cur);

  // Slot 1 is the incumbent, so the scan re-measures it under the SAME
  // conditions as its challengers - that is what lets column bias cancel.
  //
  // Seed the RNG from something that varies: Arduino's random() is a
  // deterministic PRNG, and unseeded it replays one sequence every boot, so
  // every rescue would test identical candidates.
  randomSeed(micros() ^ (uint32_t)(lv * 2654435761u));

  memcpy(cand[1], cur, 13);
  int n = 1;
  for (int s = 2; s <= 14; s++) {
    for (int tries = 0; tries < 60; tries++) {
      uint8_t c[13];
      memcpy(c, cur, 13);
      const int edits = 1 + (int)random(0, 2);      // one or two edits
      for (int e = 0; e < edits; e++) {
        const int pos = (int)random(0, 13);
        switch ((int)random(0, 3)) {
          case 0:                                    // substitute
            c[pos] = (uint8_t)random(0, 3);
            break;
          case 1:                                    // insert, shift right
            for (int i = 12; i > pos; i--) c[i] = c[i - 1];
            c[pos] = (uint8_t)random(0, 3);
            break;
          default:                                   // delete, shift left
            for (int i = pos; i < 12; i++) c[i] = c[i + 1];
            c[12] = 0;
            break;
        }
      }
      // A train of nothing but floats drives no ink; skip it rather than
      // waste a slot measuring paper.
      bool any = false;
      for (int i = 0; i < 13; i++) if (c[i]) { any = true; break; }
      if (!any) continue;
      bool dup = false;
      for (int j = 1; j <= n && !dup; j++)
        if (!memcmp(cand[j], c, 13)) dup = true;
      if (dup) continue;
      memcpy(cand[++n], c, 13);
      break;
    }
  }
  Serial.printf("[tune] rescue L%d: %d distinct candidates (raw edits)\n", lv, n);

  uint8_t t[13];
  for (int s = 1; s <= n; s++)
    epd.driver().setDecisionTrain((uint8_t)(s << 1), cand[s]);
  memset(t, 0, 13);                        // column 15: deep anchor
  for (int i = 0; i < 13; i++) t[i] = 1;
  epd.driver().setDecisionTrain((uint8_t)(15 << 1), t);

  tunePaintMatchCard();
  tuneClear();
  tunePaint();
  float ref[16], nat[16];
  if (!tuneMeasureCard(ref, nat)) { tuneUploadApplies(); return false; }

  // Judge candidates RELATIVE TO THE INCUMBENT measured in the same frame,
  // not against an absolute target.
  //
  // The first version compared each candidate's landing in column s against
  // probeRef[lv], a reference measured in column lv. That carries whatever
  // horizontal variation exists across the card, and on this panel it was
  // worth ~13 units: the incumbent shape read -12.7 on the normal card and
  // +0.5 here, so the rescue "improved" a level by adopting its own shape
  // unchanged. Slot 1 holds the incumbent precisely so that bias can be
  // cancelled - measure how far each candidate sits FROM the incumbent, and
  // apply that delta to the incumbent's known error. Column bias affects
  // both terms and drops out.
  const float g0 = nat[0] - ref[0], g15 = nat[15] - ref[15];
  const float incumbent = nat[1] - (g0 + (g15 - g0) * (1 / 15.0f));
  const float curErr = tl[lv].err;          // incumbent's error, normal card

  int best = -1;
  float bestErr = fabsf(curErr);
  for (int s = 2; s <= n; s++) {
    const float landing = nat[s] - (g0 + (g15 - g0) * (s / 15.0f));
    const float e = fabsf(curErr + (landing - incumbent));
    if (e < bestErr) { bestErr = e; best = s; }
  }

  bool improved = false;
  if (best > 1 && bestErr < before - 0.5f) {
    // The level is now rescue-owned: store the sequence literally, because
    // it may be off-grammar and (d,w,g,r) could not represent it.
    tl[lv].hasRaw = true;
    memcpy(tl[lv].raw, cand[best], 13);
    // Record the new shape as this level's best-known ONLY if it really is
    // better than what was already banked. Blindly resetting bestAbs threw
    // away a good shape the converger had found earlier, and the best-shape
    // assembly at the end then had nothing better to fall back on - which
    // cost a run more than the failed rescue itself did.
    // bestAbs/bd..br describe a GRAMMAR shape and cannot hold a raw train, so
    // invalidate the banked best rather than let the end-of-run assembly
    // reinstate a shape this level no longer uses.
    tl[lv].bestAbs = 1e9f;
    tl[lv].frozen = false;
    improved = true;
    Serial.print("[tune] rescue L");
    Serial.print(lv);
    Serial.print(": err ");
    Serial.print(before, 1);
    Serial.print(" -> ");
    Serial.print(bestErr, 1);
    Serial.print("  {");
    for (int i = 0; i < 13; i++) { Serial.print(cand[best][i]); if (i < 12) Serial.print(","); }
    Serial.println("}");
  } else {
    Serial.printf("[tune] rescue L%d: nothing beat %.1f (best challenger %.1f)\n",
                  lv, before, bestErr);
  }
  tuneUploadApplies();                     // put every level's real train back
  return improved;
}

// Pick the worst 1-2 out-of-tolerance levels and edit them.
static void tunePlanEdits() {
  int order[14];
  int n = 0;
  for (int lv = 1; lv <= 14; lv++) {
    tl[lv].lastOp = 0;
    tl[lv].edited = false;
    if (tl[lv].skip) { tl[lv].skip = false; continue; }
    if (tl[lv].frozen) continue;
    // Rescue-owned levels carry a raw sequence the (d,w,g,r) planner cannot
    // represent; editing them would silently overwrite it with a grammar
    // shape. They are the rescue's to change from here.
    if (tl[lv].hasRaw) continue;
    // Eligible on EITHER count. A level can sit well inside TUNE_TOL and
    // still be the reason the ramp bands, if its neighbour leans the other
    // way — that case was previously invisible to the planner and simply
    // never got edited.
    if (fabsf(tl[lv].err) > tuneTol() || tuneGapErr(lv) > tuneGapTol())
      order[n++] = lv;
  }
  for (int i = 0; i < n; i++)          // worst first, placement + spacing
    for (int j = i + 1; j < n; j++)
      if (tuneBadness(order[j]) > tuneBadness(order[i])) {
        int t = order[i]; order[i] = order[j]; order[j] = t;
      }
  int edited = 0;
  for (int i = 0; i < n && edited < 2; i++)
    if (tunePlanOne(order[i])) edited++;
}

static float tuneMedian(float *v, int n) {
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      if (v[j] < v[i]) { float t = v[i]; v[i] = v[j]; v[j] = t; }
  return n ? (n & 1 ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2])) : 0.0f;
}

static void tuneAbsorb(const float err[16]) {
  // drift baseline: median error change of the UNEDITED levels
  float deltas[14];
  int nd = 0;
  for (int lv = 1; lv <= 14; lv++)
    if (!tl[lv].edited) deltas[nd++] = err[lv] - tl[lv].err;
  const float drift = tuneMedian(deltas, nd);

  for (int lv = 1; lv <= 14; lv++) {
    TuneLv &L = tl[lv];
    L.lastErr = L.err;
    L.err = err[lv];

    // Remember the best shape ever measured for this level. Levels take
    // turns misbehaving, so where the loop happens to stop is not where
    // each level was at its best — without this, a good state reached at
    // cycle 7 is simply lost.
    if (fabsf(L.err) < L.bestAbs) {
      L.bestAbs = fabsf(L.err);
      L.bd = L.d; L.bw = L.w; L.bg = L.g; L.br = L.r;
    }
    if (!L.edited) continue;

    const float moved = (L.err - L.lastErr) - drift;   // what the edit itself did

    // Learn from what happened. A single-parameter edit names its own
    // gain; a restructure only tells us the model's overall scale, so
    // rescale the shape gains by the (damped) ratio it was wrong by.
    // NOTE: an overshoot means the move was BIGGER than predicted, so
    // the gain must grow. Shrinking it here — an earlier bug — made the
    // planner re-propose the same too-large move next cycle.
    if (L.lastOp && L.lastSteps) {
      const float obs = fabsf(moved / (float)L.lastSteps);
      float *gp = (L.lastOp == 1) ? &L.gd : (L.lastOp == 2) ? &L.gw
                : (L.lastOp == 3) ? &L.gr : &L.gg;
      if (obs > 1.0f) {
        *gp = constrain(0.55f * *gp + 0.45f * obs, 1.5f, 80.0f);
        if (L.nvalid < 250) L.nvalid++;
      }
    } else if (fabsf(L.predMove) > 1.0f && fabsf(moved) > 1.0f) {
      const float k = sqrtf(constrain(fabsf(moved / L.predMove), 0.4f, 3.0f));
      L.gd = constrain(L.gd * k, 1.5f, 60.0f);
      L.gw = constrain(L.gw * k, 1.5f, 80.0f);
      L.gr = constrain(L.gr * k, 1.5f, 80.0f);
      L.gg = constrain(L.gg * k, 1.0f, 40.0f);
      if (L.nvalid < 250) L.nvalid++;
    }

    // Undo any edit that left the level worse off — whether it shot past
    // zero (sign flipped) or simply moved the wrong way, which happens
    // when the model has a structural term's SIGN wrong and no amount of
    // rescaling will save it. A level that keeps doing this is frozen at
    // its best shape rather than allowed to thrash for the whole run.
    const bool flipped = (L.lastErr * L.err < 0 && fabsf(L.err) > 0.75f * fabsf(L.lastErr));
    const bool worse   = fabsf(L.err) > fabsf(L.lastErr) + 2.0f;
    if (flipped || worse) {
      L.d = L.pd; L.w = L.pw; L.g = L.pg; L.r = L.pr;
      L.skip = true;
      if (++L.bad >= 3) {
        // Freeze means STOP EDITING — leave the shape exactly as it is.
        // Snapping to the best-ever shape here was a mistake: that shape
        // was measured in a different frame, so installing it moved one
        // level 56 units in a single cycle, with no way back because the
        // level was now frozen. Picking bests is the end-of-run job.
        L.frozen = true;
        Serial.printf("[tune] L%d frozen (best so far |err| %.1f)\n", lv, L.bestAbs);
      }
    }
  }
  tl[15].err = err[15];
  if (nd) Serial.printf("[tune] drift %+.1f (n=%d)\n", drift, nd);
}

// ============================================================================
// Charge-matched removes: k darkens then k + net whitens (wr − k = net).
// ============================================================================
static void tuneRemoveOf(int lv, uint8_t out[13]) {
  memset(out, 0, 13);
  if (lv == 0) return;
  const int net = tuneNetOf(lv);          // from the real train, raw or not
  if (net <= 0) {
    out[0] = 1; out[1] = 2; out[2] = 1; out[3] = 2;   // charge-neutral scrub
    return;
  }
  // Depth proxy: the darken count of the actual train, so a raw shape sizes
  // its remove sensibly too.
  int darks = 0;
  { uint8_t t[13]; tuneTrainOf(lv, t);
    for (int i = 0; i < 13; i++) if (t[i] == 1) darks++; }
  int wr = max(net, max(3, darks / 2 + 3) + removeExtra[lv]);
  if (wr > 13) wr = 13;
  int k = wr - net;
  if (k < 0) k = 0;
  while (k + wr > 13 && k > 0) { k--; wr--; }
  if (k + wr > 13) { wr = min(13, net); k = 0; }
  int p = 0;
  for (int i = 0; i < k; i++) out[p++] = 1;
  for (int i = 0; i < wr && p < 13; i++) out[p++] = 2;
}

static void tuneUploadRemoves() {
  uint8_t t[13];
  for (int lv = 1; lv <= 15; lv++) {
    tuneRemoveOf(lv, t);
    epd.driver().setDecisionTrain((uint8_t)((lv << 1) | 1), t);
  }
}

// Optical remove check via the anchors (works on an all-white panel).
static float tuneVerifyRemoves() {
  tunePaintMatchCard();
  tuneClear();
  tunePaint();
  tuneFill(0);
  tunePaint();
  delay(300);
  tuneRepaint = tuneRepaintCardThenWhite;
  if (!tuneScan(tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm)) return -1;
  const int H = epd.height();
  float col[16];
  for (int i = 0; i < 16; i++) {
    int px0, px1, sx0, sx1, sy0, sy1;
    tuneColSpan(i, &px0, &px1);
    tunePanelRect(px0, px1, (int)(H * 0.08f), (int)(H * 0.92f), &sx0, &sx1, &sy0, &sy1);
    col[i] = tuneRectMean(sx0, sx1, sy0, sy1);
  }
  float worst = 0;
  int worstLv = 0;
  Serial.print("[tune] remove residuals: ");
  for (int i = 1; i <= 15; i++) {
    const float res = col[i] - col[0];
    Serial.printf("%+.1f ", res);
    if (fabsf(res) > fabsf(worst)) { worst = res; worstLv = i; }
    if (res < -6.0f) removeExtra[i]++;   // ghost: widen that level's erase
  }
  Serial.printf("  worst L%d %+.1f\n", worstLv, worst);
  return fabsf(worst);
}

// ============================================================================
// Reports
// ============================================================================
static void tuneDumpTables() {
  uint8_t t[13];
  Serial.println("[tune] final apply trains (d,w,g,r):");
  for (int lv = 1; lv <= 15; lv++) {
    tuneTrainOf(lv, t);
    Serial.printf("  L%-2d (%d,%d,%d,%d)  {", lv, tl[lv].d, tl[lv].w, tl[lv].g, tl[lv].r);
    for (int p = 0; p < 13; p++) Serial.printf("%d%s", t[p], p < 12 ? "," : "");
    Serial.println("}");
  }
  Serial.println("[tune] removes:");
  for (int lv = 1; lv <= 15; lv++) {
    tuneRemoveOf(lv, t);
    Serial.printf("  L%-2d {", lv);
    for (int p = 0; p < 13; p++) Serial.printf("%d%s", t[p], p < 12 ? "," : "");
    Serial.println("}");
  }
}

// ============================================================================
// Results screen: what the tune achieved, and the choice of whether to keep
// it. Painted at the end, so it is what the user reads when they pick the
// board up off the glass.
//
// Errors are shown as a percentage of FULL SCALE (the panel's own white-to-
// black span, measured this run) rather than raw scanner units, because
// that is the number that means something: 5% out is 5% of everything the
// panel can do.
// ============================================================================
static bool tuneResultsScreen(float maxerr, float removeWorst) {
  const int W = epd.width(), H = epd.height();
  const float span = max(20.0f, probeRef[0] - probeRef[15]);   // full scale
  auto pct = [&](float units) { return 100.0f * units / span; };

  uint8_t *fb = epd.getBuffer();
  memset(fb, 0, (size_t)W * H);

  // ---- header ----
  epd.fillRect(0, 0, W, 46, 15);
  epd.setTextColor(0);
  epd.setTextSize(4);
  epd.setCursor(14, 8);
  epd.print("COMPLETE");
  epd.setTextSize(2);
  epd.setCursor(270, 16);
  char hd[64];
  snprintf(hd, sizeof(hd), "worst %+.1f%%   erase %.1f%%",
           pct(maxerr), pct(removeWorst));
  epd.print(hd);

  // ---- the ramp, each level shown against its own reference ----
  // Top half of a patch is the Bayer-dithered reference for that level
  // (built only from full black and full white pixels, so it is the
  // optical truth); the bottom half is the tuned grey. A level that is
  // right has no visible seam — which is far easier to judge than looking
  // at a grey on its own. Patches are outlined so the near-white end
  // doesn't vanish into the page.
  epd.setTextColor(15);
  const int cw = W / 16, rampY = 56, rampH = 84, half = rampH / 2;
  for (int i = 0; i < 16; i++) {
    const int x = i * cw;
    const int thr = (i * 64 + 7) / 15;
    for (int y = rampY; y < rampY + half; y++)
      for (int px = x + 1; px < x + cw - 1; px++)
        fb[(size_t)y * W + px] = (tuneBayer8[y & 7][px & 7] < thr) ? 15 : 0;
    epd.fillRect(x + 1, rampY + half, cw - 2, rampH - half, (uint8_t)i);
    epd.drawRect(x + 1, rampY, cw - 2, rampH, 15);
    epd.setTextSize(1);
    char lb[6];
    snprintf(lb, sizeof(lb), "%d", i);
    epd.setCursor(x + cw / 2 - (i > 9 ? 6 : 3), rampY + rampH + 4);
    epd.print(lb);
  }
  epd.setTextSize(1);
  epd.setCursor(4, rampY - 10);
  epd.print("top = target dither, bottom = tuned grey (a good level shows no seam)");

  // ---- error bars about a zero line ----
  const int zero = 240, ppp = 4;            // pixels per percent
  epd.drawFastHLine(0, zero, W, 15);
  epd.setTextSize(1);
  epd.setCursor(4, zero - 60);
  epd.print("too light");
  epd.setCursor(4, zero + 54);
  epd.print("too dark");
  for (int i = 1; i <= 15; i++) {
    const float p = pct(tl[i].err);
    int h = (int)(fabsf(p) * ppp);
    if (h > 52) h = 52;
    const int x = i * cw + cw / 2 - 9;
    char lb[8];
    snprintf(lb, sizeof(lb), "%+.0f", p);
    const int tx = i * cw + cw / 2 - (fabsf(p) >= 9.5f ? 9 : 6);
    if (p > 0) {
      if (h > 1) epd.fillRect(x, zero - h, 18, h, 15);
      epd.setCursor(tx, zero - h - 11);     // label above the bar
    } else {
      if (h > 1) epd.fillRect(x, zero + 1, 18, h, 15);
      epd.setCursor(tx, zero + h + 4);      // label below the bar
    }
    epd.print(lb);
  }

  // ---- monotonicity: adjacent levels that came out the wrong way round
  // are visible as banding, so say so plainly rather than hiding it in a
  // max-error figure ----
  int inversions = 0;
  for (int i = 1; i < 15; i++) {
    const float a = probeRef[i] + tl[i].err, b = probeRef[i + 1] + tl[i + 1].err;
    if (b > a + 0.5f) inversions++;
  }
  epd.setTextSize(2);
  epd.setCursor(20, 322);
  if (inversions) {
    char w2[80];
    snprintf(w2, sizeof(w2), "WARNING: %d level pair%s inverted (banding)",
             inversions, inversions == 1 ? "" : "s");
    epd.print(w2);
  } else {
    epd.print("Ramp is smooth - every level darker than the last.");
  }
  epd.setCursor(20, 346);
  epd.print(pct(maxerr) <= 5.0f && !inversions
            ? "Good result - safe to keep."
            : "Coarser than the built-in tables.");

  // ---- the choice ----
  Btn save = { 60, 386, 360, 116 };
  Btn skip = { 540, 386, 360, 116 };
  drawBtn(save, "SAVE", "use on every boot");
  drawBtn(skip, "SKIP", "keep built-in");
  tuneClear();          // activate immediately before the drive
  tunePaint();
  touchRewake();          // 15 minutes of painting leaves the GT911 mute

  Serial.printf("[tune] results: worst %+.1f%% of full scale, erase %.1f%%, "
                "%d inversion(s)\n", pct(maxerr), pct(removeWorst), inversions);
  Serial.println("[tune] tap SAVE or SKIP on the panel (serial: s = save, k = skip)");

  // Serial is checked FIRST and unconditionally: if the touch controller
  // has died (it is reset by the panel's own power cycling, and comes back
  // on a different I2C address), this keyboard route is the only way out.
  for (;;) {
    if (Serial.available()) {
      const int c = Serial.read();
      if (c == 's') return true;
      if (c == 'k' || c == 'q') return false;
    }
    int px, py;
    if (touchTapped(px, py)) {
      Serial.printf("[touch] tap at %d,%d  (SAVE %d..%d/%d..%d  SKIP %d..%d)\n",
                    px, py, save.x, save.x + save.w, save.y, save.y + save.h,
                    skip.x, skip.x + skip.w);
      if (hitBtn(save, px, py)) return true;
      if (hitBtn(skip, px, py)) return false;
    }
    delay(8);
  }
}

static void tuneResultScreen(bool ok, float maxerr, int cycles) {
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width(), H = epd.height();
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      fb[(size_t)y * W + x] = (uint8_t)((x * 16) / W);
  epd.fillRect(0, H / 2 - 40, W, 80, 0);
  epd.setTextSize(3);
  epd.setTextColor(15);
  epd.setCursor(40, H / 2 - 28);
  epd.print(ok ? "SELF-TUNE COMPLETE" : "SELF-TUNE FAILED");
  epd.setTextSize(2);
  epd.setCursor(40, H / 2 + 8);
  if (ok) {
    char b[80];
    snprintf(b, sizeof(b), "max err %.1f units in %d cycles - saved to flash",
             maxerr, cycles);
    epd.print(b);
  } else {
    epd.print("see serial log");
  }
  tunePaint();
}

// Paint the results screen with the figures from a real run, so the layout
// can be checked without sitting through a 15-minute tune.
static void tunePreviewResults() {
  static const float demoErr[16] =
      { 0, 0, -3, 2, -2, 3, -6, -6, 9, -6, -5, -4, -3, 0, 3, 0 };
  for (int i = 0; i < 16; i++) {
    probeRef[i] = 155.0f - i * 6.7f;      // a typical measured reference set
    tl[i].err = demoErr[i];
  }
  // Walk the whole end-of-run flow — results page, then the outcome page —
  // so the artwork and the button placement can be checked without a tune.
  // Nothing is written to flash.
  const bool keep = tuneResultsScreen(9.0f, 3.1f);
  if (keep)
    tuneConfirmScreen("SAVED", "Your panel now uses these greys",
                      "Verified in flash - loaded on every boot (preview only)");
  else
    tuneConfirmScreen("SKIPPED", "Nothing was changed",
                      "The built-in tables are still in charge.");
}

// ============================================================================
// The whole show
// ============================================================================
static bool tuneAbortRequested() {
  while (Serial.available()) {
    if (Serial.read() == 'q') return true;
  }
  return false;
}

static bool runSelfTune() {
  if (!scPort) {
    Serial.println("[tune] no scanner selected");
    return false;
  }
  Serial.println("[tune] ===== SELF-TUNE START =====");
  Serial.println("[tune] send q to abort between cycles");

  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  if (tunePeriodUs) {
    epd.driver()._config.g16_pass_us_normal = (int)tunePeriodUs;
    Serial.printf("[tune] pass period overridden to %lu us\n",
                  (unsigned long)tunePeriodUs);
  }
  const uint32_t period = (uint32_t)epd.driver()._config.g16_pass_us_normal;
  Serial.printf("[tune] tuning at %lu us NORMAL period\n", (unsigned long)period);
  if (!epd.driver().setGreyLevels(16)) {
    Serial.println("[tune] setGreyLevels(16) refused");
    return false;
  }

  tuneSeedShapes();
  if (!tuneRegister()) { tuneResultScreen(false, 0, 0); return false; }
  if (!tuneProbe())    { tuneResultScreen(false, 0, 0); return false; }

  // ---- attempts -----------------------------------------------------------
  // A run that stalls is not a run that has found the best tables - it is one
  // that fell into a bad basin. Measured evidence: two runs of the IDENTICAL
  // binary produced +3.3% and a result bad enough to discard, so the spread
  // between attempts is as large as the differences between code versions.
  // With scanning down to ~4 s a whole attempt costs about five minutes, so
  // the cheapest way to a good table is to take the best of several rather
  // than to keep tuning the converger.
  //
  // Geometry is NOT repeated - the anchor ring is a property of where the
  // panel sits on the glass, not of the attempt. Everything downstream of it
  // starts fresh, including the probe, so each attempt gets its own
  // measurement noise and its own seeds.
  float ref[16], nat[16], err[16];
  int8_t keepD[16], keepW[16], keepG[16], keepR[16];
  float  keepScore = 1e9f;
  bool   haveKeep = false;
  bool   converged = false;
  float  mx = 999;

  for (int attempt = 1; attempt <= TUNE_MAX_ATTEMPTS; attempt++) {
  if (attempt > 1) {
    // Let the scanner settle before hammering it again - see the constant.
    // Polled rather than one long delay so 'q' still aborts promptly.
    Serial.printf("[tune] cooling down %lus before the next attempt "
                  "(scanner rate limit)\n",
                  (unsigned long)(TUNE_ATTEMPT_COOLDOWN_MS / 1000));
    const uint32_t t0 = millis();
    while (millis() - t0 < TUNE_ATTEMPT_COOLDOWN_MS) {
      if (tuneAbortRequested()) { Serial.println("[tune] aborted"); break; }
      delay(100);
    }
    if (tuneAbortRequested()) break;

    Serial.printf("[tune] ---- attempt %d of %d (previous did not converge) ----\n",
                  attempt, TUNE_MAX_ATTEMPTS);
    tuneSeedShapes();
    tuneScansPerMeasure = 1;
    if (!tuneProbe()) break;
  }

  float bestMax = 1e9f;
  int   stall = 0, cyc = 0;
  int   rescues = 0;
  converged = false;
  mx = 999;

  for (cyc = 0; cyc < TUNE_MAX_CYCLES; cyc++) {
    if (tuneAbortRequested()) { Serial.println("[tune] aborted"); break; }
    tuneUploadApplies();
    tunePaintMatchCard();
    tuneClear();
    tunePaint();
    if (!tuneMeasureCard(ref, nat)) break;
    tuneErrors(ref, nat, err);
    tuneAbsorb(err);

    mx = 0;
    for (int lv = 1; lv <= 14; lv++) mx = max(mx, fabsf(tl[lv].err));
    const float gmx = tuneGapMax();   // worst neighbour spacing error
    // Close to home: average more scans per measurement so the remaining
    // edits are made against the panel rather than scan noise. Precision has
    // to keep pace with the tolerance — TUNE_GAP_TOL is 2.5 units and one
    // scan reads to a unit or two, so the spacing figure only becomes
    // meaningful once it is averaged. Gated on the WORSE of placement and
    // spacing, matching what convergence now asks for. Root-N: the third
    // read buys less than the second, so it is only worth its extra scan
    // right at the end.
    const float close = max(mx, gmx);
    const int wantScans = (close <= tuneAvgBelow3()) ? 3
                        : (close <= tuneAvgBelow())  ? 2 : 1;
    if (wantScans > tuneScansPerMeasure) {
      tuneScansPerMeasure = wantScans;
      Serial.printf("[tune] within %.0f - averaging %d scans per measurement now\n",
                    (wantScans == 3) ? tuneAvgBelow3() : tuneAvgBelow(),
                    tuneScansPerMeasure);
    }
    Serial.printf("[tune] cycle %2d  maxerr %5.1f  gap %4.1f  |", cyc, mx, gmx);
    for (int lv = 1; lv <= 15; lv++) Serial.printf("%+3.0f", tl[lv].err);
    Serial.println();

    // The first measurement is what the gain solve needs: re-derive the
    // shapes from the now-measured physics before any editing starts.
    if (cyc == 0) {
      tuneCalibrateGains();
      tuneSeedModel(false);  // gains are solved now; take the best-fitting shapes
      for (int lv = 1; lv <= 14; lv++) tl[lv].nvalid = 1;   // gains measured now
      continue;
    }
    if (cyc < TUNE_WARMUP) { Serial.println("[tune] (warm-up, no edits)"); continue; }
    // Both conditions: a ramp whose levels are each in tolerance but whose
    // steps vary 5x is exactly the "converged, still bands" case that
    // started this. Stall tracking follows the same combined figure, so the
    // loop does not give up while spacing is still improving.
    if (mx <= tuneTol() && gmx <= tuneGapTol()) { converged = true; break; }
    const float progress = max(mx, gmx);
    if (progress < bestMax - 0.7f) { bestMax = progress; stall = 0; } else stall++;
    if (stall >= 8) {
      // Stalled means the one-edit-per-cycle search has run out of ideas, not
      // that the level is unimprovable. Before giving up, spend a whole card
      // on the worst offender - fourteen candidates in one scan beats another
      // eight cycles of the same nudges. Only once per attempt: if a rescue
      // does not help, the retry loop's fresh probe is the better lever.
      // Stalling is the trigger for the SECOND strategy, not the end of the
      // run: the one-edit-per-cycle planner has run out of ideas, so hand the
      // worst level to the rescue and carry on. Repeatedly - a run that needs
      // three levels unstuck should get three rescues, not one and then a
      // shrug. Bounded so a level that cannot be helped cannot loop forever.
      if (rescues < TUNE_MAX_RESCUES) {
        rescues++;
        int worst = 0;
        for (int lv = 1; lv <= 14; lv++)
          if (!worst || tuneBadness(lv) > tuneBadness(worst)) worst = lv;
        Serial.printf("[tune] stalled - rescue %d/%d on worst level L%d\n",
                      rescues, TUNE_MAX_RESCUES, worst);
        stall = 0;
        bestMax = 1e9f;              // give the planner a clean slate after it
        if (tuneRescueLevel(worst)) continue;
        // Nothing beat the incumbent: freeze that level so the next stall
        // picks a different one rather than retrying the same dead end.
        tl[worst].frozen = true;
        continue;
      }
      Serial.println("[tune] stalled");
      break;
    }
    tunePlanEdits();
  }

  // Assemble every level's best-known shape and MEASURE that combination.
  // Each level's best was seen in a different frame, so the assembled set
  // has to be verified rather than assumed — but it is still a far better
  // starting point than wherever the loop happened to stop.
  if (!converged) {
    tuneScansPerMeasure = 2;        // the set we keep is worth measuring well
    for (int lv = 1; lv <= 14; lv++)
      if (tl[lv].bestAbs < 1e8f) {
        tl[lv].d = tl[lv].bd; tl[lv].w = tl[lv].bw;
        tl[lv].g = tl[lv].bg; tl[lv].r = tl[lv].br;
      }
    tuneUploadApplies();
    tunePaintMatchCard();
    tuneClear();
    tunePaint();
    if (tuneMeasureCard(ref, nat)) {
      tuneErrors(ref, nat, err);
      float bmx = 0;
      for (int lv = 1; lv <= 14; lv++) {
        tl[lv].err = err[lv];
        bmx = max(bmx, fabsf(err[lv]));
      }
      Serial.printf("[tune] best-shape set measures %5.1f  |", bmx);
      for (int lv = 1; lv <= 15; lv++) Serial.printf("%+3.0f", tl[lv].err);
      Serial.println();
      mx = bmx;                       // this is the set we are keeping
    }
  }

  // Score this attempt on the WORSE of placement and spacing - the same
  // figure convergence is gated on, so "best attempt" means the same thing
  // as "closest to converged" rather than being a second opinion.
  {
    const float score = max(mx, tuneGapMax());
    Serial.printf("[tune] attempt %d scores %.1f (maxerr %.1f, gap %.1f)%s\n",
                  attempt, score, mx, tuneGapMax(),
                  (score < keepScore) ? " - best so far" : "");
    if (score < keepScore) {
      keepScore = score;
      haveKeep = true;
      for (int lv = 1; lv <= 15; lv++) {
        keepD[lv] = tl[lv].d; keepW[lv] = tl[lv].w;
        keepG[lv] = tl[lv].g; keepR[lv] = tl[lv].r;
      }
    }
  }
  if (converged) break;
  if (tuneAbortRequested()) break;
  }  // ---- end attempts ----

  // Reinstate the best attempt's shapes. Without this the tables kept would
  // be whichever attempt happened to run last, which is not the same thing.
  if (haveKeep) {
    for (int lv = 1; lv <= 15; lv++) {
      tl[lv].d = keepD[lv]; tl[lv].w = keepW[lv];
      tl[lv].g = keepG[lv]; tl[lv].r = keepR[lv];
    }
    mx = keepScore;
    // ref[] and tl[].err are currently whatever the LAST attempt left behind,
    // which is not necessarily the attempt whose shapes we just reinstated.
    // That matters: lum[] is built from ref[i] + tl[i].err and is the curve
    // the image dither quantises against - the one thing that demonstrably
    // fixed banding. Re-measure so the curve describes the set being kept.
    tuneUploadApplies();
    tunePaintMatchCard();
    tuneClear();
    tunePaint();
    if (tuneMeasureCard(ref, nat)) {
      tuneErrors(ref, nat, err);
      for (int lv = 1; lv <= 14; lv++) tl[lv].err = err[lv];
      mx = 0;
      for (int lv = 1; lv <= 14; lv++) mx = max(mx, fabsf(tl[lv].err));
      Serial.printf("[tune] kept set re-measured: maxerr %.1f gap %.1f\n",
                    mx, tuneGapMax());
    }
  }

  Serial.printf("[tune] converged=%d  final maxerr %.1f\n", converged, mx);
  tuneUploadApplies();

  Serial.println("[tune] deriving charge-matched removes...");
  float worst = -1;
  for (int round = 0; round < 3; round++) {
    tuneUploadRemoves();
    worst = tuneVerifyRemoves();
    if (worst < 0) { Serial.println("[tune] remove verify scan failed"); break; }
    if (worst <= 6.0f) break;
    Serial.println("[tune] widening ghosting removes and re-checking...");
  }

  uint8_t ap[16][13], rm[16][13];
  memset(ap, 0, sizeof(ap));
  for (int lv = 1; lv <= 15; lv++) {
    tuneTrainOf(lv, ap[lv]);
    tuneRemoveOf(lv, rm[lv]);
  }

  // What each level ACTUALLY renders as, from the final measurement:
  // reference plus the residual error, normalised so level 0 is paper
  // white and level 15 the deepest black this panel reached. The levels
  // are NOT evenly spaced, so image conversion must quantise against this
  // curve rather than assume even steps.
  //
  // ref[] here is the FINAL card's reference, not probeRef[] from the
  // ladder at the start of the run. probeRef was read minutes and dozens
  // of paints earlier, and the panel drifts over a session — mixing a
  // start-of-run reference with an end-of-run error put that drift
  // straight into the curve the image dither quantises against. Since
  // tuneErrors() defines err = (nat - ref) - gradient, ref[i] + err[i] is
  // exactly the gradient-corrected native landing, with no reconstruction.
  uint8_t lum[16];
  {
    float meas[16];
    for (int i = 0; i < 16; i++) meas[i] = ref[i] + tl[i].err;
    const float white = meas[0], black = meas[15];
    const float span = (white - black) > 1.0f ? (white - black) : 1.0f;
    Serial.print("[tune] measured level curve (255=white): ");
    for (int i = 0; i < 16; i++) {
      int v = (int)lroundf(255.0f * (meas[i] - black) / span);
      lum[i] = (uint8_t)constrain(v, 0, 255);
      Serial.printf("%d ", lum[i]);
    }
    Serial.println();
  }
  tuneDumpTables();

  // Show what was achieved and let the user decide. Nothing is written to
  // flash unless they ask for it — these tables drive every future paint,
  // so the choice belongs to them, not to a threshold of mine.
  const bool keep = tuneResultsScreen(mx, worst < 0 ? 0.0f : worst);
  bool saved = false, loaded = false;
  if (keep) {
    saved = tunedSave(epd.driver(), period, ap, rm, lum);
    // Read it straight back rather than trusting the write: this is the
    // same path every future boot will take, so if it can't be reloaded
    // now it was never really saved.
    loaded = saved && tunedLoad(epd.driver());
    Serial.printf("[tune] flash save %s, reload %s\n",
                  saved ? "OK" : "FAILED", loaded ? "OK" : "FAILED");
    if (loaded) {
      char sub[80];
      snprintf(sub, sizeof(sub), "Verified in flash - loaded at %lu us on every boot",
               (unsigned long)period);
      tuneConfirmScreen("SAVED", "Your panel now uses these greys", sub);
    } else {
      tuneConfirmScreen("SAVE FAILED", "Nothing was stored",
                        saved ? "Written, but it would not read back - flash may be full."
                              : "Could not write to storage. The old tables still apply.");
    }
  } else {
    // Put the preset tables back: the tuned trains are still live in the
    // engine from the run itself.
    epd.driver().rebuildDecisionTrains();
    Serial.println("[tune] discarded - built-in tables restored");
    tuneConfirmScreen("SKIPPED", "Nothing was changed",
                      "The built-in tables are still in charge.");
  }

  Serial.printf("[tune] ===== SELF-TUNE DONE (maxerr %.1f, removes worst %.1f, %s) =====\n",
                mx, worst, keep ? "saved" : "discarded");
  return keep && saved && loaded;
}

#else  // !TUNEUP_HAVE_JPEGDEC

static uint32_t tunePeriodUs = 0;
static bool runSelfTune() {
  Serial.println("[tune] JPEGDEC required - set TUNEUP_HAVE_JPEGDEC to 1");
  return false;
}

#endif
