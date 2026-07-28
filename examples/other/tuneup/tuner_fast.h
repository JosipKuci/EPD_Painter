// ============================================================================
// tuner_fast.h — the FAST-quality tune.
//
// FAST is a different machine from NORMAL and HIGH, not a faster setting of
// one. It has no 16-grey mode at all: seven undelayed passes cannot hold
// sixteen distinguishable doses, and setGreyLevels(16) refuses while the
// quality is FAST. So this tune is about the two things FAST actually has:
//
//   1. The four levels. Waveforms::fast_darker[3][7] draws levels 1..3 from
//      white, fast_lighter[3][7] takes them back. Level 3 is pinned to
//      seven darkens — that is as dark as FAST goes, by construction — so
//      only levels 1 and 2 are free, which is why this converges in a
//      fraction of the time the 16-grey tune takes.
//
//   2. The six grey-to-grey direct trains (dir_fast). These are what make
//      FAST animation clean: without them every grey-to-grey pixel
//      two-steps through white and punches a visible hole (the megademo's
//      bouncing-title artifact). NORMAL and the M5PaperS3 have had a tuned
//      set since July; FAST never did.
//
// WHAT LIMITS THIS TUNE, and it is worth understanding before reading the
// numbers: in 4-level mode there are only three drive trains on the glass
// at once — one per level — so a card can measure three candidates, not
// fourteen. The 16-grey tuner's whole strategy (spend a whole card on one
// level's candidates) is unavailable here. The compensation is that FAST
// paints are fast and the search space is tiny, so this converges by
// iterating rather than by breadth.
//
// The direct card is the exception and the reason phase 2 is cheap: nine
// columns carry three references and all six transitions in a single
// paint, so every pair is measured every scan.
// ============================================================================

#pragma once

#if TUNEUP_HAVE_JPEGDEC

// FAST's pass count is fixed by the engine — Waveforms::fast_* are [3][7]
// and the pass loop runs exactly 7 for this quality.
static const int F_LEN = 7;

// Placement tolerance, as a fraction of a level step. Four levels over the
// same full scale means a step is five times a 16-grey step, so the same
// fraction is a much looser absolute number — which is correct: nothing
// about FAST is asking for 16-grey precision.
static const float FAST_TOL_STEPS = 0.25f;
static const int   FAST_MAX_CYCLES = 14;
static const int   FAST_DIR_CYCLES = 10;

struct FastLv {
  int8_t d, w, g, r;
  float  err;          // gradient-corrected (native - reference), scan units
  float  bias;         // measured landing minus what the model predicted
  float  bestAbs;
  float  bestErr;      // the same measurement, sign kept (the level curve needs it)
  int8_t bd, bw, bg, br;
};
static FastLv fl[4];
static float fastDose[F_LEN + 1];   // landing of {1 x n}, n = 1..7
static float fastRef[4];            // the four dither references
static float fastGw = 20.0f, fastGr = 10.0f, fastGg = 0.0f;

// One level step in scan units, from the panel's own reference ladder.
static float fastStep() {
  const float span = fastRef[0] - fastRef[3];
  return (span > 20.0f ? span : 20.0f) / 3.0f;
}
static float fastTol() { return FAST_TOL_STEPS * fastStep(); }

// ---------------------------------------------------------------------------
// Cards
// ---------------------------------------------------------------------------

// The 4-level match card. Same idea as the 16-grey one and deliberately
// the same geometry, so tuneMeasureCardOnce() reads it unchanged: sixteen
// columns, but in four groups of four. Each group pairs a Bayer-dithered
// reference of density g/3 — built only from full black and full white
// pixels, so it is the optical truth for that level — against the native
// 4-level grey below it. Four columns per level is not waste: it averages
// out the horizontal bias that a single column carries.
static void tunePaintFastCard() {
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width(), H = epd.height(), mid = H / 2;
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      const int g = ((x * 16) / W) / 4;       // 0..3
      uint8_t v;
      if (y < mid - 3) {
        const int thr = (g * 64 + 1) / 3;
        v = (tuneBayer8[y & 7][x & 7] < thr) ? 3 : 0;
      } else if (y < mid + 3) {
        v = 3;
      } else {
        v = (uint8_t)g;
      }
      fb[(size_t)y * W + x] = v;
    }
}

static void tuneRepaintFastCard() {
  tunePaintFastCard();
  tuneClear();
  tunePaint();
}

// Measure the 4-level card: mean the four columns of each group, then
// remove the panel's vertical gradient by interpolating the deltas the
// two extreme groups show (they are the same drive top and bottom).
static bool tuneMeasureFast(float ref[4], float nat[4]) {
  float r16[16], n16[16];
  tuneRepaint = tuneRepaintFastCard;
  if (!tuneMeasureCardOnce(r16, n16)) return false;
  for (int g = 0; g < 4; g++) {
    float rs = 0, ns = 0;
    for (int c = 0; c < 4; c++) { rs += r16[g * 4 + c]; ns += n16[g * 4 + c]; }
    ref[g] = rs / 4;
    nat[g] = ns / 4;
  }
  Serial.print("[fast] ref:");
  for (int g = 0; g < 4; g++) Serial.printf(" %3.0f", ref[g]);
  Serial.print("  nat:");
  for (int g = 0; g < 4; g++) Serial.printf(" %3.0f", nat[g]);
  Serial.println();
  return true;
}

static void fastErrors(const float ref[4], const float nat[4], float err[4]) {
  const float g0 = nat[0] - ref[0], g3 = nat[3] - ref[3];
  for (int g = 0; g < 4; g++)
    err[g] = (nat[g] - ref[g]) - (g0 + (g3 - g0) * (g / 3.0f));
}

// ---------------------------------------------------------------------------
// Trains
// ---------------------------------------------------------------------------
static void fastTrainOfShape(int d, int w, int gap, int r, uint8_t out[F_LEN]) {
  memset(out, 0, F_LEN);
  int p = 0;
  for (int i = 0; i < d && p < F_LEN; i++) out[p++] = 1;
  for (int i = 0; i < w && p < F_LEN; i++) out[p++] = 2;
  p += gap;
  if (p > F_LEN) p = F_LEN;
  for (int i = 0; i < r && p < F_LEN; i++) out[p++] = 1;
}

// Write straight into the live waveform table: paint() re-reads
// Config::waveforms every frame, so an upload takes effect on the next one.
static void fastUploadApply(int lv, const uint8_t t[F_LEN]) {
  memcpy(epd.driver()._config.waveforms.fast_darker[lv - 1], t, F_LEN);
}
static void fastUploadRemove(int lv, const uint8_t t[F_LEN]) {
  memcpy(epd.driver()._config.waveforms.fast_lighter[lv - 1], t, F_LEN);
}

static void fastUploadApplies() {
  uint8_t t[F_LEN];
  for (int lv = 1; lv <= 3; lv++) {
    fastTrainOfShape(fl[lv].d, fl[lv].w, fl[lv].g, fl[lv].r, t);
    fastUploadApply(lv, t);
  }
}

static int fastNetOf(int lv) {
  uint8_t t[F_LEN];
  fastTrainOfShape(fl[lv].d, fl[lv].w, fl[lv].g, fl[lv].r, t);
  int net = 0;
  for (int i = 0; i < F_LEN; i++) { if (t[i] == 1) net++; else if (t[i] == 2) net--; }
  return net;
}

// Charge-matched erase, exactly the rule the 16-grey removes use: k
// darkens then k + net whitens, so net whitens equals the apply's net
// darkens and a paint/unpaint cycle is DC neutral. Darkens FIRST — an
// appended (darken, whiten) scrub pair erases nothing, because the darken
// undoes its whiten's gain almost exactly, whereas deepening first means
// every whiten then fires from a dark state where it is strong.
static void fastRemoveOf(int lv, uint8_t out[F_LEN]) {
  memset(out, 0, F_LEN);
  const int net = fastNetOf(lv);
  if (net <= 0) { out[0] = 1; out[1] = 2; out[2] = 1; out[3] = 2; return; }
  int wr = max(net, 3);
  if (wr > F_LEN) wr = F_LEN;
  int k = wr - net;
  if (k < 0) k = 0;
  while (k + wr > F_LEN && k > 0) { k--; wr--; }
  if (k + wr > F_LEN) { wr = min(F_LEN, net); k = 0; }
  int p = 0;
  for (int i = 0; i < k; i++) out[p++] = 1;
  for (int i = 0; i < wr && p < F_LEN; i++) out[p++] = 2;
}

static void fastUploadRemoves() {
  uint8_t t[F_LEN];
  for (int lv = 1; lv <= 3; lv++) { fastRemoveOf(lv, t); fastUploadRemove(lv, t); }
}

static float fastPredict(int d, int w, int gap, int r) {
  const int dd = constrain(d, 1, F_LEN);
  return fastDose[dd] + w * fastGw - r * fastGr + gap * fastGg;
}

// ---------------------------------------------------------------------------
// Probe: the pure-darken response, 1..7 passes.
//
// Only two slots per card (level 3 is held at seven darkens as the deep
// anchor the gradient correction needs), so the ladder takes three cards
// rather than the single card the 16-grey ladder manages. Cheap: FAST
// paints in well under a second.
// ---------------------------------------------------------------------------
static bool fastProbe() {
  Serial.println("[fast] pure-darken ladder (1..7 passes)...");
  uint8_t anchor[F_LEN];
  for (int i = 0; i < F_LEN; i++) anchor[i] = 1;
  fastUploadApply(3, anchor);

  float ref[4], nat[4];
  for (int card = 0; card < 3; card++) {
    const int n1 = card * 2 + 1, n2 = card * 2 + 2;
    uint8_t t[F_LEN];
    memset(t, 0, F_LEN);
    for (int i = 0; i < n1; i++) t[i] = 1;
    fastUploadApply(1, t);
    memset(t, 0, F_LEN);
    for (int i = 0; i < n2; i++) t[i] = 1;
    fastUploadApply(2, t);

    tunePaintFastCard();
    tuneClear();
    tunePaint();
    if (!tuneMeasureFast(ref, nat)) return false;

    // Gradient correction against this frame's own extremes, so the three
    // cards land on one scale.
    const float g0 = nat[0] - ref[0], g3 = nat[3] - ref[3];
    fastDose[n1] = nat[1] - (g0 + (g3 - g0) * (1 / 3.0f));
    fastDose[n2] = nat[2] - (g0 + (g3 - g0) * (2 / 3.0f));
    if (card == 0) for (int g = 0; g < 4; g++) fastRef[g] = ref[g];
    fastDose[F_LEN] = nat[3] - g3;
  }

  for (int n = 2; n <= F_LEN; n++)                 // saturation is monotone
    if (fastDose[n] > fastDose[n - 1]) fastDose[n] = fastDose[n - 1];

  Serial.print("[fast] dose curve: ");
  for (int n = 1; n <= F_LEN; n++) Serial.printf("%.0f ", fastDose[n]);
  Serial.printf(" (step %.1f units)\n", fastStep());
  return true;
}

// ---------------------------------------------------------------------------
// Float-gap gain, measured rather than assumed.
//
// On the 16-grey modes this knob measured as nothing, twelve times across
// two panels, and the reason is physical: the inter-pass settle floor (4 ms
// of a 15-20 ms period) has already saturated the relaxation, so an extra
// idle period adds nothing. FAST is the one place that argument does NOT
// apply — it has no inter-pass delay at all — so the question is genuinely
// open here and worth one card.
// ---------------------------------------------------------------------------
static bool fastProbeGap() {
  Serial.println("[fast] float-gap ladder...");
  uint8_t anchor[F_LEN];
  for (int i = 0; i < F_LEN; i++) anchor[i] = 1;
  fastUploadApply(3, anchor);

  float land[4] = { 0, 0, 0, 0 };   // landing at gap = 0..3
  float ref[4], nat[4];
  uint8_t t[F_LEN];
  for (int card = 0; card < 2; card++) {
    for (int slot = 0; slot < 2; slot++) {
      const int gap = card * 2 + slot;
      fastTrainOfShape(2, 1, gap, 1, t);       // (d,w,g,r) = (2,1,gap,1)
      fastUploadApply(slot + 1, t);
    }
    tunePaintFastCard();
    tuneClear();
    tunePaint();
    if (!tuneMeasureFast(ref, nat)) return false;
    const float g0 = nat[0] - ref[0], g3 = nat[3] - ref[3];
    land[card * 2]     = nat[1] - (g0 + (g3 - g0) * (1 / 3.0f));
    land[card * 2 + 1] = nat[2] - (g0 + (g3 - g0) * (2 / 3.0f));
  }

  // Least-squares slope of landing against gap length.
  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int g = 0; g < 4; g++) { sx += g; sy += land[g]; sxx += (float)g * g; sxy += g * land[g]; }
  const float den = 4 * sxx - sx * sx;
  const float slope = (fabsf(den) < 1e-3f) ? 0.0f : (4 * sxy - sx * sy) / den;

  // A measured nothing sets the gain to ZERO, not to a floor. Clamping a
  // null up to a minimum is what once made the gap the finest knob in the
  // set, and the seeder then spent train length buying units the panel
  // never delivered.
  fastGg = (fabsf(slope) >= 0.25f * fastStep()) ? fabsf(slope) : 0.0f;
  Serial.printf("[fast] gap gain %.1f units/pass (slope %+.1f, landings %.0f %.0f %.0f %.0f)%s\n",
                fastGg, slope, land[0], land[1], land[2], land[3],
                fastGg == 0.0f ? " - treating float gaps as neutral" : "");
  if (fastGg > 0.0f)
    Serial.println("[fast] NOTE: unlike NORMAL/HIGH, a float gap does something "
                   "here - FAST has no inter-pass settle to saturate it");
  return true;
}

// ---------------------------------------------------------------------------
// Convergence: two free levels, a small grammar, a learned bias.
//
// With only two candidates measurable per card there is no point in a
// breadth-first search, so this is a fixed-point iteration instead: the
// model predicts a landing, the glass says what it really did, the
// difference is carried as a per-level bias, and the shape is re-solved
// against the corrected model. Damped, because an undamped bias update
// oscillates for exactly the reason the 16-grey converger does.
// ---------------------------------------------------------------------------
static void fastSolveShape(int lv) {
  const FastLv &L = fl[lv];
  int bd = 1, bw = 0, bg = 0, br = 0;
  float best = 1e9f;
  for (int d = 1; d <= F_LEN; d++)
    for (int w = 0; w <= 3; w++)
      for (int gap = 0; gap <= 2; gap++)
        for (int r = 0; r <= 3; r++) {
          if (w == 0 && (r || gap)) continue;      // re-darkens need a whiten first
          if (r == 0 && gap) continue;             // a gap only weakens re-darkens
          if (d + w + gap + r > F_LEN) continue;
          const float land = fastPredict(d, w, gap, r) + L.bias;
          const float score = fabsf(land - fastRef[lv]) + 0.10f * (d + w + gap + r);
          if (score < best) { best = score; bd = d; bw = w; bg = gap; br = r; }
        }
  fl[lv].d = bd; fl[lv].w = bw; fl[lv].g = bg; fl[lv].r = br;
}

static bool fastConverge(float *worstOut) {
  // Level 3 is the anchor: every pass a darken, which is the darkest FAST
  // can render. Nothing to search.
  fl[3] = {}; fl[3].d = F_LEN; fl[3].bestAbs = 1e9f;
  for (int lv = 1; lv <= 2; lv++) {
    fl[lv] = {};
    fl[lv].bestAbs = 1e9f;
    fl[lv].d = lv * 2;                 // rough start; the first cycle re-solves
  }
  fastUploadApplies();

  float ref[4], nat[4], err[4];
  bool converged = false;
  float mx = 999;
  for (int cyc = 0; cyc < FAST_MAX_CYCLES; cyc++) {
    if (tuneAbortRequested()) { Serial.println("[fast] aborted"); break; }
    fastUploadApplies();
    tunePaintFastCard();
    tuneClear();
    tunePaint();
    if (!tuneMeasureFast(ref, nat)) return false;
    for (int g = 0; g < 4; g++) fastRef[g] = ref[g];
    fastErrors(ref, nat, err);

    mx = 0;
    for (int lv = 1; lv <= 2; lv++) {
      fl[lv].err = err[lv];
      mx = max(mx, fabsf(err[lv]));
      if (fabsf(err[lv]) < fl[lv].bestAbs) {
        fl[lv].bestAbs = fabsf(err[lv]);
        fl[lv].bestErr = err[lv];
        fl[lv].bd = fl[lv].d; fl[lv].bw = fl[lv].w;
        fl[lv].bg = fl[lv].g; fl[lv].br = fl[lv].r;
      }
    }
    Serial.printf("[fast] cycle %2d  L1 (%d,%d,%d,%d) %+5.1f   L2 (%d,%d,%d,%d) %+5.1f   "
                  "worst %.1f / tol %.1f\n",
                  cyc, fl[1].d, fl[1].w, fl[1].g, fl[1].r, err[1],
                  fl[2].d, fl[2].w, fl[2].g, fl[2].r, err[2], mx, fastTol());
    if (mx <= fastTol()) { converged = true; break; }

    for (int lv = 1; lv <= 2; lv++) {
      // What the glass did, minus what the model said it would: the model's
      // own error for this level, learned at half strength.
      const float measured = ref[lv] + err[lv];
      const float predicted = fastPredict(fl[lv].d, fl[lv].w, fl[lv].g, fl[lv].r);
      fl[lv].bias += 0.5f * (measured - predicted - fl[lv].bias);
      fastSolveShape(lv);
    }
  }

  // Whatever the loop ended on, keep each level's best-measured shape.
  if (!converged)
    for (int lv = 1; lv <= 2; lv++)
      if (fl[lv].bestAbs < 1e8f) {
        fl[lv].d = fl[lv].bd; fl[lv].w = fl[lv].bw;
        fl[lv].g = fl[lv].bg; fl[lv].r = fl[lv].br;
        fl[lv].err = fl[lv].bestErr;    // signed: the curve is built from it
      }
  fastUploadApplies();
  *worstOut = mx;
  return true;
}

// Optical check on the erases: paint the card, repaint white, and read what
// is left against the never-driven group 0. A remove that leaves a residual
// is a ghost.
static void tuneRepaintFastCardThenWhite() {
  tuneRepaintFastCard();
  tuneFill(0);
  tunePaint();
}

static float fastVerifyRemoves() {
  fastUploadRemoves();
  tunePaintFastCard();
  tuneClear();
  tunePaint();
  tuneFill(0);
  tunePaint();
  delay(300);
  tuneRepaint = tuneRepaintFastCardThenWhite;
  if (!tuneScan(tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm)) return -1;
  const int H = epd.height();
  float col[4];
  for (int g = 0; g < 4; g++) {
    float s = 0;
    for (int c = 0; c < 4; c++) {
      int px0, px1, sx0, sx1, sy0, sy1;
      tuneColSpan(g * 4 + c, &px0, &px1);
      tunePanelRect(px0, px1, (int)(H * 0.08f), (int)(H * 0.92f), &sx0, &sx1, &sy0, &sy1);
      s += tuneRectMean(sx0, sx1, sy0, sy1);
    }
    col[g] = s / 4;
  }
  float worst = 0;
  Serial.print("[fast] remove residuals: ");
  for (int g = 1; g <= 3; g++) {
    const float res = col[g] - col[0];
    Serial.printf("%+.1f ", res);
    if (fabsf(res) > fabsf(worst)) worst = res;
  }
  Serial.printf("  worst %+.1f\n", worst);
  return fabsf(worst);
}

// ---------------------------------------------------------------------------
// Phase 2: the six grey-to-grey direct trains.
//
// Card layout, nine columns in three groups (the grey16_testcard 'D' card):
//
//   | ref  2>1  3>1 | ref  1>2  3>2 | ref  1>3  2>3 |
//
// The group's reference column is painted from white, so each transition
// is judged against the same level arrived at the ordinary way, in the same
// frame. All six pairs are therefore measured by every scan — which is why
// this phase is cheap despite being six unknowns.
//
// DC balance is not searched for, it is imposed: a direct train's net
// darkens must equal Q(to) - Q(from), with Q(g) the net darkens of the
// level's own apply train. That makes the charge ledger path-independent —
// white>1>2>3>white nets zero along any route — so the only free parameter
// per pair is how deep the balancing run goes before the payload. Which
// direction that run takes is the shape lesson from the July work:
// darkening transitions put their whitens FIRST (so the darkens land on
// freshly whitened glass, where they are strong), lightening transitions
// put their darkens first for the same reason mirrored.
// ---------------------------------------------------------------------------
static const uint8_t DIR_FROM[9] = { 0, 2, 3,  0, 1, 3,  0, 1, 2 };
static const uint8_t DIR_TO[9]   = { 1, 1, 1,  2, 2, 2,  3, 3, 3 };

static void tunePaintDirCard(const uint8_t *lv) {
  uint8_t *fb = epd.getBuffer();
  const int W = epd.width(), H = epd.height();
  for (int i = 0; i < 9; i++) {
    const int x0 = (i * W) / 9, x1 = ((i + 1) * W) / 9;
    for (int y = 0; y < H; y++)
      memset(fb + (size_t)y * W + x0, lv[i], x1 - x0);
  }
}

// Drive the card: establish every column's FROM level, settle, then repaint
// the TO levels in ONE paint. That single paint is the direct engine's
// gate — a pair without a loaded train would two-step through white here
// and read as a bad landing, which is exactly what we want to see.
static void tuneDriveDirCard() {
  tuneClear();
  tunePaintDirCard(DIR_FROM);
  epd.paint();
  tuneWaitIdle();
  epd.paint();
  tuneWaitIdle();
  delay(300);
  tunePaintDirCard(DIR_TO);
  epd.paint();
  tuneWaitIdle();
}

static void tuneRepaintDirCard() { tuneDriveDirCard(); }

// Read the nine columns, then take out the horizontal trend the three
// reference columns reveal. That trend is real and it is large — the
// 16-grey rescue measured ~13 units across the card — so comparing a
// transition column against its group's reference without correcting it
// would bury a 3-unit landing error under a bias three times the size.
static bool tuneMeasureDirCard(float land[9]) {
  tuneRepaint = tuneRepaintDirCard;
  if (!tuneScan(tgeo.x0mm, tgeo.y0mm, tgeo.wmm, tgeo.hmm)) return false;
  const int W = epd.width(), H = epd.height();
  for (int i = 0; i < 9; i++) {
    const int px0 = (i * W) / 9 + W / 36, px1 = ((i + 1) * W) / 9 - W / 36;
    int sx0, sx1, sy0, sy1;
    tunePanelRect(px0, px1, (int)(H * 0.08f), (int)(H * 0.92f), &sx0, &sx1, &sy0, &sy1);
    land[i] = tuneRectMean(sx0, sx1, sy0, sy1);
  }
  Serial.print("[fast] dir card:");
  for (int i = 0; i < 9; i++) Serial.printf(" %3.0f", land[i]);
  Serial.println();
  return true;
}

// Net darkens of a level's tuned apply train — the charge a pixel at that
// level is holding.
static int fastQ(int lv) { return lv == 0 ? 0 : fastNetOf(lv); }

// Build a direct train for `net` charge with a balancing run of `depth`.
// Returns its length.
static int fastDirTrain(int net, int depth, uint8_t *out, int cap) {
  memset(out, 0, cap);
  int p = 0;
  if (net >= 0) {
    // Darkening: whitens first, then depth + net darkens.
    for (int i = 0; i < depth && p < cap; i++) out[p++] = 2;
    for (int i = 0; i < depth + net && p < cap; i++) out[p++] = 1;
  } else {
    // Lightening: darkens first, then depth + |net| whitens. Deepening
    // toward saturation is optically cheap, and every whiten then fires
    // from a dark state where it is strong — at zero extra net charge.
    for (int i = 0; i < depth && p < cap; i++) out[p++] = 1;
    for (int i = 0; i < depth - net && p < cap; i++) out[p++] = 2;
  }
  return p;
}

static bool fastTuneDirect(uint8_t dirtbl[4][4][EPD_Painter::DEC_WF_LEN_DIR],
                           uint16_t *loadedOut, float *worstOut) {
  if (!epd.driver().setDirectTransitions(true)) {
    Serial.println("[fast] direct engine would not enable");
    return false;
  }
  // Start every pair from the two-step so an un-converged pair is left
  // unloaded rather than loaded with a bad train.
  for (int f = 1; f <= 3; f++)
    for (int t = 1; t <= 3; t++)
      if (f != t) epd.driver().setDirectTrain((uint8_t)f, (uint8_t)t, nullptr);

  int depth[9];
  float bestErr[9];
  int   bestDepth[9];
  for (int i = 0; i < 9; i++) { depth[i] = 3; bestErr[i] = 1e9f; bestDepth[i] = 3; }

  const int cap = EPD_Painter::DEC_WF_LEN_DIR;
  uint8_t tr[EPD_Painter::DEC_WF_LEN_DIR];
  float land[9];
  float worst = 999;

  for (int cyc = 0; cyc < FAST_DIR_CYCLES; cyc++) {
    if (tuneAbortRequested()) { Serial.println("[fast] aborted"); break; }
    for (int i = 0; i < 9; i++) {
      const int f = DIR_FROM[i], t = DIR_TO[i];
      if (!f) continue;                                  // reference column
      const int net = fastQ(t) - fastQ(f);
      const int len = fastDirTrain(net, depth[i], tr, cap);
      epd.driver().setDirectTrain((uint8_t)f, (uint8_t)t, tr, len);
    }
    tuneDriveDirCard();
    if (!tuneMeasureDirCard(land)) return false;

    // Horizontal trend from the three reference columns (0, 3, 6), which
    // all hold the same thing their group's transitions are aiming at.
    // Fit a line through them and judge every column against the fit.
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int k = 0; k < 3; k++) {
      const float x = (float)(k * 3), y = land[k * 3];
      sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const float den = 3 * sxx - sx * sx;
    const float slope = fabsf(den) < 1e-3f ? 0.0f : (3 * sxy - sx * sy) / den;

    worst = 0;
    for (int i = 0; i < 9; i++) {
      const int f = DIR_FROM[i];
      if (!f) continue;
      // The group's reference, carried to this column by the fitted trend.
      const float target = land[(i / 3) * 3] + slope * (i - (i / 3) * 3);
      const float err = land[i] - target;
      if (fabsf(err) < bestErr[i]) { bestErr[i] = fabsf(err); bestDepth[i] = depth[i]; }
      if (fabsf(err) > worst) worst = fabsf(err);
      // One step of a damped 1-D search. A deeper balancing run drives the
      // pixel further toward its rail before the payload, so it lands
      // darker for a darkening pair and lighter for a lightening one.
      const int net = fastQ(DIR_TO[i]) - fastQ(f);
      const float step = err / max(2.0f, 0.35f * fastStep());
      int d = depth[i] + (int)lroundf((net >= 0 ? -step : step));
      depth[i] = constrain(d, 0, (cap - abs(net)) / 2);
    }
    Serial.printf("[fast] dir cycle %2d  worst %.1f / tol %.1f  depths", cyc, worst, fastTol());
    for (int i = 0; i < 9; i++) if (DIR_FROM[i]) Serial.printf(" %d>%d:%d", DIR_FROM[i], DIR_TO[i], depth[i]);
    Serial.println();
    if (worst <= fastTol()) break;
  }

  // Keep each pair's best-measured depth, and publish only the pairs that
  // actually landed. A pair left on the two-step is slower but correct;
  // a pair loaded with a bad train is neither.
  uint16_t loaded = 0;
  for (int i = 0; i < 9; i++) {
    const int f = DIR_FROM[i], t = DIR_TO[i];
    if (!f) continue;
    const int net = fastQ(t) - fastQ(f);
    const int len = fastDirTrain(net, bestDepth[i], tr, cap);
    if (bestErr[i] <= 1.5f * fastTol()) {
      epd.driver().setDirectTrain((uint8_t)f, (uint8_t)t, tr, len);
      memcpy(dirtbl[f][t], tr, EPD_Painter::DEC_WF_LEN_DIR);
      loaded |= (uint16_t)(1u << ((f << 2) | t));
      Serial.printf("[fast] %d>%d converged: depth %d, net %+d, %d passes, err %.1f\n",
                    f, t, bestDepth[i], net, len, bestErr[i]);
    } else {
      epd.driver().setDirectTrain((uint8_t)f, (uint8_t)t, nullptr);
      Serial.printf("[fast] %d>%d NOT converged (best err %.1f) - left on the two-step\n",
                    f, t, bestErr[i]);
    }
  }
  *loadedOut = loaded;
  *worstOut = worst;
  return true;
}

// ---------------------------------------------------------------------------
// Results.
//
// Its own screen rather than the 16-grey one: that page draws a sixteen
// level ramp out of probeRef[] and tl[].err, neither of which a FAST run
// ever fills. It would have rendered a plausible-looking page of stale
// numbers, which is worse than no page at all.
// ---------------------------------------------------------------------------
static bool fastResultsScreen(float worst, float erase, int npairs, float worstDir) {
  const int W = epd.width(), H = epd.height();
  const float span = max(20.0f, fastRef[0] - fastRef[3]);
  auto pct = [&](float u) { return 100.0f * u / span; };

  uint8_t *fb = epd.getBuffer();
  memset(fb, 0, (size_t)W * H);

  epd.fillRect(0, 0, W, 46, 3);
  epd.setTextColor(0);
  epd.setTextSize(4);
  epd.setCursor(14, 8);
  epd.print("FAST DONE");
  epd.setTextSize(2);
  epd.setCursor(300, 16);
  char hd[72];
  snprintf(hd, sizeof(hd), "levels %+.1f%%   erase %.1f%%   %d/6 pairs",
           pct(worst), pct(erase), npairs);
  epd.print(hd);

  // Four patches, dithered reference above and tuned grey below. A level
  // that is right shows no seam — far easier to judge than a grey alone.
  epd.setTextColor(3);
  const int cw = W / 4, rampY = 60, rampH = 150, half = rampH / 2;
  for (int g = 0; g < 4; g++) {
    const int x = g * cw;
    const int thr = (g * 64 + 1) / 3;
    for (int y = rampY; y < rampY + half; y++)
      for (int px = x + 2; px < x + cw - 2; px++)
        fb[(size_t)y * W + px] = (tuneBayer8[y & 7][px & 7] < thr) ? 3 : 0;
    epd.fillRect(x + 2, rampY + half, cw - 4, rampH - half, (uint8_t)g);
    epd.drawRect(x + 2, rampY, cw - 4, rampH, 3);
    char lb[24];
    snprintf(lb, sizeof(lb), "L%d  %+.1f%%", g,
             pct(g >= 1 && g <= 2 ? fl[g].err : 0.0f));
    epd.setTextSize(2);
    epd.setCursor(x + 12, rampY + rampH + 8);
    epd.print(lb);
  }
  epd.setTextSize(1);
  epd.setCursor(6, rampY - 10);
  epd.print("top = target dither, bottom = tuned level (a good level shows no seam)");

  epd.setTextSize(2);
  epd.setCursor(20, rampY + rampH + 44);
  char l2[96];
  snprintf(l2, sizeof(l2), "Grey-to-grey: %d of 6 pairs tuned, worst landing %+.1f%%",
           npairs, pct(worstDir));
  epd.print(l2);
  epd.setCursor(20, rampY + rampH + 68);
  epd.print(npairs == 6
            ? "All transitions go direct - no two-step through white."
            : "Untuned pairs keep the two-step: correct, but they flash white.");

  Btn save = { 60, H - 130, 360, 110 };
  Btn skip = { 540, H - 130, 360, 110 };
  drawBtn(save, "SAVE", "use on every boot");
  drawBtn(skip, "SKIP", "keep built-in");
  tuneClear();
  tunePaint();
  touchRewake();

  Serial.printf("[fast] results: levels %+.1f%%, erase %.1f%%, %d/6 pairs\n",
                pct(worst), pct(erase), npairs);
  Serial.println("[fast] tap SAVE or SKIP (serial: s = save, k = skip)");
  for (;;) {
    if (Serial.available()) {
      const int c = Serial.read();
      if (c == 's') return true;
      if (c == 'k' || c == 'q') return false;
    }
    int px, py;
    if (touchTapped(px, py)) {
      if (hitBtn(save, px, py)) return true;
      if (hitBtn(skip, px, py)) return false;
    }
    delay(8);
  }
}

// ---------------------------------------------------------------------------
// The whole FAST run.
// ---------------------------------------------------------------------------
static bool runSelfTuneFast() {
  if (!scPort) {
    Serial.println("[fast] no scanner selected");
    return false;
  }
  Serial.println("[fast] ===== SELF-TUNE START (FAST) =====");
  Serial.println("[fast] send q to abort between cycles");

  // 16-grey must go before the quality can: the driver refuses FAST while
  // sixteen levels are on, and it is right to.
  if (!epd.driver().setGreyLevels(4)) {
    Serial.println("[fast] setGreyLevels(4) refused");
    return false;
  }
  epd.setQuality(EPD_Painter::Quality::QUALITY_FAST);

  // The registration patterns (tuneFill(15), the quadrant, the ring) are
  // written for 16-grey and paint level 15. That is not a bug here: the
  // 4-level compaction masks each canvas byte with 0x03, so 15 arrives as
  // 3 = black. Said out loud because it is an accident of the encoding
  // rather than a decision, and a future change to the packing would break
  // registration in this mode only.

  // Keep the tables we are about to overwrite, so a SKIP really does
  // restore what the board had.
  EPD_Painter::Waveforms saved = epd.driver()._config.waveforms;

  if (!tuneRegister())  { tuneResultScreen(false, 0, 0); return false; }
  if (!fastProbe())     { tuneResultScreen(false, 0, 0); return false; }
  if (!fastProbeGap())  Serial.println("[fast] gap probe failed - assuming neutral");

  float worst4 = 999;
  if (!fastConverge(&worst4)) { tuneResultScreen(false, 0, 0); return false; }

  Serial.println("[fast] deriving charge-matched removes...");
  float erase = fastVerifyRemoves();

  // The measured landing of the four levels, normalised so 0 is paper and
  // 3 is the deepest FAST black. drawGray8() quantises against this — and
  // 4-level mode had no curve of its own until now, so it was quantising
  // against the first four entries of the 16-grey curve, which describe
  // four near-white levels rather than a full ramp.
  uint8_t lum4[4];
  {
    float meas[4];
    for (int g = 0; g < 4; g++) meas[g] = fastRef[g] + (g >= 1 && g <= 2 ? fl[g].err : 0.0f);
    const float white = meas[0], black = meas[3];
    const float span = (white - black) > 1.0f ? (white - black) : 1.0f;
    Serial.print("[fast] measured level curve (255=white): ");
    for (int g = 0; g < 4; g++) {
      lum4[g] = (uint8_t)constrain((int)lroundf(255.0f * (meas[g] - black) / span), 0, 255);
      Serial.printf("%d ", lum4[g]);
    }
    Serial.println();
  }

  Serial.println("[fast] tuning grey-to-grey direct trains...");
  static uint8_t dirtbl[4][4][EPD_Painter::DEC_WF_LEN_DIR];
  memset(dirtbl, 0, sizeof(dirtbl));
  uint16_t loaded = 0;
  float worstDir = 999;
  if (!fastTuneDirect(dirtbl, &loaded, &worstDir))
    Serial.println("[fast] direct phase failed - keeping the two-step for every pair");

  int npairs = 0;
  for (int b = 0; b < 16; b++) if (loaded & (1u << b)) npairs++;

  uint8_t dark[3][7], light[3][7];
  memcpy(dark,  epd.driver()._config.waveforms.fast_darker,  sizeof(dark));
  memcpy(light, epd.driver()._config.waveforms.fast_lighter, sizeof(light));

  Serial.printf("[fast] levels worst %.1f, erase %.1f, %d of 6 direct pairs tuned\n",
                worst4, erase < 0 ? 0.0f : erase, npairs);

  const bool keep = fastResultsScreen(worst4, erase < 0 ? 0.0f : erase,
                                     npairs, worstDir);
  bool saved_ok = false, loaded_ok = false;
  if (keep) {
    saved_ok = tunedSaveFast(epd.driver(), dark, light, dirtbl, loaded, lum4);
    loaded_ok = saved_ok && tunedLoad(epd.driver(), SET_FAST);
    Serial.printf("[fast] flash save %s, reload %s\n",
                  saved_ok ? "OK" : "FAILED", loaded_ok ? "OK" : "FAILED");
    char sub[96];
    snprintf(sub, sizeof(sub), "FAST: 4 levels + %d of 6 direct pairs, loaded every boot",
             npairs);
    tuneConfirmScreen(loaded_ok ? "SAVED" : "SAVE FAILED",
                      loaded_ok ? "Your panel now uses these FAST trains"
                                : "Nothing was stored",
                      loaded_ok ? sub : "Could not write to storage.");
  } else {
    epd.driver()._config.waveforms = saved;
    epd.driver().reloadDirectTrains();
    Serial.println("[fast] discarded - built-in tables restored");
    tuneConfirmScreen("SKIPPED", "Nothing was changed",
                      "The built-in tables are still in charge.");
  }

  // Put the sketch back the way it runs: the menu, the demo scan and the
  // 16-grey tunes all expect 16 greys at NORMAL, and leaving the board in
  // 4-level FAST with the direct engine's spill planes allocated would be
  // a surprise the next thing to run has to discover for itself.
  epd.driver().setDirectTransitions(false);
  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  epd.driver().setGreyLevels(16);

  Serial.printf("[fast] ===== SELF-TUNE DONE (%s) =====\n",
                keep ? "saved" : "discarded");
  return keep && saved_ok && loaded_ok;
}

#endif  // TUNEUP_HAVE_JPEGDEC
