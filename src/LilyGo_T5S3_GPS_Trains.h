// ============================================================================
// LilyGo_T5S3_GPS_Trains.h — every waveform the LilyGo T5 S3 GPS drives.
//
// One file per board: the 4-level waveform pairs (FAST/NORMAL/HIGH), the
// 16-grey decision-engine tables, and the grey-to-grey direct trains. The
// preset in EPD_Painter_presets.h carries pins and timing only and points
// here; auto-detection therefore selects these tables with the board. A
// tuned blob in NVS (see EPD_Painter_tuned.h and examples/other/tuneup)
// overrides them at begin().
// ============================================================================

#ifndef EPD_PAINTER_LILYGO_T5S3_GPS_TRAINS_H
#define EPD_PAINTER_LILYGO_T5S3_GPS_TRAINS_H

#include <stdint.h>

#if defined(EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS) || defined(EPD_PAINTER_PRESET_AUTO)

// ---- 4-level waveform pairs ------------------------------------------------
inline const EPD_Painter::Waveforms EPD_WF_LILYGO_T5S3_GPS = {
    .fast_lighter   = { { 1, 2, 2, 2, 2, 2, 3 },
                        { 3, 2, 2, 2, 2, 2, 3 },
                        { 2, 2, 2, 2, 2, 2, 2 } },
    .fast_darker    = { { 1, 1, 3, 3, 1, 3, 1 },
                        { 1, 3, 1, 1, 1, 1, 3 },
                        { 1, 1, 1, 1, 1, 1, 1 } },
    // NORMAL tables dither-match calibrated (extras/calibration,
    // 2026-07-18): each driven grey matches a 66%/33% black-dot
    // dither of the panel's own black+white to within ~1 scanner
    // grey level — linear-reflectance spacing. The
    // original hand-tuned rows were already near-perfect; dark grey
    // needed one darkening step (plus the row-1 DC balance fix).
    .normal_lighter = { { 1, 1, 1, 1, 2, 2, 3, 2, 2, 2, 2, 2, 2 },
                        { 3, 2, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3 },
                        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
    .normal_darker  = { { 1, 2, 1, 1, 1, 3, 1, 2, 2, 1, 2, 1, 1 },
                        { 1, 1, 1, 2, 3, 3, 1, 1, 3, 1, 3, 1, 1 },
                        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
    .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                        { 1, 1, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2 },
                        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
    .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                        { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
};

// ---- 16-grey decision-engine tables ----------------------------------------
// Tuned 20 July 2026 UNDER THE CONSTANT PASS PERIOD (15 ms NORMAL) and
// cross-verified: strictly monotonic on both the match card (max err 4.2
// scan units) and the full-height staircase. What the glass taught us
// (vs the formula library's assumptions):
//  - A whiten pass is STRONG: after a short darken run one whiten takes
//    back 20-30 units, near saturation ~10 — never a "half step".
//  - The dose response saturates hard: darken passes 8..13 buy ~3 units
//    total. Deep greys cannot be spaced by run length alone.
//  - Fine steps come from RE-DARKENING after a whiten: patterns like
//    1,1,2,1,1 climb from the lifted grey in small fresh-response steps.
inline const uint8_t TUNED16_LILYGO_T5S3_NORMAL[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1  (2,2)  */ {1,1,2,2,0,0,0,0,0,0,0,0,0},
  /*  2  (2,1)  */ {1,1,2,0,0,0,0,0,0,0,0,0,0},
  /*  3  (3,2)  */ {1,1,1,2,2,0,0,0,0,0,0,0,0},
  /*  4 (3,3,1) */ {1,1,1,2,2,2,1,0,0,0,0,0,0},
  /*  5  (4,2)  */ {1,1,1,1,2,2,0,0,0,0,0,0,0},
  /*  6 (2,2,2) */ {1,1,2,2,1,1,0,0,0,0,0,0,0},
  /*  7 (2,1,2) */ {1,1,2,1,1,0,0,0,0,0,0,0,0},
  /*  8 (3,2,2) */ {1,1,1,2,2,1,1,0,0,0,0,0,0},
  /*  9  (5,1)  */ {1,1,1,1,1,2,0,0,0,0,0,0,0},
  /* 10 (7,2,1) */ {1,1,1,1,1,1,1,2,2,1,0,0,0},
  /* 11  (7,1)  */ {1,1,1,1,1,1,1,2,0,0,0,0,0},
  /* 12 (5,1,1) */ {1,1,1,1,1,2,1,0,0,0,0,0,0},
  /* 13 (11,1)  */ {1,1,1,1,1,1,1,1,1,1,1,2,0},
  /* 14  (8,0)  */ {1,1,1,1,1,1,1,1,0,0,0,0,0},
  /* 15 (13,0)  */ {1,1,1,1,1,1,1,1,1,1,1,1,1},
};

// Charge-matched removes (phase D). The DC constraint: under the
// constant pass period every pass carries equal dose, so a train's net
// charge is simply darkens - whitens; a paint+unpaint cycle returns the
// pixel's ledger to zero only if the remove's net whitens equal the
// apply's net darkens. Scanner-tuned 21 July 2026 with the remove-ladder
// method. What the glass taught us: appended charge-neutral (1,2) scrub
// pairs erase NOTHING — the shape that works is k darkens FIRST
// (deepening toward saturation, optically cheap), then all net+k whitens
// firing from a dark state where whitens are strong. All 15 levels land
// within 1.5 scan units of clear-white.
// Apply nets: L1:0 L2:+1 L3:+1 L4:+1 L5:+2 L6:+2 L7:+3 L8:+3 L9:+4
//             L10:+6 L11:+6 L12:+5 L13:+10 L14:+8 L15:+13
inline const uint8_t TUNED16_LILYGO_T5S3_NORMAL_REMOVE[16][13] = {
  /*  0  net  0 */ {0,0,0,0,0,0,0,0,0,0,0,0,0},   // never removed
  /*  1  net  0 */ {1,2,1,2,0,0,0,0,0,0,0,0,0},   // scrub pairs (light grey)
  /*  2  net -1 */ {1,1,2,2,2,0,0,0,0,0,0,0,0},
  /*  3  net -1 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  4  net -1 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  5  net -2 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /*  6  net -2 */ {1,1,1,2,2,2,2,2,0,0,0,0,0},
  /*  7  net -3 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /*  8  net -3 */ {1,1,2,2,2,2,2,0,0,0,0,0,0},
  /*  9  net -4 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10  net -6 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},  // free budget suffices
  /* 11  net -6 */ {2,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 12  net -5 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /* 13  net -10*/ {2,2,2,2,2,2,2,2,2,2,0,0,0},
  /* 14  net -8 */ {2,2,2,2,2,2,2,2,0,0,0,0,0},
  /* 15  net -13*/ {2,2,2,2,2,2,2,2,2,2,2,2,2},  // was already balanced
};

// ---- direct grey-to-grey trains --------------------------------------------
// Tuned 21 July 2026, seeded from the M5PaperS3 finals (the panels share
// a physics family; four of six FAST trains transferred UNCHANGED).
// NORMAL all within ±4.0 of reference, FAST within ±3.3. This panel's
// re-darkens after whitening bite weaker than the M5Paper's, so most
// trains carry more charge-neutral drive mass.
// NORMAL potentials Q=(4,7,13); FAST potentials identical to the
// M5PaperS3's (4,5,7).
inline const uint8_t DIRECT_LILYGO_T5S3_NORMAL[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +3 (9)  */ {2,2,2,1,1,1,1,1,1},
    /* 1->3 net +9 (15) */ {2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -3 (7)  */ {2,2,2,2,2,1,1},
    {0},
    /* 2->3 net +6 (18) */ {2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -9 (15) */ {2,2,2,2,2,2,2,2,2,2,2,2,1,1,1},
    /* 3->2 net -6 (18) */ {2,2,2,2,2,2,2,2,2,2,2,1,2,1,1,1,1,1},
    {0},
  },
};

inline const uint8_t DIRECT_LILYGO_T5S3_FAST[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +1 (9)  */ {2,2,2,2,1,1,1,1,1},
    /* 1->3 net +3 (13) */ {2,2,2,2,2,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -1 (1)  */ {2},
    {0},
    /* 2->3 net +2 (14) */ {2,2,2,2,2,2,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -3 (5)  */ {2,2,2,2,1},
    /* 3->2 net -2 (8)  */ {2,2,2,2,2,1,1,1},
    {0},
  },
};

#endif // LilyGo T5 S3 GPS
#endif // EPD_PAINTER_LILYGO_T5S3_GPS_TRAINS_H
