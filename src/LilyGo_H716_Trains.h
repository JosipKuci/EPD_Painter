// ============================================================================
// LilyGo_H716_Trains.h — every waveform the LilyGo T5 4.7" EPD47 H716 drives.
//
// One file per board: the 4-level waveform pairs (FAST/NORMAL/HIGH), the
// 16-grey decision-engine tables, and the grey-to-grey direct trains. The
// preset in EPD_Painter_presets.h carries pins and timing only and points
// here; auto-detection therefore selects these tables with the board. A
// tuned blob in NVS (see EPD_Painter_tuned.h and examples/other/tuneup)
// overrides them at begin().
//
// Everything below was measured on one H716 panel with the self-tuning rig
// (examples/other/tuneup, flatbed scanner in the loop), 28 July 2026, at
// the preset's timing: 20 ms pass period, 4 us row charge, 1000 ns LE.
// The tables are valid ONLY at that timing.
// ============================================================================

#ifndef EPD_PAINTER_LILYGO_H716_TRAINS_H
#define EPD_PAINTER_LILYGO_H716_TRAINS_H

#include <stdint.h>

#if defined(EPD_PAINTER_PRESET_LILYGO_EPD47_H716) || defined(EPD_PAINTER_PRESET_AUTO)

// ---- 4-level waveform pairs ------------------------------------------------
// FAST and NORMAL pairs scanner-tuned 28 July 2026 against the corrected
// dose ladder (FAST levels saved to blob earlier in the session; NORMAL
// levels worst 6.3 units against a 8.0 tolerance, erase residual 6.4).
// Both came out as plain darken runs — this panel's undelayed first
// passes are strong enough that no whiten trim was needed.
//
// The HIGH 4-level pair is the 22 July dithertune set, untouched: the
// HIGH_4 combination has never been through the scanner rig.
inline const EPD_Painter::Waveforms EPD_WF_H716 = {
    .fast_lighter   = { { 1, 2, 2, 2, 0, 0, 0 },
                        { 2, 2, 2, 2, 0, 0, 0 },
                        { 2, 2, 2, 2, 2, 2, 2 } },
    .fast_darker    = { { 1, 1, 0, 0, 0, 0, 0 },
                        { 1, 1, 1, 1, 0, 0, 0 },
                        { 1, 1, 1, 1, 1, 1, 1 } },
    .normal_lighter = { { 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                        { 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
    .normal_darker  = { { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                        { 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
                        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
    .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                        { 1, 1, 2, 2, 1, 2, 2, 2, 2, 1, 2, 2, 2 },
                        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
    .high_darker    = { { 1, 3, 1, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1 },
                        { 1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 1 },
                        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
};

// Measured landing of the four FAST levels, 255 = paper white. (4-grey
// NORMAL measured {255,192,101,0} in the same session — Config carries
// one level_lum4 pointer for both 4-level modes, a limitation the mode
// records in MODES.md exist to remove. The FAST curve is wired because
// that is what a FAST tuned blob installs.)
inline const uint8_t LEVEL_LUM4_H716_FAST[4] = { 255, 201, 122, 0 };

// ---- 16-grey decision-engine tables ----------------------------------------
// Evolutionary tune, 28 July 2026 (raw drive sequences — no grammar; see
// examples/other/tuneup/tuner_evo.h). NORMAL: match card worst 2.3 scan
// units after confirmation, full-height staircase worst 4.2 with ZERO
// inversions. Seeded from this board's previous tuned set.
inline const uint8_t TUNED16_H716_NORMAL[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 255
  /*  1 */ {1,1,2,2,0,0,0,0,0,0,0,0,0},  // measured 242
  /*  2 */ {1,1,1,1,1,1,2,2,2,0,0,0,0},  // measured 225
  /*  3 */ {1,1,2,1,0,2,0,0,0,0,0,0,0},  // measured 200
  /*  4 */ {0,1,1,2,0,0,0,0,0,0,0,0,0},  // measured 187
  /*  5 */ {1,1,1,2,0,1,2,0,0,0,0,0,0},  // measured 158
  /*  6 */ {1,1,2,1,1,0,1,1,2,1,2,0,0},  // measured 129
  /*  7 */ {0,1,1,0,1,1,0,1,1,1,2,1,2},  // measured  98
  /*  8 */ {1,1,1,0,1,1,1,2,0,0,2,0,1},  // measured  77
  /*  9 */ {1,1,1,1,2,0,0,0,1,0,0,0,0},  // measured  53
  /* 10 */ {1,0,0,1,1,2,1,0,1,0,0,0,0},  // measured  40
  /* 11 */ {1,0,1,1,1,1,2,0,1,0,0,0,0},  // measured  35
  /* 12 */ {1,1,1,0,1,0,0,0,0,0,0,0,0},  // measured  27
  /* 13 */ {0,1,1,0,1,1,0,1,0,0,0,0,0},  // measured  19
  /* 14 */ {1,0,1,1,1,0,0,1,1,0,1,0,0},  // measured   6
  /* 15 */ {1,1,1,1,1,1,1,1,1,1,1,0,0},  // measured   0
};

// Charge-matched removes, optically verified same session (white-landing
// residuals within 3.7 scan units).
inline const uint8_t TUNED16_H716_NORMAL_REMOVE[16][13] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1 */ {1,2,1,2,0,0,0,0,0,0,0,0,0},
  /*  2 */ {1,1,1,2,2,2,2,2,2,0,0,0,0},
  /*  3 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  4 */ {1,1,1,1,2,2,2,2,2,0,0,0,0},
  /*  5 */ {1,1,1,1,2,2,2,2,2,2,0,0,0},
  /*  6 */ {1,1,1,2,2,2,2,2,2,2,0,0,0},
  /*  7 */ {1,1,2,2,2,2,2,2,2,2,0,0,0},
  /*  8 */ {1,1,2,2,2,2,2,2,2,0,0,0,0},
  /*  9 */ {1,1,2,2,2,2,2,2,0,0,0,0,0},
  /* 10 */ {1,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 11 */ {1,2,2,2,2,2,2,0,0,0,0,0,0},
  /* 12 */ {1,2,2,2,2,2,0,0,0,0,0,0,0},
  /* 13 */ {2,2,2,2,2,0,0,0,0,0,0,0,0},
  /* 14 */ {2,2,2,2,2,2,2,0,0,0,0,0,0},
  /* 15 */ {2,2,2,2,2,2,2,2,2,2,2,0,0},
};

// What each level ACTUALLY renders as, measured on the FULL-HEIGHT
// STAIRCASE rather than the tuning card: a photograph is staircase-like
// content, and the two curves differ (the card's rows are cheaper to
// convert, so heavy content runs a slightly different dose). Image
// dithering quantises against this; the staircase measurement repeats
// within 1-2 lum.
inline const uint8_t LEVEL_LUM_H716_NORMAL[16] =
    { 255, 242, 225, 200, 187, 158, 129, 98, 77, 53, 40, 35, 27, 19, 6, 0 };

// HIGH: the first HIGH set any board has shipped. 20 passes at the same
// 20 ms period (more slots, not slower ones). Match card worst 2.9 after
// confirmation, staircase worst 4.2, zero inversions.
inline const uint8_t TUNED16_H716_HIGH[16][32] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 255
  /*  1 */ {1,1,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 241
  /*  2 */ {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 215
  /*  3 */ {1,2,0,1,0,1,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 194
  /*  4 */ {0,1,2,1,0,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 183
  /*  5 */ {1,1,1,2,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 156
  /*  6 */ {1,2,0,1,1,1,0,1,2,1,1,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 122
  /*  7 */ {0,1,1,1,0,2,1,0,1,1,1,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured 103
  /*  8 */ {1,0,1,2,1,1,2,2,0,1,2,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured  66
  /*  9 */ {2,1,1,1,0,1,2,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured  47
  /* 10 */ {0,1,1,2,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured  36
  /* 11 */ {0,1,0,1,2,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured  30
  /* 12 */ {2,0,1,1,0,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured  18
  /* 13 */ {0,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured  17
  /* 14 */ {0,1,0,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured   6
  /* 15 */ {1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},  // measured   0
};

inline const uint8_t TUNED16_H716_HIGH_REMOVE[16][32] = {
  /*  0 (white) */ {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  1 */ {1,2,1,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  2 */ {1,1,1,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  3 */ {1,1,1,1,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  4 */ {1,1,1,1,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  5 */ {1,1,1,1,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  6 */ {1,1,1,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  7 */ {1,1,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  8 */ {1,1,1,1,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /*  9 */ {1,1,1,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 10 */ {1,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 11 */ {1,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 12 */ {1,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 13 */ {2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 14 */ {2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  /* 15 */ {2,2,2,2,2,2,2,2,2,2,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
};

inline const uint8_t LEVEL_LUM_H716_HIGH[16] =
    { 255, 241, 215, 194, 183, 156, 122, 103, 66, 47, 36, 30, 18, 17, 6, 0 };

// ---- direct grey-to-grey trains (NORMAL) -----------------------------------
// Scanner-tuned 28 July 2026, all SIX pairs converged (worst 6.4 scan
// units, tolerance 8.0) — the first complete set on this board. Both
// directions use the overshoot-and-return shape: darkening pairs put
// their charge-balancing whitens first; lightening pairs whiten PAST the
// target and let re-darkens (full fresh-response strength on this glass)
// walk the landing back down. DC-matched to the 4-grey NORMAL pairs
// above: potentials Q=(2,4,13).
inline const uint8_t DIRECT_H716_NORMAL[4][4][26] = {
  { {0}, {0}, {0}, {0} },
  { /* from 1 */
    {0},
    {0},
    /* 1->2 net +2  (6) */ {2,2,1,1,1,1},
    /* 1->3 net +11 (17) */ {2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 2 */
    {0},
    /* 2->1 net -2  (4) */ {2,2,2,1},
    {0},
    /* 2->3 net +9 (15) */ {2,2,2,1,1,1,1,1,1,1,1,1,1,1,1},
  },
  { /* from 3 */
    {0},
    /* 3->1 net -11 (15) */ {2,2,2,2,2,2,2,2,2,2,2,2,2,1,1},
    /* 3->2 net -9  (19) */ {2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1},
    {0},
  },
};

// No FAST direct set. The July FAST directs were tuned against the old
// FAST waveform pair and its potentials; promoting the newly measured
// FAST levels invalidates them (wrong net charge AND wrong optics), and
// the FAST tune session skipped its direct phase. Pairs fall back to the
// DC-correct two-step until a FAST direct session runs — the fixed
// search converges in a couple of cycles, so it is a ten-minute job.

#endif // H716
#endif // EPD_PAINTER_LILYGO_H716_TRAINS_H
