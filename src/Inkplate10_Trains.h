// ============================================================================
// Inkplate10_Trains.h — every waveform the Inkplate 10 (ED097TC2) drives.
//
// NOT YET TUNED FOR THIS PANEL. The 4-level pairs below are the M5PaperS3's
// 30 July 2026 NORMAL/FAST/HIGH set, carried over as a starting point: same
// drive-code conventions, DC-balanced row pairs, and full black / full white
// (level 3 / its remove) are saturating runs that land on any E Ink glass.
// Levels 1 and 2 are a different matter — they are doses, and a dose is only
// valid at the row timing it was calibrated at. This panel runs 825 rows at
// a 4 MHz pixel clock, so a pass here is several times longer than on the
// panel these were measured on; expect the two greys to land darker than
// intended until a tuning session replaces them.
//
// There are no 16-grey tables and no direct trains: 16-grey runs on the
// formula library and grey-to-grey transitions take the two-step path. A
// tuned blob in NVS (see EPD_Painter_tuned.h and examples/other/tuneup)
// overrides all of this at begin(), which is the intended route to real
// tables for this board.
// ============================================================================

#ifndef EPD_PAINTER_INKPLATE10_TRAINS_H
#define EPD_PAINTER_INKPLATE10_TRAINS_H

#include <stdint.h>

// ---- 4-level waveform pairs (seed, untuned on ED097TC2) --------------------
inline const EPD_Painter::Waveforms EPD_WF_INKPLATE10 = {
    .fast_lighter   = { { 1, 2, 2, 2, 2, 2, 3 },
                        { 3, 2, 2, 2, 2, 2, 3 },
                        { 2, 2, 2, 2, 2, 2, 2 } },
    .fast_darker    = { { 1, 1, 3, 3, 1, 3, 1 },
                        { 3, 1, 1, 1, 1, 1, 3 },
                        { 1, 1, 1, 1, 1, 1, 1 } },
    .normal_lighter = { { 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 1, 2 },
                        { 3, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2 },
                        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
    .normal_darker  = { { 1, 1, 1, 3, 1, 1, 1, 0, 0, 1, 2, 3, 2 },
                        { 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 2, 3 },
                        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
    .high_lighter   = { { 1, 3, 1, 1, 1, 2, 1, 2, 2, 2, 2, 2, 2 },
                        { 1, 3, 3, 1, 3, 2, 2, 2, 2, 2, 2, 2, 2 },
                        { 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2 } },
    .high_darker    = { { 1, 3, 1, 1, 2, 2, 2, 1, 2, 1, 1, 2, 1 },
                        { 3, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 2 },
                        { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
};

#endif // EPD_PAINTER_INKPLATE10_TRAINS_H
