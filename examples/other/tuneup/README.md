# TuneUp — the self-tuning rig

The shipped presets were tuned with a closed optical loop: paint a pattern,
scan the glass, measure, adjust. This example puts that loop **on the board
itself**: lay it face-down on any eSCL/AirScan network scanner and it tunes
its own 16-grey waveform trains, saves them to flash, and uses them from
then on. No PC in the loop.

**Board:** Soldered Inkplate 10 (9.7", 1200x825, ED097TC2) on an ESP32-S3.
It has no touch panel, so the whole rig is driven over the serial console
(115200; flash with `USBMode=hwcdc,CDCOnBoot=cdc`). The on-screen buttons
are still drawn, each labelled with the serial key that presses it.

**Placement:** the active area is 203 x 140 mm, so the board must lie
*landscape across the platen's short side* (216 mm on an A4/Letter
flatbed), pushed into the origin corner. The tuner corrects for the
face-down mirroring but not for a 90-degree turn, and 216 mm leaves almost
no slack — if the bezel stops the active area fitting inside the platen
width, the registration scan will clip the panel and the run will fail.

## The flow

1. **WiFi** — credentials are compiled in. Set `WIFI_SSID` / `WIFI_PASS` at
   the top of the sketch; if they're still blank, the panel shows
   instructions to edit the file and reflash.
2. **Scanner** — discovers eSCL scanners via mDNS (`_uscan._tcp`), draws
   them as numbered buttons; type yours' number over serial (`r` rescans).
   The choice is remembered in NVS, so later boots need no interaction.
3. **DEMO SCAN** — scans the bed and shows the image on the panel in 16
   native greys. Proves the plumbing.
4. **TUNE ME** — send `t`: pick which quality to tune (`f`/`n`/`h`), then
   instructions and a 30 s countdown to lay the board face-down and close
   the lid. `T` makes the same choice and starts with no countdown.

## Three qualities, three tunes, three blobs

They are not one tune with a speed setting. Each measures different
tables, and each is stored on its own — **tuning one leaves the other two
exactly as they were**, so you can tune HIGH without risking a NORMAL set
that already works.

| | levels | passes | period | what gets tuned | roughly |
|---|---|---|---|---|---|
| **FAST** | 4 | 7 | none | `fast_darker`/`fast_lighter` + the six grey-to-grey direct trains | ~10 min |
| **NORMAL** | 16 | 13 | preset | 16-grey applies + charge-matched removes | ~10–15 min |
| **HIGH** | 16 | 20 | preset | the same, at 20 passes | ~20 min |

The period is the preset's `g16_pass_us_normal` / `g16_pass_us_high`. On
the Inkplate 10 that is 75 ms (an 825-row loop at the 4 MHz pixel clock
takes ~64 ms by itself), against the 15 ms of the 960x540 boards the
numbers below were measured on — so each paint, and each tune, takes
longer here. The times in the table are from those boards.

**HIGH is NORMAL's timing with more slots** — same pass period, 20
passes instead of 13 (at 15 ms, a 300 ms full-screen paint against
NORMAL's 195 ms). It is emphatically *not* "the same trains, driven slower".

That was measured, not assumed. A dose-ladder sweep at 19 / 15 / 13 ms
(on a LilyGo T5 S3, 26 July 2026 — the history this rig was built on)
found a longer pass buys **no extra depth at all**
— all three periods bottomed out at the identical black — while making
every slot coarser *and* wasting most of them: at 19 ms the response
stopped moving after 7 passes, at 15 ms it was still resolving at 10. So
a longer pass costs accuracy, wastes slots, and takes longer.

What the extra slots are actually for is room. The panel saturates around
pass 10 whatever the period, so every level below that knee has to be
placed by whitening and re-darkening rather than by lengthening a darken
run that has stopped responding — and those shapes need spare passes to
fit. Black is anchored at the *measured* knee (typically 11 darkens), not
at the full train length: driving it for 20 when it stops responding at
10 wastes ten passes and forces a 20-whiten erase to balance them,
doubling the charge cycled through every black paint for no extra depth.

Run the sweep yourself with `D` — six scans per period, against forty for
a full tune, so the period is chosen by measurement.

**FAST** is the odd one out because it has no 16-grey mode at all — seven
undelayed passes cannot hold sixteen distinguishable doses, and
`setGreyLevels(16)` refuses while the quality is FAST. Its tune is
therefore about its four levels and, more usefully, the six grey-to-grey
transitions: without those every grey-to-grey pixel two-steps through
white and punches a visible hole in motion. One limit worth knowing —
4-level mode has only three drive trains on the glass at once, so a card
measures three candidates rather than fourteen, and the FAST tune
converges by iterating rather than by breadth.

## What the self-tune does (see tuner.h, tuner_fast.h)

- **Registration**: paints full black and takes a wide scan to find the
  panel on the bed, then a corner mark to learn the face-down mirroring.
- **Reference**: every measurement is a dither-match card — 16 columns,
  each pairing a Bayer-dithered black/white reference (the optical truth
  for level g/15) with the native waveform grey. Scanner gamma, lamp
  falloff and drift cancel inside each frame; the L0/L15 pairs calibrate
  out the panel's vertical gradient.
- **Probe**: a pure-darken dose ladder measures the panel's response curve
  at the tuning pass period — the initial gain table.
- **Converge**: damped proportional edits over the (darkens, whitens,
  re-darkens) train grammar, one edit per out-of-tolerance level per
  cycle, expected move sizes updated online from what each edit actually
  did, overshoots reverted. Typically converges in 5–15 cycles to within
  ~4 scanner units.
- **Removes**: charge-matched erase trains derived from the tuned applies
  (the DC ledger stays exact), then verified optically — paint the card,
  repaint white, scan the residual; ghosting levels get a wider erase.
- **Save**: tables + train length + tuning period go to LittleFS via
  `src/EPD_Painter_tuned.h` — one file per quality — and are reloaded
  through the same path every boot.

## Using the tuned tables in YOUR sketch

```cpp
#include <LittleFS.h>
#include "tuned_storage.h"          // the ~100-line LittleFS half
...
epd.begin();
LittleFS.begin(true);               // true = format if unformatted

// Installs every set this board has been tuned for. Pass the driver:
// EPD_PainterAdafruit wraps one, so hand over epd.driver().
Serial.printf("%d tuned sets\n", tunedLoadAll(epd.driver()));

epd.setGreyLevels(16);
```

Order doesn't matter relative to `setGreyLevels(16)` — call it either side
and the train library ends up built from the tuned tables. Loading also
restores the **pass period** each set was tuned at, which is essential:
trains are only valid at the period *and the length* they were calibrated
against, and `setQuality()` re-picks the matching set every time you
change quality.

Blobs are board-keyed (pins + geometry) and CRC-guarded; on any mismatch
they are refused and the preset tables stay in charge, so it is safe to
call unconditionally on any board. Blobs written by the pre-split version
of this sketch are still read, as the NORMAL set.

**Scope:** a board may carry any subset. An untuned quality keeps its
preset tables — or, if the preset has none for it, the formula library,
which renders sixteen monotonic greys but not accurate ones.

## Requirements

- **Adafruit GFX Library**, **JPEGDEC** (image analysis — required for
  tuning). No touch library: the Inkplate 10 has no touch controller.
- A scanner speaking plain-HTTP eSCL (`GET /eSCL/ScannerCapabilities`).
  Nearly every network scanner/MFP since ~2015 does; older USB scanners
  can be bridged with `sane-airscan`.

## Serial commands (115200)

| Key | Action |
|-----|--------|
| `s` | demo scan (the DEMO SCAN button) |
| `t` | self-tune with the 30 s lid countdown (the TUNE ME button) |
| `T` | self-tune: offers the quality choice, then starts (no countdown) |
| `o` | show the photo through each tier (the SHOW PHOTO button; `m` cycles the mode) |
| `f` / `n` / `h` | at that choice: FAST / NORMAL / HIGH |
| `q` | back out of the choice, or abort a running tune (between cycles) |
| `s` / `k` | on the results screen: SAVE / SKIP |
| any key | dismiss an OK / "scanner not responding" screen (`q` aborts the latter) |
| `P <us>` | override the tuning pass period (0 = preset) |
| `L` | which blobs are present, and the 16-grey periods and lengths |
| `X` | erase **all** tuned blobs (reboot to fall back to presets) |
| `n` | re-pick the scanner (from the menu; the CHANGE SCANNER button) |
| `m` | reprint the menu |
| `r` | reboot |

**Don't commit the sketch with your WiFi password in it.**
