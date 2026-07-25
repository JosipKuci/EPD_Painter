# Calibration, the tuning rig, and what we learned

Working notes for the self-tuning system: how it works, how to drive it, what
has been measured, and what is still wrong. Written after a long session on
two LilyGo boards so the next person does not rediscover it all.

Read alongside `DECISION_ENGINE.md` (what the trains are) and
`examples/other/tuneup/README.md` (how to run the rig).

---

## 1. The short version

The panel's 16 grey levels are **not evenly spaced**, and their spacing
differs per board. Everything here exists to measure where they actually are
and to move them where they should be.

| Thing | Where it lives |
|---|---|
| Tuner | `examples/other/tuneup/tuner.h` (~2000 lines) |
| Tuned tables | NVS/LittleFS blob, `src/EPD_Painter_tuned.h` |
| Shipped tables | `src/EPD_Painter_trains.h` |
| Scanner bridge | NAS, `escl_bridge.py` — see §7 |

The tuner paints a card, scans it on a flatbed, compares each grey against an
in-frame dithered reference, and edits the drive trains until they match.

---

## 2. Operational gotchas — read this before touching hardware

These cost hours. All are real, all bit us.

**Flash with `CDCOnBoot=cdc` or you get no serial.**
```
esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,USBMode=hwcdc,CDCOnBoot=cdc
```
The FQBN default is `CDCOnBoot=default` (disabled), which routes the sketch's
`Serial` to **UART0 physical pins**, not USB. ROM boot messages still appear
over USB-Serial-JTAG, so you see a boot log and then silence at exactly the
same line every time. Two runs' telemetry were lost to this, and two "fixes"
were written chasing a phantom before the FQBN was checked.

**Opening the serial port RESETS the board.** Attach *before* starting a run,
never during. A speculative reconnect is a reset: an earlier sniffer treated
60 s of silence as a stale endpoint and reopened, which killed a run outright.
Silence means nothing on this hardware; only reconnect if the device node is
genuinely replaced.

**The port name changes.** `usbmodem1301` and `usbmodem11301` have both
appeared for the same board across re-enumerations. Rescan, don't hardcode.

**`!` does not execute in the Claude mobile app.** Commands pasted with a `!`
prefix arrive as plain text. If a shell command matters, either get a
permission rule in place or wait for the desktop. Two printer-config attempts
were believed to have run and had not, which produced a wrong conclusion about
the printer rejecting them.

---

## 3. What the tuner does

```
registration  -> find the panel on the glass (adaptive dark threshold)
geometry      -> anchor ring, exact panel-px to scan-px mapping
dose ladder   -> measure the pure-darken response curve
gap ladder    -> measure the float-gap gain
seed model    -> derive starting shapes from the measured physics
cycles        -> measure card, attribute drift, edit 1-2 levels, repeat
  stall       -> rescue the worst level (whole card, 14 candidates)
removes       -> derive charge-matched removes, verify optically
results       -> SAVE or SKIP
```

**Trains** are per-level drive sequences of 13 passes: `0` float, `1` darken,
`2` whiten. The planner works in a `(d,w,g,r)` grammar — d darkens, w whitens,
g float gap, r re-darkens — which spans most useful shapes but **cannot**
express interior floats in arbitrary positions or alternating `d,w,d,w` tails.
The rescue (§5) works on raw sequences and can.

---

## 4. Measured facts

**Float gaps are worth nothing.** Twelve measurements across two panels:
`-0.53, -0.50, -0.20, -0.40, -0.30, -0.1, -0.2, -0.4, -0.6, ...` — all within
noise of zero, bases never agreeing on sign. The knob is switched off when
measured as nothing, *not floored*: an early version clamped a measured zero up
to 1.0, which made the gap the finest lever in the set and the seeder
sprinkled gaps buying units the panel never delivered.

Physically: the inter-pass settle floor (4 ms of a 15–20 ms period) has already
saturated the relaxation, so an extra period of rest adds nothing. **This is
settled — do not re-investigate.** Untested: FAST 4-level, which has *no*
inter-pass delay and is where the idea might still hold.

**Whiten gain differs per board and must be measured.**

| Board | measured `gw` |
|---|---|
| LilyGo A (orig) | 18.8 – 26.1, mean ~23 (nine `n=2` readings) |
| LilyGo B | 17.5 |
| built-in fallback | **31.0** — wrong on both |

`tuneCalibrateGains()` can only solve `gw` from a level carrying whitens and no
re-darkens. Nothing guaranteed one existed; when none did it silently used
31.0, measured on a different board. The seeder now reserves probes where the
constraint costs least, refusing any costing more than ~a level step.

**Tolerances must scale with the panel.** Full scale ≈ 97 scanner units over 15
intervals, so a level step ≈ 6.5 — on *that* panel, at *that* scan resolution.
All thresholds are now fractions of `tuneStep()`, measured from the panel's own
reference ladder. The registration dark threshold was likewise a hardcoded grey
of 120; a second board whose black rendered lighter put no pixels below it and
reported "no dark panel found" with the board plainly on the glass.

**A gap tolerance must be LOOSER than the placement tolerance.** A gap is the
*difference* of two neighbouring errors, so it runs larger than either and can
reach twice the worst placement error. Gating gaps tighter than placement makes
convergence unreachable by construction — a whole morning of runs ended
`converged=0`, stalling on gap while `maxerr` was already inside tolerance, with
the planner burning cycles chasing a target it could not hit. Now 1.10 steps
against 0.70 for placement.

---

## 5. The rescue

When the planner stalls, hand the worst level the **whole card**: the 16 columns
take their train from their own level index, so they can be borrowed as
experiment slots. Fourteen candidate trains, one scan, keep the best.

Two things make it trustworthy:

**Slot 1 holds the incumbent.** Candidates are judged by how far they sit from
the incumbent *in the same frame*, and that delta applied to the incumbent's
known error. Judging against a reference measured in a *different column*
carried ~13 units of horizontal bias and made a rescue "improve" a level by
adopting its own shape unchanged.

**Candidates are raw drive sequences** — substitute, insert (shifting right),
delete (shifting left) — so the search reaches shapes the grammar cannot
express. A level the rescue owns stores its train literally (`hasRaw`/`raw[13]`)
and the grammar planner leaves it alone.

Consequence handled: `tuneRemoveOf()` used to derive its charge match from
`(d,w,g,r)`. For a raw train that computes a remove for a shape the level no
longer has, and **a DC imbalance is ghosting**. It counts the actual sequence
now (`tuneNetOf`).

Observed working: L9 pinned at −5 for eight cycles, rescue found `(8,2,0,0)`,
converged two cycles later.

---

## 6. What is still wrong

**The converger oscillates.** Levels still move 15–40 units in one cycle. The
risk cap (bounding an edit's *uncertainty*, not just its predicted move) damped
it from 52 but did not cure it. A restructure trading whitens for re-darkens
predicts ~0 because the terms cancel while their errors do not.

**`converged=1` is not a quality gate.** It tests the *applies* only. One run
converged with the tightest applies of the day and `erase 19.1%` — one level's
remove residual at −19 where everything else was under −6. The removes are
derived analytically, verified in three widening rounds, then accepted whatever
they are, and **the attempt score does not see them at all**.

> **This is the highest-value next change**: score attempts on the removes as
> well as the applies. Everything done in this session improved the half that
> was already working.

**Run-to-run variance exceeds most effects being measured.** Two runs of the
*identical* binary produced `+3.3%` and `+22.2%` worst error. Every A/B
comparison of one run against one run is therefore uninformative. Measure the
noise floor — three runs of one binary — before believing any change helped.

**`src/EPD_Painter_trains.h` predates the converger.** Those tables came from
the older Python ladder (`extras/calibration/tune.py`), whose operators only
*flip* symbols in place and can never produce an interior float. Whatever the
tuner learns goes to the blob, not there.

---

## 7. The scanner rig

Canon PIXMA MG5200 — a 2010 device with no eSCL — bridged by a bespoke
`escl_bridge.py` on an Asustor NAS (`ssh admin@AS3302T-1307`,
`/volume1/.@plugins/AppCentral/entware/opt/etc/escl-bridge/`). Published as
`asustor-nas-scanner-bridge` on GitHub.

**Go through the bridge, never around it.** The bridge serialises every
acquisition on a lock. Running `scanimage` directly bypasses that and two
clients can hit the printer's one-client BJNP stack at once. Use an eSCL job
(`scanpanel.py` does this).

**It wedges on scan RATE, not length.** The printer accepts one session and
releases it lazily. Sustained scanning outruns it and the scan/print daemons
crash — HTTP and LPD stay up, BJNP 8610/8612 and 9100 stop listening. **Only a
power cycle fixes it**; three separate remote restarts (including one that
demonstrably restarted the network services) did not touch the scan engine.
Best-of-3 tuning attempts triple the load and caused exactly this, hence the
60 s cooldown between attempts.

**Port checks are not a health test.** Those ports show `connection refused`
even when healthy — BJNP only binds them once a UDP session is negotiated. The
only valid check is attempting a scan.

A persistent SANE handle removes the *second* BJNP conversation per job
(`sane_init` + backend load + discovery, which `scanimage` redid every time):
7.7 s → 3.9 s per scan. It cannot hold the TCP session open — the pixma backend
connects and disconnects *inside* each acquisition, independent of
`sane_open`/`close`.

---

## 8. Image drawing

`drawGray8()` quantises against `Config::level_lum` — what each level
*measurably* renders as, filled in by `EPD_PainterTuned::install()` from the
tuned blob. On a measured LilyGo the gaps between neighbouring levels ran 6 to
32 units against an ideal 17. Diffusion copes fine with uneven levels (it
grains more where gaps are wide); it only *bands* when it is wrong about where
they sit.

Fixing this — using the final card's `ref[]` rather than a `probeRef[]` read
minutes earlier — is what visibly cleaned up photographs. The spacing work in
the converger did *not*; do not credit it.

Everything streams (`GrayDither::begin/row/end`): ~4 KB of error rows whatever
the image size. No decoder dependency in the core; `EPD_Painter_Image.h` is
opt-in behind an explicit `#define` — **not** `__has_include`, because the
Arduino builder only puts a library on the include path when it sees an
unconditional include.

---

## 9. Driving a run remotely

`serialctl.py` (scratchpad) attaches once and takes commands through a file,
because a second port open would reset the board.

| key | at the menu | at results |
|---|---|---|
| `T` | start self-tune | — |
| `s` | demo scan | **save** |
| `k` | — | **skip** |
| `q` | — | abort between cycles |

`s` is *scan* at the menu and *save* at the results screen — an easy and
embarrassing mistake to make.

Results screen: `worst` = placement error as % of full scale, `erase` = worst
remove residual, plus an inversion count. **Compare on all three**; a good
`worst` with a bad `erase` is an unusable table.

Best set so far on LilyGo A: **worst +3.3%, erase 3.2%, 0 inversions.**
Nothing in a long session of changes beat it — see the variance note in §6
before concluding anything from that.
