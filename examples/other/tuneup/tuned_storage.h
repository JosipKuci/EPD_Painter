// ============================================================================
// tuned_storage.h — where THIS sketch keeps its tuned tables.
//
// The library (src/EPD_Painter_tuned.h) only knows how to build, validate
// and install a blob; it deliberately has no idea where the bytes live,
// so that including it never drags a filesystem into a project. This file
// is the other half for the tuneup example: LittleFS, one file per
// quality.
//
// One file per quality, because the three tunes are independent. Tuning
// HIGH must not cost you a good NORMAL set, and a board is perfectly
// entitled to carry NORMAL and nothing else. tunedLoadAll() installs
// whichever files are present and valid; the rest keep their preset
// tables.
//
// Swap this out to keep the tables somewhere else — an SD card, NVS, or
// pulled from a server at boot. Nothing else in the sketch changes.
// ============================================================================

#pragma once

#include <LittleFS.h>
#include "EPD_Painter_tuned.h"

using EPD_PainterTuned::Set;
using EPD_PainterTuned::SET_FAST;
using EPD_PainterTuned::SET_NORMAL;
using EPD_PainterTuned::SET_HIGH;

static const char *tunedPath(Set s) {
  switch (s) {
    case SET_FAST:   return "/epd_tuned_fast.bin";
    case SET_HIGH:   return "/epd_tuned_high.bin";
    default:         return "/epd_tuned_normal.bin";
  }
}

// Written by versions of this sketch that predate the per-quality split.
// Read once at boot as the NORMAL set so an already-tuned board does not
// silently lose its tables on an upgrade.
static const char *TUNED_PATH_LEGACY = "/epd_g16_tuned.bin";

static const char *tunedSetName(Set s) {
  return s == SET_FAST ? "FAST" : s == SET_HIGH ? "HIGH" : "NORMAL";
}

// Read one blob and install it. False (changing nothing) if absent,
// corrupt, or tuned on a different board.
static bool tunedLoadFile(EPD_Painter &p, const char *path) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  // Static, not stack: the blob is ~1.1 KB and these are called from deep
  // inside a tuner that already has hundreds of bytes of arrays live, on an
  // 8 KB task stack. One task, one at a time, so sharing is safe.
  static uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  const size_t n = f.read(buf, sizeof(buf));
  f.close();
  return EPD_PainterTuned::install(p, buf, n);
}

static bool tunedLoad(EPD_Painter &p, Set s) {
  return tunedLoadFile(p, tunedPath(s));
}

// Install every set this board has. Returns how many were installed.
// Legacy first, so a per-quality NORMAL file written later wins.
static int tunedLoadAll(EPD_Painter &p) {
  int n = 0;
  if (tunedLoadFile(p, TUNED_PATH_LEGACY)) n++;
  for (int s = 0; s <= (int)SET_HIGH; s++)
    if (tunedLoad(p, (Set)s)) n++;
  return n;
}

// ---- saving ---------------------------------------------------------------

// Keep one generation of history before overwriting.
//
// A save is destructive and irreversible, and a tune that took half an hour
// to produce a WORSE table will happily replace a good one - which has now
// cost two good tables in a single session, once to a mis-tapped SAVE
// button. The blob is ~1.1 KB and LittleFS has room, so there is no reason
// the previous set should not survive its replacement.
static void tunedBackup(const char *path) {
  if (!LittleFS.exists(path)) return;
  char bak[64];
  snprintf(bak, sizeof(bak), "%s.bak", path);
  LittleFS.remove(bak);                 // rename() will not overwrite
  if (!LittleFS.rename(path, bak))
    printf("[tuned] WARNING: could not back up %s before overwriting\n", path);
}

static bool tunedWrite(const char *path, const uint8_t *buf, size_t len) {
  tunedBackup(path);
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  const size_t n = f.write(buf, len);
  f.close();
  return n == len;
}

// Put the previous set back. Returns false if there is no backup to restore.
static bool tunedRestore(Set s) {
  const char *path = tunedPath(s);
  char bak[64];
  snprintf(bak, sizeof(bak), "%s.bak", path);
  if (!LittleFS.exists(bak)) return false;
  LittleFS.remove(path);
  return LittleFS.rename(bak, path);
}

// 16-grey sets (NORMAL, HIGH). train_len is the quality's pass count and
// period_us the pass period they were tuned at — a train means nothing
// without both.
static bool tunedSave16(EPD_Painter &p, Set s, uint32_t period_us, int train_len,
                        const uint8_t apply[16][EPD_Painter::DEC_WF_LEN16_MAX],
                        const uint8_t remove[16][EPD_Painter::DEC_WF_LEN16_MAX],
                        const uint8_t *level_lum = nullptr) {
  static uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  EPD_PainterTuned::build16(p, s, period_us, train_len, apply, remove,
                            level_lum, buf);
  return tunedWrite(tunedPath(s), buf, sizeof(buf));
}

// The FAST set: 4-level waveforms plus the six grey-to-grey direct trains.
static bool tunedSaveFast(EPD_Painter &p,
                          const uint8_t darker[3][7], const uint8_t lighter[3][7],
                          const uint8_t direct[4][4][EPD_Painter::DEC_WF_LEN_DIR],
                          uint16_t direct_loaded,
                          const uint8_t *level_lum4 = nullptr) {
  static uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  EPD_PainterTuned::buildFast(p, darker, lighter, direct, direct_loaded,
                              level_lum4, buf);
  return tunedWrite(tunedPath(SET_FAST), buf, sizeof(buf));
}

// ---- status ---------------------------------------------------------------

// Does this file hold a valid blob for THIS board and THIS quality?
// Checking the set matters: the three files are the same size and the
// same shape, so a plain validity test would call a HIGH blob sitting in
// the NORMAL slot "present".
static bool tunedPresentPath(EPD_Painter &p, const char *path, Set want) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  static uint8_t buf[EPD_PainterTuned::BLOB_SIZE];
  const size_t n = f.read(buf, sizeof(buf));
  f.close();
  return EPD_PainterTuned::setOf(p, buf, n) == (int)want;
}

static bool tunedPresent(EPD_Painter &p, Set s) {
  if (tunedPresentPath(p, tunedPath(s), s)) return true;
  // A blob written before the per-quality split IS the NORMAL set, and
  // tunedLoadAll() installs it — so it has to report present, or a board
  // that is tuned reads as untuned on the very screen where that matters.
  return s == SET_NORMAL && tunedPresentPath(p, TUNED_PATH_LEGACY, s);
}

static bool tunedErase(Set s) { return LittleFS.remove(tunedPath(s)); }

static void tunedEraseAll() {
  for (int s = 0; s <= (int)SET_HIGH; s++) LittleFS.remove(tunedPath((Set)s));
  LittleFS.remove(TUNED_PATH_LEGACY);
}
