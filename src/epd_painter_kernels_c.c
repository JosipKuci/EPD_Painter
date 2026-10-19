// =============================================================================
// Portable C versions of the pixel kernels in EPD_Painter.S.
//
// EPD_Painter.S is ESP32-S3 (Xtensa PIE) assembly and only builds there.
// Every other target, the ESP32-S31 first, links these instead. Same
// symbols, same arguments, same output bit for bit: the kernels are pure
// per-pixel bit logic, done here on 32-bit words (16 pixels each) instead of
// 128-bit vectors.
//
// Layout reminders (see How_It_Works.md):
//   - packed 2bpp: 4 pixels a byte, first pixel in the top two bits
//   - rows are handled in 16-byte chunks (64 pixels); chunk masks put
//     chunk 0 in bit 31
//   - all buffers are at least 4-byte aligned and a multiple of 16 long
//
// Not tuned yet. The S31 has a 128-bit SIMD unit on one core (RISC-V PIE,
// SOC_CPU_HAS_PIE) that a later pass can use; it is not the S3 instruction
// set, so EPD_Painter.S does not carry over.
// =============================================================================
#if !defined(__XTENSA__)

#include <stdint.h>

#define LO2 0x55555555u   // low bit of every 2-bit pixel
#define HI2 0xAAAAAAAAu   // high bit of every 2-bit pixel

// 8bpp canvas (values 0..3, upper bits ignored) -> packed 2bpp.
// size = input bytes, a multiple of 16.
void epd_painter_compact_pixels(const uint8_t *input, uint8_t *output, uint32_t size) {
  const uint32_t *in = (const uint32_t *)input;
  uint32_t *out = (uint32_t *)output;
  for (uint32_t n = size / 16; n; n--, in += 4) {
    uint32_t r = 0;
    for (int k = 0; k < 4; k++) {
      // Little-endian word: byte 0 is the first pixel of the four.
      const uint32_t v = in[k] & 0x03030303u;
      const uint32_t b = (v << 6 | v >> 4 | v >> 14 | v >> 24) & 0xFFu;
      r |= b << (8 * k);
    }
    *out++ = r;
  }
}

// One row of packed pixels -> ink drive codes for one pass. waveform[0..2]
// are the drive bytes (code * 0x55) for pixel values 3, 2 and 1; value 0
// drives nothing. Only chunks flagged in chunk_flags are written.
static inline uint32_t epd_convert_word(uint32_t x, uint32_t w3, uint32_t w2, uint32_t w1) {
  const uint32_t lo = x & LO2;
  const uint32_t hi = (x >> 1) & LO2;
  uint32_t m3 = lo & hi, m2 = hi & ~lo, m1 = lo & ~hi;
  m3 |= m3 << 1; m2 |= m2 << 1; m1 |= m1 << 1;
  return (w3 & m3) | (w2 & m2) | (w1 & m1);
}

void epd_painter_convert_packed_fb_to_ink(const uint8_t *packed_fb, uint8_t *output, uint32_t length,
                                          const uint8_t *waveform, uint32_t chunk_flags) {
  const uint32_t w3 = waveform[0] * 0x01010101u;
  const uint32_t w2 = waveform[1] * 0x01010101u;
  const uint32_t w1 = waveform[2] * 0x01010101u;
  const uint32_t *in = (const uint32_t *)packed_fb;
  uint32_t *out = (uint32_t *)output;
  for (uint32_t n = length / 16; n; n--, in += 4, out += 4, chunk_flags <<= 1) {
    if (!(chunk_flags & 0x80000000u)) continue;
    out[0] = epd_convert_word(in[0], w3, w2, w1);
    out[1] = epd_convert_word(in[1], w3, w2, w1);
    out[2] = epd_convert_word(in[2], w3, w2, w1);
    out[3] = epd_convert_word(in[3], w3, w2, w1);
  }
}

// Same, OR-ed into the output (the dark and light planes are pixel-disjoint).
void epd_painter_convert_packed_fb_to_ink_or(const uint8_t *packed_fb, uint8_t *output, uint32_t length,
                                             const uint8_t *waveform, uint32_t chunk_flags) {
  const uint32_t w3 = waveform[0] * 0x01010101u;
  const uint32_t w2 = waveform[1] * 0x01010101u;
  const uint32_t w1 = waveform[2] * 0x01010101u;
  const uint32_t *in = (const uint32_t *)packed_fb;
  uint32_t *out = (uint32_t *)output;
  for (uint32_t n = length / 16; n; n--, in += 4, out += 4, chunk_flags <<= 1) {
    if (!(chunk_flags & 0x80000000u)) continue;
    out[0] |= epd_convert_word(in[0], w3, w2, w1);
    out[1] |= epd_convert_word(in[1], w3, w2, w1);
    out[2] |= epd_convert_word(in[2], w3, w2, w1);
    out[3] |= epd_convert_word(in[3], w3, w2, w1);
  }
}

// Spread a per-pixel "any bit set" across both bits of the pixel.
static inline uint32_t epd_spread(uint32_t x) {
  x |= (x << 1) & HI2;
  x |= (x >> 1) & LO2;
  return x;
}

// Delta detection, both drive directions (see EPD_Painter.S):
//   dark plane  = new values where the screen is white       -> fbD (always written)
//   light plane = screen values of changed non-white pixels  -> fbL (written when non-empty)
//   screen'     = (screen & ~changed) | dark                 -> sb  (written when driven)
// Returns the dark chunk mask; the light one goes to *maskL_out.
uint32_t epd_painter_ink_dual(const uint8_t *packed_paintbuffer, uint8_t *packed_fastbuffer,
                              uint8_t *packed_lightbuffer, uint8_t *packed_screenbuffer,
                              uint32_t length, uint32_t *maskL_out) {
  const uint32_t *nw = (const uint32_t *)packed_paintbuffer;
  uint32_t *fd = (uint32_t *)packed_fastbuffer;
  uint32_t *fl = (uint32_t *)packed_lightbuffer;
  uint32_t *sb = (uint32_t *)packed_screenbuffer;
  uint32_t maskD = 0, maskL = 0, bit = 0x80000000u;

  for (uint32_t n = length / 16; n; n--, nw += 4, fd += 4, fl += 4, sb += 4, bit >>= 1) {
    uint32_t dk[4], lt[4], sc[4], anyD = 0, anyL = 0;
    for (int k = 0; k < 4; k++) {
      const uint32_t s = sb[k], v = nw[k];
      const uint32_t white = ~epd_spread(s);      // both bits set where screen == 0
      const uint32_t changed = epd_spread(v ^ s);
      dk[k] = v & white;
      lt[k] = s & changed;
      sc[k] = (s & ~changed) | dk[k];
      anyD |= dk[k];
      anyL |= lt[k];
    }
    fd[0] = dk[0]; fd[1] = dk[1]; fd[2] = dk[2]; fd[3] = dk[3];
    if (anyD) maskD |= bit;
    if (anyL) {
      maskL |= bit;
      fl[0] = lt[0]; fl[1] = lt[1]; fl[2] = lt[2]; fl[3] = lt[3];
    }
    if (anyD | anyL) {
      sb[0] = sc[0]; sb[1] = sc[1]; sb[2] = sc[2]; sb[3] = sc[3];
    }
  }
  *maskL_out = maskL;
  return maskD;
}

#endif  // !__XTENSA__
