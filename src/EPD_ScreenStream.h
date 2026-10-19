// EPD_ScreenStream: the receiving end of screen_stream.py, over any Stream.
//
// The examples inkplate10_screen_stream (UART / USB Serial) and
// inkplate10_screen_stream_hs (USB 2.0 high speed) share this. A host script
// grabs a monitor, dithers it to 1 bit per pixel and sends only the rows that
// changed; this class undoes that into a 1bpp copy of the screen, expands the
// changed rows to the driver's 2bpp packed layout and hands them to
// paintPackedRows(): no 8bpp canvas, and only the changed rows are compared
// and scanned.
//
// Flow control: every frame is answered with one ACK byte once the driver
// has taken it, which is when the previous frame is on the glass. The host
// sends the next frame (grabbed meanwhile) only then, so the picture is
// never more than one frame behind and nothing queues up. Every 5 s both
// sides print where each frame's time went; see "[stream]" below.
//
// Wire format (host -> board), little endian:
//   "EPDF"  u8 flags  u8 enc  u16 y0  u16 rows  u32 len  payload[len]  u32 crc32
//   flags bit0 KEY:   forget the old frame (treat it as all white) first
//   flags bit1 CLEAR: full panel clear before painting (wipes ghosting)
//   enc 0 RAW:        payload = rows * ROW1 bytes, XOR against the old rows
//   enc 1 RLE:        the same bytes coded as: c < 0x80 -> c+1 literal bytes
//                     follow; c >= 0x80 -> (c & 0x7F)+1 zero bytes
//   enc 2 ZXOR:       the RAW bytes, raw deflate (no zlib header)
//   enc 3 ZNEW:       the new rows themselves (not XOR-ed), raw deflate.
//                     Wins on scrolling: shifted dither XORs to noise.
//   The host sends whichever is smallest. Inflate is the ROM's miniz; it
//   is tested at boot, and READY ends in " z" only if it passed (without
//   it the host sticks to RAW and RLE).
//   rows are PHYSICAL panel rows, 1 bit per pixel, first pixel in the MSB
//   (the host applies the rotation). crc32 = zlib's, over the payload.
//   Bytes between frames are skipped while looking for the next "EPDF", so
//   a transport may pad a frame (the USB one does, to end a bulk transfer).
// Board -> host: ACK (0x06) per good frame, NAK (0x15) on a bad one (the
// host then sends a KEY frame), preceded by a "[stream] NAK: why" line.
// Anything else on the line is log text.

#ifndef EPD_SCREEN_STREAM_H
#define EPD_SCREEN_STREAM_H

#include <Arduino.h>
#include <miniz.h>   // ROM tinfl (esp_rom), on the S3 and the S31
#include "EPD_Painter.h"

class EPD_ScreenStream {
public:
  static constexpr uint8_t ACK = 0x06, NAK = 0x15;

  EPD_ScreenStream(EPD_Painter &epd, Stream &link) : _epd(epd), _io(link) {}

  // Every `ms`, a full clear (the flashing HARD wipe) and a repaint of the
  // whole frame, to remove the ghosting that partial updates build up. Only
  // when something was painted since the last one: a still screen is left
  // alone. It runs between frames, or with the next frame if one is
  // waiting. 0 = never (the host can still ask with --clear-every).
  void setFullRefreshMs(uint32_t ms) { _fullMs = ms; }

  // After epd.begin() and the link's own begin(): allocates the buffers,
  // tests the ROM inflate, clears the panel and announces READY. False if
  // out of memory.
  bool begin() {
    _W = _epd.getConfig().width;
    _H = _epd.getConfig().height;
    _row1 = _W / 8;
    _row2 = _epd.packedRowBytes();
    const size_t f1 = (size_t)_row1 * _H;
    _frame1   = (uint8_t *)heap_caps_calloc(f1, 1, MALLOC_CAP_SPIRAM);
    _packed2  = (uint8_t *)heap_caps_calloc(_epd.packedSize(), 1, MALLOC_CAP_SPIRAM);
    _payload  = (uint8_t *)heap_caps_malloc(f1 + 16, MALLOC_CAP_SPIRAM);
    _inflated = (uint8_t *)heap_caps_malloc(f1, MALLOC_CAP_SPIRAM);
    // ~11 KB of inflate state: on the heap, never on a task stack.
    _inflator = (tinfl_decompressor *)heap_caps_malloc(sizeof(tinfl_decompressor),
                                                      MALLOC_CAP_INTERNAL);
    _rowChanged = (uint8_t *)heap_caps_calloc(_H, 1, MALLOC_CAP_INTERNAL);
    if (!_frame1 || !_packed2 || !_payload || !_inflated || !_inflator || !_rowChanged) {
      _io.println("[stream] out of memory");
      return false;
    }
    for (int b = 0; b < 256; b++) {
      uint16_t v = 0;
      for (int i = 0; i < 8; i++)
        if (b & (0x80 >> i)) v |= 3u << (14 - 2 * i);   // black = 3
      _expand[b] = (uint16_t)((v >> 8) | (v << 8));
    }
    for (uint32_t i = 0; i < 256; i++) {
      uint32_t c = i;
      for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      _crcTable[i] = c;
    }
    _io.println("[stream] testing ROM inflate...");
    _io.flush();
    _zOk = inflateSelftest();
    _io.printf("[stream] ROM inflate %s\n", _zOk ? "ok" : "FAILED: deflate frames disabled");

    _epd.clear();
    _io.printf("EPDSTREAM READY %d %d%s\n", _W, _H, _zOk ? " z" : "");
    _io.flush();
    _tAck = esp_timer_get_time();
    _t0 = _lastFull = millis();
    return true;
  }

  // Receive and paint one frame (waits for it). Call from loop().
  void poll() {
    waitMagic();
    const int64_t tHdr = esp_timer_get_time();
    uint8_t h[10];
    if (!readExact(h, sizeof h)) { nak("header timeout"); return; }
    const uint8_t flags = h[0], enc = h[1];
    const int y0 = h[2] | h[3] << 8, rows = h[4] | h[5] << 8;
    const uint32_t len = h[6] | h[7] << 8 | (uint32_t)h[8] << 16 | (uint32_t)h[9] << 24;
    if (enc > 3 || rows < 1 || y0 + rows > _H || len > (uint32_t)_row1 * _H + 16) {
      nak("bad header");
      return;
    }
    if (enc >= 2 && !_zOk) {
      nak("deflate frame, but inflate failed its selftest");
      return;
    }
    uint8_t c[4];
    if (!readExact(_payload, len) || !readExact(c, 4)) { nak("payload timeout"); return; }
    const int64_t tRx = esp_timer_get_time();
    const uint32_t crc = c[0] | c[1] << 8 | (uint32_t)c[2] << 16 | (uint32_t)c[3] << 24;
    if (crc32(_payload, len) != crc) { nak("CRC mismatch"); return; }

    if (flags & 1) {          // KEY: the payload is relative to white
      memset(_frame1, 0, (size_t)_row1 * _H);
      memset(_rowChanged, 1, _H);   // every row is new, white ones included
    }
    if (!applyPayload(enc, y0, rows, _payload, len)) {
      nak(enc >= 2 ? "inflate failed" : "bad payload");
      return;
    }
    // Only rows whose bits really changed are expanded and painted: the
    // host sends one band from the first to the last changed row, and on a
    // mostly still screen most of it is unchanged.
    int lo = -1, hi = -1;
    for (int y = y0; y < y0 + rows; y++) {
      if (!_rowChanged[y]) continue;
      _rowChanged[y] = 0;
      expandRow(y);
      if (lo < 0) lo = y;
      hi = y;
    }
    const int64_t tDec = esp_timer_get_time();

    if (lo >= 0) _dirty = true;
    if ((flags & 2) || fullRefreshDue()) {   // CLEAR, or the timer
      fullRefresh();
    } else if (lo >= 0) {
      // Just the changed rows. (The GIF player paints the union with the
      // previous frame's rows so a grey frame's mop-up cycle is not cut
      // short; in black and white that cycle has nothing to do.)
      _epd.paintPackedRows(_packed2, lo, hi);   // waits for the previous frame
    }
    _io.write(ACK);
    _io.flush();
    const int64_t tDone = esp_timer_get_time();

    _sHost   += tHdr - _tAck;
    _sRx     += tRx - tHdr;
    _sDecode += tDec - tRx;
    _sHand   += tDone - tDec;
    _sRows   += rows;
    if (len > _sMaxLen) _sMaxLen = len;
    _tAck = tDone;
    _frames++;
    _bytesIn += len + 18;
    report();
  }

private:
  EPD_Painter &_epd;
  Stream &_io;
  int _W = 0, _H = 0, _row1 = 0, _row2 = 0;   // width, height, 1bpp / 2bpp row bytes
  uint8_t *_frame1 = nullptr;     // the frame on the glass, 1bpp, physical rows
  uint8_t *_packed2 = nullptr;    // the same in the driver's 2bpp layout
  uint8_t *_payload = nullptr;    // one received payload (worst case raw)
  uint8_t *_inflated = nullptr;   // a deflate payload, inflated
  tinfl_decompressor *_inflator = nullptr;
  bool _zOk = false;
  // 8 pixels (1bpp) -> 2 bytes of 2bpp, byte-swapped so a little-endian
  // 16-bit half of a 32-bit store lands them in bus order.
  uint16_t _expand[256];
  uint8_t *_rowChanged = nullptr; // per row: bits changed since it was expanded
  uint32_t _crcTable[256];
  uint32_t _fullMs = 0, _lastFull = 0;
  bool _dirty = false;           // painted since the last full refresh

  bool fullRefreshDue() const {
    return _fullMs && _dirty && millis() - _lastFull >= _fullMs;
  }

  // Wipe the panel, then paint the whole current frame.
  void fullRefresh() {
    while (!_epd.paintIdle()) delay(1);
    _epd.clear();
    _epd.paintPackedRows(_packed2, 0, _H - 1);
    _lastFull = millis();
    _dirty = false;
  }

  // Where each frame's time goes, summed over a 5 s report window (us):
  //   host   ACK sent -> first header byte: the host grabbing, dithering and
  //          coding the next frame (its share of the round trip)
  //   rx     header -> last CRC byte: the link
  //   decode CRC + inflate/XOR/RLE into frame1 + 1bpp -> 2bpp expansion
  //   handoff paintPackedRows() blocked waiting for the paint task to pick
  //          up the previous frame: the panel
  // The four add up to the frame interval, so the biggest one is the limit.
  uint64_t _sHost = 0, _sRx = 0, _sDecode = 0, _sHand = 0;
  uint32_t _sRows = 0, _sMaxLen = 0, _frames = 0, _bytesIn = 0, _t0 = 0;
  int64_t _tAck = 0;

  void report() {
    if (millis() - _t0 < 5000) return;
    const float s = (millis() - _t0) / 1000.0f;
    const float n = _frames ? _frames : 1;
    _io.printf("[stream] %.1f fps, %.1f KB/s in (%.1f KB/frame, max %.1f), %.0f rows/frame | "
               "per frame ms: host %.1f  rx %.1f  decode %.1f  handoff %.1f | "
               "link %.0f KB/s while receiving\n",
               _frames / s, _bytesIn / s / 1024, _bytesIn / n / 1024, _sMaxLen / 1024.0f,
               _sRows / n, _sHost / n / 1000, _sRx / n / 1000, _sDecode / n / 1000,
               _sHand / n / 1000, _sRx ? _bytesIn / (_sRx / 1e6) / 1024 : 0.0);
    _io.flush();
    _frames = _bytesIn = 0;
    _sHost = _sRx = _sDecode = _sHand = 0;
    _sRows = _sMaxLen = 0;
    _t0 = millis();
  }

  void nak(const char *why) {
    _io.printf("[stream] NAK: %s\n", why);
    _io.write(NAK);
    _io.flush();
  }

  uint32_t crc32(const uint8_t *p, size_t n) const {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) c = _crcTable[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return ~c;
  }

  // Exactly n bytes from the link, or false after a 2 s stall.
  bool readExact(uint8_t *dst, size_t n) {
    size_t got = 0;
    uint32_t last = millis();
    while (got < n) {
      const int a = _io.available();
      if (a > 0) {
        got += _io.readBytes(dst + got, min((size_t)a, n - got));
        last = millis();
      } else if (millis() - last > 2000) {
        return false;
      } else {
        delay(0);
      }
    }
    return true;
  }

  // Resynchronise on the "EPDF" magic, byte by byte.
  void waitMagic() {
    static const uint8_t magic[4] = {'E', 'P', 'D', 'F'};
    int m = 0;
    while (m < 4) {
      while (_io.available() <= 0) {
        if (fullRefreshDue()) fullRefresh();   // the screen went still
        delay(1);
      }
      const uint8_t b = _io.read();
      m = (b == magic[m]) ? m + 1 : (b == magic[0] ? 1 : 0);
    }
  }

  // Inflate a raw deflate stream to exactly n bytes of _inflated.
  bool inflateTo(const uint8_t *p, size_t len, size_t n) {
    tinfl_init(_inflator);
    size_t in = len, out = n;
    const tinfl_status st = tinfl_decompress(_inflator, p, &in, _inflated, _inflated, &out,
                                             TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF);
    return st == TINFL_STATUS_DONE && out == n;
  }

  // Boot check of the ROM inflate: a known dynamic-Huffman stream (the
  // selftest text below, deflated by Python's zlib) must come back exactly.
  bool inflateSelftest() {
    static const uint8_t zTest[] = {
      0x85, 0xd3, 0xbb, 0x8d, 0xc3, 0x40, 0x10, 0x04, 0x51, 0x5f, 0x51, 0x30, 0x84, 0x9d, 0xd9, 0xf9,
      0xfa, 0x92, 0xaf, 0x14, 0x68, 0x50, 0x80, 0x00, 0xdd, 0x19, 0x47, 0xe6, 0x8f, 0x53, 0x04, 0x5b,
      0x6d, 0x97, 0xf5, 0x80, 0x7e, 0x3c, 0xef, 0xdb, 0x79, 0xfd, 0x1d, 0xfb, 0xcf, 0xf6, 0xfe, 0x7d,
      0x7d, 0xf6, 0xeb, 0xd8, 0xce, 0xe3, 0xf3, 0xba, 0x8e, 0xf3, 0xda, 0xc6, 0x77, 0xb7, 0xc7, 0x32,
      0x98, 0x09, 0x41, 0xda, 0x3a, 0x10, 0x11, 0x08, 0xac, 0x20, 0x28, 0x5f, 0x07, 0xaa, 0x0a, 0x81,
      0x37, 0x04, 0x1d, 0xeb, 0x60, 0xce, 0x09, 0x41, 0x82, 0xa4, 0x0d, 0x90, 0x34, 0x03, 0x49, 0x2b,
      0x90, 0x74, 0x01, 0x49, 0x77, 0x90, 0xf4, 0x06, 0xc9, 0x50, 0x90, 0x8c, 0x00, 0xc9, 0x1c, 0x20,
      0x99, 0x06, 0x92, 0x99, 0x20, 0x59, 0x02, 0x92, 0xe5, 0x20, 0x59, 0x05, 0x92, 0xad, 0x20, 0xd9,
      0x01, 0x92, 0xdd, 0x6b, 0x49, 0x19, 0x33, 0x20, 0xc8, 0xb5, 0xe4, 0xf7, 0x79, 0x03, 0x02, 0x4b,
      0x08, 0x6a, 0x2d, 0x29, 0xaa, 0x02, 0x81, 0x17, 0x04, 0xbd, 0x96, 0x94, 0x39, 0x15, 0x82, 0x00,
      0x49, 0x1b, 0x20, 0x69, 0x06, 0x92, 0x56, 0x20, 0xe9, 0x02, 0x92, 0xee, 0x20, 0xe9, 0x0d, 0x92,
      0xa1, 0x20, 0x19, 0x01, 0x92, 0x39, 0x40, 0x32, 0x67, 0xdf, 0xfe, 0x01
    };
    static char want[48 * 33 + 1];
    size_t n = 0;
    for (int i = 0; i < 48; i++)
      n += snprintf(want + n, sizeof want - n, "EPD stream inflate selftest %04d\n", i * 37);
    return inflateTo(zTest, sizeof zTest, n) && memcmp(_inflated, want, n) == 0;
  }

  // Apply the payload to frame1 rows y0..y0+rows-1: XOR it in, or for ZNEW
  // copy it over, flagging every row whose bits change in _rowChanged.
  // False if it does not decode to exactly rows * ROW1 bytes.
  bool applyPayload(int enc, int y0, int rows, const uint8_t *p, size_t len) {
    const size_t want = (size_t)rows * _row1;
    if (enc >= 2) {
      if (!inflateTo(p, len, want)) return false;
      p = _inflated;
      len = want;
      enc = enc == 3 ? 3 : 0;
    }
    if (enc == 0 || enc == 3) {
      if (len != want) return false;
      // Row by row in 16-bit words: ROW1 (150) is even, and every row of
      // frame1 and of the payload starts 2-byte aligned.
      const int words = _row1 / 2;
      for (int r = 0; r < rows; r++) {
        uint16_t *o = (uint16_t *)(_frame1 + (size_t)(y0 + r) * _row1);
        const uint16_t *q = (const uint16_t *)(p + (size_t)r * _row1);
        uint16_t diff = 0;
        if (enc == 0) {                     // XOR: changed bits are the payload
          for (int i = 0; i < words; i++) { diff |= q[i]; o[i] ^= q[i]; }
        } else {                            // ZNEW: compare, then copy
          for (int i = 0; i < words; i++) { diff |= o[i] ^ q[i]; o[i] = q[i]; }
        }
        if (diff) _rowChanged[y0 + r] = 1;
      }
      return true;
    }
    uint8_t *o = _frame1 + (size_t)y0 * _row1;
    size_t done = 0;
    const uint8_t *e = p + len;
    while (p < e) {
      const uint8_t c = *p++;
      const size_t n = (c & 0x7F) + 1;
      if (done + n > want) return false;
      if (c & 0x80) {
        done += n;                                      // zero run: unchanged
      } else {
        if (p + n > e) return false;
        for (size_t i = 0; i < n; i++) {
          if (p[i]) {
            o[done + i] ^= p[i];
            _rowChanged[y0 + (done + i) / _row1] = 1;
          }
        }
        p += n;
        done += n;
      }
    }
    return done == want;
  }

  // One frame1 row -> packed2, two input bytes (16 pixels) per 32-bit store.
  // The 2bpp row's padding stays zero.
  void expandRow(int y) {
    const uint8_t *s = _frame1 + (size_t)y * _row1;
    uint32_t *d = (uint32_t *)(_packed2 + (size_t)y * _row2);
    for (int i = 0; i < _row1; i += 2)
      *d++ = _expand[s[i]] | (uint32_t)_expand[s[i + 1]] << 16;
  }
};

#endif
