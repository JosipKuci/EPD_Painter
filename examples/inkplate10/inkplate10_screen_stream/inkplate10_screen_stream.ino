// Inkplate 10 screen mirror: shows a computer monitor, streamed over a
// serial link.
//
// The host script (screen_stream.py, next to this sketch) grabs a monitor,
// dithers it to 1 bit per pixel and sends only the rows that changed. The
// receiving end, the wire format and the flow control live in
// EPD_ScreenStream.h; this sketch only picks the link and the panel config.
// For USB 2.0 high speed on the S31, see inkplate10_screen_stream_hs.
//
// Transport: the sketch talks on `Serial`, and the FQBN decides which port
// that is. Pick the one your cable is plugged into:
//   - USB Serial/JTAG (GPIO33/34, shows up as /dev/cu.usbmodem*): USB full
//     speed, ~0.5-1 MB/s. Build with CDCOnBoot=cdc.
//   - USB-UART bridge on UART0 (the CP2102, /dev/cu.usbserial-*): build with
//     CDCOnBoot=default. Runs at STREAM_BAUD; the CP2102N tops out at
//     3 Mbaud, ~290 KB/s.
// The boot ROM's log stays at 115200 and shows as garbage at STREAM_BAUD.
//
// Build:
//   S31, USB:  arduino-cli compile --fqbn esp32:esp32:esp32s31:USBMode=hwcdc,CDCOnBoot=cdc
//   S31, UART: arduino-cli compile --fqbn esp32:esp32:esp32s31:CDCOnBoot=default
//   S3:        arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app,USBMode=hwcdc,CDCOnBoot=cdc
// Then on the computer:
//   pip install numpy pillow pyserial mss
//   python3 screen_stream.py --port /dev/cu.usbmodemXXXX      (USB)
//   python3 screen_stream.py --port /dev/cu.usbserial-10      (UART, 3 Mbaud)

#include <EPD_Painter.h>
#include <EPD_ScreenStream.h>

// UART baud when Serial is UART0 (CDCOnBoot=default); ignored by USB CDC.
// Must match the script's --baud (default 3000000).
#define STREAM_BAUD 3000000

// 1: the driver also prints one [paintprof] line per drive cycle (passes,
// row loops, gaps, closing sweep, cycle period). Off by default: it is a
// line per cycle, and every log byte competes with frames on a UART link.
#define STREAM_PAINT_PROFILE 0

// Full clear + repaint of the whole screen every this many ms (if anything
// changed since the last one), to wipe ghosting. 0 = off.
#define STREAM_FULL_REFRESH_MS 10000

static EPD_Painter::Config streamConfig() {
  EPD_Painter::Config c = EPD_PAINTER_PRESET;
  c.pclk_hz        = 10000000;
  c.skip_idle_rows = true;
  c.skip_idle_cols = false;   // true leaves columns past the cut undriven
  c.min_pass_us    = 4000;
  c.quality        = EPD_Painter::Quality::QUALITY_BW;
  c.bw_passes      = 4;       // raise if blacks look grey, lower for speed
  return c;
}

EPD_Painter epd(streamConfig());
EPD_ScreenStream stream(epd, Serial);

void setup() {
  // Room for a whole raw frame (1200x825 / 8 + header): the host may send
  // one while the board is busy with a full refresh, and a UART cannot
  // make it wait the way USB does.
  Serial.setRxBufferSize(128 * 1024);
  Serial.begin(STREAM_BAUD);
  delay(500);
  // printf goes to the console UART directly: readable at 115200 if the
  // rate change failed, at STREAM_BAUD if it worked.
  printf("[stream] Serial at %lu baud (asked %lu)\n",
         (unsigned long)Serial.baudRate(), (unsigned long)STREAM_BAUD);

  if (!epd.begin()) {
    Serial.println("epd.begin() failed - see the log above");
    while (true) delay(1000);
  }
  epd.setIdleTimeout(30);   // keep the PMIC up between frames
  epd.setPaintProfile(STREAM_PAINT_PROFILE);
  stream.setFullRefreshMs(STREAM_FULL_REFRESH_MS);
  if (!stream.begin()) {
    while (true) delay(1000);
  }
}

void loop() {
  stream.poll();
}
