// Inkplate 10 screen mirror over USB 2.0 High-Speed (480 Mbit/s), ESP32-S31.
//
// Same stream, wire format and host script as inkplate10_screen_stream
// (see EPD_ScreenStream.h), on the S31's USB 2.0 OTG High-Speed port instead
// of a UART. The port runs as a USB *device* here: the computer is the host,
// and two hosts cannot talk to each other. The OTG controller does either
// role; the datasheet: "it can be configured as either a Host or a Device".
//
// The link is a vendor-specific interface with one bulk OUT and one bulk IN
// endpoint (512-byte packets), driven by a small TinyUSB class driver below.
// The core's USBCDC moves every byte through a FreeRTOS queue, which would
// cap the link near UART speeds; here each OUT transfer is a 16 KB DMA into
// internal RAM, copied in one memcpy into a stream buffer the receiver reads.
// No driver is needed on the computer: libusb opens vendor interfaces as is.
//
// !! Hardware: the board's high-speed port is wired as a HOST port (USB-A,
// !! "up to 500 mA output current"): it drives 5 V onto VBUS. A computer
// !! port drives 5 V too. Joining the two with an A-to-A cable shorts two
// !! supplies together and can damage the computer port or the board.
// !! Use a cable or adapter with VBUS (pin 1, red) cut or taped over, so only
// !! D+, D- and GND connect, or switch the board's VBUS supply off if it has
// !! a jumper or enable for it. The S31 does not need VBUS in device mode.
//
// Logs: Serial stays on UART0 (the CP2102, 115200) for anything printed
// outside the stream; the stream's own [stream] lines go over USB to the
// script, as on the serial version.
//
// Build (TinyUSB on the OTG port, Serial on UART0):
//   arduino-cli compile --fqbn esp32:esp32:esp32s31:USBMode=default,CDCOnBoot=default
// Then on the computer:
//   brew install libusb            (macOS; Linux: apt install libusb-1.0-0)
//   pip install numpy pillow pyusb mss
//   python3 ../inkplate10_screen_stream/screen_stream.py --usb
// Linux needs a udev rule (or root) to open the device; macOS does not.

#include <EPD_Painter.h>
#include <EPD_ScreenStream.h>

#if !CONFIG_IDF_TARGET_ESP32S31 || ARDUINO_USB_MODE
#error "Build for the ESP32-S31 with USBMode=default (USB-OTG / TinyUSB)"
#endif

#include "USB.h"
#include "esp32-hal-tinyusb.h"
#include "device/usbd_pvt.h"
#include "freertos/stream_buffer.h"

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

// ---------------------------------------------------------------------------
// USB link: a vendor interface the script finds by class/subclass/protocol.
// ---------------------------------------------------------------------------
static constexpr uint8_t  ITF_SUBCLASS = 0x45, ITF_PROTOCOL = 0x50;   // "EP"
static constexpr uint16_t BULK_MPS     = 512;          // high-speed bulk
static constexpr size_t   RX_CHUNK     = 16 * 1024;    // one OUT transfer
static constexpr size_t   RX_BUF       = 96 * 1024;    // receiver backlog
static constexpr size_t   TX_BUF       = 4 * 1024;
// IN transfers stay below one packet so each ends in a short packet and the
// host's read returns at once, without waiting for a zero-length packet.
static constexpr size_t   TX_CHUNK     = BULK_MPS - 1;

static uint8_t s_rhport, s_epOut, s_epIn;
static volatile bool s_open = false, s_rxPaused = false, s_txBusy = false;
static uint8_t *s_rxDma, *s_txDma;               // internal, DMA capable
static StreamBufferHandle_t s_rx, s_tx;

// All endpoint arming runs in the TinyUSB task (from its callbacks, or via
// usbd_defer_func), so there is never a second caller racing it.
static void armOut(void *) {
  if (!s_open || usbd_edpt_busy(s_rhport, s_epOut)) return;
  if (xStreamBufferSpacesAvailable(s_rx) < RX_CHUNK) {
    s_rxPaused = true;               // the receiver re-arms once it has read
    return;
  }
  s_rxPaused = false;
  usbd_edpt_xfer(s_rhport, s_epOut, s_rxDma, RX_CHUNK, false);
}

static void kickIn(void *) {
  if (!s_open || s_txBusy) return;
  const size_t n = xStreamBufferReceive(s_tx, s_txDma, TX_CHUNK, 0);
  if (!n) return;
  s_txBusy = true;
  if (!usbd_edpt_xfer(s_rhport, s_epIn, s_txDma, n, false)) s_txBusy = false;
}

static void drvInit() {}
static bool drvDeinit() { return true; }

static void drvReset(uint8_t) {
  s_open = false;
  s_txBusy = false;
  s_rxPaused = false;
  xStreamBufferReset(s_rx);   // a half-received frame is resynced on "EPDF"
}

static uint16_t drvOpen(uint8_t rhport, tusb_desc_interface_t const *d, uint16_t max_len) {
  const uint16_t len = sizeof(tusb_desc_interface_t) + 2 * sizeof(tusb_desc_endpoint_t);
  if (d->bInterfaceClass != TUSB_CLASS_VENDOR_SPECIFIC || d->bInterfaceSubClass != ITF_SUBCLASS ||
      d->bInterfaceProtocol != ITF_PROTOCOL || max_len < len)
    return 0;
  if (!usbd_open_edpt_pair(rhport, tu_desc_next(d), 2, TUSB_XFER_BULK, &s_epOut, &s_epIn))
    return 0;
  s_rhport = rhport;
  s_open = true;
  armOut(nullptr);
  kickIn(nullptr);
  return len;
}

static bool drvControl(uint8_t, uint8_t, tusb_control_request_t const *) {
  return false;                      // no class requests: stall them
}

static bool drvXfer(uint8_t, uint8_t ep, xfer_result_t result, uint32_t n) {
  if (ep == s_epOut) {
    if (result == XFER_RESULT_SUCCESS && n)
      xStreamBufferSend(s_rx, s_rxDma, n, 0);   // space was checked when armed
    armOut(nullptr);
  } else if (ep == s_epIn) {
    s_txBusy = false;
    kickIn(nullptr);
  }
  return true;
}

static const usbd_class_driver_t s_driver = {
  .name            = "EPDSTREAM",
  .init            = drvInit,
  .deinit          = drvDeinit,
  .reset           = drvReset,
  .open            = drvOpen,
  .control_xfer_cb = drvControl,
  .xfer_cb         = drvXfer,
  .xfer_isr        = nullptr,
  .sof             = nullptr,
};

// TinyUSB asks the application for extra class drivers; they are tried
// before its own, so this one claims the interface below.
extern "C" const usbd_class_driver_t *usbd_app_driver_get_cb(uint8_t *count) {
  *count = 1;
  return &s_driver;
}

// A high-speed device must answer GET_DESCRIPTOR(DEVICE_QUALIFIER); the
// core leaves it to TinyUSB's default, which stalls. Same class codes as the
// core's device descriptor (IAD composite).
extern "C" const uint8_t *tud_descriptor_device_qualifier_cb(void) {
  static const tusb_desc_device_qualifier_t q = {
    .bLength            = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType    = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 1,
    .bReserved          = 0,
  };
  return (uint8_t const *)&q;
}

// The interface's descriptor, appended to the core's configuration.
static uint16_t loadDescriptor(uint8_t *dst, uint8_t *itf) {
  const uint8_t str = tinyusb_add_string_descriptor("Inkplate screen stream");
  const uint8_t ep = tinyusb_get_free_duplex_endpoint();
  const uint8_t d[] = {
    9, TUSB_DESC_INTERFACE, *itf, 0, 2, TUSB_CLASS_VENDOR_SPECIFIC, ITF_SUBCLASS, ITF_PROTOCOL, str,
    7, TUSB_DESC_ENDPOINT, ep, TUSB_XFER_BULK, U16_TO_U8S_LE(BULK_MPS), 0,
    7, TUSB_DESC_ENDPOINT, (uint8_t)(0x80 | ep), TUSB_XFER_BULK, U16_TO_U8S_LE(BULK_MPS), 0,
  };
  *itf += 1;
  memcpy(dst, d, sizeof d);
  return sizeof d;
}

// The link as an Arduino Stream, for EPD_ScreenStream.
class UsbHsLink : public Stream {
public:
  bool begin() {
    s_rx = xStreamBufferCreate(RX_BUF, 1);
    s_tx = xStreamBufferCreate(TX_BUF, 1);
    s_rxDma = (uint8_t *)heap_caps_aligned_alloc(64, RX_CHUNK, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_txDma = (uint8_t *)heap_caps_aligned_alloc(64, BULK_MPS, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!s_rx || !s_tx || !s_rxDma || !s_txDma) return false;
    if (tinyusb_enable_interface(USB_INTERFACE_CUSTOM, 9 + 7 + 7, loadDescriptor) != ESP_OK)
      return false;
    USB.productName("Inkplate screen stream");
    USB.manufacturerName("Soldered");
    return USB.begin();
  }

  int available() override {
    resume();
    if (_peek >= 0) return 1 + (int)xStreamBufferBytesAvailable(s_rx);
    return (int)xStreamBufferBytesAvailable(s_rx);
  }
  int read() override {
    if (_peek >= 0) { const int c = _peek; _peek = -1; return c; }
    uint8_t c;
    const bool got = xStreamBufferReceive(s_rx, &c, 1, 0) == 1;
    resume();
    return got ? c : -1;
  }
  int peek() override {
    if (_peek < 0) {
      uint8_t c;
      if (xStreamBufferReceive(s_rx, &c, 1, 0) == 1) _peek = c;
    }
    return _peek;
  }
  size_t readBytes(uint8_t *buf, size_t n) override {
    size_t got = 0;
    if (n && _peek >= 0) { buf[got++] = (uint8_t)_peek; _peek = -1; }
    got += xStreamBufferReceive(s_rx, buf + got, n - got, 0);
    resume();
    return got;
  }
  size_t readBytes(char *buf, size_t n) override { return readBytes((uint8_t *)buf, n); }

  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t *p, size_t n) override {
    // Nobody reading (no host, or the script not running): drop rather than
    // stall the stream; the script resyncs on READY and ACK/NAK.
    if (!s_open) return n;
    size_t done = 0;
    while (done < n) {
      const size_t k = xStreamBufferSend(s_tx, p + done, n - done, pdMS_TO_TICKS(20));
      usbd_defer_func(kickIn, nullptr, false);
      if (!k) break;
      done += k;
    }
    return n;
  }
  void flush() override { usbd_defer_func(kickIn, nullptr, false); }

private:
  int _peek = -1;

  // OUT was paused for lack of room: re-arm once a chunk fits again.
  void resume() {
    if (s_rxPaused && s_open && xStreamBufferSpacesAvailable(s_rx) >= RX_CHUNK) {
      s_rxPaused = false;
      usbd_defer_func(armOut, nullptr, false);
    }
  }
};

UsbHsLink usbLink;
EPD_Painter epd(streamConfig());
EPD_ScreenStream stream(epd, usbLink);

void setup() {
  Serial.begin(115200);   // UART0: boot and driver log
  if (!usbLink.begin()) {
    Serial.println("[stream] USB high-speed link failed to start");
    while (true) delay(1000);
  }
  if (!epd.begin()) {
    Serial.println("epd.begin() failed - see the log above");
    while (true) delay(1000);
  }
  epd.setIdleTimeout(30);   // keep the PMIC up between frames
  epd.setPaintProfile(STREAM_PAINT_PROFILE);
  stream.setFullRefreshMs(STREAM_FULL_REFRESH_MS);
  // Give the computer a moment to enumerate, so READY reaches a listener.
  for (int i = 0; i < 30 && !tud_mounted(); i++) delay(100);
  Serial.printf("[stream] USB %s\n", tud_mounted()
                ? (tud_speed_get() == TUSB_SPEED_HIGH ? "mounted, high speed" : "mounted, NOT high speed")
                : "not mounted yet (no host?)");
  if (!stream.begin()) {
    while (true) delay(1000);
  }
  Serial.println("[stream] ready: run screen_stream.py --usb");
}

void loop() {
  stream.poll();
}
