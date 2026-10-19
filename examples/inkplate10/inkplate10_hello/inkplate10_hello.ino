// Inkplate 10 first light.
//
// Brings the panel up on the ESP32-S3 breakout, reports what the PMIC sees,
// then draws an orientation marker, the four grey levels and some text, and
// repaints a counter every few seconds so the delta engine has work to do.
//
// Build:
//   arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app,USBMode=hwcdc,CDCOnBoot=cdc
// PSRAM must be enabled (~2 MB of buffers). USBMode/CDCOnBoot route Serial
// to the native USB port.
//
// What to check on the glass:
//   - the filled triangle sits in the TOP-LEFT corner: orientation matches
//     the Inkplate library (preset rotation ROTATION_180)
//   - the frame touches all four edges: no lost rows or columns
//     (vscan_skip_lines / row padding)
//   - four distinct bars, white to black. Levels 1 and 2 run on untuned
//     seed waveforms, so expect them uneven until the panel is tuned.

#include <EPD_Painter_Adafruit.h>
#include <Fonts/FreeSansBold24pt7b.h>

EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

static void drawStatic() {
  const int W = epd.width(), H = epd.height();
  epd.fillScreen(0);                                  // 0 = white ... 3 = black

  epd.drawRect(0, 0, W, H, 3);                        // edge-to-edge frame
  epd.drawRect(1, 1, W - 2, H - 2, 3);
  epd.fillTriangle(10, 10, 130, 10, 10, 130, 3);      // top-left marker

  const int barW = (W - 200) / 4, barY = 200, barH = 260;
  for (int g = 0; g < 4; g++) {
    epd.fillRect(100 + g * barW, barY, barW, barH, g);
    epd.drawRect(100 + g * barW, barY, barW, barH, 3);
  }

  epd.setFont(&FreeSansBold24pt7b);
  epd.setTextColor(3);
  epd.setCursor(100, 150);
  epd.print("EPD_Painter on Inkplate 10");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!epd.begin()) {
    Serial.println("epd.begin() failed - see the log above");
    while (true) delay(1000);
  }
  Serial.printf("panel %d x %d, temperature %d C, VCOM %d mV\n",
                epd.width(), epd.height(),
                epd.driver().readTemperature(), epd.driver().readVcomMv());

  epd.clear();
  drawStatic();
  epd.paint();
}

void loop() {
  static int n = 0;
  delay(5000);

  epd.fillRect(100, 520, 600, 80, 0);
  epd.setCursor(100, 580);
  epd.printf("refresh #%d", ++n);

  const int64_t t0 = esp_timer_get_time();
  epd.paint();
  while (!epd.driver().paintIdle()) delay(1);
  // S31: bytes clocked to the panel that were not row data (FIFO empty).
  // Should stay 0; always 0 on the S3.
  Serial.printf("refresh %d: %lld ms, LCD invalid reads %lu\n", n,
                (esp_timer_get_time() - t0) / 1000,
                (unsigned long)epd.driver().lcdInvalidReads());
}
