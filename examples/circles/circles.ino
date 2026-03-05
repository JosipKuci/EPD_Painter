// Choose your board.
#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS
//#define EPD_PAINTER_PRESET_M5PAPER_S3


#include <Arduino.h>
#include "EPD_Painter_Adafruit.h"
#include "EPD_Painter_presets.h"


EPD_PainterAdafruit epd(EPD_PAINTER_PRESET);

void setup() {
if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1);
  }

 epd.clear();
 epd.fillScreen(1);


}

void loop() {
 for (int y=0; y<500; y+=100){
  epd.fillRect(0, y, 960, 50, 0);

 }
 epd.paint();
 delay(3000);
  for (int y=0; y<500; y+=100){
  epd.fillRect(0, y, 960, 50, 1);
 }
   epd.paint();
  delay(3000);
}
