#define EPD_PAINTER_PRESET_LILYGO_T5_S3_GPS

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"



// =============================================================================
// Display driver instance
// =============================================================================
EPD_PainterLVGL display(EPD_PAINTER_PRESET);

// =============================================================================
// LVGL tick source
// =============================================================================
static uint32_t my_tick() {
    return millis();
}

// =============================================================================
// setup()
// =============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("EPD Hello World");

    // 1. Initialise LVGL
    lv_init();
    lv_tick_set_cb(my_tick);

    // 2. Initialise display driver — allocates framebuffer and inits hardware
    if (!display.begin()) {
        Serial.println("ERROR: display.begin() failed");
        while (1) delay(1000);
    }

    Serial.println("Display ready");

    // 3. Create a simple Hello World label, centred on screen
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello World");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_color(label, lv_color_make(0xFF, 0xFF, 0xFF), LV_PART_MAIN);

}

// =============================================================================
// loop()
// =============================================================================
void loop() {
    lv_timer_handler();     // let LVGL do its work — triggers flush when ready
    delay(500);             // eInk doesn't need frequent updates
}
