// LVGL widget demo. The Inkplate 10 has no touch controller, so instead of
// a pointer input device a timer "plays" the widgets: it steps the sliders
// and flips a switch every few seconds, which exercises the same partial
// redraw path a touch would.

#include <Arduino.h>
#include <lvgl.h>
#include "EPD_Painter_presets.h"
#include "EPD_Painter_LVGL.h"

EPD_PainterLVGL display(EPD_PAINTER_PRESET);

static uint32_t my_tick_cb() {
    return millis();
}

static lv_obj_t *slider1_label;
static lv_obj_t *slider2_label;

static void slider1_event_cb(lv_event_t *e) {
    int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(slider1_label, buf);
//    Serial.printf("Slider 1: %d\n", val);
}

static void slider2_event_cb(lv_event_t *e) {
    int val = (int)lv_slider_get_value(lv_event_get_target_obj(e));
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", val);
    lv_label_set_text(slider2_label, buf);
//    Serial.printf("Slider 2: %d\n", val);
}

static void switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target_obj(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);
 //   Serial.printf("Switch %d: %s\n", (int)(intptr_t)lv_event_get_user_data(e), on ? "ON" : "OFF");
}

static void button_event_cb(lv_event_t *e) {
 //   Serial.printf("Button %d pressed\n", (int)(intptr_t)lv_event_get_user_data(e));
}
// Stands in for touch: nudges the widgets every DEMO_STEP_MS.
static const uint32_t DEMO_STEP_MS = 3000;
static lv_obj_t *demo_slider1, *demo_slider2, *demo_switch1, *demo_switch2;

static void demo_timer_cb(lv_timer_t *) {
    static int step = 0;
    step++;
    auto bump = [](lv_obj_t *s, int by) {
        int v = (lv_slider_get_value(s) + by) % 101;
        lv_slider_set_value(s, v, LV_ANIM_OFF);
        lv_obj_send_event(s, LV_EVENT_VALUE_CHANGED, nullptr);
    };
    bump(demo_slider1, 10);
    bump(demo_slider2, 15);
    lv_obj_t *sw = (step & 1) ? demo_switch1 : demo_switch2;
    if (lv_obj_has_state(sw, LV_STATE_CHECKED)) lv_obj_clear_state(sw, LV_STATE_CHECKED);
    else                                         lv_obj_add_state(sw, LV_STATE_CHECKED);
}

void setup() {
    Serial.begin(115200);

    lv_init();
    lv_tick_set_cb(my_tick_cb);

    if (!display.begin()) {
        Serial.println("Display init failed!");
        while (1) delay(1000);
    }


    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();

    lv_obj_set_style_bg_color(lv_screen_active(), EPD_PainterLVGL::WHITE, 0);

    // Title
    lv_obj_t *title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "LVGL Demo");
    lv_obj_set_style_text_color(title, EPD_PainterLVGL::BLACK, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_48, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20);

    // ── Sliders ──────────────────────────────────────────────────────────────
    // Layout is for the 1200x825 Inkplate 10 panel.
    auto make_slider = [](lv_obj_t *parent, int y_ofs, int init_val,
                          lv_event_cb_t cb, lv_obj_t **out_label) -> lv_obj_t * {
        lv_obj_t *s = lv_slider_create(parent);
        lv_obj_set_size(s, 900, 50);
        lv_slider_set_range(s, 0, 100);
        lv_slider_set_value(s, init_val, LV_ANIM_OFF);
        lv_obj_align(s, LV_ALIGN_TOP_MID, -60, y_ofs);
        lv_obj_set_style_bg_color(s, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
        lv_obj_set_style_border_color(s, EPD_PainterLVGL::BLACK, LV_PART_MAIN);
        lv_obj_set_style_border_width(s, 2, LV_PART_MAIN);
        lv_obj_set_style_bg_color(s, EPD_PainterLVGL::DK_GREY, LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(s, EPD_PainterLVGL::BLACK, LV_PART_KNOB);
        lv_obj_add_event_cb(s, cb, LV_EVENT_VALUE_CHANGED, nullptr);

        *out_label = lv_label_create(parent);
        char buf[8];
        lv_snprintf(buf, sizeof(buf), "%d%%", init_val);
        lv_label_set_text(*out_label, buf);
        lv_obj_set_style_text_color(*out_label, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(*out_label, &lv_font_montserrat_48, 0);
        lv_obj_align_to(*out_label, s, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
        return s;
    };

    demo_slider1 = make_slider(lv_screen_active(), 140, 50, slider1_event_cb, &slider1_label);
    demo_slider2 = make_slider(lv_screen_active(), 240, 25, slider2_event_cb, &slider2_label);

    // ── Switches ─────────────────────────────────────────────────────────────
    auto make_switch = [](lv_obj_t *parent, lv_align_t align, int x_ofs, int y_ofs, int id) -> lv_obj_t * {
        lv_obj_t *sw = lv_switch_create(parent);
        lv_obj_set_size(sw, 200, 100);
        lv_obj_align(sw, align, x_ofs, y_ofs);
        lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::LT_GREY, LV_PART_MAIN);
        lv_obj_set_style_border_color(sw, EPD_PainterLVGL::BLACK, LV_PART_MAIN);
        lv_obj_set_style_border_width(sw, 2, LV_PART_MAIN);
        lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::DK_GREY, LV_PART_MAIN | LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(sw, EPD_PainterLVGL::BLACK, LV_PART_KNOB);
        lv_obj_add_event_cb(sw, switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)(intptr_t)id);
        return sw;
    };

    demo_switch1 = make_switch(lv_screen_active(), LV_ALIGN_TOP_LEFT,  120,  400, 1);
    demo_switch2 = make_switch(lv_screen_active(), LV_ALIGN_TOP_RIGHT, -120, 400, 2);

    // ── Buttons ───────────────────────────────────────────────────────────────
    const char *btn_labels[] = { "Button 1", "Button 2", "Button 3" };
    for (int i = 0; i < 3; i++) {
        lv_obj_t *btn = lv_button_create(lv_screen_active());
        lv_obj_set_size(btn, 320, 120);
        lv_obj_align(btn, LV_ALIGN_TOP_MID, (i - 1) * 380, 620);
        lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::LT_GREY, 0);
        lv_obj_set_style_bg_color(btn, EPD_PainterLVGL::DK_GREY, LV_STATE_PRESSED);
        lv_obj_set_style_radius(btn, 8, 0);
        lv_obj_add_event_cb(btn, button_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)(i + 1));

        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, btn_labels[i]);
        lv_obj_set_style_text_color(lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_48, 0);
        lv_obj_set_style_outline_width(lbl, 1, 0);
        lv_obj_set_style_outline_color(lbl, EPD_PainterLVGL::BLACK, 0);
        lv_obj_center(lbl);
    }

    // No touch panel on this board: drive the widgets from a timer instead.
    lv_timer_create(demo_timer_cb, DEMO_STEP_MS, nullptr);
}

void loop() {
    lv_timer_handler();
    delay(5);
}