/**
 * lv_conf.h
 * Configuration for LVGL v9 on ESP32 with eInk display (8bpp, 960x540)
 * Place this file in: Arduino/libraries/lv_conf.h
 */

#if 1   /* Set to 1 to enable — do not change */

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS
 *====================*/

/** Color depth: 8 = RGB332, matches our 8bpp eInk framebuffer */
#define LV_COLOR_DEPTH 8

/*=========================
   MEMORY SETTINGS
 *=========================*/

/** Size of the memory available for `lv_malloc()` in bytes (48KB) */
#define LV_MEM_SIZE (48 * 1024U)

/** Use the standard `memcpy` and `memset` */
#define LV_MEMCPY_MEMSET_STD 1

/*====================
   HAL SETTINGS
 *====================*/

/** Default display refresh period in ms — keep high for eInk */
#define LV_DEF_REFR_PERIOD 500

/** Input device read period in ms */
#define LV_INDEV_DEF_READ_PERIOD 100

/*=======================
   RENDERING SETTINGS
 *=======================*/

/** Use the simple (faster) software renderer — sufficient for eInk */
#define LV_USE_DRAW_SW 1

/** Number of rows to use in draw buffer when not using full-screen mode */
#define LV_DRAW_SW_SHADOW_CACHE_SIZE 0

/*=====================
   LOGGING
 *=====================*/

/** Enable LVGL log — useful during development */
#define LV_USE_LOG 1

#if LV_USE_LOG
    /** Log level: TRACE / INFO / WARN / ERROR / USER / NONE */
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN

    /** Print logs via `printf` — Arduino overrides this to Serial */
    #define LV_LOG_PRINTF 1
#endif

/*=================
   ASSERT
 *=================*/

/** Halt on assert failure — helpful during development */
#define LV_USE_ASSERT_NULL          1
#define LV_USE_ASSERT_MALLOC        1
#define LV_USE_ASSERT_STYLE         0
#define LV_USE_ASSERT_MEM_INTEGRITY 0
#define LV_USE_ASSERT_OBJ           0

/*================
   FONTS
 *================*/

/** Built-in fonts — enable what you need */
#define LV_FONT_MONTSERRAT_8   0
#define LV_FONT_MONTSERRAT_10  0
#define LV_FONT_MONTSERRAT_12  1
#define LV_FONT_MONTSERRAT_14  1
#define LV_FONT_MONTSERRAT_16  1
#define LV_FONT_MONTSERRAT_18  0
#define LV_FONT_MONTSERRAT_20  0
#define LV_FONT_MONTSERRAT_22  0
#define LV_FONT_MONTSERRAT_24  1
#define LV_FONT_MONTSERRAT_26  0
#define LV_FONT_MONTSERRAT_28  0
#define LV_FONT_MONTSERRAT_30  0
#define LV_FONT_MONTSERRAT_32  0
#define LV_FONT_MONTSERRAT_34  0
#define LV_FONT_MONTSERRAT_36  0
#define LV_FONT_MONTSERRAT_38  0
#define LV_FONT_MONTSERRAT_40  0
#define LV_FONT_MONTSERRAT_48  0

/** Default font used by widgets */
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/** Enable generic font handling */
#define LV_USE_FONT_PLACEHOLDER 1

/*=================
   WIDGETS
 *=================*/

#define LV_USE_ANIMIMG     0
#define LV_USE_ARC         1
#define LV_USE_BAR         1
#define LV_USE_BTN         1
#define LV_USE_BTNMATRIX   1
#define LV_USE_CALENDAR    0
#define LV_USE_CANVAS      0
#define LV_USE_CHART       0
#define LV_USE_CHECKBOX    1
#define LV_USE_DROPDOWN    1
#define LV_USE_IMG         1
#define LV_USE_IMGBTN      0
#define LV_USE_KEYBOARD    0
#define LV_USE_LABEL       1
#define LV_USE_LED         0
#define LV_USE_LINE        1
#define LV_USE_LIST        1
#define LV_USE_MENU        0
#define LV_USE_METER       0
#define LV_USE_MSGBOX      0
#define LV_USE_ROLLER      1
#define LV_USE_SCALE       0
#define LV_USE_SLIDER      1
#define LV_USE_SPAN        0
#define LV_USE_SPINBOX     0
#define LV_USE_SPINNER     0
#define LV_USE_SWITCH      1
#define LV_USE_TABLE       0
#define LV_USE_TABVIEW     0
#define LV_USE_TEXTAREA    1
#define LV_USE_TILEVIEW    0
#define LV_USE_WIN         0

/*==================
   THEMES
 *==================*/

/** Simple monochrome theme — well suited for eInk */
#define LV_USE_THEME_DEFAULT  0
#define LV_USE_THEME_SIMPLE   1
#define LV_USE_THEME_MONO     1

/*==================
   LAYOUTS
 *==================*/

#define LV_USE_FLEX  1
#define LV_USE_GRID  0

/*==================
   OTHERS
 *==================*/

#define LV_USE_MONKEY        0
#define LV_USE_GRIDNAV       0
#define LV_USE_FRAGMENT      0
#define LV_USE_IMGFONT       0
#define LV_USE_OBSERVER      0
#define LV_USE_SNAPSHOT      0
#define LV_USE_SYSMON        0
#define LV_USE_PROFILER      0
#define LV_USE_LODEPNG       0
#define LV_USE_LIBPNG        0
#define LV_USE_BMP           0
#define LV_USE_TJPGD         0
#define LV_USE_LIBJPEG_TURBO 0
#define LV_USE_GIF           0
#define LV_USE_QRCODE        0
#define LV_USE_BARCODE       0
#define LV_USE_FREETYPE      0
#define LV_USE_TINY_TTF      0
#define LV_USE_RLOTTIE       0
#define LV_USE_FFMPEG        0

#endif /* LV_CONF_H */
#endif /* if 1 */
