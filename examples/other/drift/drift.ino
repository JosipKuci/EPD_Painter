// =============================================================================
//  Drift  —  Julia-set morph demo for EPD_Painter / ESP32-S3
//
//  The Mandelbrot set is the complete atlas of Julia sets.  Each point c in
//  the complex plane defines a unique Julia set J_c — the set of starting
//  values z₀ for which z_{n+1} = z² + c stays bounded.  Points on the
//  boundary of the Mandelbrot set give the most intricate J_c.
//
//  This demo slowly drifts c around that boundary, producing a continuous
//  procession of fractal forms: seahorse spirals → dendrite lightning →
//  rabbit ears → cauliflower whorls → and back.  Each frame is a complete
//  artwork, printed fresh onto the e-paper panel.
//
//  Technique:
//    · Smooth escape-time colouring (Böttcher / Milnor log-log formula)
//      gives a continuous float luminance rather than hard iteration bands.
//    · Bayer 4×4 ordered dithering quantises that float to the panel's
//      4 grey levels while preserving the illusion of a full tonal range.
//    · Dual-core rendering splits the frame between ESP32-S3 cores 0 and 1.
//
//  Render target : 240×165  (upscaled 5× to the 1200×825 panel; the render
//                  size is the panel size / SCALE, computed at runtime)
//  Render time   : ~1.5–2.5 s on ESP32-S3 @ 240 MHz
// =============================================================================

#include "EPD_Painter_presets.h"
#include "esp_heap_caps.h"

static EPD_Painter display(EPD_PAINTER_PRESET);

static const int SCALE = 5;             // render pixel -> SCALE×SCALE panel pixels
static int DW, DH;                       // panel size, from the preset
static int RW, RH;                       // render size = DW/SCALE × DH/SCALE

static uint8_t *render_buf  = nullptr;   // RW*RH  — internal SRAM
static uint8_t *display_buf = nullptr;   // DW*DH  — PSRAM

// =============================================================================
// Smooth Julia-set iteration
//
// z_{n+1} = z² + c,  starting from z₀ = (zx, zy).
//
// Returns a normalised float in [0, 1]:
//   0.0  — escaped at the first iteration  (far outside → white)
//   1.0  — never escaped                   (inside the set → black)
//
// Smooth colouring via the Böttcher potential / Milnor formula eliminates
// the hard iteration-count bands:
//   μ = iter + 1 − log₂(log₂|z_iter|)
//
// Escape radius² = 256 (|z| ≥ 16) ensures log₂(log₂|z|) > 0 at escape.
// =============================================================================

static const int   MAX_ITER  = 128;
static const float BAND_SIZE = 8.f;   // escape iterations per tonal band

// Returns a float for dithering:
//   [0, 1)  — cyclic band value for escaped pixels (repeating dark→light rings)
//   1.0     — inside the Julia set (always rendered solid black)
//
// Cyclic banding wraps the smooth escape time modulo BAND_SIZE, creating
// repeating tonal rings across the whole escape region.  This fills the screen
// with structure even when c is outside the Mandelbrot set (Julia set = dust),
// rather than fading to a single near-white tone.
static float julia_banded(float zx, float zy, float cx, float cy) {
    float zx2 = zx*zx, zy2 = zy*zy;
    for (int i = 0; i < MAX_ITER; i++) {
        if (zx2 + zy2 > 256.f) {
            float log2_r = logf(zx2 + zy2) * (0.5f * 1.4426950f); // log₂(|z|)
            float nu     = logf(log2_r)     * 1.4426950f;          // log₂(log₂|z|)
            float smooth = (float)(i + 1) - nu;
            return fmodf(smooth, BAND_SIZE) / BAND_SIZE;           // [0, 1)
        }
        float nx = zx2 - zy2 + cx;
        zy  = 2.f * zx * zy + cy;
        zx  = nx;
        zx2 = zx * zx;
        zy2 = zy * zy;
    }
    return 1.f;   // inside the Julia set
}

// =============================================================================
// Bayer 4×4 ordered dithering
//
// The 4×4 matrix provides 16 distinct spatial thresholds.  Adding one before
// quantising a float level in [0, 3] creates up to 4×16+1 = 65 apparent grey
// steps from the display's 4 physical grey levels — the same technique used
// in classic halftone printing.
//
// Buffer format: 0 = white, 3 = black  (EPD native format).
// =============================================================================

static const uint8_t bayer4[4][4] = {
    { 0,  8,  2, 10 },
    {12,  4, 14,  6 },
    { 3, 11,  1,  9 },
    {15,  7, 13,  5 }
};

static inline uint8_t dither(float v, int x, int y) {
    float threshold = (bayer4[y & 3][x & 3] + 0.5f) * (1.f / 16.f);
    int   q         = (int)(v * 3.f + threshold);
    return (uint8_t)(q < 0 ? 0 : q > 3 ? 3 : q);
}

// =============================================================================
// Parameter path — c slowly orbits the Mandelbrot boundary
//
// Radius 0.7885 ≈ √0.622 passes through the cardioid boundary at many angles,
// giving the richest variety of Julia set morphologies per orbit:
//
//   θ ≈ 0         c ≈ (+0.79,  0.00)   spiralling islands
//   θ ≈ π/3       c ≈ (+0.39, +0.68)   tangled filaments
//   θ ≈ π/2       c ≈ ( 0.00, +0.79)   cauliflower / Siegel disc
//   θ ≈ 2π/3      c ≈ (−0.39, +0.68)   rabbit fractal
//   θ ≈ π         c ≈ (−0.79,  0.00)   seahorse valley, dendrites
//   θ ≈ 4π/3      c ≈ (−0.39, −0.68)   lightning bolt
//
// A slow radial wobble (±0.08) drifts in and out of the set boundary,
// toggling between connected filament Julia sets and fractal dust.
//
// One full orbit at the default time step takes ~1047 frames ≈ ~26 minutes.
// =============================================================================

static void julia_c(float t, float& cx, float& cy) {
    float a = t * 0.006f + 3.14159265f;        // start near seahorse valley
    float r = 0.7885f + 0.08f * sinf(t * 0.017f);
    cx = r * cosf(a);
    cy = r * sinf(a);
}

// =============================================================================
// Dual-core rendering
//
// The helper task (pinned to core 0) renders the bottom half of the frame
// while the main loop (core 1) renders the top half concurrently.
// =============================================================================

struct DriftJob {
    float    cx, cy;
    int      row0, row1;
    uint8_t *buf;
    SemaphoreHandle_t start_sem;
    SemaphoreHandle_t done_sem;
};

static DriftJob     g_job;
static TaskHandle_t g_helper = nullptr;

static void render_rows(float cx, float cy, int row0, int row1, uint8_t *buf) {
    // View window: centred at origin, width 3.6 units, square pixels (the
    // height follows the panel's aspect ratio).
    const float xs     = 3.6f   / RW;
    const float ys     = xs;
    const float x_orig = -1.8f            + xs * 0.5f;
    const float y_orig = -0.5f * ys * RH  + ys * 0.5f;

    for (int y = row0; y < row1; y++) {
        float zy = y_orig + y * ys;
        for (int x = 0; x < RW; x++) {
            float zx = x_orig + x * xs;
            float v  = julia_banded(zx, zy, cx, cy);
            // v >= 1.0 means inside the set: force solid black, no dithering.
            buf[y * RW + x] = (v >= 1.f) ? 3 : dither(v, x, y);
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // keeps WDT fed on both cores
    }
}

static void helper_task(void*) {
    for (;;) {
        xSemaphoreTake(g_job.start_sem, portMAX_DELAY);
        render_rows(g_job.cx, g_job.cy, g_job.row0, g_job.row1, g_job.buf);
        xSemaphoreGive(g_job.done_sem);
    }
}

// =============================================================================
// Upscale render_buf (RW×RH) → display_buf (DW×DH)
//
// Each source byte is replicated SCALE× horizontally; each source row SCALE×
// vertically. Any remainder columns/rows (DW, DH not multiples of SCALE)
// stay white from the initial memset.
// =============================================================================

static void upscale() {
    for (int sy = 0; sy < RH; sy++) {
        const uint8_t *src = render_buf + sy * RW;
        for (int rep = 0; rep < SCALE; rep++) {
            uint8_t *dst = display_buf + (size_t)(sy*SCALE + rep) * DW;
            for (int x = 0; x < RW; x++) {
                memset(dst, src[x], SCALE);
                dst += SCALE;
            }
        }
    }
}

// =============================================================================
// Arduino entry points
// =============================================================================

static float g_time = 0.f;

void setup() {
    Serial.begin(115200);
    Serial.println("[Drift] Starting...");

    DW = display.getConfig().width;
    DH = display.getConfig().height;
    RW = DW / SCALE;
    RH = DH / SCALE;
    render_buf  = (uint8_t*)heap_caps_malloc(RW * RH,           MALLOC_CAP_INTERNAL);
    display_buf = (uint8_t*)heap_caps_malloc((size_t)DW * DH,   MALLOC_CAP_SPIRAM);

    if (!render_buf || !display_buf) {
        Serial.println("[Drift] FATAL: allocation failed");
        while (1) delay(1000);
    }
    memset(display_buf, 0, (size_t)DW * DH);

    if (!display.begin()) {
        Serial.println("[Drift] FATAL: display init failed");
        while (1) delay(1000);
    }
    display.setQuality(EPD_Painter::Quality::QUALITY_HIGH);
    display.clear();

    g_job.start_sem = xSemaphoreCreateBinary();
    g_job.done_sem  = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(helper_task, "drift", 4096, nullptr,
                            5, &g_helper, 0);

    Serial.println("[Drift] Ready.");
}

void loop() {
    float cx, cy;
    julia_c(g_time, cx, cy);

    Serial.printf("[Drift] t=%.1f  c=(%+.5f, %+.5f)\n", g_time, cx, cy);
    uint32_t t0 = millis();

    // Core 0: bottom half of frame
    g_job.cx   = cx;  g_job.cy   = cy;
    g_job.row0 = RH/2;  g_job.row1 = RH;
    g_job.buf  = render_buf;
    xSemaphoreGive(g_job.start_sem);

    // Core 1: top half concurrently
    render_rows(cx, cy, 0, RH/2, render_buf);

    xSemaphoreTake(g_job.done_sem, portMAX_DELAY);

    uint32_t render_ms = millis() - t0;
    Serial.printf("[Drift] Render: %u ms  (%.0f px/s) — painting...\n",
                  render_ms, (float)(RW * RH) * 1000.f / render_ms);

    upscale();
    display.paint(display_buf);

    g_time += 1.5f;
}
