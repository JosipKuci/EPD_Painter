#if defined(__ASSEMBLER__) && defined(__XTENSA__)   // S3 PIE; other targets use epd_painter_kernels_c.c
// =============================================================================
// EPD Painter Assembly Functions
//
// Uses ESP32-S3 PIE (128-bit vector) instructions to process pixels 16 bytes
// at a time. The processor handles all pipeline interlocks automatically;
// instructions are written in logical order.
//
// Vector registers q0–q4 are scratch; q5–q7 hold constants loaded in each
// function's prologue.
// =============================================================================

    .text
    .align 4

// =============================================================================
// epd_painter_compact_pixels
//
// Converts a GFX canvas (1 byte per pixel, values 0–3) to a packed 2bpp
// framebuffer (4 pixels per byte, most-significant pixel first).
//
// Parameters:
//   a2 - Input pointer  (1 byte per pixel)
//   a3 - Output pointer (2bpp packed)
//   a4 - Size in bytes  (multiple of 16)
// =============================================================================
    .global epd_painter_compact_pixels
    .type   epd_painter_compact_pixels, @function

epd_painter_compact_pixels:
    entry   a1, 32

    ee.zero.q       q7              // q7 = zero (used as shift source)

    // Broadcast 2-bit pixel mask 0x03 to all lanes of q5
    movi            a10, 0b00000011
    s32i            a10, a1, 0
    ee.vldbc.8      q5, a1

    // SAR_BYTE=1 for ee.src.q (byte shift), SAR=2 for ee.vsl.32 (2-bit shift)
    movi            a10, 1
    wur.sar_byte    a10
    movi            a10, 2
    wsr.sar         a10

    mov             a11, a4         // loop counter

.compact_process_pixels:
    // Load 16 pixels and strip each to its 2-bit value
    ee.vld.128.ip   q0, a2, 16
    ee.andq         q0, q0, q5

    // Fold 16 bytes into 4 by shifting and OR-ing in three passes.
    // Each pass halves the active byte count and doubles the pixels per byte:
    //   pass 1: 16 bytes, 1 pixel/byte  →  8 bytes, 2 pixels/byte
    //   pass 2: 8 bytes,  2 pixels/byte →  4 bytes, 4 pixels/byte (interleaved)
    //   pass 3: consolidates into the low 4 bytes
    ee.src.q        q1, q0, q7      // q1 = q0 >> 8 bytes
    ee.vsl.32       q0, q0          // q0 <<= 2 bits
    ee.orq          q0, q0, q1

    ee.src.q        q1, q1, q7
    ee.vsl.32       q0, q0
    ee.orq          q0, q0, q1

    ee.src.q        q1, q1, q7
    ee.vsl.32       q0, q0
    ee.orq          q0, q0, q1

    // Deinterleave so the 4 packed result bytes are contiguous
    ee.vunzip.8     q0, q1
    ee.vunzip.8     q0, q1

    // Store the 4 output bytes then advance the output pointer
    ee.stxq.32      q0, q7, a3, 0, 0
    addi            a3, a3, 4

    addi            a11, a11, -16
    bnez            a11, .compact_process_pixels

    retw.n


// =============================================================================
// epd_painter_convert_packed_fb_to_ink
//
// Converts one row of packed 2bpp framebuffer data into EPD ink drive bytes
// for one waveform pass. Only chunks whose bit is set in chunk_flags are
// processed; others have their pointers advanced without writing.
//
// Parameters:
//   a2 - Input pointer  (packed 2bpp framebuffer row)
//   a3 - Output pointer (ink drive data)
//   a4 - Length in bytes (multiple of 16)
//   a5 - Pointer to 3-byte waveform entry (one byte per pixel value 1/2/3)
//   a6 - chunk_flags (bit 31 = first chunk; consumed MSB-first each iteration)
// =============================================================================
    .text
    .align 4
    .global epd_painter_convert_packed_fb_to_ink
    .type   epd_painter_convert_packed_fb_to_ink, @function

epd_painter_convert_packed_fb_to_ink:
    entry       a1, 32

    // SAR=1 for ee.vsr.32 (1-bit right shift)
    movi        a7, 1
    wsr.sar     a7

    // Broadcast 0x55 to q7: selects the lo (even) bit of each pixel pair
    movi        a10, 0b01010101
    s32i        a10, a1, 0
    ee.vldbc.8  q7, a1

    // Load the three waveform bytes into broadcast registers q4, q5, q6
    // (q4 = waveform for pixel value 3, q5 = value 2, q6 = value 1)
    ee.vldbc.8.ip   q4, a5, 1
    ee.vldbc.8.ip   q5, a5, 1
    ee.vldbc.8.ip   q6, a5, 1

.convert_loop:
    // Consume the MSB of chunk_flags to decide whether to process this chunk
    extui       a9, a6, 31, 1
    slli        a6, a6, 1

    beqz        a9, .convert_skip

    // --- Active path: process this 16-byte chunk ---
    ee.vld.128.ip   q0, a2, 16

    // Separate the lo bit (bit0) and hi bit (bit1) of each pixel pair
    ee.andq         q3, q0, q7      // q3 = lo bits
    ee.vsr.32       q2, q0          // shift right to bring bit1 down to bit0 position
    ee.andq         q2, q2, q7      // q2 = hi bits

    // Build a presence mask for each pixel value:
    //   q1 = pixels with value 3 (lo=1 AND hi=1)
    //   q0 = pixels with value 2 (lo=0 AND hi=1)
    //   q2 = pixels with value 1 (lo=1 AND hi=0)
    ee.andq         q1, q3, q2
    ee.notq         q0, q3
    ee.andq         q0, q2, q0
    ee.notq         q2, q2
    ee.andq         q2, q3, q2

    // Expand each mask to cover both bit positions of its pixel pair
    ee.vsl.32       q3, q1
    ee.orq          q3, q3, q1

    ee.vsl.32       q1, q0
    ee.orq          q0, q0, q1

    ee.vsl.32       q1, q2
    ee.orq          q2, q2, q1

    // AND each expanded mask with its waveform byte and OR all three together
    ee.andq         q3, q4, q3
    ee.andq         q0, q5, q0
    ee.orq          q3, q3, q0
    ee.andq         q2, q6, q2
    ee.orq          q3, q3, q2

    ee.vst.128.ip   q3, a3, 16
    j               .convert_next

.convert_skip:
    // Chunk not active: advance both pointers without writing
    addi        a2, a2, 16
    addi        a3, a3, 16

.convert_next:
    addi        a4, a4, -16
    bnez        a4, .convert_loop

    retw.n



// =============================================================================
// epd_painter_convert_packed_fb_to_ink_or
//
// Identical to epd_painter_convert_packed_fb_to_ink but ORs the generated
// ink bytes into the output instead of overwriting. Lets the dark and light
// planes merge into one row transmission: drive codes are per-pixel and the
// planes are pixel-disjoint, so OR composes them losslessly.
// =============================================================================
    .text
    .align 4
    .global epd_painter_convert_packed_fb_to_ink_or
    .type   epd_painter_convert_packed_fb_to_ink_or, @function

epd_painter_convert_packed_fb_to_ink_or:
    entry       a1, 32

    movi        a7, 1
    wsr.sar     a7

    movi        a10, 0b01010101
    s32i        a10, a1, 0
    ee.vldbc.8  q7, a1

    ee.vldbc.8.ip   q4, a5, 1
    ee.vldbc.8.ip   q5, a5, 1
    ee.vldbc.8.ip   q6, a5, 1

.convor_loop:
    extui       a9, a6, 31, 1
    slli        a6, a6, 1

    beqz        a9, .convor_skip

    ee.vld.128.ip   q0, a2, 16

    ee.andq         q3, q0, q7
    ee.vsr.32       q2, q0
    ee.andq         q2, q2, q7

    ee.andq         q1, q3, q2
    ee.notq         q0, q3
    ee.andq         q0, q2, q0
    ee.notq         q2, q2
    ee.andq         q2, q3, q2

    ee.vsl.32       q3, q1
    ee.orq          q3, q3, q1

    ee.vsl.32       q1, q0
    ee.orq          q0, q0, q1

    ee.vsl.32       q1, q2
    ee.orq          q2, q2, q1

    ee.andq         q3, q4, q3
    ee.andq         q0, q5, q0
    ee.orq          q3, q3, q0
    ee.andq         q2, q6, q2
    ee.orq          q3, q3, q2

    // OR into the existing output instead of overwriting
    ee.vld.128.ip   q1, a3, 0
    ee.orq          q3, q3, q1
    ee.vst.128.ip   q3, a3, 16
    j               .convor_next

.convor_skip:
    addi        a2, a2, 16
    addi        a3, a3, 16

.convor_next:
    addi        a4, a4, -16
    bnez        a4, .convor_loop

    retw.n

// =============================================================================
// epd_painter_ink_dual
//
// Delta-detects changed pixels between the new frame and the current screen
// state and emits BOTH drive directions as separate planes:
//
//   dark plane  (fbD, internal) — new pixel values where the screen is white;
//                                 driven with the darker waveform
//   light plane (fbL, PSRAM)    — current screen values of changed non-white
//                                 pixels; driven with the lighter waveform
//
// Unlike the old either/or chunk scheme there is no direction priority and
// no deferral: a chunk containing both darkening and lightening work sets
// its bit in both masks, and the paint task resolves it by latching the row
// twice (once per plane) before advancing the gate — so mixed chunks fully
// update in a single paint cycle.
//
// The two planes are disjoint per pixel by construction: dark data is
// masked to screen==0 pixels, light data is screen&changed which is zero
// wherever the screen is white.
//
// Screenbuffer bookkeeping matches the old routine: darkened pixels take
// their new value, lightened pixels are recorded as white. The chunk is
// written back only when something is driven.
//
// Parameters:
//   a2 - paintbuffer row         (new desired pixel state, read-only)
//   a3 - packed_fastbuffer row   (dark plane out; always written)
//   a4 - packed_lightbuffer row  (light plane out; written when non-empty)
//   a5 - packed_screenbuffer row (current physical screen state)
//   a6 - Length in bytes (multiple of 16)
//   a7 - uint32_t* out: light-plane chunk mask
//
// Returns (a2): dark-plane chunk mask
// =============================================================================
    .global epd_painter_ink_dual
    .type   epd_painter_ink_dual, @function
    .text
    .align 4

epd_painter_ink_dual:
    entry           a1, 32

    // SAR=1 for ee.vsl.32 / ee.vsr.32 (1-bit shifts)
    movi            a10, 1
    wsr.sar         a10

    // q6 = 0x55 broadcast (lo bit mask), q5 = 0xAA broadcast (hi bit mask)
    movi            a10, 0b01010101
    s32i            a10, a1, 0
    ee.vldbc.8      q6, a1
    ee.vsl.32       q5, q6

    movi            a11, 0          // maskD accumulator
    movi            a9,  0          // maskL accumulator

    // a12 = rotating single-bit mask, starts at bit 31
    movi            a12, 31
    ssl             a12
    movi            a12, 1
    sll             a12, a12

    movi            a10, 1
    wsr.sar         a10

.inkdual_loop:
    ee.vld.128.ip   q2, a2, 16      // q2 = new (paintbuffer)
    ee.vld.128.ip   q3, a5, 16      // q3 = current screen

    // q0 = spread(screen) inverted: both bits set wherever screen pixel == 0
    ee.vsl.32       q1, q3
    ee.andq         q1, q1, q5
    ee.orq          q0, q3, q1
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1
    ee.notq         q0, q0

    ee.andq         q4, q2, q0      // q4 = dark plane: new values on white ground

    // q0 = spread(new ^ screen): both bits set for every changed pixel
    ee.xorq         q0, q2, q3
    ee.vsl.32       q1, q0
    ee.andq         q1, q1, q5
    ee.orq          q0, q0, q1
    ee.vsr.32       q1, q0
    ee.andq         q1, q1, q6
    ee.orq          q0, q0, q1

    ee.andq         q1, q3, q0      // q1 = light plane: screen values of changed pixels
                                    //      (zero where screen is white — disjoint from q4)

    // screen' = (screen & ~changed) | dark
    ee.notq         q0, q0
    ee.andq         q3, q3, q0
    ee.orq          q3, q3, q4

    // Dark plane: store unconditionally (internal RAM), set mask bit if any
    ee.vst.128.ip   q4, a3, 16
    ee.movi.32.a    q4, a10, 0
    ee.movi.32.a    q4, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q4, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q4, a14, 3
    or              a10, a10, a14

    beqz            a10, .inkdual_no_dark
    or              a11, a11, a12
.inkdual_no_dark:
    mov             a8, a10         // a8 accumulates "anything driven"

    // Light plane: store only when non-empty (PSRAM — skip idle writes)
    ee.movi.32.a    q1, a10, 0
    ee.movi.32.a    q1, a14, 1
    or              a10, a10, a14
    ee.movi.32.a    q1, a14, 2
    or              a10, a10, a14
    ee.movi.32.a    q1, a14, 3
    or              a10, a10, a14

    beqz            a10, .inkdual_no_light
    or              a9, a9, a12
    ee.vst.128.ip   q1, a4, 16
    j               .inkdual_light_done
.inkdual_no_light:
    addi            a4, a4, 16
.inkdual_light_done:
    or              a8, a8, a10

    // Screenbuffer: write back only if this chunk drove anything
    beqz            a8, .inkdual_next
    addi            a10, a5, -16
    ee.vst.128.ip   q3, a10, 0

.inkdual_next:
    srli            a12, a12, 1
    addi            a6, a6, -16
    bnez            a6, .inkdual_loop

    s32i            a9, a7, 0       // light-plane mask out
    mov             a2, a11         // dark-plane mask return
    retw.n
#endif
