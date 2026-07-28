// jpeg_decode.h - Decode a UART JPEG frame to RGB565 (and to grayscale) using
// JPEGDEC. Frames are decoded at half scale (320x240 -> 160x120) which suits
// both the live preview canvas and the room fitter's Hough input.

#pragma once
#include <stdint.h>

// Decode `jpeg`/`len` into `out_rgb565` (must hold >= 160*120 pixels). On
// success returns true and sets *w,*h to the decoded dimensions.
bool jpeg_decode_half_rgb565(const uint8_t* jpeg, uint32_t len,
                             uint16_t* out_rgb565, int* w, int* h);

// Convert an RGB565 buffer to 8-bit luma in-place into `gray` (w*h bytes).
void rgb565_to_gray(const uint16_t* rgb565, int w, int h, uint8_t* gray);
