// jpeg_decode.cpp
#include "jpeg_decode.h"
#include <JPEGDEC.h>

namespace {
    JPEGDEC   s_jpeg;
    uint16_t* s_target = nullptr;
    int       s_target_w = 0, s_target_h = 0;

    // JPEGDEC hands us decoded blocks; copy each into the target raster.
    int drawBlock(JPEGDRAW* d) {
        for (int row = 0; row < d->iHeight; ++row) {
            int ty = d->y + row;
            if (ty < 0 || ty >= s_target_h) continue;
            const uint16_t* src = &d->pPixels[row * d->iWidth];
            for (int col = 0; col < d->iWidth; ++col) {
                int tx = d->x + col;
                if (tx < 0 || tx >= s_target_w) continue;
                s_target[ty * s_target_w + tx] = src[col];
            }
        }
        return 1;  // continue decoding
    }
}

bool jpeg_decode_half_rgb565(const uint8_t* jpeg, uint32_t len,
                             uint16_t* out_rgb565, int* w, int* h) {
    if (!jpeg || !out_rgb565 || len == 0) return false;
    if (!s_jpeg.openRAM((uint8_t*)jpeg, (int)len, drawBlock)) return false;

    s_jpeg.setPixelType(RGB565_LITTLE_ENDIAN);
    int fullW = s_jpeg.getWidth(), fullH = s_jpeg.getHeight();
    int outW = fullW / 2, outH = fullH / 2;   // JPEG_SCALE_HALF

    s_target = out_rgb565;
    s_target_w = outW;
    s_target_h = outH;

    int ok = s_jpeg.decode(0, 0, JPEG_SCALE_HALF);
    s_jpeg.close();
    s_target = nullptr;
    if (!ok) return false;

    if (w) *w = outW;
    if (h) *h = outH;
    return true;
}

void rgb565_to_gray(const uint16_t* rgb565, int w, int h, uint8_t* gray) {
    for (int i = 0; i < w * h; ++i) {
        uint16_t p = rgb565[i];
        int r = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
        // scale to 0..255 and apply luma weights (approx 0.30/0.59/0.11)
        int R = (r * 255) / 31, G = (g * 255) / 63, B = (b * 255) / 31;
        gray[i] = (uint8_t)((R * 77 + G * 151 + B * 28) >> 8);
    }
}
