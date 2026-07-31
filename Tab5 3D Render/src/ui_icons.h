// ui_icons.h - Vector glyphs drawn with primitives. No bitmap assets, no
// icon fonts — fully portable across displays (modularity principle).
// Every icon draws centred at (cx, cy) within a nominal s x s box.

#pragma once
#include <M5GFX.h>

namespace ui_icons {

// wireframe cube — the 3D viewer
inline void cube(LovyanGFX* d, int cx, int cy, int s, uint16_t col) {
    int h = s / 2, o = s / 5;                       // half size, iso offset
    int x0 = cx - h, y0 = cy - h + o, x1 = cx + h - o, y1 = cy + h;
    // front face
    d->drawRect(x0, y0, x1 - x0, y1 - y0, col);
    // back face (offset)
    d->drawRect(x0 + o, y0 - o, x1 - x0, y1 - y0, col);
    // connectors
    d->drawLine(x0, y0, x0 + o, y0 - o, col);
    d->drawLine(x1, y0, x1 + o, y0 - o, col);
    d->drawLine(x0, y1, x0 + o, y1 - o, col);
    d->drawLine(x1, y1, x1 + o, y1 - o, col);
}

// radar sweep — the room scanner
inline void radar(LovyanGFX* d, int cx, int cy, int s, uint16_t col) {
    int r = s / 2;
    d->drawCircle(cx, cy, r, col);
    d->drawCircle(cx, cy, (r * 2) / 3, col);
    d->drawCircle(cx, cy, r / 3, col);
    // sweep beam at ~40 degrees
    d->drawLine(cx, cy, cx + (int)(r * 0.77f), cy - (int)(r * 0.64f), col);
    d->fillCircle(cx + r / 2, cy + r / 3, 2, col);      // a blip
}

// antenna mast — the antenna suite
inline void mast(LovyanGFX* d, int cx, int cy, int s, uint16_t col) {
    int h = s / 2;
    d->drawLine(cx, cy - h, cx, cy + h, col);            // mast
    d->drawLine(cx - h / 2, cy + h, cx + h / 2, cy + h, col);   // base
    // radiating arcs (approximated with short circle segments)
    for (int r = s / 4; r <= s / 2; r += s / 4) {
        d->drawArc(cx, cy - h, r, r, 300, 60, col);
    }
}

// trash can — destructive actions
inline void trash(LovyanGFX* d, int cx, int cy, int s, uint16_t col) {
    int w = (s * 3) / 5, h = s / 2;
    int x0 = cx - w / 2, y0 = cy - h / 2 + s / 10;
    d->drawRect(x0, y0, w, h, col);                      // body
    d->drawFastHLine(x0 - s / 10, y0 - s / 12, w + s / 5, col);   // lid
    d->drawFastVLine(cx - w / 4, y0 + 3, h - 6, col);    // ribs
    d->drawFastVLine(cx,         y0 + 3, h - 6, col);
    d->drawFastVLine(cx + w / 4, y0 + 3, h - 6, col);
}

// wifi arcs — connectivity chip
inline void wifi(LovyanGFX* d, int cx, int cy, int s, uint16_t col) {
    for (int i = 1; i <= 3; ++i)
        d->drawArc(cx, cy, (s * i) / 6, (s * i) / 6, 225, 315, col);
    d->fillCircle(cx, cy, 2, col);
}

} // namespace ui_icons
