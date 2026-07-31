// ui_theme.h - Shared visual language for Project Platypus.
//
// One place for the palette, typography, and touch-feedback flash so every
// applet reads as the same product (and re-theming is a one-file edit).
//
// Typography rule: real vector faces (FreeSans family, bundled with
// LovyanGFX) for STATIC chrome — titles, buttons, list rows, modals. The
// classic 6x8 mono font stays for DYNAMIC telemetry text (counters, HUDs)
// because GFX faces don't background-fill on reprint. Call one of the font
// helpers at the start of any draw block; never assume the previous state.

#pragma once
#include <M5GFX.h>

namespace ui_theme {

// ---- palette (RGB565) -------------------------------------------------
static const uint16_t BG        = 0x0841;   // app background
static const uint16_t SURFACE   = 0x2104;   // cards / rows
static const uint16_t SURFACE_2 = 0x2945;   // raised buttons
static const uint16_t ACCENT    = 0x0300;   // green actions (live, scan+)
static const uint16_t DANGER    = 0x8000;   // destructive
static const uint16_t TEXT      = 0xFFFF;
static const uint16_t SUBTEXT   = 0x8410;
static const uint16_t DIVIDER   = 0x4208;

// ---- typography -------------------------------------------------------
inline void font_title(LovyanGFX* d)  { d->setFont(&fonts::FreeSansBold18pt7b); d->setTextSize(1); }
inline void font_button(LovyanGFX* d) { d->setFont(&fonts::FreeSansBold12pt7b); d->setTextSize(1); }
inline void font_body(LovyanGFX* d)   { d->setFont(&fonts::FreeSans9pt7b);      d->setTextSize(1); }
inline void font_mono(LovyanGFX* d)   { d->setFont(&fonts::Font0);              d->setTextSize(2); }
inline void font_mono1(LovyanGFX* d)  { d->setFont(&fonts::Font0);              d->setTextSize(1); }

// ---- touch feedback ---------------------------------------------------
// Brief inverted flash over a control on the press edge; the action's own
// redraw restores it. Pairs with ui_feedback's tick.
inline void press_flash(int x, int y, int w, int h) {
    M5.Display.fillRoundRect(x, y, w, h, 6, TEXT);
    delay(45);
}

} // namespace ui_theme
