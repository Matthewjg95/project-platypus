#pragma once
#include <M5Unified.h>
#include <M5GFX.h>
#include <vector>
#include <SD_MMC.h>
#include "applet.h"
#include "ui_constants.h"
#include "ui_theme.h"
#include "ui_icons.h"

// ============================================================
// shell.h — App shell for M5View
// ============================================================



class Shell {
public:
    void register_applet(Applet* applet) {
        _applets.push_back(applet);
    }

    void begin(int display_w, int display_h) {
        _w = display_w;
        _h = display_h;
        _load_ui_cfg();
        M5.Speaker.setVolume(_ui_vol);
        M5.Display.setBrightness(_ui_bright);
        _state           = STATE_HOME;
        _active          = -1;
        _settings_open   = false;
        _settings_y      = _h;
        _settings_target = _h;
        _dirty           = true;
        _settings_dirty  = false;
    }

    void update() {
        M5.update();
        _handle_touch();
        // (Settings panel no longer animates — the slide left a trail of
        // part-drawn frames behind it because nothing repainted the strip it
        // revealed while closing. Instant open/close, one clean draw each.)
    }

    void render() {
        if (_state == STATE_HOME) {
            if (_dirty) {
                _draw_home_full();
                _dirty = false;
                _settings_dirty = (_settings_open || _settings_y < _h);
            }
            if (_settings_dirty) {
                if (_settings_open || _settings_y < _h)
                    _draw_settings_panel();
                _settings_dirty = false;
            }

        } else if (_state == STATE_APPLET && _active >= 0) {
            if (!_applets[_active]->on_update()) {
                _exit_applet();
                return;
            }
            _applets[_active]->on_render();

            // Fullscreen applets own the whole screen — no shell chrome.
            if (_applets[_active]->is_fullscreen()) return;

            if (_dirty) {
                _draw_top_bar_applet();
                _dirty = false;
            }

            if (_settings_dirty) {
                if (_settings_open || _settings_y < _h)
                    _draw_settings_panel();
                else {
                    // Panel fully closed — redraw top bar and applet content
                    _dirty = true;
                }
                _settings_dirty = false;
            }
        }
    }

    bool is_in_applet() const { return _state == STATE_APPLET; }

private:
    enum State { STATE_HOME, STATE_APPLET };

    int                  _w = 0, _h = 0;
    std::vector<Applet*> _applets;
    State                _state          = STATE_HOME;
    int                  _active         = -1;
    bool                 _settings_open  = false;
    int                  _settings_y     = 0;
    int                  _settings_target = 0;
    bool                 _dirty          = true;
    bool                 _settings_dirty = false;

    int  _prev_tx = -1, _prev_ty = -1;
    bool _prev_touched = false;

    // ---- real settings: volume + brightness sliders, persisted ---------
    uint8_t _ui_vol = 130, _ui_bright = 255;
    int     _adj = 0;                       // 0 none, 1 volume, 2 brightness
    static constexpr const char* UI_CFG = "/m5view_ui.cfg";

    void _load_ui_cfg() {
        File f = SD_MMC.open(UI_CFG, FILE_READ);
        if (!f) return;
        char m[4];
        if (f.read((uint8_t*)m, 4) == 4 && memcmp(m, "UIC1", 4) == 0) {
            f.read(&_ui_vol, 1);
            f.read(&_ui_bright, 1);
            if (_ui_bright < 20) _ui_bright = 20;   // never save yourself blind
        }
        f.close();
    }
    void _save_ui_cfg() {
        File f = SD_MMC.open(UI_CFG, FILE_WRITE);
        if (!f) return;
        f.write((const uint8_t*)"UIC1", 4);
        f.write(&_ui_vol, 1);
        f.write(&_ui_bright, 1);
        f.close();
    }

    // slider geometry (shared by draw + touch)
    void _slider_rect(int which, int& x, int& y, int& w, int& h) {
        x = 250; w = _w - x - 60; h = 40;
        y = _settings_y + (which == 1 ? 84 : 164);
    }

    // continuous slider handling while the settings panel is open.
    // returns true if the touch was consumed by a slider.
    bool _settings_sliders(bool touched, int tx, int ty) {
        if (!_settings_open) return false;
        if (!touched) {
            if (_adj) {                      // release: persist + audible preview
                _save_ui_cfg();
                if (_adj == 1) M5.Speaker.tone(800, 45);   // low pip, not a squeak
                _adj = 0;
            }
            return false;
        }
        for (int which = 1; which <= 2; ++which) {
            int x, y, w, h; _slider_rect(which, x, y, w, h);
            bool inside = tx >= x - 10 && tx < x + w + 10 &&
                          ty >= y - 10 && ty < y + h + 10;
            if ((_adj == which) || (_adj == 0 && inside)) {
                _adj = which;
                int v = (tx - x) * 255 / (w > 0 ? w : 1);
                if (v < 0) v = 0; if (v > 255) v = 255;
                if (which == 1) { _ui_vol = (uint8_t)v; M5.Speaker.setVolume(_ui_vol); }
                else { if (v < 20) v = 20; _ui_bright = (uint8_t)v;
                       M5.Display.setBrightness(_ui_bright); }
                // targeted row repaint — a full panel redraw per drag frame
                // flickered and ghosted ("hazy"); the row clears itself
                _draw_slider_row(which);
                return true;
            }
        }
        return false;
    }

    // draw one slider row, clearing its strip first (handle overhang incl.)
    void _draw_slider_row(int which) {
        int x, y, w, h; _slider_rect(which, x, y, w, h);
        M5.Display.startWrite();
        M5.Display.fillRect(0, y - 6, _w - 12, h + 12, COL_PANEL);
        ui_theme::font_body(&M5.Display);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(24, y + 8);
        M5.Display.print(which == 1 ? "Volume" : "Brightness");
        uint8_t val = which == 1 ? _ui_vol : _ui_bright;
        M5.Display.fillRoundRect(x, y + h/2 - 5, w, 10, 5, 0x2945);
        int fx = x + (int)val * w / 255;
        M5.Display.fillRoundRect(x, y + h/2 - 5, fx - x, 10, 5, ui_theme::ACCENT);
        M5.Display.fillCircle(fx, y + h/2, 14, COL_TEXT);
        ui_theme::font_mono1(&M5.Display);
        M5.Display.setTextColor(COL_SUBTEXT, COL_PANEL);
        M5.Display.setCursor(x + w + 14, y + h/2 - 4);
        M5.Display.printf("%3d", val);
        ui_theme::font_mono(&M5.Display);
        M5.Display.endWrite();
    }

    // Pause/resume the active applet generically (no concrete-type knowledge).
    void _pause_active()  { if (_active >= 0) _applets[_active]->on_pause(); }
    void _resume_active() { if (_active >= 0) _applets[_active]->on_resume(); }

    void _handle_touch() {
        bool touched = (M5.Touch.getCount() > 0);
        int tx = -1, ty = -1;
        if (touched) {
            auto t = M5.Touch.getDetail(0);
            tx = t.x; ty = t.y;
        }

        // live slider drags own the touch while the settings panel is open
        if (_settings_sliders(touched, tx, ty)) {
            _prev_touched = touched; _prev_tx = tx; _prev_ty = ty;
            return;
        }

        bool tapped = (_prev_touched && !touched);
        int tap_x = _prev_tx, tap_y = _prev_ty;
        _prev_touched = touched;
        _prev_tx = tx; _prev_ty = ty;

        if (!tapped) return;

        if (_settings_open) {
            if (tap_y < _settings_y) _close_settings();
            return;
        }

        if (_state == STATE_HOME) {
            int tile = _tile_at(tap_x, tap_y);
            if (tile >= 0 && tile < (int)_applets.size())
                _launch_applet(tile);

        } else if (_state == STATE_APPLET) {
            // Fullscreen applets handle all their own touch (own nav + exit).
            if (_active >= 0 && _applets[_active]->is_fullscreen()) return;
            if (_tap_in_back(tap_x, tap_y)) {
                // Ask the applet if it wants to handle back internally
                if (!_applets[_active]->on_back()) {
                    _exit_applet();
                }
                return;
            }
            if (_tap_in_settings_btn(tap_x, tap_y)) { _open_settings(); return; }
        }
    }

    bool _tap_in_back(int x, int y) {
        return x >= 0 && x < BACK_BTN_W && y >= 0 && y < BAR_H;
    }
    bool _tap_in_settings_btn(int x, int y) {
        return x >= _w - SETTINGS_BTN_W && y >= 0 && y < BAR_H;
    }
    int _tile_at(int x, int y) {
        if (y < BAR_H) return -1;
        int cy = BAR_H + TILE_PAD;
        int ch = _h - cy - TILE_PAD;
        int cw = _w - TILE_PAD * 2;
        int tw = (cw - TILE_PAD*(TILE_COLS-1)) / TILE_COLS;
        int th = (ch - TILE_PAD*(TILE_ROWS-1)) / TILE_ROWS;
        for (int r = 0; r < TILE_ROWS; r++) {
            for (int c = 0; c < TILE_COLS; c++) {
                int tx = TILE_PAD + c*(tw+TILE_PAD);
                int ty = cy       + r*(th+TILE_PAD);
                if (x >= tx && x < tx+tw && y >= ty && y < ty+th)
                    return r*TILE_COLS + c;
            }
        }
        return -1;
    }

    void _launch_applet(int index) {
        _active = index;
        _state  = STATE_APPLET;
        _prev_touched = false; _prev_tx = _prev_ty = -1;   // fresh touch state
        M5.Display.fillScreen(TFT_BLACK);
        _applets[index]->on_enter();
        _dirty = true;
    }

    void _exit_applet() {
        if (_active >= 0) _applets[_active]->on_exit();
        _active = -1;
        _state  = STATE_HOME;
        _prev_touched = false; _prev_tx = _prev_ty = -1;   // fresh touch state
        _dirty  = true;
        if (_settings_open) _close_settings_immediate();
    }

    void _open_settings() {
        // Pause the active applet so its drawing doesn't overwrite the panel
        _pause_active();
        _settings_open   = true;
        _settings_y      = _h - PANEL_H;   // instant: no slide, no trail
        _settings_target = _settings_y;
        _settings_dirty  = true;
    }

    void _close_settings() {
        _settings_open   = false;
        _settings_y      = _h;             // instant: gone in one frame
        _settings_target = _h;
        _settings_dirty  = false;
        _dirty = true;                     // home / top-bar full repaint
        // Applets repair their own content in on_resume (the panel overdrew
        // regions their normal draw path never repaints).
        _resume_active();
    }

    void _close_settings_immediate() {
        _settings_open   = false;
        _settings_y      = _h;
        _settings_target = _h;
        _resume_active();
    }

    // ---- Drawing -------------------------------------------

    void _draw_home_full() {
        M5.Display.startWrite();
        M5.Display.fillScreen(COL_BG);
        M5.Display.fillRect(0, 0, _w, BAR_H, COL_BAR);
        ui_theme::font_button(&M5.Display);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(14, 12);
        M5.Display.print("PLATYPUS");
        ui_theme::font_mono1(&M5.Display);
        M5.Display.setTextColor(COL_SUBTEXT);
        M5.Display.setCursor(_w / 2 - 40, 22);
        M5.Display.print("m5view shell");
        M5.Display.drawFastHLine(0, BAR_H, _w, COL_DIVIDER);
        _draw_tiles();
        ui_theme::font_mono(&M5.Display);   // leave state predictable
        M5.Display.endWrite();
    }

    void _draw_top_bar_applet() {
        M5.Display.startWrite();
        M5.Display.fillRect(0, 0, _w, BAR_H, COL_BAR);

        M5.Display.fillRoundRect(6, 8, BACK_BTN_W-6, BAR_H-16, 8, COL_BTN);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(18, 18);
        M5.Display.print("< Back");

        M5.Display.fillRoundRect(_w-SETTINGS_BTN_W, 8,
                                  SETTINGS_BTN_W-6, BAR_H-16, 8, COL_BTN);
        M5.Display.setCursor(_w-SETTINGS_BTN_W+12, 18);
        M5.Display.print("Settings");

        M5.Display.drawFastHLine(0, BAR_H, _w, COL_DIVIDER);
        M5.Display.endWrite();
    }

    void _draw_tiles() {
        int cy = BAR_H + TILE_PAD;
        int ch = _h - cy - TILE_PAD;
        int cw = _w - TILE_PAD * 2;
        int tw = (cw - TILE_PAD*(TILE_COLS-1)) / TILE_COLS;
        int th = (ch - TILE_PAD*(TILE_ROWS-1)) / TILE_ROWS;

        for (int i = 0; i < (int)_applets.size() && i < TILE_COLS*TILE_ROWS; i++) {
            int r = i/TILE_COLS, c = i%TILE_COLS;
            int tx = TILE_PAD + c*(tw+TILE_PAD);
            int ty = cy       + r*(th+TILE_PAD);

            M5.Display.fillRoundRect(tx, ty, tw, th, TILE_RADIUS, COL_TILE);

            // vector glyph chosen by applet name (shell stays type-agnostic)
            const char* lbl = _applets[i]->name();
            // Glyph scales with tile height: 84px was sized for the old 2-row
            // grid and looked lost in a full-height tile.
            int isz = th > 400 ? 150 : 84;
            int icx = tx + tw/2, icy = ty + th/2 - 24;
            if      (strstr(lbl, "View"))    ui_icons::cube (&M5.Display, icx, icy, isz, COL_TEXT);
            else if (strstr(lbl, "Scan"))    ui_icons::radar(&M5.Display, icx, icy, isz, COL_TEXT);
            else if (strstr(lbl, "Antenna")) ui_icons::mast (&M5.Display, icx, icy, isz, COL_TEXT);
            else {
                M5.Display.setTextSize(4); M5.Display.setTextColor(COL_TEXT);
                M5.Display.setCursor(icx - 16, icy - 16);
                M5.Display.print(_applets[i]->icon());
            }

            ui_theme::font_button(&M5.Display);
            M5.Display.setTextColor(COL_TEXT);
            int lw = M5.Display.textWidth(lbl);
            M5.Display.setCursor(tx + tw/2 - lw/2, ty + th - 40);
            M5.Display.print(lbl);
        }
        ui_theme::font_mono(&M5.Display);
    }

    void _draw_settings_panel() {
        if (_settings_y >= _h) return;

        M5.Display.startWrite();
        M5.Display.fillRoundRect(0, _settings_y, _w, _h - _settings_y + 20, 20, COL_PANEL);

        // Handle bar
        M5.Display.fillRoundRect(_w/2-30, _settings_y+10, 60, 6, 3, COL_HANDLE);

        ui_theme::font_button(&M5.Display);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(_w/2 - 60, _settings_y + 22);
        M5.Display.print("Settings");
        M5.Display.drawFastHLine(12, _settings_y+54, _w-24, COL_DIVIDER);

        M5.Display.endWrite();
        // two REAL sliders: volume + brightness (live, persisted on release)
        _draw_slider_row(1);
        _draw_slider_row(2);
    }
};