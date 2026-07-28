#pragma once
#include <M5Unified.h>
#include <M5GFX.h>
#include <vector>
#include "applet.h"
#include "ui_constants.h"

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

        // Animate settings panel
        if (_settings_y != _settings_target) {
            int diff = _settings_target - _settings_y;
            int step = diff / 4;
            if (step == 0) step = (diff > 0) ? 1 : -1;
            _settings_y += step;
            if (abs(_settings_y - _settings_target) < 3)
                _settings_y = _settings_target;
            _settings_dirty = true;
        }
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
        _settings_target = _h - PANEL_H;
        _settings_dirty  = true;
    }

    void _close_settings() {
        _settings_open   = false;
        _settings_target = _h;
        _settings_dirty  = true;
        // Resume the active applet and force full redraw to clear ghost
        _resume_active();
        _dirty = true;
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
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(12, 18);
        M5.Display.print("M5View");
        M5.Display.drawFastHLine(0, BAR_H, _w, COL_DIVIDER);
        _draw_tiles();
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

            M5.Display.setTextSize(4);
            M5.Display.setTextColor(COL_TEXT);
            M5.Display.setCursor(tx + tw/2 - 16, ty + th/2 - 32);
            M5.Display.print(_applets[i]->icon());

            M5.Display.setTextSize(2);
            M5.Display.setTextColor(COL_SUBTEXT);
            const char* lbl = _applets[i]->name();
            int lw = strlen(lbl) * 12;
            M5.Display.setCursor(tx + tw/2 - lw/2, ty + th - 32);
            M5.Display.print(lbl);
        }
    }

    void _draw_settings_panel() {
        if (_settings_y >= _h) return;

        M5.Display.startWrite();
        M5.Display.fillRoundRect(0, _settings_y, _w, _h - _settings_y + 20, 20, COL_PANEL);

        // Handle bar
        M5.Display.fillRoundRect(_w/2-30, _settings_y+10, 60, 6, 3, COL_HANDLE);

        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(_w/2 - 52, _settings_y + 28);
        M5.Display.print("Settings");
        M5.Display.drawFastHLine(12, _settings_y+54, _w-24, COL_DIVIDER);

        const char* items[] = {
            "Display brightness",
            "Rotation speed",
            "Lighting preset",
            "About M5View"
        };
        for (int i = 0; i < 4; i++) {
            int iy = _settings_y + 70 + i * 60;
            if (iy > _h - 20) break;
            M5.Display.setTextColor(COL_SUBTEXT);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(24, iy);
            M5.Display.print(items[i]);
            M5.Display.setCursor(_w - 30, iy);
            M5.Display.print(">");
            if (i < 3) M5.Display.drawFastHLine(12, iy+44, _w-24, 0x2945);
        }
        M5.Display.endWrite();
    }
};