#pragma once
#include "../applet.h"
#include "../ui_constants.h"
#include "../ui_feedback.h"
#include "../scanner/uart_receiver.h"
#include "ss_geometry.h"
#include "ss_silhouette.h"
#include "ss_export.h"

#include <M5Unified.h>
#include <M5GFX.h>
#include <esp_heap_caps.h>
#include <vector>

// ============================================================
// shadowscan_applet.h — ShadowScan applet for M5View.
//
// Silhouette -> polygon -> extruded STL, for shadow-box inserts and foam
// cutouts. Ported from the standalone "UnitV Outline Capture" project, but
// integrated the M5View way:
//
//   * The Unit V keeps its existing MeshScan firmware — we consume the same
//     FRAME (JPEG) stream on Serial1 that the Room Scan applet uses. All
//     vision work (threshold/blob/trace/simplify) runs here on the P4, so
//     there is no second camera firmware and no protocol change.
//   * Immediate-mode M5GFX UI in the house style (no LVGL).
//   * Exports: /shadowscan/<name>.stl (+.obj) in mm, and /models/<name>.mesh
//     so every scan also appears in the 3D Viewer applet.
//
// Flow:  LIVE    -> streaming preview, threshold controls, CAPTURE
//        REVIEW  -> outline editing: smooth/undo, rotate, scale, confirm
//        THICK   -> thickness presets + slider, wireframe extrusion preview
//        DONE    -> export summary
//
// The Unit V's QVGA JPEG is decoded at full 320x240 for contour quality
// (Room Scan decodes at 160x120 — its fitter wants speed, we want edges).
class ShadowScanApplet : public Applet {
public:
    const char* name() const override { return "ShadowScan"; }
    const char* icon() const override { return "O"; }

    void on_enter() override {
        if (!_ready) _init();
        _state = LIVE;
        _have_frame = false;
        _need_redraw = true;
        _reset_touch();
    }

    void on_exit() override { /* keep buffers for fast re-entry */ }

    void on_pause()  override { _paused = true; }
    void on_resume() override { _paused = false; _need_redraw = true; }

    bool on_back() override {
        switch (_state) {
            case REVIEW: _state = LIVE;   _need_redraw = true; _reset_touch(); return true;
            case THICK:  _state = REVIEW; _need_redraw = true; _reset_touch(); return true;
            case DONE:   _state = LIVE;   _need_redraw = true; _reset_touch(); return true;
            case LIVE:   return false;    // shell exits to home
        }
        return false;
    }

    bool on_update() override {
        if (_alloc_failed) return true;
        if (_state == LIVE) _uart.update();

        int tx, ty;
        if (_poll_tap(tx, ty)) {
            switch (_state) {
                case LIVE:   _tap_live(tx, ty);   break;
                case REVIEW: _tap_review(tx, ty); break;
                case THICK:  _tap_thick(tx, ty);  break;
                case DONE:   _tap_done(tx, ty);   break;
            }
        }
        if (_state == THICK) _drag_slider();
        return true;
    }

    void on_render() override {
        if (_paused) return;
        if (_alloc_failed) { _draw_alloc_failed(); return; }
        switch (_state) {
            case LIVE:   _draw_live();   break;
            case REVIEW: if (_need_redraw) _draw_review(); break;
            case THICK:  if (_need_redraw) _draw_thick();  break;
            case DONE:   if (_need_redraw) _draw_done();   break;
        }
    }

private:
    enum State { LIVE, REVIEW, THICK, DONE };
    State _state = LIVE;
    bool _paused = false, _ready = false, _alloc_failed = false;
    bool _need_redraw = true;

    // Rough scale at the default working distance; the object's true size
    // rarely matters for shadow-box inserts (Scale +/- covers the rest).
    static constexpr float PX_PER_MM = 3.0f;
    static const int CAM_W = 320, CAM_H = 240;

    // ---- camera link (same wire + protocol as Room Scan) ----
    UartReceiver _uart;
    uint8_t*  _jpeg = nullptr;         // latest frame copy, PSRAM
    uint16_t  _jpeg_len = 0;
    volatile bool _jpeg_fresh = false;
    bool      _have_frame = false;
    M5Canvas* _decode = nullptr;       // 320x240 full-res decode
    uint8_t*  _gray = nullptr;         // 320x240 grayscale scratch, PSRAM
    uint32_t  _last_frame_ms = 0;

    // ---- extraction / edit state ----
    int  _manual_thresh = -1;          // -1 = auto (Otsu)
    ss::Silhouette _sil;
    std::vector<ss::Poly> _hist;       // smooth history; [0] = as captured
    float _rot_deg = 0, _scale = 1.0f;
    float _thick_mm = 6.35f;           // 0.25in default
    ss::Poly _contour_mm;
    char _export_name[24] = {0};
    int  _export_tris = 0;
    bool _export_ok = false, _export_mesh_ok = false;

    // ---- touch (tap = press+release with little movement, like Room Scan) ----
    bool _prev_touch = false;
    int  _down_x = -1, _down_y = -1, _cur_x = -1, _cur_y = -1;
    bool _moved = false;
    void _reset_touch() { _prev_touch = false; _moved = false; _down_x = _down_y = -1; }

    bool _poll_tap(int& ox, int& oy) {
        bool touched = M5.Touch.getCount() > 0;
        if (touched) {
            auto t = M5.Touch.getDetail(0);
            _cur_x = t.x; _cur_y = t.y;
            if (!_prev_touch) { _down_x = t.x; _down_y = t.y; _moved = false; }
            else if (abs(t.x - _down_x) > 15 || abs(t.y - _down_y) > 15) _moved = true;
        }
        bool tap = false;
        if (!touched && _prev_touch && !_moved && _down_y >= BAR_H) {
            ox = _down_x; oy = _down_y;
            tap = true;
        }
        _prev_touch = touched;
        return tap;
    }

    // ====================================================
    void _init() {
        _decode = new M5Canvas(&M5.Display);
        _decode->setColorDepth(16);
        bool ok = _decode->createSprite(CAM_W, CAM_H) != nullptr;
        _jpeg = (uint8_t*)heap_caps_malloc(UART_MAX_JPEG, MALLOC_CAP_SPIRAM);
        _gray = (uint8_t*)heap_caps_malloc(CAM_W * CAM_H, MALLOC_CAP_SPIRAM);
        if (!ok || !_jpeg || !_gray) {
            _alloc_failed = true;
            Serial.println("[shadowscan] alloc FAILED");
            return;
        }
        _decode->fillSprite(TFT_BLACK);
        // Serial1 may already be running from the Room Scan applet with the
        // same pins/baud — re-begin is harmless, and only the foreground
        // applet drains the FIFO, so the two receivers never fight.
        _uart.begin(Serial1, UNITV_UART_BAUD, UNITV_UART_RX, UNITV_UART_TX);
        _uart.onFrame(_on_frame, this);
        _ready = true;
        Serial.println("[shadowscan] ready");
    }

    static void _on_frame(const uint8_t* jpeg, uint16_t len, void* u) {
        auto* self = (ShadowScanApplet*)u;
        if (len > UART_MAX_JPEG) return;
        memcpy(self->_jpeg, jpeg, len);
        self->_jpeg_len = len;
        self->_jpeg_fresh = true;
    }

    // ---- shared chrome ------------------------------------
    void _clear_content() {
        M5.Display.fillRect(0, BAR_H, M5.Display.width(),
                            M5.Display.height() - BAR_H, COL_BG);
    }

    void _btn(int x, int y, int w, int h, const char* label, uint16_t col,
              uint16_t txt_col = COL_TEXT) {
        M5.Display.fillRoundRect(x, y, w, h, 8, col);
        M5.Display.setTextColor(txt_col, col);
        M5.Display.setTextSize(2);
        int tw = M5.Display.textWidth(label);
        M5.Display.setCursor(x + (w - tw) / 2, y + h / 2 - 8);
        M5.Display.print(label);
    }

    static bool _in(int x, int y, int bx, int by, int bw, int bh) {
        return x >= bx && x < bx + bw && y >= by && y < by + bh;
    }

    void _draw_alloc_failed() {
        if (!_need_redraw) return;
        _need_redraw = false;
        _clear_content();
        M5.Display.setTextColor(TFT_RED, COL_BG);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(24, BAR_H + 40);
        M5.Display.print("LOW MEMORY - ShadowScan unavailable");
    }

    // ================= LIVE =================
    // Layout: 640x480 preview left, controls column right.
    static const int PV_X = 24, PV_W2 = 640, PV_H2 = 480;
    int _pv_y() const { return BAR_H + 24; }
    int _ctl_x() const { return PV_X + PV_W2 + 40; }

    void _draw_live_chrome() {
        _clear_content();
        int cx = _ctl_x(), dw = M5.Display.width();
        M5.Display.drawRect(PV_X - 2, _pv_y() - 2, PV_W2 + 4, PV_H2 + 4, COL_DIVIDER);

        M5.Display.setTextColor(COL_TEXT, COL_BG);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(PV_X, _pv_y() + PV_H2 + 16);
        M5.Display.print("Object on a contrasting background, fill the frame.");

        _btn(cx, BAR_H + 40, dw - cx - 24, 130, "CAPTURE", 0x0300);
        // threshold row
        M5.Display.setTextColor(COL_SUBTEXT, COL_BG);
        M5.Display.setCursor(cx, BAR_H + 210);
        M5.Display.print("Threshold");
        _btn(cx,       BAR_H + 240, 150, 70, _manual_thresh < 0 ? "AUTO *" : "AUTO",
             _manual_thresh < 0 ? 0x0300 : COL_BTN);
        _btn(cx + 170, BAR_H + 240, 100, 70, "-", COL_BTN);
        _btn(cx + 290, BAR_H + 240, 100, 70, "+", COL_BTN);
        _draw_thresh_value();
    }

    void _draw_thresh_value() {
        int cx = _ctl_x();
        M5.Display.fillRect(cx + 410, BAR_H + 240, 140, 70, COL_BG);
        M5.Display.setTextColor(COL_TEXT, COL_BG);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(cx + 415, BAR_H + 262);
        if (_manual_thresh < 0) M5.Display.print("(otsu)");
        else                    M5.Display.printf("%d", _manual_thresh);
    }

    void _draw_live() {
        if (_need_redraw) { _need_redraw = false; _draw_live_chrome(); }
        if (_jpeg_fresh) {
            _jpeg_fresh = false;
            _decode->fillSprite(TFT_BLACK);
            _decode->drawJpg(_jpeg, _jpeg_len, 0, 0, CAM_W, CAM_H);
            _decode->pushRotateZoom(&M5.Display, PV_X + PV_W2 / 2,
                                    _pv_y() + PV_H2 / 2, 0.0f, 2.0f, 2.0f);
            _have_frame = true;
            _last_frame_ms = millis();
        }
        // link indicator (stale feed after 3s = probably in the wrong applet's
        // hands or the camera is off)
        static uint32_t last_ind = 0;
        if (millis() - last_ind > 500) {
            last_ind = millis();
            bool live = _have_frame && millis() - _last_frame_ms < 3000;
            M5.Display.fillCircle(_ctl_x() + 20, BAR_H + 16, 8,
                                  live ? 0x0300 : TFT_RED);
            M5.Display.setTextColor(COL_SUBTEXT, COL_BG);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(_ctl_x() + 40, BAR_H + 8);
            M5.Display.print(live ? "Camera live " : "No feed     ");
        }
    }

    void _tap_live(int x, int y) {
        int cx = _ctl_x(), dw = M5.Display.width();
        if (_in(x, y, cx, BAR_H + 40, dw - cx - 24, 130)) { _capture(); return; }
        if (_in(x, y, cx, BAR_H + 240, 150, 70)) {         // AUTO toggle
            _manual_thresh = (_manual_thresh < 0) ? 128 : -1;
            ui_feedback::tick();
            _need_redraw = true;
            return;
        }
        if (_in(x, y, cx + 170, BAR_H + 240, 100, 70)) {
            if (_manual_thresh < 0) _manual_thresh = 128;
            _manual_thresh = std::max(0, _manual_thresh - 8);
            ui_feedback::tick(); _draw_thresh_value(); return;
        }
        if (_in(x, y, cx + 290, BAR_H + 240, 100, 70)) {
            if (_manual_thresh < 0) _manual_thresh = 128;
            _manual_thresh = std::min(255, _manual_thresh + 8);
            ui_feedback::tick(); _draw_thresh_value(); return;
        }
    }

    void _capture() {
        if (!_have_frame) { ui_feedback::buzz(); return; }
        ui_feedback::tick();
        // Freeze the latest frame: decode once more, then grayscale it.
        _decode->fillSprite(TFT_BLACK);
        _decode->drawJpg(_jpeg, _jpeg_len, 0, 0, CAM_W, CAM_H);
        const uint16_t* px = (const uint16_t*)_decode->getBuffer();
        for (int i = 0; i < CAM_W * CAM_H; ++i) {
            uint16_t p = px[i];
            int r = (p >> 11) & 0x1F, g = (p >> 5) & 0x3F, b = p & 0x1F;
            _gray[i] = (uint8_t)(((r * 255 / 31) * 77 + (g * 255 / 63) * 151 +
                                  (b * 255 / 31) * 28) >> 8);
        }
        if (!ss::extract_silhouette(_gray, CAM_W, CAM_H, _manual_thresh, _sil)) {
            ui_feedback::buzz();
            M5.Display.fillRect(PV_X, _pv_y() + PV_H2 + 40, 700, 30, COL_BG);
            M5.Display.setTextColor(TFT_RED, COL_BG);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(PV_X, _pv_y() + PV_H2 + 44);
            M5.Display.print(_sil.error ? _sil.error : "Extraction failed");
            return;
        }
        ui_feedback::buzz2();
        _hist.clear();
        _hist.push_back(_sil.pts);
        _rot_deg = 0; _scale = 1.0f;
        _state = REVIEW;
        _need_redraw = true;
        _reset_touch();
    }

    // ================= REVIEW =================
    // Outline in a 560x560 box on the left, tools on the right.
    static const int RV_X = 40, RV_S = 560;
    int _rv_y() const { return BAR_H + 30; }

    ss::Poly _edited() const {
        return ss::transform(_hist.back(), _rot_deg * (float)M_PI / 180.0f, _scale);
    }

    void _draw_review() {
        _need_redraw = false;
        _clear_content();
        int cx = RV_X + RV_S + 60, dw = M5.Display.width();

        M5.Display.drawRect(RV_X - 2, _rv_y() - 2, RV_S + 4, RV_S + 4, COL_DIVIDER);
        ss::Poly poly = _edited();

        // Fit + draw the outline.
        ss::V2 mn, mx;
        ss::bounds(poly, mn, mx);
        float sx = std::max(mx.x - mn.x, 1.0f), sy = std::max(mx.y - mn.y, 1.0f);
        float sc = 0.9f * std::min(RV_S / sx, RV_S / sy);
        float ox = RV_X + (RV_S - sx * sc) * 0.5f, oy = _rv_y() + (RV_S - sy * sc) * 0.5f;
        for (size_t i = 0, n = poly.size(); i < n; ++i) {
            const ss::V2& a = poly[i];
            const ss::V2& b = poly[(i + 1) % n];
            M5.Display.drawLine((int)((a.x - mn.x) * sc + ox), (int)((a.y - mn.y) * sc + oy),
                                (int)((b.x - mn.x) * sc + ox), (int)((b.y - mn.y) * sc + oy),
                                TFT_YELLOW);
        }

        M5.Display.setTextColor(COL_SUBTEXT, COL_BG);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(RV_X, _rv_y() + RV_S + 14);
        M5.Display.printf("%d pts   ~%.0f x %.0f mm   rot %d   scale %d%%",
                          (int)poly.size(), sx / PX_PER_MM * _scale,
                          sy / PX_PER_MM * _scale, (int)_rot_deg, (int)(_scale * 100));

        int bw = (dw - cx - 24 - 20) / 2, bh = 84, gap = 20;
        _btn(cx,            BAR_H + 40,           bw, bh, "SMOOTH",   COL_BTN);
        _btn(cx + bw + gap, BAR_H + 40,           bw, bh, "UNDO",
             _hist.size() > 1 ? COL_BTN : 0x2104);
        _btn(cx,            BAR_H + 40 + (bh+gap),   bw, bh, "ROT -15",  COL_BTN);
        _btn(cx + bw + gap, BAR_H + 40 + (bh+gap),   bw, bh, "ROT +15",  COL_BTN);
        _btn(cx,            BAR_H + 40 + 2*(bh+gap), bw, bh, "SCALE -",  COL_BTN);
        _btn(cx + bw + gap, BAR_H + 40 + 2*(bh+gap), bw, bh, "SCALE +",  COL_BTN);
        _btn(cx, BAR_H + 40 + 3*(bh+gap) + 20, bw * 2 + gap, 110, "CONFIRM >", 0x0300);
    }

    void _tap_review(int x, int y) {
        int cx = RV_X + RV_S + 60, dw = M5.Display.width();
        int bw = (dw - cx - 24 - 20) / 2, bh = 84, gap = 20;
        bool changed = false;
        if (_in(x, y, cx, BAR_H + 40, bw, bh)) {                       // smooth
            _hist.push_back(ss::chaikin(_hist.back(), 256));
            if (_hist.size() > 7) _hist.erase(_hist.begin() + 1);
            changed = true;
        } else if (_in(x, y, cx + bw + gap, BAR_H + 40, bw, bh)) {     // undo
            if (_hist.size() > 1) { _hist.pop_back(); changed = true; }
        } else if (_in(x, y, cx, BAR_H + 40 + (bh+gap), bw, bh)) {
            _rot_deg -= 15; changed = true;
        } else if (_in(x, y, cx + bw + gap, BAR_H + 40 + (bh+gap), bw, bh)) {
            _rot_deg += 15; changed = true;
        } else if (_in(x, y, cx, BAR_H + 40 + 2*(bh+gap), bw, bh)) {
            _scale = std::max(0.2f, _scale - 0.05f); changed = true;
        } else if (_in(x, y, cx + bw + gap, BAR_H + 40 + 2*(bh+gap), bw, bh)) {
            _scale = std::min(4.0f, _scale + 0.05f); changed = true;
        } else if (_in(x, y, cx, BAR_H + 40 + 3*(bh+gap) + 20, bw * 2 + gap, 110)) {
            _confirm_geometry();
            return;
        }
        if (changed) { ui_feedback::tick(); _need_redraw = true; }
    }

    void _confirm_geometry() {
        ui_feedback::tick();
        // Camera px -> mm: centre on centroid, flip Y (image Y grows down).
        ss::Poly px = _edited();
        ss::V2 c = ss::centroid(px);
        _contour_mm.clear();
        _contour_mm.reserve(px.size());
        for (const auto& v : px)
            _contour_mm.push_back(ss::V2((v.x - c.x) / PX_PER_MM,
                                         -(v.y - c.y) / PX_PER_MM));
        ss::ensure_ccw(_contour_mm);
        _state = THICK;
        _need_redraw = true;
        _reset_touch();
    }

    // ================= THICK =================
    static const int TH_PV_X = 40, TH_PV_W = 540, TH_PV_H = 460;
    int _slider_x() const { return TH_PV_X + TH_PV_W + 70; }
    int _slider_w() const { return M5.Display.width() - _slider_x() - 40; }
    static const int SLIDER_Y_OFF = 330;   // below presets, from BAR_H
    bool _slider_drag = false;

    void _draw_thick() {
        _need_redraw = false;
        _clear_content();
        int cx = _slider_x(), sw = _slider_w();

        _draw_wireframe();

        M5.Display.setTextColor(COL_TEXT, COL_BG);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(cx, BAR_H + 16);
        M5.Display.printf("Thickness: %.3f in  (%.1f mm)", _thick_mm / 25.4f, _thick_mm);

        static const char* labels[4] = {"1/8 in", "1/4 in", "1/2 in", "1 in"};
        int bw = (sw - 3 * 16) / 4;
        for (int i = 0; i < 4; ++i)
            _btn(cx + i * (bw + 16), BAR_H + 60, bw, 90, labels[i], COL_BTN);

        // custom slider: 1..50mm
        M5.Display.setTextColor(COL_SUBTEXT, COL_BG);
        M5.Display.setCursor(cx, BAR_H + SLIDER_Y_OFF - 36);
        M5.Display.print("CUSTOM (drag)  1 - 50 mm");
        _draw_slider();

        _btn(cx, BAR_H + SLIDER_Y_OFF + 90, sw, 120, "EXPORT STL", 0x0300);
    }

    void _draw_slider() {
        int cx = _slider_x(), sw = _slider_w(), sy = BAR_H + SLIDER_Y_OFF;
        M5.Display.fillRect(cx - 4, sy - 24, sw + 8, 52, COL_BG);
        M5.Display.fillRoundRect(cx, sy, sw, 10, 5, COL_BTN);
        int knob = cx + (int)((_thick_mm - 1.0f) / 49.0f * sw);
        M5.Display.fillCircle(knob, sy + 5, 20, 0x0300);
    }

    void _drag_slider() {
        bool touched = M5.Touch.getCount() > 0;
        int sy = BAR_H + SLIDER_Y_OFF;
        if (touched) {
            auto t = M5.Touch.getDetail(0);
            if (!_slider_drag && abs(t.y - (sy + 5)) < 45 &&
                t.x >= _slider_x() - 20 && t.x <= _slider_x() + _slider_w() + 20)
                _slider_drag = true;
            if (_slider_drag) {
                float f = (float)(t.x - _slider_x()) / _slider_w();
                f = std::min(1.0f, std::max(0.0f, f));
                float mm = 1.0f + f * 49.0f;
                if (fabsf(mm - _thick_mm) > 0.15f) {
                    _thick_mm = mm;
                    _draw_slider();
                    M5.Display.fillRect(_slider_x(), BAR_H + 8, _slider_w(), 30, COL_BG);
                    M5.Display.setTextColor(COL_TEXT, COL_BG);
                    M5.Display.setTextSize(2);
                    M5.Display.setCursor(_slider_x(), BAR_H + 16);
                    M5.Display.printf("Thickness: %.3f in  (%.1f mm)",
                                      _thick_mm / 25.4f, _thick_mm);
                }
            }
        } else if (_slider_drag) {
            _slider_drag = false;
            _need_redraw = true;        // refresh wireframe at final thickness
        }
    }

    void _tap_thick(int x, int y) {
        int cx = _slider_x(), sw = _slider_w();
        static const float mm[4] = {3.175f, 6.35f, 12.7f, 25.4f};
        int bw = (sw - 3 * 16) / 4;
        for (int i = 0; i < 4; ++i) {
            if (_in(x, y, cx + i * (bw + 16), BAR_H + 60, bw, 90)) {
                _thick_mm = mm[i];
                ui_feedback::tick();
                _need_redraw = true;
                return;
            }
        }
        if (_in(x, y, cx, BAR_H + SLIDER_Y_OFF + 90, sw, 120)) _export();
    }

    // Isometric wireframe of the extrusion: bottom outline dim, top outline
    // bright, sparse verticals. Straight display draws — cheap and sufficient.
    void _draw_wireframe() {
        int x0 = TH_PV_X, y0 = BAR_H + 20;
        M5.Display.drawRect(x0 - 2, y0 - 2, TH_PV_W + 4, TH_PV_H + 4, COL_DIVIDER);
        const ss::Poly& p = _contour_mm;
        if (p.size() < 3) return;

        auto proj = [&](float x, float y, float z, float& px, float& py) {
            px = x - y * 0.5f;
            py = (x + y) * 0.25f - z;
        };
        float mnx = 1e9f, mny = 1e9f, mxx = -1e9f, mxy = -1e9f;
        for (const auto& v : p) {
            for (float z : {0.0f, _thick_mm}) {
                float qx, qy; proj(v.x, v.y, z, qx, qy);
                mnx = std::min(mnx, qx); mny = std::min(mny, qy);
                mxx = std::max(mxx, qx); mxy = std::max(mxy, qy);
            }
        }
        float sc = 0.85f * std::min(TH_PV_W / std::max(mxx - mnx, 1e-3f),
                                    TH_PV_H / std::max(mxy - mny, 1e-3f));
        float ox = x0 + (TH_PV_W - (mxx - mnx) * sc) * 0.5f - mnx * sc;
        float oy = y0 + (TH_PV_H - (mxy - mny) * sc) * 0.5f - mny * sc;
        auto scr = [&](float x, float y, float z, int& sx, int& sy) {
            float qx, qy; proj(x, y, z, qx, qy);
            sx = (int)(qx * sc + ox); sy = (int)(qy * sc + oy);
        };

        size_t n = p.size(), step = std::max<size_t>(1, n / 40);
        int ax, ay, bx, by;
        for (size_t i = 0; i < n; i += step) {          // verticals
            scr(p[i].x, p[i].y, 0, ax, ay);
            scr(p[i].x, p[i].y, _thick_mm, bx, by);
            M5.Display.drawLine(ax, ay, bx, by, COL_HANDLE);
        }
        for (size_t i = 0; i < n; ++i) {                // bottom, dim
            const ss::V2& a = p[i];
            const ss::V2& b = p[(i + 1) % n];
            scr(a.x, a.y, 0, ax, ay); scr(b.x, b.y, 0, bx, by);
            M5.Display.drawLine(ax, ay, bx, by, COL_HANDLE);
        }
        for (size_t i = 0; i < n; ++i) {                // top, bright
            const ss::V2& a = p[i];
            const ss::V2& b = p[(i + 1) % n];
            scr(a.x, a.y, _thick_mm, ax, ay); scr(b.x, b.y, _thick_mm, bx, by);
            M5.Display.drawLine(ax, ay, bx, by, 0x07E6);
        }
    }

    // ================= EXPORT / DONE =================
    void _export() {
        ui_feedback::tick();
        std::vector<ss::Tri> tris = ss::extrude(_contour_mm, _thick_mm);
        _export_tris = (int)tris.size();
        _export_ok = _export_mesh_ok = false;
        if (!tris.empty() && ss::next_export_name(_export_name, sizeof(_export_name))) {
            _export_ok = ss::export_stl(_export_name, tris);
            if (_export_ok) {
                ss::export_obj(_export_name, tris);              // best-effort
                _export_mesh_ok = ss::export_viewer_mesh(_export_name, tris);
            }
        }
        if (_export_ok) ui_feedback::buzz2(); else ui_feedback::buzz();
        _state = DONE;
        _need_redraw = true;
        _reset_touch();
    }

    void _draw_done() {
        _need_redraw = false;
        _clear_content();
        int dw = M5.Display.width();
        M5.Display.setTextSize(2);
        if (_export_ok) {
            M5.Display.setTextColor(0x07E6, COL_BG);
            M5.Display.setCursor(60, BAR_H + 60);
            M5.Display.print("EXPORT COMPLETE");
            M5.Display.setTextColor(COL_TEXT, COL_BG);
            M5.Display.setCursor(60, BAR_H + 120);
            M5.Display.printf("/shadowscan/%s.stl   (%d triangles, %.1f mm thick)",
                              _export_name, _export_tris, _thick_mm);
            M5.Display.setCursor(60, BAR_H + 160);
            M5.Display.printf("/shadowscan/%s.obj", _export_name);
            M5.Display.setCursor(60, BAR_H + 200);
            if (_export_mesh_ok)
                M5.Display.printf("/models/%s.mesh   -> open it in the 3D Viewer",
                                  _export_name);
            else
                M5.Display.print("(viewer .mesh skipped)");
        } else {
            M5.Display.setTextColor(TFT_RED, COL_BG);
            M5.Display.setCursor(60, BAR_H + 60);
            M5.Display.print("EXPORT FAILED - check SD card");
        }
        _btn(60,  BAR_H + 300, 420, 120, "SCAN ANOTHER", COL_BTN);
        _btn(520, BAR_H + 300, 420, 120, "ADJUST THICKNESS", COL_BTN);
        (void)dw;
    }

    void _tap_done(int x, int y) {
        if (_in(x, y, 60, BAR_H + 300, 420, 120)) {
            ui_feedback::tick();
            _state = LIVE; _need_redraw = true; _reset_touch();
        } else if (_in(x, y, 520, BAR_H + 300, 420, 120)) {
            ui_feedback::tick();
            _state = THICK; _need_redraw = true; _reset_touch();
        }
    }
};
