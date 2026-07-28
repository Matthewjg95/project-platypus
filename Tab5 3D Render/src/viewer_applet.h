#pragma once
#include "applet.h"
#include "ui_constants.h"
#include "mesh.h"
#include "mesh_loader.h"
#include "renderer.h"
#include "joystick_input.h"
#include <SD_MMC.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>

// ============================================================
// viewer_applet.h — 3D Mesh Viewer
//
// Scroll: uses M5GFX scroll() to shift pixel rows, then draws
//   only the newly revealed row. No full redraw on scroll.
//
// Joystick:
//   - Speed proportional to deflection (1x to 3x)
//   - Y axis inverted (push forward = model tilts up)
//   - Short tap (<300ms) = zoom out
//   - Hold (>300ms) = zoom in continuously
//   - Double press = reset position/orientation


// ============================================================

#include "vm3.h"   // VM3 trackball (shared with the Room Scanner mesh view)

static const int BR_ROW_H   = 60;
static const int BR_START_Y = BAR_H + 44;

// Joystick base rotation speed and max multiplier
static constexpr float JOY_ROT_BASE  = 0.06f;  // base speed (tripled)
static constexpr float JOY_ROT_MAX   = 3.0f;   // max multiplier at full deflection
static constexpr float JOY_ZOOM_SPEED = 0.06f;

class ViewerApplet : public Applet {
public:
    const char* name() const override { return "3D Viewer"; }
    const char* icon() const override { return "#"; }

    void on_pause() override {
        // Wait for the display task to finish its current push before the
        // shell draws the Settings panel, so the 3D canvas can't overdraw it.
        if (!_paused && _done_sem) {
            xSemaphoreTake(_done_sem, pdMS_TO_TICKS(200));
            xSemaphoreGive(_done_sem);
        }
        _paused = true;
    }
    void on_resume() override { _paused = false; }

    void on_enter() override {
        if (!_pipeline_ready) _init_pipeline();

        _state         = STATE_FILE_BROWSER;
        _paused        = false;
        _enter_time    = millis();
        _prev_touched  = false;
        _prev_tx = _prev_ty = -1;
        _prev_dist     = -1.0f;
        _was_pinching  = false;
        _skip_rotation_frame = false;

        _scroll_offset     = 0;
        _drawn_offset      = -1;
        _need_full_redraw  = true;
        _need_rows_update  = false;
        _drag_anchor_y     = -1;
        _drag_start_offset = 0;
        _drag_active       = false;
        _drag_total_px     = 0;
        _last_touch_y      = -1;


        if (_mesh_loaded) { _mesh.free_data(); _mesh_loaded = false; }
        _scan_files();
    }

    // Back button: if viewing a model, return to file list.
    // If already in file browser, let shell exit to home screen.
    bool on_back() override {
        if (_state == STATE_VIEWING) {
            // Clear entire content area immediately to prevent 3D canvas
            // remnants (e.g. scrollbar/debug corner) persisting on screen
            int dw = M5.Display.width(), dh = M5.Display.height();
            M5.Display.fillRect(0, BAR_H, dw, dh - BAR_H, COL_BG);

            _state            = STATE_FILE_BROWSER;
            _need_full_redraw = true;
            _prev_touched     = false;
            _prev_tx = _prev_ty = -1;
            // Reset drag state so a residual touch doesn't immediately select a file
            _drag_active      = false;
            _drag_anchor_y    = -1;
            _drag_total_px    = 0;
            _last_touch_y     = -1;
            return true;
        }
        return false;
    }

    void on_exit() override {
        on_pause();
        Serial.println("[viewer] on_exit");
    }

    bool on_update() override {
        _joystick.update();
        if (_state == STATE_FILE_BROWSER) return _update_browser();
        return _update_viewer();
    }

private:
    enum VState { STATE_FILE_BROWSER, STATE_VIEWING };

    // 3D pipeline
    M5Canvas*         _canvas[2]      = {nullptr, nullptr};
    int               _cv_w           = 0, _cv_h = 0;
    int               _cv_render      = 0, _cv_display = 1;
    SemaphoreHandle_t _swap_sem       = nullptr;
    SemaphoreHandle_t _done_sem       = nullptr;
    TaskHandle_t      _disp_task      = nullptr;
    Renderer          _renderer;
    bool              _pipeline_ready = false;

    // Mesh + state
    Mesh        _mesh;
    bool        _mesh_loaded       = false;
    RenderState _render_state;
    RenderState _default_render_state;
    VM3         _rotation          = VM3::identity();
    VM3         _default_rotation;
    VState      _state             = STATE_FILE_BROWSER;
    bool        _paused            = false;
    uint32_t    _enter_time        = 0;

    // File list
    std::vector<std::string> _files;

    // Scroll state
    int  _scroll_offset     = 0;   // PIXELS (matches the scanner's smooth list)
    uint32_t _press_ms      = 0, _last_scr_draw = 0;
    int  _drawn_offset      = -1;
    bool _need_full_redraw  = true;
    bool _need_rows_update  = false;
    int  _drag_anchor_y     = -1;
    int  _drag_start_offset = 0;
    bool _drag_active       = false;
    int  _drag_total_px     = 0;
    int  _last_touch_y      = -1;

    // Touch (viewer)
    int   _prev_tx = -1, _prev_ty = -1;
    bool  _prev_touched = false;
    float _prev_dist    = -1.0f;
    bool  _was_pinching = false;
    bool  _skip_rotation_frame = false;

    // Joystick
    JoystickInput _joystick;
    uint32_t      _btn_press_time  = 0;
    bool          _btn_was_held    = false;

    // ---- Pipeline ------------------------------------------

    void _init_pipeline() {
        _cv_w = M5.Display.width()  / 2;
        _cv_h = M5.Display.height() / 2;
        _cv_render = 0; _cv_display = 1;

        for (int i = 0; i < 2; i++) {
            _canvas[i] = new M5Canvas(&M5.Display);
            _canvas[i]->setColorDepth(16);
            _canvas[i]->createSprite(_cv_w, _cv_h);
        }

        _swap_sem = xSemaphoreCreateBinary();
        _done_sem = xSemaphoreCreateBinary();
        xSemaphoreGive(_done_sem);

        _renderer.begin(_canvas[0], _cv_w, _cv_h, 65536);

        xTaskCreatePinnedToCore(
            _display_task_fn, "viewer_disp",
            8192, this, 2, &_disp_task, 0
        );

        _joystick.begin(0x63);
        _pipeline_ready = true;
        Serial.printf("[viewer] Pipeline %dx%d\n", _cv_w, _cv_h);
    }

    static void _display_task_fn(void* arg) {
        ViewerApplet* self = (ViewerApplet*)arg;
        for (;;) {
            xSemaphoreTake(self->_swap_sem, portMAX_DELAY);
            int ox = (M5.Display.width()  - self->_cv_w) / 2;
            int oy = (M5.Display.height() - self->_cv_h) / 2;
            M5.Display.pushImage(ox, oy, self->_cv_w, self->_cv_h,
                (uint16_t*)self->_canvas[self->_cv_display]->getBuffer());
            xSemaphoreGive(self->_done_sem);
        }
    }

    // ---- File scanning -------------------------------------

    void _scan_files() {
        _files.clear();
        File dir = SD_MMC.open("/models");
        if (!dir) return;
        while (true) {
            File f = dir.openNextFile();
            if (!f) break;
            std::string n = f.name();
            if (n.size() > 5 && n.substr(n.size()-5) == ".mesh")
                _files.push_back(n);
            f.close();
        }
        dir.close();
        std::sort(_files.begin(), _files.end());
        Serial.printf("[viewer] %d mesh files\n", (int)_files.size());
    }

    // ---- Browser row drawing -------------------------------

    int _max_scroll_px(int dh) const {
        int m = (int)_files.size() * BR_ROW_H - (dh - BR_START_Y);
        return m > 0 ? m : 0;
    }

    void _draw_scrollbar(int dw, int dh) {
        int max_px = _max_scroll_px(dh);
        if (max_px <= 0) return;
        int list_h   = dh - BR_START_Y;
        int total_px = (int)_files.size() * BR_ROW_H;
        int bar_len  = max(20, list_h * list_h / total_px);
        int bar_y    = BR_START_Y + (list_h - bar_len) * _scroll_offset / max_px;
        M5.Display.fillRect(dw-8, BR_START_Y, 6, list_h,  0x2945);
        M5.Display.fillRect(dw-8, bar_y,       6, bar_len, COL_HANDLE);
    }

    // Pixel-smooth visible-rows render (same model as the Room Scan list):
    // clip to the list area, draw partially-visible rows at pixel offsets.
    void _do_rows_update() {
        int dw = M5.Display.width(), dh = M5.Display.height();
        M5.Display.startWrite();
        M5.Display.setClipRect(0, BR_START_Y, dw, dh - BR_START_Y);
        M5.Display.fillRect(0, BR_START_Y, dw, dh - BR_START_Y, COL_BG);
        int first = _scroll_offset / BR_ROW_H;
        int y = BR_START_Y - (_scroll_offset % BR_ROW_H);
        for (int idx = first; idx < (int)_files.size() && y < dh;
             ++idx, y += BR_ROW_H) {
            uint16_t bg = (idx%2==0) ? (uint16_t)0x2104 : (uint16_t)0x2945;
            M5.Display.fillRect(0, y, dw, BR_ROW_H-2, bg);
            M5.Display.setTextColor(COL_TEXT);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(16, y+18);
            M5.Display.print(_files[idx].c_str());
            M5.Display.setCursor(dw-24, y+18);
            M5.Display.print(">");
        }
        M5.Display.clearClipRect();
        _draw_scrollbar(dw, dh);
        M5.Display.endWrite();
        _drawn_offset     = _scroll_offset;
        _need_rows_update = false;
    }

    // Full browser redraw — header + rows
    void _do_full_redraw() {
        int dw = M5.Display.width(), dh = M5.Display.height();
        M5.Display.startWrite();
        M5.Display.fillRect(0, BAR_H, dw, dh-BAR_H, COL_BG);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(12, BAR_H+10);
        M5.Display.print("Select a model:");
        M5.Display.drawFastHLine(0, BAR_H+40, dw, COL_DIVIDER);
        M5.Display.endWrite();

        if (_files.empty()) {
            M5.Display.startWrite();
            M5.Display.setTextColor(0x7BEF);
            M5.Display.setTextSize(2);
            M5.Display.setCursor(12, BR_START_Y+10);
            M5.Display.print("No .mesh files found");
            M5.Display.endWrite();
        } else {
            _do_rows_update();
        }
        _need_full_redraw = false;
    }

    // ---- Browser update — absolute scroll model ------------

    bool _update_browser() {
        if (millis() - _enter_time < 400) return true;

        bool touched       = (M5.Touch.getCount() > 0);
        bool just_pressed  = (!_prev_touched && touched);
        bool just_released = (_prev_touched && !touched);

        int cur_tx = -1, cur_ty = -1;
        if (touched) {
            auto t = M5.Touch.getDetail(0);
            cur_tx = t.x; cur_ty = t.y;
            if (cur_ty >= BAR_H) _last_touch_y = cur_ty;
        }

        if (just_pressed && cur_ty >= BAR_H) {
            _drag_anchor_y     = cur_ty;
            _drag_start_offset = _scroll_offset;
            _drag_active       = true;
            _drag_total_px     = 0;
            _press_ms          = millis();
        }

        // pixel-continuous scroll (matches the Room Scan list feel)
        if (touched && _drag_active && cur_ty >= BAR_H) {
            int displacement = _drag_anchor_y - cur_ty;
            if (abs(displacement) > _drag_total_px)
                _drag_total_px = abs(displacement);
            int dh = M5.Display.height();
            int new_off = constrain(_drag_start_offset + displacement,
                                    0, _max_scroll_px(dh));
            if (new_off != _scroll_offset &&
                millis() - _last_scr_draw >= 30) {
                _last_scr_draw    = millis();
                _scroll_offset    = new_off;
                _need_rows_update = true;
            }
        }

        if (just_released) {
            // tap = barely moved AND brief (dropout-drags can't fake this)
            bool was_tap   = _drag_total_px < 15 && (millis() - _press_ms) < 350;
            _drag_active   = false;
            _drag_anchor_y = -1;
            _need_rows_update = true;              // settle final position

            if (was_tap && _last_touch_y >= BR_START_Y) {
                int idx = (_last_touch_y - BR_START_Y + _scroll_offset) / BR_ROW_H;
                if (idx >= 0 && idx < (int)_files.size()) {
                    _load_file(_files[idx]);
                    return true;
                }
            }
        }

        _prev_touched = touched;
        if (touched) { _prev_tx = cur_tx; _prev_ty = cur_ty; }
        return true;
    }

    // ---- Mesh loading --------------------------------------

    void _load_file(const std::string& filename) {
        if (_mesh_loaded) { _mesh.free_data(); _mesh_loaded = false; }

        int dw = M5.Display.width(), dh = M5.Display.height();
        M5.Display.startWrite();
        M5.Display.fillRect(0, BAR_H, dw, dh-BAR_H, TFT_BLACK);
        M5.Display.setTextColor(TFT_WHITE);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(20, dh/2 - 20);
        M5.Display.print("Loading...");
        M5.Display.setTextColor(COL_SUBTEXT);
        M5.Display.setCursor(20, dh/2 + 10);
        M5.Display.print(filename.c_str());
        M5.Display.endWrite();

        std::string path = "/models/" + filename;
        if (!load_mesh(path.c_str(), _mesh)) {
            M5.Display.startWrite();
            M5.Display.setTextColor(TFT_RED);
            M5.Display.setCursor(20, dh/2 + 50);
            M5.Display.print("Load FAILED");
            M5.Display.endWrite();
            delay(1500);
            _need_full_redraw = true;
            return;
        }

        _mesh_loaded          = true;
        _render_state         = fit_to_canvas(_mesh, _cv_h, 0.7f);
        _rotation             = VM3::rot_x(0.4f) * VM3::rot_y(0.6f);
        _default_render_state = _render_state;
        _default_rotation     = _rotation;

        _state               = STATE_VIEWING;
        _prev_touched        = false;
        _prev_tx = _prev_ty  = -1;
        _prev_dist           = -1.0f;
        _was_pinching        = false;
        _skip_rotation_frame = false;

        M5.Display.fillRect(0, BAR_H, dw, dh-BAR_H, TFT_BLACK);
        Serial.printf("[viewer] Loaded %u verts %u faces\n",
                      _mesh.vertex_count, _mesh.face_count);
    }

    // ---- Viewer update -------------------------------------

    bool _update_viewer() {
        if (!_mesh_loaded) return true;

        // ---- Joystick input --------------------------------
        if (_joystick.connected()) {
            // Button press: track when it went down
            if (_joystick.button_pressed()) {
                _btn_press_time = millis();
            }

            // Hold (>300ms) = zoom in continuously
            if (_joystick.button_held() &&
                (millis() - _btn_press_time) > 300) {
                _render_state.cam_z -= JOY_ZOOM_SPEED;
                _render_state.cam_z  = constrain(
                    _render_state.cam_z, 0.5f, 20.0f);
            }

            // Short tap (released before 300ms) = zoom out one step
            bool btn_now = _joystick.button_held();
            if (_btn_was_held && !btn_now &&
                (millis() - _btn_press_time) <= 300) {
                _render_state.cam_z += JOY_ZOOM_SPEED * 6.0f;
                _render_state.cam_z  = constrain(
                    _render_state.cam_z, 0.5f, 20.0f);
            }
            _btn_was_held = btn_now;

            // Axes: speed proportional to deflection (1x to 3x)
            // Use raw normalised values for speed scaling before deadzone
            float jx = _joystick.x();
            float jy = _joystick.y();  // already inverted in joystick_input.h

            if (fabsf(jx) > 0.0f) {
                float speed = JOY_ROT_BASE * (1.0f + (JOY_ROT_MAX - 1.0f) * fabsf(jx));
                _rotation = VM3::rot_y(-jx * speed) * _rotation;
            }
            if (fabsf(jy) > 0.0f) {
                float speed = JOY_ROT_BASE * (1.0f + (JOY_ROT_MAX - 1.0f) * fabsf(jy));
                _rotation = VM3::rot_x(jy * speed) * _rotation;
            }
        }

        // ---- Touch -----------------------------------------
        int count = M5.Touch.getCount();
        if (count >= 2) {
            auto t0 = M5.Touch.getDetail(0);
            auto t1 = M5.Touch.getDetail(1);
            float dx = t0.x-t1.x, dy = t0.y-t1.y;
            float dist = sqrtf(dx*dx+dy*dy);
            if (_was_pinching && _prev_dist > 0.0f) {
                float delta = dist - _prev_dist;
                if (fabsf(delta) > 1.0f) {
                    _render_state.cam_z -= delta * 0.005f;
                    _render_state.cam_z  = constrain(
                        _render_state.cam_z, 0.5f, 20.0f);
                }
            }
            _prev_dist = dist; _was_pinching = true;
            _prev_touched = false; _prev_tx = _prev_ty = -1;

        } else if (count == 1) {
            auto t = M5.Touch.getDetail(0);
            if (t.y < BAR_H) {
                _prev_touched = false; _prev_tx = _prev_ty = -1;
                return true;
            }
            if (_was_pinching) {
                _was_pinching = false; _prev_dist = -1.0f;
                _skip_rotation_frame = true;
                _prev_tx = t.x; _prev_ty = t.y; _prev_touched = true;
                return true;
            }
            if (_skip_rotation_frame) {
                _skip_rotation_frame = false;
                _prev_tx = t.x; _prev_ty = t.y; _prev_touched = true;
                return true;
            }
            if (_prev_touched && _prev_tx >= 0) {
                int rdx = t.x-_prev_tx, rdy = t.y-_prev_ty;
                if (abs(rdx)<150 && abs(rdy)<150) {
                    if (abs(rdx)>3)
                        _rotation = VM3::rot_y(-rdx*0.005f) * _rotation;
                    if (abs(rdy)>3)
                        _rotation = VM3::rot_x( rdy*0.005f) * _rotation;
                }
            }
            _prev_tx = t.x; _prev_ty = t.y; _prev_touched = true;
        } else {
            if (_was_pinching) _skip_rotation_frame = true;
            _was_pinching = false; _prev_dist = -1.0f;
            _prev_touched = false; _prev_tx = _prev_ty = -1;
            bool joy = _joystick.connected() &&
                       (fabsf(_joystick.x())>0.01f ||
                        fabsf(_joystick.y())>0.01f);
            if (!joy) _rotation = VM3::rot_y(0.002f) * _rotation;
        }

        _render_state.rot_x = asinf(-_rotation.m[1][2]);
        _render_state.rot_y = atan2f(_rotation.m[0][2], _rotation.m[2][2]);
        _render_state.rot_z = atan2f(_rotation.m[1][0], _rotation.m[1][1]);
        return true;
    }

    // ---- XYZ indicator -------------------------------------

    void _draw_axis_indicator() {
        const int ox=40, oy=_cv_h-40, len=28;
        float xx,xy,xz,yx,yy,yz,zx,zy,zz;
        _rotation.transform(1,0,0,xx,xy,xz);
        _rotation.transform(0,1,0,yx,yy,yz);
        _rotation.transform(0,0,1,zx,zy,zz);
        int xex=ox+(int)(xx*len),xey=oy-(int)(xy*len);
        int yex=ox+(int)(yx*len),yey=oy-(int)(yy*len);
        int zex=ox+(int)(zx*len),zey=oy-(int)(zy*len);
        _canvas[_cv_render]->drawLine(ox,oy,xex,xey,0xF800);
        _canvas[_cv_render]->drawLine(ox,oy,yex,yey,0x07E0);
        _canvas[_cv_render]->drawLine(ox,oy,zex,zey,0x001F);
        _canvas[_cv_render]->setTextSize(1);
        _canvas[_cv_render]->setTextColor(0xF800);
        _canvas[_cv_render]->setCursor(xex+2,xey-4); _canvas[_cv_render]->print("X");
        _canvas[_cv_render]->setTextColor(0x07E0);
        _canvas[_cv_render]->setCursor(yex+2,yey-4); _canvas[_cv_render]->print("Y");
        _canvas[_cv_render]->setTextColor(0x001F);
        _canvas[_cv_render]->setCursor(zex+2,zey-4); _canvas[_cv_render]->print("Z");
        _canvas[_cv_render]->fillCircle(ox,oy,3,TFT_WHITE);
    }

    // ---- Viewer render -------------------------------------

    void _render_viewer_frame() {
        if (!_mesh_loaded || !_pipeline_ready) return;
        xSemaphoreTake(_done_sem, portMAX_DELAY);
        _renderer.render_frame(_canvas[_cv_render], _mesh,
                                _render_state, TFT_BLACK);
        _draw_axis_indicator();
        if (_joystick.connected()) {
            uint16_t col = _joystick.button_held() ? 0x07E0 : 0x4228;
            _canvas[_cv_render]->fillCircle(_cv_w-8, 8, 5, col);
        }
        _canvas[_cv_render]->setCursor(4,4);
        _canvas[_cv_render]->setTextColor(TFT_GREEN, TFT_BLACK);
        _canvas[_cv_render]->setTextSize(1);
        _canvas[_cv_render]->printf("V:%u F:%u ",
                                     _mesh.vertex_count, _mesh.face_count);
        std::swap(_cv_render, _cv_display);
        xSemaphoreGive(_swap_sem);
    }

public:
    void on_render() override {
        if (_state == STATE_FILE_BROWSER) {
            if (_need_full_redraw)
                _do_full_redraw();
            else if (_need_rows_update)
                _do_rows_update();
        } else {
            if (!_paused) _render_viewer_frame();
        }
    }
};