#pragma once
#include "../applet.h"
#include "../ui_constants.h"
#include "../mesh.h"
#include "../mesh_loader.h"
#include "../renderer.h"
#include "../vm3.h"      // shared trackball, identical feel to the 3D Viewer

#include "uart_receiver.h"
#include "frame_buffer.h"
#include "depth_estimator.h"
#include "room_fitter.h"
#include "object_labels.h"
#include "project_manager.h"
#include "scan_geometry.h"
#include "scan_mesh_writer.h"
#include "rf_survey.h"
#include "room_objdb.h"
#include "../rf_switch.h"
#include "../ui_feedback.h"
#include "../ui_theme.h"
#include "../ui_icons.h"
#include <WiFi.h>
#include "cv/slam_pipeline.h"
#include "cv/surface_recon.h"

#include <M5Unified.h>
#include <M5GFX.h>
#include <vector>
#include <string>
#include <math.h>

// ============================================================
// scanner_applet.h — MeshScan room-scanning applet for M5View.
//
// Flow:  BROWSE  -> pick/scan a room
//        SCAN    -> Unit V streams JPEG+detections over UART; rotate to sweep;
//                   Phase 1 fits a box room + places objects by known size,
//                   Phase 2 runs ORB SfM into a point cloud.
//        VIEW    -> writes the renderer's native .mesh, load_mesh()s it, and
//                   shows it with the existing Renderer + touch orbit.
//
// Storage: /meshscan/buildings/<building>/<room>.mesh on SD_MMC (renderer
// format). A single building "home" is used until building management UI lands.
//
// UART pins: the Unit V is plugged into the Tab5's single Grove port (the
// joystick was removed). That port is GPIO53 (SDA) / GPIO54 (SCL); with no
// I2C device on it, we route Serial1 onto those pads. A straight Grove cable
// connects the two signal lines 1:1, so the camera's TX lands on one of 53/54
// and its RX on the other — if there's no live feed, swap these two numbers.
#ifndef UNITV_UART_RX
#define UNITV_UART_RX 53   // Tab5 Grove SDA pad  <- Unit V TX
#endif
#ifndef UNITV_UART_TX
#define UNITV_UART_TX 54   // Tab5 Grove SCL pad  -> Unit V RX
#endif
#ifndef UNITV_UART_BAUD
#define UNITV_UART_BAUD 460800   // 4x of 115200. 921600 was too fast for the K210 UART.
#endif

class ScannerApplet : public Applet {
public:
    const char* name() const override { return "Room Scan"; }
    const char* icon() const override { return "S"; }

    void on_enter() override {
        if (!_ready) _init();
        _state = BROWSE;
        _need_redraw = true;
        _scanning = false;
        _reset_touch();
        _refresh_rooms();
    }

    void on_exit() override { /* keep pipeline allocated for fast re-entry */ }

    // Clear the content area below the shell bar. Called on every state
    // transition — VIEW only pushes its centred canvas, so without this the
    // previous state's chrome persists around it (survey leftovers, scan HUD).
    void _clear_content() {
        M5.Display.fillRect(0, BAR_H, M5.Display.width(),
                            M5.Display.height() - BAR_H, COL_BG);
    }

    bool on_back() override {
        if (_state == SURVEY) {
            if (_rf_picking && _rf.ssid[0]) {   // cancel picker, keep current AP
                _rf_picking = false; _rf_redraw = true; _reset_touch();
                return true;
            }
            _rf_picking = false;
            _state = VIEW; _reset_touch();      // survey backs out to the 3D view
            _clear_content();
            return true;
        }
        if (_state == VIEW || _state == SCAN || _state == LIVE) {
            if (_scanning) _abort_scan();
            _state = BROWSE; _need_redraw = true;
            _reset_touch();
            return true;
        }
        return false;   // let shell exit to home
    }

    // Settings opened over us: stop drawing so we don't overwrite the panel.
    void on_pause()  override { _paused = true; }
    void on_resume() override { _paused = false; _need_redraw = true; }

    bool on_update() override {
        _uart.update();                         // drain Unit V stream
        // Bring-up diagnostic: 1 Hz UART receiver stats to the serial monitor.
        // frames>0 => Unit V link is good; frames==0 => wrong RX pin / Unit V
        // config pins; crcErr high => data arriving but baud/framing mismatch.
        static uint32_t _dbg = 0;
        if (millis() - _dbg >= 1000) {
            _dbg = millis();
            Serial.printf("[scan] uart frames=%lu detects=%lu crcErr=%lu resync=%lu\n",
                          (unsigned long)_uart.framesOk(), (unsigned long)_uart.detectsOk(),
                          (unsigned long)_uart.crcErrors(), (unsigned long)_uart.resyncs());
        }
        switch (_state) {
            case BROWSE: return _update_browse();
            case SCAN:   return _update_scan();
            case VIEW:   return _update_view();
            case SURVEY: return _update_survey();
            case LIVE:   return true;             // passive: draw-only state
        }
        return true;
    }

    void on_render() override {
        if (_paused) return;        // Settings panel is up; don't overdraw it
        switch (_state) {
            case BROWSE: if (_need_redraw) _draw_browse(); break;
            case SCAN:   _draw_scan();  break;
            case VIEW:   _draw_view();  break;
            case SURVEY: if (_rf_redraw) _draw_survey(); break;
            case LIVE:   _draw_live();  break;
        }
    }

private:
    enum State { BROWSE, SCAN, VIEW, SURVEY, LIVE };
    State _state = BROWSE;
    bool  _paused = false;     // Settings panel open over us

    // ---- subsystems ----
    UartReceiver  _uart;
    FrameBuffer   _fb;
    ProjectManager _pm;
    DepthEstimator _depth{160, 120, 60.0f};
    RoomFitter     _fitter{160, 120};
    ScanGeometry   _geom;
    Renderer       _renderer;
    cv::CameraIntrinsics _K{138.56f, 138.56f, 80.0f, 60.0f};
    cv::SlamPipeline     _slam;

    M5Canvas* _canvas = nullptr;     // render target (VIEW)
    M5Canvas* _decode = nullptr;     // 160x120 JPEG decode + preview
    int _cv_w = 0, _cv_h = 0;
    bool _ready = false;
    bool _phase2 = false;            // toggle SfM; default robust box fit

    // ---- mesh / view ----
    Mesh        _mesh;
    bool        _mesh_loaded = false;
    RenderState _rs;
    VM3 _rotation = VM3::rot_x(0.4f) * VM3::rot_y(0.6f);   // trackball, like the Viewer
    int   _prev_tx = -1, _prev_ty = -1; bool _prev_touch = false;
    bool  _pipeline_ok = true;   // false => an _init() allocation failed

    // Touch tracking is shared by BROWSE/SCAN/VIEW; clear it on every state
    // transition so a tap in one state can't become a phantom drag in the next
    // (e.g. tapping a room used to feed a stale anchor into VIEW's first orbit).
    void _reset_touch() {
        _prev_touch = false; _prev_tx = _prev_ty = -1; _prev_dist = -1.0f;
    }

    // ---- browse ----
    const char* _building = "home";
    std::vector<std::string> _rooms;
    bool _need_redraw = true;

    // ---- scan ----
    bool     _scanning = false;
    uint32_t _scan_start = 0, _last_sample = 0, _last_seq = 0;
    char     _scan_room[48] = {0};
    static const uint32_t SCAN_MS = 20000, SAMPLE_MS = 1000;

    struct AccObj { uint8_t cls; float x,y,z,h; int n; };
    AccObj _acc[64]; int _acc_n = 0;

    // Additive scanning: the room's persistent object database, and whether
    // the current scan ADDS to it (rescan of an open room) or starts fresh.
    RoomObjDB _objdb;
    bool      _additive = false;
    uint8_t _gray[160*120];

    // ====================================================
    void _init() {
        // Render at half resolution and push centred — matches ViewerApplet's
        // proven pipeline (a full-size sprite is large and slower to push).
        _cv_w = M5.Display.width() / 2; _cv_h = M5.Display.height() / 2;
        _canvas = new M5Canvas(&M5.Display); _canvas->setColorDepth(16);
        _decode = new M5Canvas(&M5.Display); _decode->setColorDepth(16);

        // Every begin() below allocates (PSRAM mostly). A silent failure used
        // to surface later as a null-pointer crash mid-scan; check each, flag
        // _pipeline_ok, and let BROWSE show the error + refuse to scan.
        auto req = [this](bool ok, const char* what) {
            if (!ok) { _pipeline_ok = false; Serial.printf("[scanner] %s alloc FAILED\n", what); }
        };
        req(_canvas->createSprite(_cv_w, _cv_h) != nullptr, "render canvas");
        req(_decode->createSprite(160, 120)     != nullptr, "decode sprite");
        if (_decode->getBuffer()) _decode->fillSprite(TFT_BLACK);   // never show uninitialised pixels
        req(_renderer.begin(_canvas, _cv_w, _cv_h, 65536), "renderer");
        req(_fb.begin(),            "frame buffer");
        req(_fitter.begin(),        "room fitter");
        req(_geom.begin(),          "scan geometry");
        req(_slam.begin(_K, 20000), "SfM pipeline");
        if (!_pm.begin()) Serial.println("[scanner] SD project dir unavailable (non-fatal)");
        _pm.createBuilding(_building);

        _uart.begin(Serial1, UNITV_UART_BAUD, UNITV_UART_RX, UNITV_UART_TX);
        _uart.onFrame(_on_frame, this);
        _uart.onDetect(_on_detect, this);
        _ready = true;   // init ran; _pipeline_ok says whether it's usable
        Serial.printf("[scanner] pipeline %s\n", _pipeline_ok ? "ready" : "DEGRADED");
    }

    static void _on_frame(const uint8_t* jpeg, uint16_t len, void* u) {
        ((ScannerApplet*)u)->_fb.pushFrame(jpeg, len);
    }
    static void _on_detect(const DetectPacket& d, void* u) {
        ((ScannerApplet*)u)->_fb.attachDetections(d);
    }

    // ---- BROWSE ----------------------------------------
    void _refresh_rooms() {
        NameList nl; _pm.listRooms(_building, nl);
        _rooms.clear();
        for (uint16_t i = 0; i < nl.count; ++i) _rooms.push_back(nl.items[i]);
        if (_scroll_px > _max_scroll_px()) _scroll_px = _max_scroll_px();
        _need_redraw = true;
    }

    // Browse list scrolling — pixel-continuous drag (not row jumps), partial
    // rows clipped to the list area, scrollbar thumb tracks continuously.
    // A release counts as a tap only if it moved < 15px AND lasted < 350ms
    // (touch dropouts mid-drag used to fire phantom taps / jump the list).
    static const int BR_ROW_H  = 52;
    static const int BR_LIST_Y = BAR_H + 116;      // scan button stays fixed above
    int  _scroll_px = 0;
    int  _drag_anchor = -1, _drag_start_px = 0, _drag_px = 0;
    int  _last_ty = -1, _last_tx = -1;
    bool _drag_active = false;
    uint32_t _press_ms = 0, _last_scroll_draw = 0;

    int _list_h() const { return M5.Display.height() - BR_LIST_Y - 8; }
    int _max_scroll_px() const {
        int m = (int)_rooms.size() * BR_ROW_H - _list_h();
        return m > 0 ? m : 0;
    }

    void _draw_browse() {
        int dw = M5.Display.width(), dh = M5.Display.height();
        M5.Display.startWrite();
        M5.Display.fillRect(0, BAR_H, dw, dh - BAR_H, COL_BG);
        ui_theme::font_button(&M5.Display);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(12, BAR_H + 12);
        M5.Display.printf("Rooms in '%s'  (%d)", _building, (int)_rooms.size());
        if (!_pipeline_ok) {
            ui_theme::font_mono1(&M5.Display);
            M5.Display.setTextColor(TFT_RED);
            M5.Display.setCursor(dw - 340, BAR_H + 14);
            M5.Display.print("LOW MEMORY - scan disabled");
        }
        M5.Display.drawFastHLine(0, BAR_H + 40, dw, COL_DIVIDER);

        // fixed buttons: Scan new room | Live detection viewfinder
        int split = dw * 2 / 3;
        M5.Display.fillRoundRect(12, BAR_H + 52, split - 24, 50, 6, ui_theme::SURFACE_2);
        ui_theme::font_button(&M5.Display);
        M5.Display.setTextColor(COL_TEXT); M5.Display.setCursor(58, BAR_H + 64);
        M5.Display.print("Scan new room");
        ui_icons::radar(&M5.Display, 36, BAR_H + 77, 30, COL_TEXT);
        M5.Display.fillRoundRect(split, BAR_H + 52, dw - split - 12, 50, 6, ui_theme::ACCENT);
        M5.Display.setCursor(split + 48, BAR_H + 64);
        M5.Display.print("Live view");
        ui_icons::cube(&M5.Display, split + 28, BAR_H + 77, 26, COL_TEXT);

        // visible window of rooms, pixel-scrolled; clip so partial rows never
        // bleed into the header/scan-button area
        int lh = _list_h();
        M5.Display.setClipRect(0, BR_LIST_Y, dw, lh);
        int first = _scroll_px / BR_ROW_H;
        int y = BR_LIST_Y - (_scroll_px % BR_ROW_H);
        ui_theme::font_body(&M5.Display);
        for (int idx = first; idx < (int)_rooms.size() && y < BR_LIST_Y + lh;
             ++idx, y += BR_ROW_H) {
            M5.Display.fillRoundRect(12, y, dw - 44, 44, 6, ui_theme::SURFACE);
            M5.Display.setTextColor(COL_TEXT); M5.Display.setCursor(28, y + 12);
            M5.Display.print(_rooms[idx].c_str());
            M5.Display.setCursor(dw - 60, y + 12); M5.Display.print(">");
        }
        M5.Display.clearClipRect();
        ui_theme::font_mono(&M5.Display);
        // continuous scrollbar (only when the list overflows)
        int max_px = _max_scroll_px();
        if (max_px > 0) {
            int total_px = (int)_rooms.size() * BR_ROW_H;
            int bar_h = lh * lh / total_px; if (bar_h < 24) bar_h = 24;
            int bar_y = BR_LIST_Y + (lh - bar_h) * _scroll_px / max_px;
            M5.Display.fillRect(dw - 20, BR_LIST_Y, 8, lh, 0x2945);
            M5.Display.fillRoundRect(dw - 20, bar_y, 8, bar_h, 3, COL_HANDLE);
        }
        // delete-confirmation modal (opened by long-pressing a room row)
        if (_del_idx >= 0 && _del_idx < (int)_rooms.size()) {
            int mx = dw/2 - 220, my = dh/2 - 80;
            M5.Display.fillRoundRect(mx, my, 440, 170, 10, ui_theme::SURFACE);
            M5.Display.drawRoundRect(mx, my, 440, 170, 10, COL_DIVIDER);
            ui_theme::font_button(&M5.Display);
            M5.Display.setTextColor(COL_TEXT);
            M5.Display.setCursor(mx + 66, my + 18);
            M5.Display.printf("Delete '%s'?", _rooms[_del_idx].c_str());
            ui_icons::trash(&M5.Display, mx + 38, my + 32, 30, COL_TEXT);
            ui_theme::font_mono1(&M5.Display);
            M5.Display.setTextColor(COL_SUBTEXT);
            M5.Display.setCursor(mx + 20, my + 60);
            M5.Display.print("Removes the mesh, labels, and RF survey.");
            ui_theme::font_button(&M5.Display);
            M5.Display.fillRoundRect(mx + 20, my + 90, 190, 60, 6, ui_theme::DANGER);
            M5.Display.setTextColor(TFT_WHITE);
            M5.Display.setCursor(mx + 60, my + 106); M5.Display.print("Delete");
            M5.Display.fillRoundRect(mx + 230, my + 90, 190, 60, 6, ui_theme::SURFACE_2);
            M5.Display.setCursor(mx + 278, my + 106); M5.Display.print("cancel");
            ui_theme::font_mono(&M5.Display);
        }
        M5.Display.endWrite();
        _need_redraw = false;
    }

    // Long-press-to-delete: index of the room awaiting confirmation, or -1.
    int _del_idx = -1;

    bool _update_browse() {
        bool touched = M5.Touch.getCount() > 0;
        int tx = -1, ty = -1;
        if (touched) { auto t = M5.Touch.getDetail(0); tx = t.x; ty = t.y; }

        // delete-confirmation modal swallows all input while open
        if (_del_idx >= 0) {
            if (touched && !_prev_touch) {
                int dw = M5.Display.width(), dh = M5.Display.height();
                int mx = dw/2 - 220, my = dh/2 - 80;
                if (ty >= my + 90 && ty < my + 150) {
                    if (tx >= mx + 20 && tx < mx + 210) {          // DELETE
                        if (_del_idx < (int)_rooms.size())
                            _pm.deleteRoom(_building, _rooms[_del_idx].c_str());
                        ui_feedback::buzz();
                        _refresh_rooms();
                    } else ui_feedback::tick();                    // cancel
                    _del_idx = -1; _need_redraw = true;            // either btn closes
                }
            }
            _prev_touch = touched;
            return true;
        }

        // long-press on a row (held >700ms, barely moved) opens the modal
        if (touched && _drag_active && _drag_px < 15 &&
            millis() - _press_ms > 700 && _last_ty >= BR_LIST_Y) {
            int idx = (_scroll_px + (_last_ty - BR_LIST_Y)) / BR_ROW_H;
            if (idx >= 0 && idx < (int)_rooms.size()) {
                ui_feedback::buzz();               // long-press recognized
                _del_idx = idx; _drag_active = false; _need_redraw = true;
                _prev_touch = touched;
                return true;
            }
        }

        // press edge: anchor a potential drag
        if (touched && !_prev_touch && ty >= BAR_H) {
            _drag_anchor = ty; _drag_start_px = _scroll_px;
            _drag_active = true; _drag_px = 0; _last_ty = ty; _last_tx = tx;
            _press_ms = millis();
        }
        // move: pixel-continuous scroll, redraw throttled to ~30ms
        if (touched && _drag_active) {
            int disp = _drag_anchor - ty;
            if (abs(disp) > _drag_px) _drag_px = abs(disp);
            _last_ty = ty; _last_tx = tx;
            int px = _drag_start_px + disp;
            int max_px = _max_scroll_px();
            if (px < 0) px = 0; if (px > max_px) px = max_px;
            if (px != _scroll_px) {
                _scroll_px = px;
                if (millis() - _last_scroll_draw >= 30) {
                    _last_scroll_draw = millis();
                    _need_redraw = true;
                }
            }
        }
        // release: tap = barely moved AND brief (dropout-drag can't fake this)
        if (!touched && _prev_touch && _drag_active) {
            _drag_active = false;
            _need_redraw = true;                       // settle final position
            bool tap = _drag_px < 15 && (millis() - _press_ms) < 350;
            if (tap && _last_ty >= 0) {
                int dw_ = M5.Display.width(), split_ = dw_ * 2 / 3;
                if (_last_ty >= BAR_H + 52 && _last_ty < BAR_H + 102) {
                    ui_feedback::tick();
                    if (_last_tx >= split_) {
                        ui_theme::press_flash(split_, BAR_H + 52, dw_ - split_ - 12, 50);
                        _enter_live();
                    } else if (_pipeline_ok) {
                        ui_theme::press_flash(12, BAR_H + 52, split_ - 24, 50);
                        _start_scan();
                    }
                } else if (_last_ty >= BR_LIST_Y) {
                    int idx = (_scroll_px + (_last_ty - BR_LIST_Y)) / BR_ROW_H;
                    if (idx >= 0 && idx < (int)_rooms.size()) {
                        ui_feedback::tick();
                        int ry = BR_LIST_Y + idx * BR_ROW_H - _scroll_px;
                        ui_theme::press_flash(12, ry, dw_ - 44, 44);
                        _open_room(_rooms[idx].c_str());
                    }
                }
            }
        }
        _prev_touch = touched;
        return true;
    }

    // ---- SCAN ------------------------------------------
    // IMU-measured yaw: the camera is tethered to the tablet, so the Tab5's
    // BMI270 gyro measures the sweep directly. Projecting the gyro vector onto
    // gravity (from the accelerometer) gives the world-vertical rotation rate
    // regardless of how the tablet is held. Replaces the old assumption of a
    // perfectly uniform 20-second rotation — the largest placement-error
    // source. Flip YAW_SIGN to -1 if scanned rooms come out mirrored.
    static constexpr float    YAW_SIGN   = 1.0f;
    static const     uint32_t BIAS_MS    = 900;   // hold-still gyro bias window
    static const     uint32_t PREVIEW_MS = 400;   // live preview decode cadence
    // No scan timeout: a scan runs until a measured 360-degree sweep or the
    // on-screen FINISH button. (Shell back = abort without saving.)

    float    _yaw_rad = 0, _yaw_bias_dps = 0;
    float    _bias_sum = 0; uint32_t _bias_n = 0;
    uint32_t _imu_last_ms = 0, _last_decode_ms = 0, _last_decode_seq = 0;

    // ---- sweep dial: progress ring + detection pips --------------------
    static const int RING_R0 = 215, RING_R1 = 228;
    int _ring_deg = 0;                    // arc already drawn (degrees)
    struct Pip { float deg; uint8_t cls; };
    Pip _pips[64]; int _pip_n = 0;

    void _ring_center(int& cx, int& cy) {
        cx = M5.Display.width() / 2;
        cy = BAR_H + 240;
    }

    void _start_scan() { _start_scan_impl(false); }

    // Rescan the currently open room: new observations are registered against
    // the room's object database (yaw+translation solved from the objects
    // themselves) and merged, so the model improves instead of resetting.
    void _start_rescan() {
        if (!_mesh_loaded) return;
        _objdb.loadBeside(_mesh_path);        // may be empty for pre-DB rooms
        _start_scan_impl(true);
    }

    void _start_scan_impl(bool additive) {
        _additive = additive;
        if (!additive) {
            _pm.suggestRoomName(_building, _scan_room, sizeof(_scan_room));
            _objdb.clear();
        }
        _acc_n = 0; _fitter.reset(); _slam.reset();
        _fb.invalidateAll();          // never sample a previous scan's frame
        _scanning = true; _scan_start = millis(); _last_sample = 0; _last_seq = 0;
        _yaw_rad = 0; _yaw_bias_dps = 0; _bias_sum = 0; _bias_n = 0;
        _imu_last_ms = millis(); _last_decode_ms = 0; _last_decode_seq = 0;
        _state = SCAN;
        _reset_touch();
        _decode->fillSprite(TFT_BLACK);   // blank until the first frame decodes
        M5.Display.fillRect(0, BAR_H, M5.Display.width(), M5.Display.height()-BAR_H, TFT_BLACK);
        // FINISH button (static chrome; ends the scan and builds the mesh)
        int dh = M5.Display.height();
        M5.Display.fillRoundRect(12, dh - 70, 220, 56, 8, ui_theme::SURFACE_2);
        ui_theme::font_button(&M5.Display);
        M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(40, dh - 56); M5.Display.print("Finish scan");
        ui_theme::font_mono(&M5.Display);
        // sweep-dial ring track (progress arc fills it as you rotate)
        _ring_deg = 0; _pip_n = 0;
        int cx, cy; _ring_center(cx, cy);
        M5.Display.fillArc(cx, cy, RING_R0, RING_R1, 0, 360, ui_theme::SURFACE);
    }
    void _abort_scan() { _scanning = false; }

    // World-vertical rotation rate in deg/s: gyro projected onto gravity.
    float _yaw_rate_dps() {
        M5.Imu.update();
        float gx, gy, gz, ax, ay, az;
        M5.Imu.getGyro(&gx, &gy, &gz);
        M5.Imu.getAccel(&ax, &ay, &az);
        float an = sqrtf(ax*ax + ay*ay + az*az);
        if (an < 0.5f) return 0.0f;       // free-fall/garbage guard
        return (gx*ax + gy*ay + gz*az) / an;
    }

    void _imu_tick(uint32_t now, uint32_t el) {
        float dt = (now - _imu_last_ms) * 0.001f;
        _imu_last_ms = now;
        if (dt <= 0 || dt > 0.25f) return;            // skip stalls/jumps
        float rate = _yaw_rate_dps();
        if (el < BIAS_MS) {                            // hold-still calibration
            _bias_sum += rate; _bias_n++;
            if (_bias_n) _yaw_bias_dps = _bias_sum / _bias_n;
            return;
        }
        _yaw_rad += YAW_SIGN * (rate - _yaw_bias_dps) * (float)M_PI / 180.0f * dt;
    }

    void _acc_add(uint8_t cls, const ObjectEstimate& e) {
        for (int i = 0; i < _acc_n; ++i)
            if (_acc[i].cls == cls) {
                float dx=_acc[i].x-e.x, dz=_acc[i].z-e.z;
                if (dx*dx+dz*dz <= 0.36f) {
                    int n=_acc[i].n;
                    _acc[i].x=(_acc[i].x*n+e.x)/(n+1); _acc[i].y=(_acc[i].y*n+e.y)/(n+1);
                    _acc[i].z=(_acc[i].z*n+e.z)/(n+1); _acc[i].h=(_acc[i].h*n+e.real_height)/(n+1);
                    _acc[i].n=n+1; return;
                }
            }
        if (_acc_n < 64) _acc[_acc_n++] = { cls, e.x, e.y, e.z, e.real_height, 1 };
    }

    bool _preview_dirty = false;   // push the rotated preview only when it changed

    // ---- detection display policy (pure logic, portable) -----------------
    // "Confident and discerning": a detection is shown only if it clears a
    // confidence floor AND its class was also present in the previous
    // inference (~300ms persistence) — kills one-frame flicker.
    static const uint8_t DISPLAY_CONF = 22;
    uint32_t _seen_prev = 0, _seen_cur = 0, _seen_seq = 0;

    void _det_note_frame(const FrameSlot* s) {
        if (s->seq == _seen_seq) return;
        _seen_seq = s->seq;
        _seen_prev = _seen_cur;
        _seen_cur = 0;
        for (uint8_t i = 0; i < s->det_count; ++i)
            if (s->dets[i].class_id < 20 && s->dets[i].confidence >= DISPLAY_CONF)
                _seen_cur |= (1u << s->dets[i].class_id);
    }
    bool _det_display(const Detection& d) const {
        return d.class_id < 20 && d.confidence >= DISPLAY_CONF &&
               ((_seen_prev >> d.class_id) & 1);
    }

    static uint16_t _cls565(uint8_t cls) {
        uint32_t col = object_labels::color(cls);
        return ((col >> 8) & 0xF800) | ((col >> 5) & 0x07E0) | ((col >> 3) & 0x001F);
    }

    // draw display-worthy detection boxes onto the decode sprite
    void _draw_dets_on_decode(const FrameSlot* s) {
        for (uint8_t i = 0; i < s->det_count; ++i) {
            const Detection& d = s->dets[i];
            if (!_det_display(d)) continue;
            _decode->drawRect(d.x / 2, d.y / 2, d.w / 2, d.h / 2, _cls565(d.class_id));
        }
    }

    // Decode-only path for the live preview (no grayscale / fitter work).
    void _decode_slot(const FrameSlot* slot) {
        _decode->fillSprite(TFT_BLACK);
        _decode->drawJpg(slot->jpeg, slot->jpeg_len, 0, 0, 160, 120, 0, 0, 0.5f, 0.5f);
        _preview_dirty = true;
        _last_decode_seq = slot->seq;
    }

    void _sample(const FrameSlot* slot, float yaw) {
        // decode the JPEG to 160x120 via M5GFX, derive grayscale
        _decode->fillSprite(TFT_BLACK);
        _decode->drawJpg(slot->jpeg, slot->jpeg_len, 0, 0, 160, 120, 0, 0, 0.5f, 0.5f);
        _preview_dirty = true;
        _last_decode_seq = slot->seq;
        const uint16_t* px = (const uint16_t*)_decode->getBuffer();
        for (int i = 0; i < 160*120; ++i) {
            uint16_t p = px[i];
            int r=(p>>11)&0x1F, g=(p>>5)&0x3F, b=p&0x1F;
            _gray[i] = (uint8_t)(((r*255/31)*77 + (g*255/63)*151 + (b*255/31)*28) >> 8);
        }
        if (_phase2) _slam.addFrame(_gray, 160, 120);
        else         _fitter.addFrame(_gray, yaw);

        // detection boxes on the preview (drawn AFTER the grayscale extract
        // so the fitter never sees the overlay pixels)
        _det_note_frame(slot);
        _draw_dets_on_decode(slot);

        int before = _acc_n;
        for (uint8_t i = 0; i < slot->det_count; ++i) {
            const Detection& d = slot->dets[i];
            if (!_depth.isKnown(d.class_id)) continue;
            ObjectEstimate e = _depth.estimate(d.class_id, d.x/2, d.w/2, d.h/2, yaw);
            if (e.valid) _acc_add(d.class_id, e);
        }
        // dial pip for every NEWLY discovered object at the current bearing
        float deg = fabsf(yaw) * 180.0f / (float)M_PI;
        for (int i = before; i < _acc_n && _pip_n < 64; ++i)
            _pips[_pip_n++] = { deg, _acc[i].cls };
    }

    bool _update_scan() {
        if (!_scanning) return true;
        uint32_t now = millis(), el = now - _scan_start;
        _imu_tick(now, el);
        // Finish on a completed 360-degree sweep (measured, not assumed) or
        // when the user taps FINISH. No timeout — the sweep is theirs to pace.
        bool full_turn = fabsf(_yaw_rad) >= 2.0f * (float)M_PI;
        if (full_turn && el > 5000) { ui_feedback::buzz2(); _finish_scan(); return true; }
        if (M5.Touch.getCount() > 0) {
            if (!_prev_touch) {
                _prev_touch = true;
                auto t = M5.Touch.getDetail(0);
                int dh = M5.Display.height();
                if (t.x >= 12 && t.x < 232 && t.y >= dh - 70 && t.y < dh - 14) {
                    ui_feedback::buzz();
                    ui_theme::press_flash(12, dh - 70, 220, 56);
                    _finish_scan();
                    return true;
                }
            }
        } else _prev_touch = false;

        // live preview: decode newly arrived frames between samples,
        // WITH detection boxes (user request: live feed inside the scan too)
        if (now - _last_decode_ms >= PREVIEW_MS) {
            const FrameSlot* p = _fb.latest();
            if (p && p->seq != _last_decode_seq) {
                _decode_slot(p);
                _det_note_frame(p);
                _draw_dets_on_decode(p);
                _last_decode_ms = now;
            }
        }

        if (now - _last_sample >= SAMPLE_MS) {
            const FrameSlot* s = _fb.latest();
            // scan-path diagnostic: log the gate decision at most once/sec
            static uint32_t _g = 0;
            bool fresh = s && s->seq != _last_seq;
            if (now - _g >= 1000) {
                _g = now;
                Serial.printf("[gate] t=%lus slot=%s seq=%lu last=%lu dets=%u len=%u -> %s\n",
                              (unsigned long)(el/1000), s ? "yes" : "NULL",
                              s ? (unsigned long)s->seq : 0, (unsigned long)_last_seq,
                              s ? s->det_count : 0, s ? s->jpeg_len : 0,
                              fresh ? "SAMPLE" : "skip");
            }
            if (fresh) {
                _last_seq = s->seq;
                uint32_t t0 = millis();
                _sample(s, _yaw_rad);       // measured bearing, not a timer guess
                Serial.printf("[gate] sample took %lums (total %d)\n",
                              (unsigned long)(millis() - t0),
                              _phase2 ? _slam.frames() : _fitter.framesProcessed());
                _last_sample = now;
            }
        }
        return true;
    }

    // The Unit V sensor is mounted rotated ~90 deg CCW relative to how the
    // Tab5 is held, so the preview is pushed rotated +90 (CW) to compensate.
    // Display-only: the fitter/SLAM grayscale stays in sensor orientation.
    // If your unit reads upside-down instead, change to 270.
    static constexpr float PREVIEW_ROT_DEG = 90.0f;

    void _draw_scan() {
        int dw = M5.Display.width();
        // live preview: rotated push, centred near top (120x160 footprint).
        // Only when a new frame decoded — pushRotateZoom every render pass
        // starved the UART drain loop and contributed to RX overflow.
        int cx, cy; _ring_center(cx, cy);
        if (_preview_dirty) {
            _preview_dirty = false;
            // 2x preview centred inside the sweep dial
            _decode->pushRotateZoom(&M5.Display, cx, cy,
                                    PREVIEW_ROT_DEG, 2.0f, 2.0f);
        }
        uint32_t el = millis() - _scan_start;
        float deg = fabsf(_yaw_rad) * 180.0f / (float)M_PI;
        if (deg > 360.0f) deg = 360.0f;

        // incremental progress arc: 12 o'clock, clockwise, accent-filled.
        // Drawing only the new slice avoids flicker on the ring.
        if ((int)deg > _ring_deg) {
            float a0 = 270.0f + _ring_deg, a1 = 270.0f + deg;
            if (a1 <= 360.0f)
                M5.Display.fillArc(cx, cy, RING_R0, RING_R1, a0, a1, ui_theme::ACCENT);
            else if (a0 >= 360.0f)
                M5.Display.fillArc(cx, cy, RING_R0, RING_R1, a0 - 360.0f, a1 - 360.0f, ui_theme::ACCENT);
            else {
                M5.Display.fillArc(cx, cy, RING_R0, RING_R1, a0, 360.0f, ui_theme::ACCENT);
                M5.Display.fillArc(cx, cy, RING_R0, RING_R1, 0.0f, a1 - 360.0f, ui_theme::ACCENT);
            }
            _ring_deg = (int)deg;
        }
        // detection pips on the ring at the bearing each object was found
        for (int i = 0; i < _pip_n; ++i) {
            float a = (270.0f + _pips[i].deg) * (float)M_PI / 180.0f;
            int px = cx + (int)(cosf(a) * (RING_R0 - 12));
            int py = cy + (int)(sinf(a) * (RING_R0 - 12));
            M5.Display.fillCircle(px, py, 6, _cls565(_pips[i].cls));
        }

        int pct = (int)(100.0f * deg / 360.0f); if (pct > 100) pct = 100;
        M5.Display.setTextColor(COL_TEXT, COL_BG); M5.Display.setTextSize(2);
        M5.Display.setCursor(12, BAR_H + 490);
        if (el < BIAS_MS)
            M5.Display.print("Face a corner, hold still...");
        else
            M5.Display.printf("Sweep %3d\xF8 %d%% (360=done) ", (int)deg, pct);
        M5.Display.setCursor(12, BAR_H + 520);
        M5.Display.printf("%s objs:%d  frames:%d   ",
                          _phase2 ? "SfM" : "box", _acc_n,
                          _phase2 ? _slam.frames() : _fitter.framesProcessed());
    }

    // ---- finish: build geometry, write renderer format, view ----
    void _finish_scan() {
        _scanning = false;
        char path[200];
        if (!_pm.meshPath(_building, _scan_room, path, sizeof(path))) { _state = BROWSE; _need_redraw = true; return; }
        bool ok = _phase2 ? _write_phase2(path) : _write_phase1(path);
        {   // save-verification log for the "SCAN+ didn't save?" question
            File chk = SD_MMC.open(path, FILE_READ);
            Serial.printf("[scanner] wrote %s -> %s (%u bytes)\n", path,
                          ok ? "ok" : "FAILED",
                          chk ? (unsigned)chk.size() : 0u);
            if (chk) chk.close();
        }
        if (ok && load_mesh(path, _mesh)) {
            strncpy(_mesh_path, path, sizeof(_mesh_path)-1); _mesh_path[sizeof(_mesh_path)-1]='\0';
            _mesh_loaded = true;
            _rs = fit_to_canvas(_mesh, _cv_h, 0.7f);
            _rotation = VM3::rot_x(0.4f) * VM3::rot_y(0.6f);
            _state = VIEW;
            _clear_content();             // wipe scan HUD from around the canvas
        } else {
            Serial.println("[scanner] finish failed");
            _state = BROWSE; _need_redraw = true;
        }
        _reset_touch();
        _refresh_rooms();
    }

    // An accumulated object must be observed at least this many times to earn
    // a marker — kills one-off false positives at the low detect threshold.
    static const int MIN_OBSERVATIONS = 2;

    bool _write_phase1(const char* path) {
        // ---- resolve this scan's observations into the object database ----
        if (_additive && _acc_n > 0) {
            static RoomObj nw[64];
            for (int i = 0; i < _acc_n; ++i)
                nw[i] = { _acc[i].cls, _acc[i].x, _acc[i].y, _acc[i].z,
                          _acc[i].h, (uint16_t)_acc[i].n };
            RoomObjDB::Transform T = _objdb.registerScan(nw, _acc_n);
            Serial.printf("[scanner] rescan registration: %d landmark matches, "
                          "rot=(%.2f,%.2f) t=(%.2f,%.2f)\n",
                          T.matches, T.cosr, T.sinr, T.tx, T.tz);
            for (int i = 0; i < _acc_n; ++i) {
                float x = nw[i].x, z = nw[i].z;
                RoomObjDB::apply(T, x, z);
                _objdb.merge(nw[i].cls, x, nw[i].y, z, nw[i].h, nw[i].n);
            }
        } else {
            for (int i = 0; i < _acc_n; ++i)
                _objdb.merge(_acc[i].cls, _acc[i].x, _acc[i].y, _acc[i].z,
                             _acc[i].h, (uint16_t)_acc[i].n);
        }

        // ---- geometry + labels come from the database (n-confirmed) -------
        // Display policy: LAYOUT furniture (chair/table/sofa/tv) + LIVING
        // things (person/dog/cat — the fun ones) render; small clutter
        // (bottles etc.) is hidden by default until the per-room object menu
        // exists. The floor plate sizes to contain everything DISPLAYED, so
        // markers can never float outside the room — while hidden clutter
        // can't stretch the layout.
        auto show_cls = [](uint8_t c) {
            return c == 8 || c == 10 || c == 17 || c == 19    // layout
                || c == 14 || c == 11 || c == 7;              // person, dog, cat
        };
        for (int i = 0; i < _objdb.count; ++i)
            if (_objdb.objs[i].n >= MIN_OBSERVATIONS && show_cls(_objdb.objs[i].cls))
                _fitter.addObjectExtent(_objdb.objs[i].x, _objdb.objs[i].z);
        _fitter.addObjectExtent(0.0f, 0.0f);   // the plate always covers the origin
        RoomBox box = _fitter.fit();
        _geom.reset();
        // Floor plate only — no presumptive walls/ceiling (we can't localize
        // walls monocularly; Phase 2 will place evidence-based thin walls).
        _geom.addFloorPlate(box.cx, box.cz, box.width, box.depth);
        _geom.addOriginArrow();                // where the (first) scan began

        struct LblTmp { float x, y, z; uint8_t cls; };
        static LblTmp tmp[64];        // static: keep ~832B off the loop stack
        int tn = 0;
        for (int i = 0; i < _objdb.count; ++i) {
            const RoomObj& o = _objdb.objs[i];
            if (o.n < MIN_OBSERVATIONS || !show_cls(o.cls)) continue;
            uint32_t col = object_labels::color(o.cls);
            float hy = o.h * 0.5f;
            _geom.addObjectMarker(object_labels::name(o.cls),
                                  o.x, hy, o.z, 0.25f, hy, 0.25f,
                                  object_labels::red(col), object_labels::green(col),
                                  object_labels::blue(col));
            if (tn < 64) tmp[tn++] = { o.x, o.h, o.z, o.cls };
        }
        Serial.printf("[scanner] db: %d objects (%d confirmed) after %s\n",
                      _objdb.count, tn, _additive ? "rescan" : "scan");
        if (!_objdb.saveBeside(path))
            Serial.printf("[scanner] WARN: .objs save FAILED for %s\n", path);

        float c[3], s; ScanMeshWriter::quantisation(_geom.bounds(), c, s);
        static ScanMeshWriter w;      // static: its 4KB buffer overflowed the
                                      // 8KB loop stack (intermittent finish crash)
        if (!w.begin(path, c, s)) return false;
        const float* V = _geom.vertices();
        for (uint32_t i = 0; i < _geom.vertexCount(); ++i)
            w.addVertexModel(V[i*3], V[i*3+1], V[i*3+2]);
        const uint16_t* F = _geom.faces();
        for (uint32_t i = 0; i < _geom.faceCount(); ++i)
            w.addFace(F[i*3], F[i*3+1], F[i*3+2]);
        if (!w.finish()) return false;

        // Label anchors (marker tops), quantised into the mesh's int16 space
        // so the viewer projects them with zero knowledge of the scan frame.
        _label_n = 0;
        {   // "start" tag at the origin arrow (pseudo-class 200)
            RoomLabel& L = _labels[_label_n++];
            L.x = (int16_t)((0.0f - c[0]) * s);
            L.y = (int16_t)(-(0.30f - c[1]) * s);
            L.z = (int16_t)((0.0f - c[2]) * s);
            L.cls = 200;
        }
        for (int i = 0; i < tn && _label_n < 64; ++i) {
            RoomLabel& L = _labels[_label_n++];
            L.x = (int16_t)((tmp[i].x - c[0]) * s);
            L.y = (int16_t)(-(tmp[i].y - c[1]) * s);   // mirror Y like addVertexModel
            L.z = (int16_t)((tmp[i].z - c[2]) * s);
            L.cls = tmp[i].cls;
        }
        _write_labels(path);
        return true;
    }

    // ---- label sidecar: <room>.lbl beside <room>.mesh --------------------
    // cls 200 = the origin/"start" pseudo-class (not a VOC id)
    static const char* _label_name(uint8_t cls) {
        return cls == 200 ? "start" : object_labels::name(cls);
    }
    static uint16_t _label_color(uint8_t cls) {
        if (cls == 200) return TFT_WHITE;
        uint32_t col = object_labels::color(cls);
        return ((col >> 8) & 0xF800) | ((col >> 5) & 0x07E0) | ((col >> 3) & 0x001F);
    }
    struct RoomLabel { int16_t x, y, z; uint8_t cls; };
    RoomLabel _labels[64]; int _label_n = 0;

    void _lbl_path(const char* mesh_path, char* out, size_t n) {
        strncpy(out, mesh_path, n - 1); out[n - 1] = '\0';
        char* dot = strrchr(out, '.');
        if (dot && (size_t)(dot - out) + 5 < n) strcpy(dot, ".lbl");
    }
    void _write_labels(const char* mesh_path) {
        char p[208]; _lbl_path(mesh_path, p, sizeof(p));
        File f = SD_MMC.open(p, FILE_WRITE);
        if (!f) return;
        f.write((const uint8_t*)"LBL1", 4);
        uint8_t n = (uint8_t)_label_n;
        f.write(&n, 1);
        for (int i = 0; i < _label_n; ++i)
            f.write((const uint8_t*)&_labels[i], sizeof(RoomLabel));
        f.close();
    }
    void _read_labels(const char* mesh_path) {
        _label_n = 0;
        char p[208]; _lbl_path(mesh_path, p, sizeof(p));
        File f = SD_MMC.open(p, FILE_READ);
        if (!f) return;
        char magic[4];
        if (f.read((uint8_t*)magic, 4) == 4 && memcmp(magic, "LBL1", 4) == 0) {
            uint8_t n = 0; f.read(&n, 1);
            if (n > 64) n = 64;
            for (uint8_t i = 0; i < n; ++i)
                if (f.read((uint8_t*)&_labels[i], sizeof(RoomLabel)) == sizeof(RoomLabel))
                    _label_n = i + 1;
        }
        f.close();
    }

    // streamed SfM cloud -> greedy surface -> renderer format
    static void _face_sink(uint16_t a, uint16_t b, uint16_t c, void* u) {
        ((ScanMeshWriter*)u)->addFace(a, b, c);
    }
    bool _write_phase2(const char* path) {
        _label_n = 0;                     // no object labels in the SfM path yet
        _write_labels(path);
        const cv::PointCloud& pc = _slam.cloud();
        if (pc.count < 16) { Serial.println("[scanner] phase2: too few points"); return false; }
        float c[3], s; ScanMeshWriter::quantisation(pc.bounds, c, s);
        static ScanMeshWriter w;      // static: 4KB buffer off the loop stack
        if (!w.begin(path, c, s)) return false;
        uint32_t vmax = pc.count > 65000 ? 65000 : pc.count;
        for (uint32_t i = 0; i < vmax; ++i)
            w.addVertexModel(pc.xyz[i*3], pc.xyz[i*3+1], pc.xyz[i*3+2]);
        cv::SurfaceReconParams prm; prm.max_edge = 0.5f; prm.k = 8; prm.max_faces = 20000;
        cv::greedy_triangulate(pc, prm, _face_sink, &w);
        return w.finish();
    }

    // ---- VIEW ------------------------------------------
    bool _update_view() {
        int n = M5.Touch.getCount();
        if (n == 1) {
            auto t = M5.Touch.getDetail(0);
            // [RF] button hit-test on the press edge (canvas top-right)
            if (!_prev_touch) {
                int ox = (M5.Display.width() - _cv_w) / 2;
                int oy = (M5.Display.height() - _cv_h) / 2;
                if (t.x >= ox + _cv_w - 104 && t.x < ox + _cv_w - 4 &&
                    t.y >= oy + 4 && t.y < oy + 56) {
                    ui_feedback::tick();
                    _enter_survey();
                    return true;
                }
                if (t.x >= ox + _cv_w - 220 && t.x < ox + _cv_w - 112 &&
                    t.y >= oy + 4 && t.y < oy + 56) {
                    ui_feedback::tick();
                    _start_rescan();
                    return true;
                }
            }
            if (t.y >= BAR_H && _prev_touch && _prev_tx >= 0) {
                int dx = t.x - _prev_tx, dy = t.y - _prev_ty;
                if (abs(dx) < 150 && abs(dy) < 150) {
                    if (abs(dx) > 3) _rotation = VM3::rot_y(-dx * 0.005f) * _rotation;
                    if (abs(dy) > 3) _rotation = VM3::rot_x( dy * 0.005f) * _rotation;
                }
            }
            _prev_tx = t.x; _prev_ty = t.y; _prev_touch = true;
        } else if (n >= 2) {
            auto a = M5.Touch.getDetail(0); auto b = M5.Touch.getDetail(1);
            float d = sqrtf((float)((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y)));
            if (_prev_dist > 0) { _rs.cam_z -= (d - _prev_dist) * 0.005f;
                                  _rs.cam_z = constrain(_rs.cam_z, 0.5f, 20.0f); }
            _prev_dist = d; _prev_touch = false;
        } else { _prev_touch = false; _prev_tx = _prev_ty = -1; _prev_dist = -1; }
        return true;
    }
    float _prev_dist = -1;

    void _draw_view() {
        if (!_mesh_loaded) return;
        // map orbit angles to RenderState euler (renderer applies rx,ry,rz)
        // extract Euler from the trackball matrix (same as the Viewer applet)
        _rs.rot_x = asinf(-_rotation.m[1][2]);
        _rs.rot_y = atan2f(_rotation.m[0][2], _rotation.m[2][2]);
        _rs.rot_z = atan2f(_rotation.m[1][0], _rotation.m[1][1]);
        _renderer.render_frame(_canvas, _mesh, _rs, TFT_BLACK);

        // Object name billboards: project each label anchor (already in the
        // mesh's int16 model space) with the same transform the renderer uses,
        // and draw its VOC name just above the marker.
        if (_label_n > 0) {
            float cx_ = cosf(_rs.rot_x), sx_ = sinf(_rs.rot_x);
            float cy_ = cosf(_rs.rot_y), sy_ = sinf(_rs.rot_y);
            float cz_ = cosf(_rs.rot_z), sz_ = sinf(_rs.rot_z);
            float m00 = cy_*cz_ + sy_*sx_*sz_, m01 = -cy_*sz_ + sy_*sx_*cz_, m02 = sy_*cx_;
            float m10 = cx_*sz_,               m11 = cx_*cz_,                m12 = -sx_;
            float m20 = -sy_*cz_ + cy_*sx_*sz_, m21 = sy_*sz_ + cy_*sx_*cz_, m22 = cy_*cx_;
            _canvas->setTextSize(1);
            for (int i = 0; i < _label_n; ++i) {
                float vx = _labels[i].x * _rs.scale;
                float vy = _labels[i].y * _rs.scale;
                float vz = _labels[i].z * _rs.scale;
                float rx = m00*vx + m01*vy + m02*vz;
                float ry = m10*vx + m11*vy + m12*vz;
                float rz = m20*vx + m21*vy + m22*vz;
                float camz = rz + _rs.cam_z;
                if (camz < 0.05f) continue;              // behind the camera
                float inv = _rs.fov / camz;
                int sxp = (int)(rx * inv) + _cv_w / 2;
                int syp = (int)(ry * inv) + _cv_h / 2;
                if (sxp < 2 || sxp >= _cv_w - 2 || syp < 10 || syp >= _cv_h - 2) continue;
                const char* nm = _label_name(_labels[i].cls);
                uint16_t c565 = _label_color(_labels[i].cls);
                int tw = (int)strlen(nm) * 6;
                _canvas->setTextColor(c565, TFT_BLACK);
                _canvas->setCursor(sxp - tw / 2, syp - 10);   // just above the marker top
                _canvas->print(nm);
            }
        }

        _canvas->setTextColor(TFT_WHITE, TFT_BLACK); _canvas->setTextSize(1);
        _canvas->setCursor(4, 4);
        _canvas->printf("%s  V:%u F:%u objs:%d", _scan_room,
                        _mesh.vertex_count, _mesh.face_count, _label_n);
        // [RF] button, canvas top-right -> opens the WiFi survey heatmap
        _canvas->fillRoundRect(_cv_w - 104, 4, 100, 48, 6, 0x2945);
        _canvas->setTextSize(3); _canvas->setTextColor(TFT_WHITE);
        _canvas->setCursor(_cv_w - 88, 16); _canvas->print("RF");
        // [SCAN+] button -> additive rescan of this room (registers + merges)
        _canvas->fillRoundRect(_cv_w - 220, 4, 108, 48, 6, 0x0300);
        _canvas->setTextSize(2);
        _canvas->setCursor(_cv_w - 208, 18); _canvas->print("SCAN+");
        int ox = (M5.Display.width() - _cv_w) / 2;
        int oy = (M5.Display.height() - _cv_h) / 2;
        _canvas->pushSprite(ox, oy);
    }

    void _open_room(const char* room) {
        strncpy(_scan_room, room, sizeof(_scan_room) - 1); _scan_room[sizeof(_scan_room)-1] = '\0';
        char path[200];
        if (_pm.meshPath(_building, room, path, sizeof(path)) && load_mesh(path, _mesh)) {
            strncpy(_mesh_path, path, sizeof(_mesh_path)-1); _mesh_path[sizeof(_mesh_path)-1]='\0';
            _read_labels(path);           // object names, if a sidecar exists
            _mesh_loaded = true; _rs = fit_to_canvas(_mesh, _cv_h, 0.7f);
            _rotation = VM3::rot_x(0.4f) * VM3::rot_y(0.6f); _state = VIEW;
            _reset_touch();
            _clear_content();             // wipe browse list from around the canvas
        }
    }

    // ==================== LIVE DETECTION VIEWFINDER =====================
    // Continuous camera feed with detection boxes + names — no scan, no
    // geometry, just "what does the AI see right now". Boxes are drawn on the
    // decode sprite (so they rotate with the preview); names are drawn after
    // the rotated push using the same rotation mapping, so text stays upright.
    uint32_t _live_last = 0;
    // last frame's name-tag rects — erased each frame so tags can't linger
    // after their object leaves the frame ("descriptions persist too long")
    struct TagRect { int16_t x, y, w, h; };
    TagRect _live_tags[8]; int _live_tag_n = 0;

    void _enter_live() {
        _state = LIVE; _reset_touch();
        _clear_content();
        int dw = M5.Display.width(), dh = M5.Display.height();
        M5.Display.setTextColor(COL_SUBTEXT, COL_BG); M5.Display.setTextSize(2);
        M5.Display.setCursor(12, dh - 34); M5.Display.print("LIVE  (back to exit)");
        (void)dw;
        _live_last = 0;
    }

    void _draw_live() {
        uint32_t now = millis();
        if (now - _live_last < 150) return;            // ~6fps cap
        const FrameSlot* s = _fb.latest();
        if (!s || s->seq == _last_decode_seq) return;
        _live_last = now;
        _decode_slot(s);                                // decode into _decode
        _det_note_frame(s);
        _draw_dets_on_decode(s);                        // confidence+persistence filtered

        // rotated, 2x-zoomed push, centred in the content area
        int dw = M5.Display.width(), dh = M5.Display.height();
        float zoom = 2.0f;
        int cx = dw / 2, cy = BAR_H + (dh - BAR_H) / 2;

        // erase last frame's name tags FIRST (regions outside the pushed
        // sprite are never repainted otherwise — tags lingered for ages)
        for (int i = 0; i < _live_tag_n; ++i)
            M5.Display.fillRect(_live_tags[i].x, _live_tags[i].y,
                                _live_tags[i].w, _live_tags[i].h, COL_BG);
        _live_tag_n = 0;

        _decode->pushRotateZoom(&M5.Display, cx, cy, PREVIEW_ROT_DEG, zoom, zoom);

        // upright name tags via the same +90-degree mapping the push used:
        // sprite (sx,sy) -> screen (cx - (sy-60)*z, cy + (sx-80)*z)
        M5.Display.setTextSize(2);
        for (uint8_t i = 0; i < s->det_count && _live_tag_n < 8; ++i) {
            const Detection& d = s->dets[i];
            if (!_det_display(d)) continue;             // discerning tags too
            float sx = (d.x + d.w * 0.5f) / 2.0f, sy = d.y / 2.0f;
            int lx = cx - (int)((sy - 60) * zoom);
            int ly = cy + (int)((sx - 80) * zoom);
            uint16_t c565 = _cls565(d.class_id);
            M5.Display.setTextColor(c565, TFT_BLACK);
            M5.Display.setCursor(lx + 4, ly - 8);
            M5.Display.printf("%s %d%%", object_labels::name(d.class_id), d.confidence);
            _live_tags[_live_tag_n++] = { (int16_t)(lx + 2), (int16_t)(ly - 10),
                                          200, 22 };
        }
        // objs counter
        M5.Display.setTextColor(COL_TEXT, COL_BG);
        M5.Display.setCursor(12, BAR_H + 10);
        M5.Display.printf("objs: %-2d", (int)s->det_count);
    }

    // ==================== RF SURVEY (heatmap MVP) =======================
    // docs/HEATMAP_SPEC.md — walk the room, tap your position on the floor
    // plan, sample the target AP's RSSI, and paint an IDW heat grid over the
    // mesh footprint. Samples persist in "<room>.rf" beside the mesh.
    RfSurvey _rf;
    char     _mesh_path[200] = {0};
    bool     _rf_redraw = false;
    bool     _rf_heat = true;
    bool     _rf_wifi_ok = false;
    uint8_t  _rf_ant = 0;                 // active layer/antenna: 0=INT 1=EXT
    int16_t  _rf_bx0=0,_rf_bx1=0,_rf_bz0=0,_rf_bz1=0;   // mesh XZ bounds
    int16_t  _rf_tap_x=0,_rf_tap_z=0; bool _rf_tap_set=false;
    int8_t   _rf_last=127;
    // plan-view screen rect (computed in _draw_survey, used by touch mapping)
    int _pl_x=0,_pl_y=0,_pl_w=1,_pl_h=1;

    void _survey_bounds_from_mesh() {
        _rf_bx0 = _rf_bz0 = 32767; _rf_bx1 = _rf_bz1 = -32768;
        for (uint32_t i = 0; i < _mesh.vertex_count; ++i) {
            int16_t x = _mesh.vertices[i].x, z = _mesh.vertices[i].z;
            if (x < _rf_bx0) _rf_bx0 = x;  if (x > _rf_bx1) _rf_bx1 = x;
            if (z < _rf_bz0) _rf_bz0 = z;  if (z > _rf_bz1) _rf_bz1 = z;
        }
        if (_rf_bx1 <= _rf_bx0) _rf_bx1 = _rf_bx0 + 1;
        if (_rf_bz1 <= _rf_bz0) _rf_bz1 = _rf_bz0 + 1;
    }

    bool _wifi_up() {
        if (_rf_wifi_ok) return true;
        WiFi.setPins(12, 13, 11, 10, 9, 8, 15);   // M5Tab5 C6 hosted-SDIO
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();
        delay(100);
        _rf_wifi_ok = true;                       // C6 link init is one-shot
        return true;
    }

    void _enter_survey() {
        if (!_mesh_loaded) return;
        _state = SURVEY; _reset_touch();
        _rf_tap_set = false; _rf_last = 127;
        _survey_bounds_from_mesh();
        M5.Display.fillRect(0, BAR_H, M5.Display.width(),
                            M5.Display.height() - BAR_H, COL_BG);
        M5.Display.setTextColor(COL_TEXT); M5.Display.setTextSize(2);
        M5.Display.setCursor(12, BAR_H + 12);
        M5.Display.print("Starting WiFi + loading survey...");
        _wifi_up();
        rf_switch::init();
        rf_switch::setExternal(_rf_ant == 1);
        if (!_rf.loadBeside(_mesh_path)) _rf.clear();
        // AP priority: this room's saved survey AP -> the global saved AP ->
        // ask the user (picker). Selection persists across sessions; the AP
        // row in the panel is tappable to change it later.
        if (_rf.ssid[0] == '\0' && !_load_saved_ap()) { _open_ap_picker(); return; }
        _rf.computeGrid(_rf_bx0, _rf_bx1, _rf_bz0, _rf_bz1, _rf_ant);
        _rf_redraw = true;
    }

    // ---- AP selection: persistent global choice + explicit picker --------
    static constexpr const char* WIFI_CFG = "/meshscan/wifi.cfg";
    struct APPick { char ssid[33]; uint8_t bssid[6]; uint8_t ch; int8_t rssi; };
    APPick _rf_picks[10]; int _rf_pick_n = 0;
    bool _rf_picking = false;

    bool _load_saved_ap() {
        File f = SD_MMC.open(WIFI_CFG, FILE_READ);
        if (!f) return false;
        char magic[4]; bool ok = f.read((uint8_t*)magic,4)==4 && memcmp(magic,"WAP1",4)==0;
        if (ok) {
            f.read((uint8_t*)_rf.ssid, 33); _rf.ssid[32]='\0';
            f.read(_rf.bssid, 6);
            f.read(&_rf.channel, 1);
        }
        f.close();
        return ok && _rf.ssid[0] != '\0';
    }
    void _save_ap() {
        File f = SD_MMC.open(WIFI_CFG, FILE_WRITE);
        if (!f) return;
        f.write((const uint8_t*)"WAP1", 4);
        f.write((const uint8_t*)_rf.ssid, 33);
        f.write(_rf.bssid, 6);
        f.write(&_rf.channel, 1);
        f.close();
    }

    void _open_ap_picker() {
        int dw = M5.Display.width();
        M5.Display.fillRect(0, BAR_H, dw, M5.Display.height()-BAR_H, COL_BG);
        M5.Display.setTextColor(COL_TEXT); M5.Display.setTextSize(2);
        M5.Display.setCursor(12, BAR_H + 12); M5.Display.print("Scanning WiFi networks...");
        int n = WiFi.scanNetworks(false, false, false, 120);
        // keep the strongest 10, sorted by RSSI
        _rf_pick_n = 0;
        for (int i = 0; i < n; ++i) {
            int8_t r = (int8_t)WiFi.RSSI(i);
            int pos = _rf_pick_n;
            while (pos > 0 && _rf_picks[pos-1].rssi < r) pos--;
            if (pos >= 10) continue;
            if (_rf_pick_n < 10) _rf_pick_n++;
            for (int j = _rf_pick_n - 1; j > pos; --j) _rf_picks[j] = _rf_picks[j-1];
            strncpy(_rf_picks[pos].ssid, WiFi.SSID(i).c_str(), 32);
            _rf_picks[pos].ssid[32]='\0';
            memcpy(_rf_picks[pos].bssid, WiFi.BSSID(i), 6);
            _rf_picks[pos].ch   = (uint8_t)WiFi.channel(i);
            _rf_picks[pos].rssi = r;
        }
        WiFi.scanDelete();
        _rf_picking = true; _rf_redraw = true;
    }

    void _draw_ap_picker() {
        int dw = M5.Display.width(), dh = M5.Display.height();
        M5.Display.startWrite();
        M5.Display.fillRect(0, BAR_H, dw, dh - BAR_H, COL_BG);
        M5.Display.setTextColor(COL_TEXT); M5.Display.setTextSize(2);
        M5.Display.setCursor(12, BAR_H + 10); M5.Display.print("Pick WiFi network to survey");
        for (int i = 0; i < _rf_pick_n; ++i) {
            int y = BAR_H + 48 + i * 50;
            if (y + 44 > dh) break;
            M5.Display.fillRoundRect(12, y, dw - 24, 44, 6, 0x2104);
            M5.Display.setCursor(24, y + 12);
            M5.Display.printf("%-24.24s ch%-2d %ddBm",
                              _rf_picks[i].ssid, _rf_picks[i].ch, _rf_picks[i].rssi);
        }
        M5.Display.endWrite();
    }

    void _pick_row_tapped(int row) {
        if (row < 0 || row >= _rf_pick_n) return;
        strncpy(_rf.ssid, _rf_picks[row].ssid, 32); _rf.ssid[32]='\0';
        memcpy(_rf.bssid, _rf_picks[row].bssid, 6);
        _rf.channel = _rf_picks[row].ch;
        _save_ap();                                    // persists across sessions
        _rf_picking = false; _rf_last = 127;
        _rf.computeGrid(_rf_bx0, _rf_bx1, _rf_bz0, _rf_bz1, _rf_ant);
        _rf_redraw = true;
    }

    // Sample burst: 2 single-channel scans, average the target AP's RSSI.
    int8_t _rf_measure(int8_t& mn, int8_t& mx) {
        if (_rf.channel < 1) return 127;
        int sum = 0, cnt = 0; mn = 0; mx = -127;
        for (int burst = 0; burst < 2; ++burst) {
            int n = WiFi.scanNetworks(false, false, false, 120, _rf.channel);
            for (int i = 0; i < n; ++i) {
                if (memcmp(WiFi.BSSID(i), _rf.bssid, 6) != 0) continue;
                int8_t r = (int8_t)WiFi.RSSI(i);
                sum += r; ++cnt;
                if (mx < r) mx = r;
                if (cnt == 1 || r < mn) mn = r;
            }
            WiFi.scanDelete();
        }
        return cnt ? (int8_t)(sum / cnt) : 127;
    }

    bool _update_survey() {
        if (M5.Touch.getCount() == 0) { _prev_touch = false; return true; }
        if (_prev_touch) return true;             // act on press edge
        _prev_touch = true;
        auto t = M5.Touch.getDetail(0);
        int dw = M5.Display.width(), dh = M5.Display.height();
        int panel_x = dw * 2 / 3;

        if (_rf_picking) {                        // AP picker modal
            int row = (t.y - (BAR_H + 48)) / 50;
            if (t.y >= BAR_H + 48) _pick_row_tapped(row);
            return true;
        }
        // tappable AP row (top of the panel): passive "change network" option
        if (t.x >= panel_x && t.y >= BAR_H + 34 && t.y < BAR_H + 62) {
            _open_ap_picker();
            return true;
        }

        if (t.x >= panel_x) {                     // right panel buttons
            int by = BAR_H + 150;
            if (t.y >= by && t.y < by + 56) {          // [SAMPLE HERE]
                if (_rf_tap_set && _rf.ssid[0]) {
                    ui_theme::press_flash(panel_x + 10, by, dw - panel_x - 22, 56);
                    int8_t mn, mx;
                    int8_t avg = _rf_measure(mn, mx);
                    _rf_last = avg;
                    if (avg != 127) {
                        _rf.add(_rf_tap_x, _rf_tap_z, avg, mn, mx, _rf_ant);
                        _rf.computeGrid(_rf_bx0, _rf_bx1, _rf_bz0, _rf_bz1, _rf_ant);
                        _rf.saveBeside(_mesh_path);
                        ui_feedback::tick();               // sample landed
                    } else ui_feedback::buzz();            // AP not seen
                    _rf_redraw = true;
                }
            } else if (t.y >= by + 66 && t.y < by + 110) {   // [ANT] layer flip
                ui_feedback::tick();
                _rf_ant ^= 1;
                rf_switch::setExternal(_rf_ant == 1);
                delay(800);                        // RF path settle before sampling
                _rf.computeGrid(_rf_bx0, _rf_bx1, _rf_bz0, _rf_bz1, _rf_ant);
                _rf_last = 127;
                _rf_redraw = true;
            } else if (t.y >= by + 120 && t.y < by + 164) {  // [HEAT] toggle
                _rf_heat = !_rf_heat; _rf_redraw = true;
            }
            return true;
        }
        // tap inside the plan: set the "I'm standing here" crosshair
        if (t.x >= _pl_x && t.x < _pl_x + _pl_w &&
            t.y >= _pl_y && t.y < _pl_y + _pl_h) {
            _rf_tap_x = (int16_t)(_rf_bx0 + (int32_t)(t.x - _pl_x) * (_rf_bx1 - _rf_bx0) / _pl_w);
            _rf_tap_z = (int16_t)(_rf_bz0 + (int32_t)(t.y - _pl_y) * (_rf_bz1 - _rf_bz0) / _pl_h);
            _rf_tap_set = true; _rf_redraw = true;
        }
        return true;
    }

    void _draw_survey() {
        _rf_redraw = false;
        if (_rf_picking) { _draw_ap_picker(); return; }
        int dw = M5.Display.width(), dh = M5.Display.height();
        int panel_x = dw * 2 / 3;
        M5.Display.startWrite();
        M5.Display.fillRect(0, BAR_H, dw, dh - BAR_H, COL_BG);

        // ---- plan area (left 2/3): heat grid, footprint, labels, samples --
        _pl_x = 16; _pl_y = BAR_H + 16;
        _pl_w = panel_x - 32; _pl_h = dh - BAR_H - 32;
        if (_rf_heat) {
            for (int row = 0; row < RfSurvey::GRID_H; ++row)
                for (int col = 0; col < RfSurvey::GRID_W; ++col) {
                    uint16_t c = RfSurvey::colorFor(_rf.gridValue(col, row));
                    int x0 = _pl_x + col * _pl_w / RfSurvey::GRID_W;
                    int x1 = _pl_x + (col + 1) * _pl_w / RfSurvey::GRID_W;
                    int y0 = _pl_y + row * _pl_h / RfSurvey::GRID_H;
                    int y1 = _pl_y + (row + 1) * _pl_h / RfSurvey::GRID_H;
                    M5.Display.fillRect(x0, y0, x1 - x0, y1 - y0, c);
                }
        }
        M5.Display.drawRect(_pl_x, _pl_y, _pl_w, _pl_h, COL_DIVIDER);

        // object labels as landmarks
        M5.Display.setTextSize(1);
        for (int i = 0; i < _label_n; ++i) {
            int sx = _pl_x + (int32_t)(_labels[i].x - _rf_bx0) * _pl_w / (_rf_bx1 - _rf_bx0);
            int sy = _pl_y + (int32_t)(_labels[i].z - _rf_bz0) * _pl_h / (_rf_bz1 - _rf_bz0);
            M5.Display.fillRect(sx - 3, sy - 3, 6, 6, TFT_WHITE);
            M5.Display.setTextColor(TFT_WHITE);
            M5.Display.setCursor(sx + 5, sy - 4);
            M5.Display.print(_label_name(_labels[i].cls));
        }
        // sample discs — active antenna layer only
        for (int i = 0; i < _rf.count(); ++i) {
            const RfSurvey::Sample& sp = _rf.sample(i);
            if (sp.antenna != _rf_ant) continue;
            int sx = _pl_x + (int32_t)(sp.x - _rf_bx0) * _pl_w / (_rf_bx1 - _rf_bx0);
            int sy = _pl_y + (int32_t)(sp.z - _rf_bz0) * _pl_h / (_rf_bz1 - _rf_bz0);
            M5.Display.fillCircle(sx, sy, 8, RfSurvey::colorFor(sp.rssi));
            M5.Display.drawCircle(sx, sy, 8, TFT_BLACK);
        }
        // crosshair
        if (_rf_tap_set) {
            int sx = _pl_x + (int32_t)(_rf_tap_x - _rf_bx0) * _pl_w / (_rf_bx1 - _rf_bx0);
            int sy = _pl_y + (int32_t)(_rf_tap_z - _rf_bz0) * _pl_h / (_rf_bz1 - _rf_bz0);
            M5.Display.drawLine(sx - 10, sy, sx + 10, sy, TFT_WHITE);
            M5.Display.drawLine(sx, sy - 10, sx, sy + 10, TFT_WHITE);
        }

        // ---- right panel --------------------------------------------------
        int px = panel_x + 10, py = BAR_H + 12;
        M5.Display.setTextColor(COL_TEXT, COL_BG); M5.Display.setTextSize(2);
        M5.Display.setCursor(px, py);      M5.Display.print("RF Survey");
        M5.Display.setTextSize(1);
        M5.Display.fillRoundRect(px - 4, py + 24, dw - px - 8, 24, 4, 0x18E3);   // tappable
        M5.Display.setCursor(px, py + 28); M5.Display.printf("AP: %.18s ch%d >", _rf.ssid, _rf.channel);
        M5.Display.setCursor(px, py + 44);
        M5.Display.printf("INT:%d  EXT:%d samples", _rf.countFor(0), _rf.countFor(1));
        M5.Display.setTextSize(3);
        M5.Display.setCursor(px, py + 66);
        if (_rf_last != 127) M5.Display.printf("%d dBm", (int)_rf_last);
        else                 M5.Display.print("-- dBm");

        int by = BAR_H + 150;
        M5.Display.fillRoundRect(px, by, dw - px - 12, 56, 6, 0x2945);
        M5.Display.setTextSize(2); M5.Display.setTextColor(COL_TEXT);
        M5.Display.setCursor(px + 12, by + 18);
        M5.Display.print(_rf_tap_set ? "SAMPLE HERE" : "tap plan first");
        // antenna layer toggle: which antenna samples AND which heat layer shows
        M5.Display.fillRoundRect(px, by + 66, dw - px - 12, 44, 6,
                                 _rf_ant ? 0x7800 : 0x0300);
        M5.Display.setCursor(px + 12, by + 78);
        M5.Display.printf("ANT: %s", _rf_ant ? "EXT MMCX" : "INTERNAL");
        M5.Display.fillRoundRect(px, by + 120, dw - px - 12, 44, 6, 0x2104);
        M5.Display.setCursor(px + 12, by + 132);
        M5.Display.printf("HEAT %s", _rf_heat ? "ON " : "OFF");

        M5.Display.setTextSize(1); M5.Display.setTextColor(COL_SUBTEXT, COL_BG);
        M5.Display.setCursor(px, dh - 40);
        M5.Display.print("tap plan = your position");
        M5.Display.setCursor(px, dh - 26);
        M5.Display.print("back = 3D view");
        M5.Display.endWrite();
    }
};
