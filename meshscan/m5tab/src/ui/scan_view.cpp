// scan_view.cpp - Live preview + sweep progress during a scan.
//
// Layout: large camera canvas (left), progress arc + status + "Stop" (right).
// The canvas is fed RGB565 thumbnails via scan_view_update_preview(); detection
// boxes are drawn by main.cpp before handing the buffer over (or extend here).
//
// Phase 1 status: progress + status + stop wired. Preview canvas allocates an
// LVGL image buffer; main.cpp decodes the latest JPEG into it.

#include "scan_view.h"
#include <string.h>

#if MESHSCAN_HAVE_LVGL

static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_canvas = nullptr;
static lv_obj_t* s_arc = nullptr;
static lv_obj_t* s_status = nullptr;
static lv_color_t* s_canvas_buf = nullptr;   // CANVAS_W*CANVAS_H

static const int CANVAS_W = 320;
static const int CANVAS_H = 240;

static void stop_clicked(lv_event_t*) { app_request_scan_stop(); }

void scan_view_create(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_ROW);

    // camera canvas
    s_canvas = lv_canvas_create(s_root);
    s_canvas_buf = (lv_color_t*)lv_mem_alloc(sizeof(lv_color_t) * CANVAS_W * CANVAS_H);
    if (s_canvas_buf) {
        lv_canvas_set_buffer(s_canvas, s_canvas_buf, CANVAS_W, CANVAS_H, LV_IMG_CF_TRUE_COLOR);
        lv_canvas_fill_bg(s_canvas, lv_color_black(), LV_OPA_COVER);
    }

    // right column
    lv_obj_t* right = lv_obj_create(s_root);
    lv_obj_set_flex_grow(right, 1);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_COLUMN);

    s_arc = lv_arc_create(right);
    lv_arc_set_range(s_arc, 0, 100);
    lv_arc_set_value(s_arc, 0);
    lv_obj_set_size(s_arc, 200, 200);

    s_status = lv_label_create(right);
    lv_label_set_text(s_status, "Rotate slowly 360\xC2\xB0");

    lv_obj_t* stop = lv_btn_create(right);
    lv_obj_add_event_cb(stop, stop_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(stop), LV_SYMBOL_STOP " Stop");
}

void scan_view_set_progress(int pct) {
    if (s_arc) lv_arc_set_value(s_arc, pct);
}

void scan_view_set_status(const char* text) {
    if (s_status && text) lv_label_set_text(s_status, text);
}

void scan_view_update_preview(const uint16_t* rgb565, int w, int h) {
    if (!s_canvas || !s_canvas_buf || !rgb565) return;
    // Copy/clip into the canvas buffer. Assumes LV_COLOR_DEPTH == 16 (RGB565).
    int cw = w < CANVAS_W ? w : CANVAS_W;
    int ch = h < CANVAS_H ? h : CANVAS_H;
    for (int y = 0; y < ch; ++y)
        memcpy(&s_canvas_buf[y * CANVAS_W], &rgb565[y * w], cw * sizeof(uint16_t));
    lv_obj_invalidate(s_canvas);
    // TODO: draw detection bounding boxes + labels over the canvas here.
}

void scan_view_destroy() {
    if (s_canvas_buf) { lv_mem_free(s_canvas_buf); s_canvas_buf = nullptr; }
    if (s_root) { lv_obj_del(s_root); s_root = nullptr; }
    s_canvas = s_arc = s_status = nullptr;
}

#else  // no LVGL

void scan_view_create(lv_obj_t*) {}
void scan_view_destroy() {}
void scan_view_set_progress(int) {}
void scan_view_set_status(const char*) {}
void scan_view_update_preview(const uint16_t*, int, int) {}

#endif
