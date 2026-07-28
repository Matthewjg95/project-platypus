// mesh_view.cpp - 3D view with touch orbit/zoom over the existing renderer.
//
// A full-screen canvas holds the renderer's RGB565 output. Drag = orbit,
// pinch/two-finger or the +/- buttons = zoom. A back button returns to the
// project browser; an edit button opens the annotation view.
//
// Phase 1 status: orbit-on-drag + zoom buttons wired; pinch-zoom left as a TODO
// (LVGL gesture plumbing depends on the touch driver).

#include "mesh_view.h"

#if MESHSCAN_HAVE_LVGL

static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_canvas = nullptr;
static lv_obj_t* s_title = nullptr;
static lv_color_t* s_buf = nullptr;
static lv_point_t s_last;
static bool s_dragging = false;

static const int RW = 480;     // render canvas size; scale to your panel
static const int RH = 360;

static void redraw() {
    if (!s_canvas || !s_buf) return;
    app_render_mesh((uint16_t*)s_buf, RW, RH);
    lv_obj_invalidate(s_canvas);
}

static void canvas_pressing(lv_event_t* e) {
    lv_indev_t* indev = lv_indev_get_act();
    if (!indev) return;
    lv_point_t p; lv_indev_get_point(indev, &p);
    if (!s_dragging) { s_last = p; s_dragging = true; return; }
    float daz = (p.x - s_last.x) * 0.01f;
    float del = (p.y - s_last.y) * 0.01f;
    s_last = p;
    app_mesh_orbit(daz, del);
    redraw();
}
static void canvas_released(lv_event_t*) { s_dragging = false; }

static void zoom_in (lv_event_t*) { app_mesh_zoom(-2000.0f); redraw(); }
static void zoom_out(lv_event_t*) { app_mesh_zoom(+2000.0f); redraw(); }
static void back_clicked(lv_event_t*) { app_show_view(AppView::Project); }
static void edit_clicked(lv_event_t*) { app_show_view(AppView::Edit); }

void mesh_view_create(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));

    s_canvas = lv_canvas_create(s_root);
    s_buf = (lv_color_t*)lv_mem_alloc(sizeof(lv_color_t) * RW * RH);
    if (s_buf) {
        lv_canvas_set_buffer(s_canvas, s_buf, RW, RH, LV_IMG_CF_TRUE_COLOR);
        lv_canvas_fill_bg(s_canvas, lv_color_black(), LV_OPA_COVER);
    }
    lv_obj_align(s_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(s_canvas, canvas_pressing, LV_EVENT_PRESSING, nullptr);
    lv_obj_add_event_cb(s_canvas, canvas_released, LV_EVENT_RELEASED, nullptr);

    s_title = lv_label_create(s_root);
    lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 4);
    lv_label_set_text(s_title, "Mesh");

    lv_obj_t* back = lv_btn_create(s_root);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 4, 4);
    lv_obj_add_event_cb(back, back_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(back), LV_SYMBOL_LEFT);

    lv_obj_t* edit = lv_btn_create(s_root);
    lv_obj_align(edit, LV_ALIGN_TOP_RIGHT, -4, 4);
    lv_obj_add_event_cb(edit, edit_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(edit), LV_SYMBOL_EDIT);

    lv_obj_t* zi = lv_btn_create(s_root);
    lv_obj_align(zi, LV_ALIGN_BOTTOM_RIGHT, -4, -64);
    lv_obj_add_event_cb(zi, zoom_in, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(zi), LV_SYMBOL_PLUS);

    lv_obj_t* zo = lv_btn_create(s_root);
    lv_obj_align(zo, LV_ALIGN_BOTTOM_RIGHT, -4, -4);
    lv_obj_add_event_cb(zo, zoom_out, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(zo), LV_SYMBOL_MINUS);

    redraw();
}

void mesh_view_set_title(const char* text) {
    if (s_title && text) lv_label_set_text(s_title, text);
}

void mesh_view_destroy() {
    if (s_buf) { lv_mem_free(s_buf); s_buf = nullptr; }
    if (s_root) { lv_obj_del(s_root); s_root = nullptr; }
    s_canvas = s_title = nullptr;
    s_dragging = false;
}

#else  // no LVGL

void mesh_view_create(lv_obj_t*) {}
void mesh_view_destroy() {}
void mesh_view_set_title(const char*) {}

#endif
