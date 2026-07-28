// edit_view.cpp - Rename a room and edit object annotations.
//
// Phase 1 status: scaffold. Shows the current room name in a text area with an
// on-screen keyboard and a Save button wired to app_save_room_label(). Object
// annotation editing (per-marker label/colour) is stubbed with TODOs — it needs
// the mesh's object list surfaced from main.cpp.

#include "edit_view.h"
#include <string.h>

#if MESHSCAN_HAVE_LVGL

static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_name_ta = nullptr;
static lv_obj_t* s_kb = nullptr;

static void save_clicked(lv_event_t*) {
    const char* newname = s_name_ta ? lv_textarea_get_text(s_name_ta) : nullptr;
    if (newname && newname[0])
        app_save_room_label(g_sel_building, g_sel_room, newname);
    app_show_view(AppView::Mesh);
}
static void back_clicked(lv_event_t*) { app_show_view(AppView::Mesh); }

void edit_view_create(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* title = lv_label_create(s_root);
    lv_label_set_text(title, "Edit room");

    s_name_ta = lv_textarea_create(s_root);
    lv_textarea_set_one_line(s_name_ta, true);
    lv_textarea_set_text(s_name_ta, g_sel_room);
    lv_obj_set_width(s_name_ta, LV_PCT(80));

    lv_obj_t* row = lv_obj_create(s_root);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_size(row, LV_PCT(100), LV_SIZE_CONTENT);

    lv_obj_t* back = lv_btn_create(row);
    lv_obj_add_event_cb(back, back_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(back), LV_SYMBOL_LEFT " Cancel");

    lv_obj_t* save = lv_btn_create(row);
    lv_obj_add_event_cb(save, save_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(save), LV_SYMBOL_OK " Save");

    // TODO: list MeshObjects (label + colour swatch) with per-item edit rows.

    s_kb = lv_keyboard_create(s_root);
    lv_keyboard_set_textarea(s_kb, s_name_ta);
}

void edit_view_destroy() {
    if (s_root) { lv_obj_del(s_root); s_root = nullptr; }
    s_name_ta = s_kb = nullptr;
}

#else  // no LVGL

void edit_view_create(lv_obj_t*) {}
void edit_view_destroy() {}

#endif
