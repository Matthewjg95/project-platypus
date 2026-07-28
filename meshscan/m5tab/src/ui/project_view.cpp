// project_view.cpp - Buildings + rooms browser.
//
// Layout (1024x600): left column = buildings list + "New building"; right column
// = rooms in the selected building + "Scan room" / "Open" / "Delete".
//
// Phase 1 status: functional listing and navigation. TODOs marked for the text
// entry dialog (on-screen keyboard) used when creating/renaming.

#include "project_view.h"
#include <string.h>
#include <stdio.h>

char g_sel_building[48] = {0};
char g_sel_room[48]     = {0};

#if MESHSCAN_HAVE_LVGL

static lv_obj_t* s_root = nullptr;
static lv_obj_t* s_buildings_list = nullptr;
static lv_obj_t* s_rooms_list = nullptr;

static void rooms_reload();

static void building_clicked(lv_event_t* e) {
    const char* txt = lv_list_get_btn_text(s_buildings_list, (lv_obj_t*)lv_event_get_target(e));
    if (!txt) return;
    strncpy(g_sel_building, txt, sizeof(g_sel_building) - 1);
    g_sel_room[0] = '\0';
    rooms_reload();
}

static void room_clicked(lv_event_t* e) {
    const char* txt = lv_list_get_btn_text(s_rooms_list, (lv_obj_t*)lv_event_get_target(e));
    if (!txt) return;
    strncpy(g_sel_room, txt, sizeof(g_sel_room) - 1);
    app_open_room(g_sel_building, g_sel_room);   // -> switches to mesh view
}

static void new_building_clicked(lv_event_t*) {
    // TODO: show an lv_keyboard + lv_textarea dialog and pass the typed name.
    // Placeholder auto-names so the flow is testable end-to-end now.
    char tmp[48]; static int n = 1; snprintf(tmp, sizeof(tmp), "building_%d", n++);
    app_create_building(tmp);
    project_view_refresh();
}

static void scan_room_clicked(lv_event_t*) {
    if (g_sel_building[0] == '\0') return;
    // Room name auto-suggested by app side if empty.
    app_request_scan_start(g_sel_building, g_sel_room[0] ? g_sel_room : "");
}

static void rooms_reload() {
    if (!s_rooms_list) return;
    lv_obj_clean(s_rooms_list);
    if (g_sel_building[0] == '\0') return;
    char names[32][48];
    int n = app_list_rooms(g_sel_building, names, 32);
    for (int i = 0; i < n; ++i) {
        lv_obj_t* btn = lv_list_add_btn(s_rooms_list, LV_SYMBOL_FILE, names[i]);
        lv_obj_add_event_cb(btn, room_clicked, LV_EVENT_CLICKED, nullptr);
    }
}

void project_view_refresh() {
    if (!s_buildings_list) return;
    lv_obj_clean(s_buildings_list);
    char names[32][48];
    int n = app_list_buildings(names, 32);
    for (int i = 0; i < n; ++i) {
        lv_obj_t* btn = lv_list_add_btn(s_buildings_list, LV_SYMBOL_DIRECTORY, names[i]);
        lv_obj_add_event_cb(btn, building_clicked, LV_EVENT_CLICKED, nullptr);
    }
    rooms_reload();
}

void project_view_create(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_ROW);

    // left: buildings
    lv_obj_t* left = lv_obj_create(s_root);
    lv_obj_set_size(left, LV_PCT(48), LV_PCT(100));
    lv_obj_set_flex_flow(left, LV_FLEX_FLOW_COLUMN);
    lv_obj_t* lt = lv_label_create(left); lv_label_set_text(lt, "Buildings");
    s_buildings_list = lv_list_create(left);
    lv_obj_set_flex_grow(s_buildings_list, 1);
    lv_obj_t* nb = lv_btn_create(left);
    lv_obj_add_event_cb(nb, new_building_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(nb), LV_SYMBOL_PLUS " New building");

    // right: rooms
    lv_obj_t* right = lv_obj_create(s_root);
    lv_obj_set_size(right, LV_PCT(48), LV_PCT(100));
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_COLUMN);
    lv_obj_t* rt = lv_label_create(right); lv_label_set_text(rt, "Rooms");
    s_rooms_list = lv_list_create(right);
    lv_obj_set_flex_grow(s_rooms_list, 1);
    lv_obj_t* sb = lv_btn_create(right);
    lv_obj_add_event_cb(sb, scan_room_clicked, LV_EVENT_CLICKED, nullptr);
    lv_label_set_text(lv_label_create(sb), LV_SYMBOL_EYE_OPEN " Scan room");

    project_view_refresh();
}

void project_view_destroy() {
    if (s_root) { lv_obj_del(s_root); s_root = nullptr; }
    s_buildings_list = s_rooms_list = nullptr;
}

#else  // no LVGL — no-op stubs so the module still compiles

void project_view_create(lv_obj_t*) {}
void project_view_destroy() {}
void project_view_refresh() {}

#endif
