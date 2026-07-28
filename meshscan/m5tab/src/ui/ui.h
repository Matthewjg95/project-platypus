// ui.h - Shared UI types, view lifecycle, and app-action callbacks.
//
// Each view is a small module that builds its LVGL widgets onto a parent object
// and tears them down on exit. Views never touch the scan/mesh subsystems
// directly — they call the app_* action hooks (implemented in main.cpp), which
// keeps the UI decoupled from the pipeline. main.cpp owns view switching.

#pragma once
#include <stdint.h>

#if __has_include(<lvgl.h>)
  #include <lvgl.h>
  #define MESHSCAN_HAVE_LVGL 1
#else
  // Allows the UI sources to compile for static analysis without LVGL present.
  typedef struct _lv_obj_t lv_obj_t;
  #define MESHSCAN_HAVE_LVGL 0
#endif

enum class AppView : uint8_t { Project, Scan, Mesh, Edit };

// ---- app actions: implemented in main.cpp, called by the views ----
void app_show_view(AppView v);
void app_create_building(const char* name);
void app_delete_building(const char* name);
void app_open_room(const char* building, const char* room);
void app_delete_room(const char* building, const char* room);
void app_request_scan_start(const char* building, const char* room);
void app_request_scan_stop();
void app_save_room_label(const char* building, const char* room, const char* new_label);

// ---- data providers: implemented in main.cpp, read by the views ----
int  app_list_buildings(char names[][48], int max);
int  app_list_rooms(const char* building, char names[][48], int max);

// ---- selection state shared across views (set by project_view) ----
extern char g_sel_building[48];
extern char g_sel_room[48];

// ---- view lifecycle (each implemented in its own .cpp) ----
void scan_view_create(lv_obj_t* parent);
void scan_view_destroy();
void scan_view_set_progress(int pct);              // 0..100 sweep progress
void scan_view_set_status(const char* text);
// live preview: RGB565 thumbnail + detection overlays come via these hooks
void scan_view_update_preview(const uint16_t* rgb565, int w, int h);

void mesh_view_create(lv_obj_t* parent);
void mesh_view_destroy();
void mesh_view_set_title(const char* text);

void project_view_create(lv_obj_t* parent);
void project_view_destroy();
void project_view_refresh();                       // re-read SD listing

void edit_view_create(lv_obj_t* parent);
void edit_view_destroy();
