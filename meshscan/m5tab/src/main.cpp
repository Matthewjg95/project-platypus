// main.cpp - MeshScan M5Tab firmware entry point.
//
// Wiring:
//   Unit V --UART--> UartReceiver --> FrameBuffer (JPEG + detections)
//   Scan: collect ~20 frames over a 360 sweep -> DepthEstimator places objects,
//         RoomFitter fits a box -> MeshBuilder assembles -> MeshWriter -> SD
//         -> MeshRenderer loads & displays. ProjectManager owns the SD layout.
//
// The display/touch panel driver is hardware-specific (M5Tab panel). LVGL init
// below is a scaffold: register your real flush + touchpad read callbacks where
// marked TODO. Everything else is functional.

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <esp_heap_caps.h>

#include "uart_receiver.h"
#include "frame_buffer.h"
#include "project_manager.h"
#include "mesh_builder.h"
#include "mesh_writer.h"
#include "mesh_renderer.h"
#include "depth_estimator.h"
#include "room_fitter.h"
#include "object_labels.h"
#include "jpeg_decode.h"
#include "mesh_stream_writer.h"
#include "cv/slam_pipeline.h"
#include "cv/surface_recon.h"
#include "ui/ui.h"

// ---- build-flag config (see platformio.ini) ----
#ifndef UNITV_UART_RX
#define UNITV_UART_RX 18
#endif
#ifndef UNITV_UART_TX
#define UNITV_UART_TX 17
#endif
#ifndef UNITV_UART_BAUD
#define UNITV_UART_BAUD 115200
#endif
#ifndef SD_CS_PIN
#define SD_CS_PIN 4
#endif
#ifndef SD_SCK_PIN
#define SD_SCK_PIN 36
#endif
#ifndef SD_MISO_PIN
#define SD_MISO_PIN 37
#endif
#ifndef SD_MOSI_PIN
#define SD_MOSI_PIN 35
#endif

// ---- subsystems ----
static UartReceiver  g_uart;
static FrameBuffer   g_fb;
static ProjectManager g_pm;
static MeshRenderer  g_renderer;
static DepthEstimator g_depth(160, 120, 60.0f);   // we run depth on half-scale frames
static RoomFitter    g_fitter(160, 120);
static MeshBuilder   g_builder;

// ---- Phase 2: structure-from-motion ----
// Intrinsics for the half-scale (160x120) frames the scan path decodes.
//   fx = (W/2)/tan(HFOV/2);  principal point at the image centre.
static cv::CameraIntrinsics g_K = { 138.56f, 138.56f, 80.0f, 60.0f };
static cv::SlamPipeline     g_slam;
#ifdef MESHSCAN_PHASE2_DEFAULT
static bool g_phase2 = true;       // dense SfM mesh instead of the box fit
#else
static bool g_phase2 = false;      // Phase 1 box-room fit (default, robust)
#endif

// ---- decode scratch (half-scale 160x120) ----
static uint16_t g_rgb[160 * 120];
static uint8_t  g_gray[160 * 120];

// ---- app state ----
static AppView g_view = AppView::Project;

// ---- scan state ----
static const uint32_t SCAN_DURATION_MS  = 20000;   // ~20 s for a slow 360 sweep
static const uint32_t SCAN_SAMPLE_MS    = 1000;    // -> ~20 frames
static const float    MERGE_DIST_M      = 0.6f;    // same-class merge radius

struct AccObj { uint8_t cls; float x, y, z, h; int n; };
static AccObj   g_acc[64];
static int      g_acc_count = 0;
static bool     g_scanning = false;
static uint32_t g_scan_start = 0;
static uint32_t g_last_sample = 0;
static uint32_t g_last_sampled_seq = 0;
static char     g_scan_building[48] = {0};
static char     g_scan_room[48] = {0};

// ============================ UART callbacks ===============================
static void on_frame(const uint8_t* jpeg, uint16_t len, void*) {
    g_fb.pushFrame(jpeg, len);
}
static void on_detect(const DetectPacket& det, void*) {
    g_fb.attachDetections(det);
}

// ============================ scan pipeline ================================
static void acc_add(uint8_t cls, const ObjectEstimate& e) {
    for (int i = 0; i < g_acc_count; ++i) {
        if (g_acc[i].cls != cls) continue;
        float dx = g_acc[i].x - e.x, dz = g_acc[i].z - e.z;
        if (dx*dx + dz*dz <= MERGE_DIST_M * MERGE_DIST_M) {
            // running average
            int n = g_acc[i].n;
            g_acc[i].x = (g_acc[i].x*n + e.x) / (n+1);
            g_acc[i].y = (g_acc[i].y*n + e.y) / (n+1);
            g_acc[i].z = (g_acc[i].z*n + e.z) / (n+1);
            g_acc[i].h = (g_acc[i].h*n + e.real_height) / (n+1);
            g_acc[i].n = n + 1;
            return;
        }
    }
    if (g_acc_count < (int)(sizeof(g_acc)/sizeof(g_acc[0]))) {
        g_acc[g_acc_count++] = { cls, e.x, e.y, e.z, e.real_height, 1 };
    }
}

static void scan_sample(const FrameSlot* slot, float yaw) {
    // decode for preview + grayscale Hough input
    int w = 0, h = 0;
    if (jpeg_decode_half_rgb565(slot->jpeg, slot->jpeg_len, g_rgb, &w, &h)) {
        scan_view_update_preview(g_rgb, w, h);
        rgb565_to_gray(g_rgb, w, h, g_gray);
        if (g_phase2) g_slam.addFrame(g_gray, w, h);   // ORB SfM -> point cloud
        else          g_fitter.addFrame(g_gray, yaw);  // Hough box fit
    }
    // place each detected, known-size object
    for (uint8_t i = 0; i < slot->det_count; ++i) {
        const Detection& d = slot->dets[i];
        if (!g_depth.isKnown(d.class_id)) continue;
        // detections are in full-res (320x240) pixels; depth estimator is set up
        // for 160x120, so halve the bbox to match.
        ObjectEstimate e = g_depth.estimate(d.class_id, d.x/2, d.w/2, d.h/2, yaw);
        if (e.valid) acc_add(d.class_id, e);
    }
}

// Build the labelled-object list shared by both finish paths.
static void build_object_list(MeshObject* objs, uint8_t& n) {
    n = 0;
    for (int i = 0; i < g_acc_count && n < MESH_MAX_OBJECTS; ++i) {
        uint32_t col = object_labels::color(g_acc[i].cls);
        MeshObject& o = objs[n++];
        strncpy(o.label, object_labels::name(g_acc[i].cls), MESH_MAX_LABEL);
        o.label[MESH_MAX_LABEL] = '\0';
        o.bbox[0] = g_acc[i].x - 0.25f; o.bbox[1] = g_acc[i].x + 0.25f;
        o.bbox[2] = 0.0f;               o.bbox[3] = g_acc[i].h;
        o.bbox[4] = g_acc[i].z - 0.25f; o.bbox[5] = g_acc[i].z + 0.25f;
        o.r = object_labels::red(col); o.g = object_labels::green(col); o.b = object_labels::blue(col);
    }
}

// --- Phase 1: Hough box room + object markers (MeshBuilder + MeshWriter) ---
static bool scan_finish_phase1(const char* path, uint32_t room_id, uint32_t ts) {
    for (int i = 0; i < g_acc_count; ++i) g_fitter.addObjectExtent(g_acc[i].x, g_acc[i].z);
    RoomBox box = g_fitter.fit();

    g_builder.reset();
    g_builder.addBoxRoom(box);
    for (int i = 0; i < g_acc_count; ++i) {
        uint32_t col = object_labels::color(g_acc[i].cls);
        float hy = g_acc[i].h * 0.5f;
        g_builder.addObjectMarker(object_labels::name(g_acc[i].cls),
                                  g_acc[i].x, hy, g_acc[i].z, 0.25f, hy, 0.25f,
                                  object_labels::red(col), object_labels::green(col),
                                  object_labels::blue(col));
    }
    g_builder.finalize();
    return MeshWriter::writeMesh(path, g_builder, room_id, ts);
}

// face sink: stream each reconstructed triangle straight to the writer
static void face_sink(uint16_t a, uint16_t b, uint16_t c, void* user) {
    static_cast<MeshStreamWriter*>(user)->addFace(a, b, c);
}

// --- Phase 2: SfM point cloud -> greedy surface -> incremental SD write ---
static bool scan_finish_phase2(const char* path, uint32_t room_id, uint32_t ts) {
    const cv::PointCloud& pc = g_slam.cloud();
    Serial.printf("[scan] phase2 cloud: %u pts, %d frames\n",
                  (unsigned)pc.count, g_slam.frames());
    if (pc.count < 16) { Serial.println("[scan] phase2: too few points"); return false; }
    if (pc.count > 65000) { Serial.println("[scan] phase2: >65535 verts, clamping"); }

    MeshStreamWriter w;
    if (!w.begin(path, room_id, ts, pc.bounds)) return false;

    w.beginVertices();
    uint32_t vmax = pc.count > 65000 ? 65000 : pc.count;   // uint16 face indices
    for (uint32_t i = 0; i < vmax; ++i)
        w.addVertex(pc.xyz[i*3], pc.xyz[i*3+1], pc.xyz[i*3+2]);
    w.endVertices();

    w.beginFaces();
    cv::SurfaceReconParams prm; prm.max_edge = 0.5f; prm.k = 8; prm.max_faces = 20000;
    uint32_t nf = cv::greedy_triangulate(pc, prm, face_sink, &w);
    w.endFaces();

    MeshObject objs[MESH_MAX_OBJECTS]; uint8_t no;
    build_object_list(objs, no);
    w.writeObjects(objs, no);
    w.finish();
    Serial.printf("[scan] phase2 mesh written: %u verts, %u faces\n", (unsigned)vmax, (unsigned)nf);
    return true;
}

static void scan_finish() {
    g_scanning = false;
    char path[200];
    bool ok = false;
    if (g_pm.meshPath(g_scan_building, g_scan_room, path, sizeof(path))) {
        uint32_t room_id = (uint32_t)millis();
        uint32_t ts = (uint32_t)time(nullptr);
        ok = g_phase2 ? scan_finish_phase2(path, room_id, ts)
                      : scan_finish_phase1(path, room_id, ts);
        if (ok) g_renderer.loadFromFile(path);
        else    Serial.println("[scan] mesh write failed");
    }
    app_show_view(AppView::Mesh);
    mesh_view_set_title(g_scan_room);
}

static void scan_tick() {
    if (!g_scanning) return;
    uint32_t now = millis();
    uint32_t elapsed = now - g_scan_start;
    if (elapsed >= SCAN_DURATION_MS) { scan_finish(); return; }

    if (now - g_last_sample >= SCAN_SAMPLE_MS) {
        const FrameSlot* slot = g_fb.latest();
        if (slot && slot->seq != g_last_sampled_seq) {
            g_last_sampled_seq = slot->seq;
            float yaw = 2.0f * (float)M_PI * (float)elapsed / (float)SCAN_DURATION_MS;
            scan_sample(slot, yaw);
            g_last_sample = now;
            scan_view_set_progress((int)(100 * elapsed / SCAN_DURATION_MS));
        }
    }
}

// ============================ app actions =================================
void app_show_view(AppView v) {
    // tear down current
    switch (g_view) {
        case AppView::Project: project_view_destroy(); break;
        case AppView::Scan:    scan_view_destroy();    break;
        case AppView::Mesh:    mesh_view_destroy();    break;
        case AppView::Edit:    edit_view_destroy();    break;
    }
    g_view = v;
#if MESHSCAN_HAVE_LVGL
    lv_obj_t* scr = lv_scr_act();
#else
    lv_obj_t* scr = nullptr;
#endif
    switch (v) {
        case AppView::Project: project_view_create(scr); break;
        case AppView::Scan:    scan_view_create(scr);    break;
        case AppView::Mesh:    mesh_view_create(scr);    break;
        case AppView::Edit:    edit_view_create(scr);    break;
    }
}

void app_create_building(const char* name) { g_pm.createBuilding(name); }
void app_delete_building(const char* name) { g_pm.deleteBuilding(name); }
void app_delete_room(const char* b, const char* r) { g_pm.deleteRoom(b, r); }

void app_open_room(const char* building, const char* room) {
    char path[200];
    if (g_pm.meshPath(building, room, path, sizeof(path)) && g_renderer.loadFromFile(path)) {
        app_show_view(AppView::Mesh);
        mesh_view_set_title(room);
    } else {
        Serial.printf("[app] open_room failed: %s/%s\n", building, room);
    }
}

void app_request_scan_start(const char* building, const char* room) {
    strncpy(g_scan_building, building, sizeof(g_scan_building) - 1);
    g_scan_building[sizeof(g_scan_building)-1] = '\0';
    if (room && room[0]) {
        strncpy(g_scan_room, room, sizeof(g_scan_room) - 1);
    } else {
        g_pm.suggestRoomName(building, g_scan_room, sizeof(g_scan_room));
    }
    g_scan_room[sizeof(g_scan_room)-1] = '\0';

    g_acc_count = 0;
    g_fitter.reset();
    g_slam.reset();
    g_scanning = true;
    g_scan_start = millis();
    g_last_sample = 0;
    g_last_sampled_seq = 0;

    app_show_view(AppView::Scan);
    scan_view_set_status("Rotate slowly 360 degrees");
    scan_view_set_progress(0);
}

void app_request_scan_stop() {
    if (g_scanning) scan_finish();
    else app_show_view(AppView::Project);
}

void app_save_room_label(const char* building, const char* room, const char* new_label) {
    // TODO: rename the .mesh file and (optionally) rewrite its room_id/label.
    Serial.printf("[app] save label %s/%s -> %s\n", building, room, new_label);
    (void)building; (void)room; (void)new_label;
}

int app_list_buildings(char names[][48], int max) {
    NameList nl; g_pm.listBuildings(nl);
    int n = nl.count < max ? nl.count : max;
    for (int i = 0; i < n; ++i) { strncpy(names[i], nl.items[i], 47); names[i][47] = '\0'; }
    return n;
}
int app_list_rooms(const char* building, char names[][48], int max) {
    NameList nl; g_pm.listRooms(building, nl);
    int n = nl.count < max ? nl.count : max;
    for (int i = 0; i < n; ++i) { strncpy(names[i], nl.items[i], 47); names[i][47] = '\0'; }
    return n;
}

void app_set_scan_phase2(bool on) { g_phase2 = on; }   // UI toggle hook (TODO: button)
bool app_get_scan_phase2() { return g_phase2; }

void app_render_mesh(uint16_t* fb, int /*w*/, int /*h*/) { g_renderer.draw(fb); }
void app_mesh_orbit(float daz, float del) { g_renderer.orbit(daz, del); }
void app_mesh_zoom(float dd) { g_renderer.zoom(dd); }

// ============================ display / LVGL =============================
#if MESHSCAN_HAVE_LVGL
// TODO: replace these scaffolds with the M5Tab panel driver (M5GFX/LovyanGFX).
// A real flush_cb pushes the dirty area to the panel; a real touch read_cb
// reports the touch controller state. Without them LVGL renders nowhere.
static lv_disp_draw_buf_t s_draw_buf;
static lv_color_t*        s_lvbuf = nullptr;
static const int          LV_HOR = 1024, LV_VER = 600;

static void lv_flush_cb(lv_disp_drv_t* drv, const lv_area_t* /*area*/, lv_color_t* /*px*/) {
    // TODO: panel->pushImageDMA(area, px);
    lv_disp_flush_ready(drv);
}
static void lv_touch_cb(lv_indev_drv_t*, lv_indev_data_t* data) {
    // TODO: read your touch controller; set data->state + data->point.
    data->state = LV_INDEV_STATE_REL;
}

static void lvgl_init() {
    lv_init();
    size_t px = LV_HOR * 40;
    s_lvbuf = (lv_color_t*)heap_caps_malloc(px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    lv_disp_draw_buf_init(&s_draw_buf, s_lvbuf, nullptr, px);

    static lv_disp_drv_t ddrv; lv_disp_drv_init(&ddrv);
    ddrv.hor_res = LV_HOR; ddrv.ver_res = LV_VER;
    ddrv.flush_cb = lv_flush_cb; ddrv.draw_buf = &s_draw_buf;
    lv_disp_drv_register(&ddrv);

    static lv_indev_drv_t idrv; lv_indev_drv_init(&idrv);
    idrv.type = LV_INDEV_TYPE_POINTER; idrv.read_cb = lv_touch_cb;
    lv_indev_drv_register(&idrv);
}
#endif

// ============================ setup / loop ===============================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("MeshScan M5Tab boot");

    // SD over SPI
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
    if (!SD.begin(SD_CS_PIN, SPI)) Serial.println("[boot] SD init FAILED");

    // subsystems
    if (!g_fb.begin())      Serial.println("[boot] frame buffer alloc FAILED");
    if (!g_fitter.begin())  Serial.println("[boot] room fitter alloc FAILED");
    if (!g_builder.begin()) Serial.println("[boot] mesh builder alloc FAILED");
    if (!g_slam.begin(g_K, 20000)) Serial.println("[boot] SfM pipeline alloc FAILED");
    g_pm.begin();

    // UART from Unit V
    if (!g_uart.begin(Serial2, UNITV_UART_BAUD, UNITV_UART_RX, UNITV_UART_TX))
        Serial.println("[boot] UART jpeg buffer alloc FAILED");
    g_uart.onFrame(on_frame, nullptr);
    g_uart.onDetect(on_detect, nullptr);

#if MESHSCAN_HAVE_LVGL
    lvgl_init();
#endif
    app_show_view(AppView::Project);
}

void loop() {
    g_uart.update();      // drain UART, fire frame/detect callbacks
    scan_tick();          // advance the scan state machine
#if MESHSCAN_HAVE_LVGL
    lv_timer_handler();   // LVGL tick + rendering
#endif
    delay(2);
}
