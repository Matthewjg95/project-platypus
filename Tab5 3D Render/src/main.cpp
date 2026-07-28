// ============================================================
// main.cpp — M5View Shell Entry Point
// ============================================================

#define MESH_LOADER_IMPL
#define RENDERER_IMPL

#include <M5Unified.h>
#include <M5GFX.h>
#include <math.h>
#include <esp_heap_caps.h>
#include <SD_MMC.h>
#include <algorithm>

#include <WiFi.h>
#include <ArduinoOTA.h>

#include "mesh.h"
#include "mesh_loader.h"
#include "renderer.h"
#include "applet.h"
#include "ui_constants.h"
#include "shell.h"
#include "viewer_applet.h"
#include "scanner/scanner_applet.h"
#include "antenna/antenna_applet.h"
#include "wifi_creds.h"          // gitignored: OTA_WIFI_SSID/PASS/HOSTNAME

// The scan-finish path (mesh writer + geometry + SD + renderer frames deep in
// call chains) brushed the default 8KB Arduino loop stack — intermittent panic
// at scan completion. Give the loop task real headroom.
SET_LOOP_TASK_STACK_SIZE(16 * 1024);

static Shell shell;

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 3000) delay(10);
    Serial.println("\n=== M5View booting ===");

    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setRotation(1);
    M5.Display.fillScreen(TFT_BLACK);

    Serial.printf("Display: %dx%d\n",
                  M5.Display.width(), M5.Display.height());

    // SD card
    SD_MMC.setPins(12, 13, 11, 10, 9, 8);
    if (!SD_MMC.begin("/sdcard", false)) {
        Serial.println("SD mount FAILED");
        M5.Display.setTextColor(TFT_YELLOW);
        M5.Display.setTextSize(2);
        M5.Display.setCursor(10, 10);
        M5.Display.println("SD card not found");
        M5.Display.println("Insert SD and restart");
        delay(3000);
    } else {
        Serial.println("SD mounted OK");
    }

    // Register applets — Settings is panel-only, not a tile
    shell.register_applet(new ViewerApplet());
    shell.register_applet(new ScannerApplet());
    shell.register_applet(new AntennaApplet());
    // Future applets go here:
    // shell.register_applet(new NotesApplet());

    shell.begin(M5.Display.width(), M5.Display.height());

    Serial.println("M5View ready");
}

// ---- serial screenshot -------------------------------------
// PC sends the two bytes "SS" over USB-CDC; we reply
//   "SCRB" | u16 width | u16 height | width*height RGB565 (LE)
// streamed row by row (2.5KB buffer). tools/tab5_screenshot.py drives this
// and saves a PNG/BMP — pixel-perfect captures with no SD involved, works in
// every applet state.
static void serial_screenshot() {
    int w = M5.Display.width(), h = M5.Display.height();
    static uint16_t row[1280];
    if (w > 1280) w = 1280;
    Serial.write((const uint8_t*)"SCRB", 4);
    uint16_t dims[2] = { (uint16_t)w, (uint16_t)h };
    Serial.write((const uint8_t*)dims, 4);
    for (int y = 0; y < h; ++y) {
        M5.Display.readRect(0, y, w, 1, row);
        Serial.write((const uint8_t*)row, w * 2);
    }
    Serial.flush();
}

static void poll_serial_commands() {
    static uint8_t last = 0;
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();
        if (last == 'S' && b == 'S') { serial_screenshot(); last = 0; }
        else last = b;
    }
}

// ---- WiFi OTA (home screen only) ---------------------------
// Cable-free flashing: `pio run -e tab5-ota -t upload`. Active ONLY while the
// shell sits on the home screen so it never fights the applets that manage
// WiFi themselves (antenna suite, RF survey). Leaving home tears the state
// down; returning re-joins and re-arms OTA. C6 SDIO pins are set once here.
// (PT_ prefix: ArduinoOTA.h already claims OTA_IDLE)
enum PtOtaState { PT_OTA_IDLE, PT_OTA_JOINING, PT_OTA_READY };
static PtOtaState ota_state = PT_OTA_IDLE;
static uint32_t ota_join_start = 0;
static bool     wifi_pins_set  = false;

static void ota_tick(bool at_home) {
    if (!at_home) {
        if (ota_state != PT_OTA_IDLE) {
            ArduinoOTA.end();
            ota_state = PT_OTA_IDLE;   // applets own the radio from here
        }
        return;
    }
    switch (ota_state) {
    case PT_OTA_IDLE:
        if (!wifi_pins_set) {
            WiFi.setPins(12, 13, 11, 10, 9, 8, 15);   // M5Tab5 C6 hosted-SDIO
            wifi_pins_set = true;
        }
        WiFi.mode(WIFI_STA);
        WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);
        ota_join_start = millis();
        ota_state = PT_OTA_JOINING;
        break;
    case PT_OTA_JOINING:
        if (WiFi.status() == WL_CONNECTED) {
            ArduinoOTA.setHostname(OTA_HOSTNAME);
            ArduinoOTA.begin();
            Serial.printf("[ota] ready: %s @ %s\n", OTA_HOSTNAME,
                          WiFi.localIP().toString().c_str());
            ota_state = PT_OTA_READY;
        } else if (millis() - ota_join_start > 20000) {
            ota_join_start = millis();  // keep retrying quietly
            WiFi.disconnect();
            WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASS);
        }
        break;
    case PT_OTA_READY:
        if (WiFi.status() != WL_CONNECTED) { ota_state = PT_OTA_IDLE; break; }
        ArduinoOTA.handle();
        break;
    }
}

void loop() {
    poll_serial_commands();
    ota_tick(!shell.is_in_applet());
    shell.update();
    shell.render();
}