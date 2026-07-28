/*
 * M5Stack Tab5 — 2.4 GHz Antenna Test Suite
 *
 * Screens (default = Oscilloscope; others enabled in Settings):
 *   Network Select → Oscilloscope → Menu → [Channel Scanner | Polar Plot |
 *   Walk Test | AP Timeline | Multipath | A/B Dwell | Polarisation | Settings]
 *
 * RF switch: PI4IOE5V6408-1  I2C 0x43  pin P0  (LOW=INT, HIGH=EXT)
 * Libraries: M5Unified ≥ 0.1.15 · Arduino-ESP32 ≥ 2.0
 */

#include "antenna_applet.h"
#include <M5Unified.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <cstring>
#include <cmath>

// Packet-loss/ping test needs the external ESPping lib. Off by default so the
// applet builds without it; set to 1 and add `espping` to platformio.ini to use.
#define ANT_ENABLE_PKTLOSS 0
#if ANT_ENABLE_PKTLOSS
#include <ESPping.h>
#endif

// All sketch globals + functions live in this namespace so they don't collide
// with the rest of the firmware. The AntennaApplet methods (defined at the
// bottom of this file) call in.
namespace antenna {

// ═══════════════════════════════════════════════════════════════════════════
//  CONSTANTS
// ═══════════════════════════════════════════════════════════════════════════
static const uint8_t IO_EXP_ADDR  = 0x43;
static const uint8_t IO_EXP_PIN   = 0;
// PI4IOE5V6408 register map — NOT PCA9536-style. 0x01 is Chip ID/Control
// (bit0 = SOFTWARE RESET; writing it blanked the Tab5's screen since other
// rails share this expander). Real registers: 0x03 direction (1=output),
// 0x05 output state, 0x07 output Hi-Z (1=Hi-Z, the power-on default).
static const uint8_t PI4_REG_OUT  = 0x05;
static const uint8_t PI4_REG_DIR  = 0x03;
static const uint8_t PI4_REG_HIZ  = 0x07;

static const uint32_t SAMPLE_MS   = 1000;
static const uint32_t REDRAW_MS   = 500;
static const uint32_t SCAN_DWELL  = 60;
static const int      PLOT_SAMPLES= 200;
static const int      RSSI_TOP    = -30;
static const int      RSSI_BOT    = -100;
static const char*    LOG_FILE    = "/ant_test.csv";

// Set your WiFi password here to enable the Packet Loss test.
// The test connects to the SELECTED network and pings its gateway.
#define PKT_WIFI_PASSWORD  ""   // e.g. "mypassword123"

// ── Colours ──────────────────────────────────────────────────────────────
#define C_BG     0x0000u
#define C_GRID   0x10A2u
#define C_INT    0x07E0u
#define C_EXT    0xFD20u
#define C_AXIS   0x3186u
#define C_HDRBG  0x0841u
#define C_SEP    0x4208u
#define C_WHITE  0xFFFFu
#define C_RED    0xF800u
#define C_INACT  0x2104u
#define C_BINT   0x0300u
#define C_BEXT   0x7800u
#define C_BLOG   0x6000u
#define C_BMENU  0x000Fu
#define C_GOLD   0xFEA0u
#define C_CYAN   0x07FFu
#define C_PURP   0x780Fu

// ═══════════════════════════════════════════════════════════════════════════
//  ENUMS  — avoid "EXTERNAL" (Arduino.h #defines it to 0)
// ═══════════════════════════════════════════════════════════════════════════
enum AntMode { ANT_INT = 0, ANT_EXT = 1 };

enum Screen {
    SCR_SELECT, SCR_SCOPE, SCR_CHANNEL, SCR_POLAR,
    SCR_WALK,   SCR_TIMELINE, SCR_MULTIPATH,
    SCR_ABDWELL,SCR_POL, SCR_PKTLOSS, SCR_SETTINGS, SCR_MENU
};

// ═══════════════════════════════════════════════════════════════════════════
//  STRUCTS
// ═══════════════════════════════════════════════════════════════════════════
struct Stats {
    int8_t cur=-127,mn=0,mx=-127;
    float  avg=0,sum=0; uint32_t n=0;
    void reset(){cur=-127;mn=0;mx=-127;avg=0;sum=0;n=0;}
    void push(int8_t v){
        cur=v; if(!n||v<mn)mn=v; if(v>mx)mx=v;
        sum+=v; n++; avg=sum/n;
    }
    float stddev(int8_t* buf, int len){
        if(len<2) return 0;
        float s=0; for(int i=0;i<len;i++) s+=buf[i];
        float m=s/len; float ss=0;
        for(int i=0;i<len;i++) ss+=(buf[i]-m)*(buf[i]-m);
        return sqrtf(ss/len);
    }
};

struct APInfo {
    char ssid[64]; uint8_t bssid[6];
    int8_t rssi; uint8_t channel; bool encrypted;
};

struct ChanData { int8_t best; uint8_t count; };

struct Settings {
    bool channelScan  = false;
    bool polarPlot    = false;
    bool walkTest     = false;
    bool apTimeline   = false;
    bool multipath    = false;
    bool abDwell      = false;
    bool polarisation = false;
    bool pktLoss      = false;
    uint32_t dwellSec = 30;
    uint32_t walkMs   = 500;
};

// ═══════════════════════════════════════════════════════════════════════════
//  GLOBALS
// ═══════════════════════════════════════════════════════════════════════════
// — display layout —
static int W,H,PX,PY,PW,PH,BY;
static const int HH=52, BH=46;

// — app state —
static Screen   g_screen    = SCR_SELECT;
static Settings g_cfg;
static bool     g_dirty     = true;
static uint32_t g_tDraw     = 0;
static bool     g_wantExit  = false;   // set by Menu->EXIT; leaves the applet

// — antenna —
static AntMode  g_ant       = ANT_INT;
static bool     g_antPending= false;
static AntMode  g_antQueued = ANT_INT;
static uint8_t  g_expCache  = 0x00;
static uint32_t g_settleUntil = 0;

// — WiFi scan —
static bool     g_scanning  = false;
static uint32_t g_tSample   = 0;
static char     g_ssid[64]  = {};
static uint8_t  g_bssid[6]  = {};
static uint8_t  g_targetChannel = 0;   // channel of selected AP — scan only this!
static bool     g_locked    = false;

// — oscilloscope / core —
static int8_t   g_hist[PLOT_SAMPLES];
static AntMode  g_hant[PLOT_SAMPLES];
static int      g_head=0; static bool g_full=false;
static Stats    g_si, g_se;

// — SD logging —
static bool     g_logging   = false;
static File     g_logFile;
static bool     g_sdOk      = false;

// — network selection —
static APInfo   g_apList[40];
static int      g_apCount=0, g_apScroll=0;

// — channel scanner —
static ChanData g_channels[14] = {};   // index 1-13

// — AP timeline —
static const int TL_LEN = 200;
static uint8_t  g_timeline[TL_LEN] = {};
static int      g_tlHead=0; static bool g_tlFull=false;

// — polar plot —
static const int POLAR_N = 36;         // 10° steps
static int8_t   g_polInt[POLAR_N], g_polExt[POLAR_N];
static int      g_polAngle = 0;        // current angle index

// — multipath —
static const int VAR_WIN = 30;
static int8_t   g_varBuf[VAR_WIN] = {};
static int      g_varHead = 0;
static bool     g_varFull = false;
static float    g_varStd  = 0;

// — packet loss test —
static char     g_wifiPass[64] = "";        // set via PKT_WIFI_PASSWORD below
static bool     g_pktConnected = false;
static bool     g_pktConnecting= false;
static uint32_t g_pktConnStart = 0;
static uint32_t g_pktSent=0, g_pktLost=0;
static uint32_t g_pktLastPing=0;
static int32_t  g_pktRttMs=-1;
static float    g_pktAvgRtt=0;
static const int PKT_HIST=60;
static int8_t   g_pktHist[PKT_HIST]={};     // 1=ok 0=lost
static int      g_pktHead=0;

// — A/B dwell & polarisation —
static bool     g_abRunning = false;
static uint32_t g_abLast    = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════════════════════════
bool     expWrite(uint8_t r, uint8_t v);
uint8_t  expRead(uint8_t r);
void     initExpander();
void     setAntenna(AntMode m);
void     doAntSwitch();
void     kickScan();
void     onScanComplete(int n);
void     pushSample(int8_t rssi);
void     navigateTo(Screen s);
void     drawCurrentScreen();
// screens
void     scanForSelection(); void drawSelectScreen(); void handleSelectTouch();
void     drawScopeScreen();  void handleScopeTouch();
void     drawHeader();       void drawGrid(); void drawTrace(); void drawScopeButtons();
void     drawChannelScreen();void handleChannelTouch();
void     drawPolarScreen();  void handlePolarTouch();
void     drawWalkScreen();   void handleWalkTouch();
void     drawTimelineScreen();void handleTimelineTouch();
void     drawMultipathScreen();void handleMultipathTouch();
void     drawAbDwellScreen();void handleAbDwellTouch();
void     drawPolScreen();    void handlePolTouch();
void     drawPktLossScreen();void handlePktLossTouch();
void     pktLossService();
void     drawSettingsScreen();void handleSettingsTouch();
void     drawMenuScreen();   void handleMenuTouch();
// utils
void     startLog(); void stopLog(); void writeLog(uint32_t ts, int8_t rssi);
int      rssiY(int8_t v);
int      rssiToSignalBars(int8_t rssi);
const char* antStr(AntMode m);
void     drawBtnBar(const char* b0, uint16_t c0,
                    const char* b1, uint16_t c1,
                    const char* b2, uint16_t c2,
                    const char* b3, uint16_t c3);

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
// Was setup(): M5/Serial are already initialised by the shell. SD is mounted by
// the shell as SD_MMC, so we just probe it rather than calling begin().
void app_setup() {
    W=M5.Display.width(); H=M5.Display.height();
    BY=H-BH; PX=32; PY=HH; PW=W-PX; PH=BY-PY;
    g_wantExit=false;
    Serial.printf("[ANT] %dx%d\n",W,H);

    for(int i=0;i<PLOT_SAMPLES;i++){g_hist[i]=(int8_t)RSSI_BOT;g_hant[i]=ANT_INT;}
    for(int i=0;i<POLAR_N;i++){g_polInt[i]=-127;g_polExt[i]=-127;}
    M5.Display.fillScreen(C_BG);

    initExpander();
    setAntenna(ANT_INT);
    // ESP32-P4 has no radio: WiFi runs on the onboard ESP32-C6 over a hosted
    // SDIO link. The arduino-esp32 default targets the P4 EV-board pins, which
    // are WRONG for the M5Tab5 -> "H_SDIO_DRV: sdio card init failed" abort.
    // Override with the Tab5's actual C6 SDIO pins BEFORE any WiFi init.
    //   clk=12 cmd=13 d0=11 d1=10 d2=9 d3=8 reset=15  (slot 1)
    WiFi.setPins(12, 13, 11, 10, 9, 8, 15);
    WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(100);
    g_sdOk = (SD_MMC.cardType() != CARD_NONE);
    Serial.printf("[SD] %s\n", g_sdOk?"ok":"no card");

    g_screen=SCR_SELECT;
    scanForSelection();
    drawSelectScreen();
    Serial.println("[ANT READY]");
}

// Released when the applet is left (Menu->EXIT or shell exit).
void app_teardown() {
    stopLog();
    g_abRunning=false; g_scanning=false; g_antPending=false;
    WiFi.disconnect(); delay(20);
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
// Was loop(): called once per frame from AntennaApplet::on_update().
void app_loop() {
    static uint32_t loopEnter=0;
    loopEnter++;
    if(loopEnter<10 || loopEnter%50==0){
        Serial.printf("[LOOP %lu] enter screen=%d\n", loopEnter, g_screen);
        Serial.flush();
    }
    M5.update();

    // Touch dispatch
    switch(g_screen){
        case SCR_SELECT:    handleSelectTouch();   delay(20); return;
        case SCR_SCOPE:     handleScopeTouch();    break;
        case SCR_CHANNEL:   handleChannelTouch();  break;
        case SCR_POLAR:     handlePolarTouch();    break;
        case SCR_WALK:      handleWalkTouch();     break;
        case SCR_TIMELINE:  handleTimelineTouch(); break;
        case SCR_MULTIPATH: handleMultipathTouch();break;
        case SCR_ABDWELL:   handleAbDwellTouch();  break;
        case SCR_POL:       handlePolTouch();      break;
        case SCR_PKTLOSS:   handlePktLossTouch();  break;
        case SCR_SETTINGS:  handleSettingsTouch(); break;
        case SCR_MENU:      handleMenuTouch();     break;
        default: break;
    }
    
    // Debug: print loop status every 100 ticks
    static uint32_t loopCounter=0;
    if((++loopCounter % 100)==0) 
        Serial.printf("[LOOP tick %lu] screen=%d locked=%d scanning=%d\n", loopCounter, g_screen, g_locked, g_scanning);

    uint32_t now = millis();

    // Packet loss test service (connection mgmt + pings)
    if(g_screen==SCR_PKTLOSS) pktLossService();

    // A/B auto-switch (dwell + polarisation screens)
    if(g_abRunning && (g_screen==SCR_ABDWELL||g_screen==SCR_POL)){
        if(now - g_abLast >= g_cfg.dwellSec*1000UL){
            setAntenna(g_ant==ANT_INT ? ANT_EXT : ANT_INT);
            g_abLast = now;
        }
    }

    // Antenna switch (when radio idle) - check touch right before & after
    if(g_antPending && !g_scanning){
        M5.update();  // drain touch events first
        doAntSwitch();
        M5.update();
    }

    // Kick scan — but FIRST give touch handling another pass
    M5.update();
    switch(g_screen){
        case SCR_SCOPE:     handleScopeTouch();    break;
        case SCR_CHANNEL:   handleChannelTouch();  break;
        case SCR_POLAR:     handlePolarTouch();    break;
        case SCR_WALK:      handleWalkTouch();     break;
        case SCR_TIMELINE:  handleTimelineTouch(); break;
        case SCR_MULTIPATH: handleMultipathTouch();break;
        case SCR_ABDWELL:   handleAbDwellTouch();  break;
        case SCR_POL:       handlePolTouch();      break;
        case SCR_PKTLOSS:   handlePktLossTouch();  break;
        case SCR_SETTINGS:  handleSettingsTouch(); break;
        case SCR_MENU:      handleMenuTouch();     break;
        default: break;
    }
    
    uint32_t interval = SAMPLE_MS;
    if(g_screen==SCR_MULTIPATH) interval = 200;
    else if(g_screen==SCR_WALK) interval = g_cfg.walkMs;
    if(g_screen!=SCR_PKTLOSS
       && !g_antPending && !g_scanning && now>g_settleUntil
       && (now-g_tSample>=interval)){
        g_tSample=now;
        kickScan();
        M5.update();  // drain touch events that came in during scan
    }

    // Redraw
    if(g_dirty && now-g_tDraw>=REDRAW_MS){
        g_tDraw=now; g_dirty=false;
        drawCurrentScreen();
    }
    delay(5);  // shorter delay for more responsive touch
}

// ═══════════════════════════════════════════════════════════════════════════
//  CORE: EXPANDER + ANTENNA
// ═══════════════════════════════════════════════════════════════════════════
// The PI4IOE5V6408 RF-switch expander sits on the Tab5's INTERNAL I2C bus.
// The standalone sketch used raw `Wire` without begin() (the Arduino M5 board
// package did it implicitly), which under this shell produced "NULL TX buffer"
// errors and a dead INT/EXT switch. Use M5Unified's already-initialised
// internal bus instead of raw Wire.
static const uint32_t EXP_I2C_FREQ = 100000;

bool expWrite(uint8_t reg, uint8_t val){
    return M5.In_I2C.writeRegister8(IO_EXP_ADDR, reg, val, EXP_I2C_FREQ);
}
uint8_t expRead(uint8_t reg){
    return M5.In_I2C.readRegister8(IO_EXP_ADDR, reg, EXP_I2C_FREQ);
}
void initExpander(){
    uint8_t dir=expRead(PI4_REG_DIR); dir|=(1u<<IO_EXP_PIN);   // 1 = output
    expWrite(PI4_REG_DIR,dir);
    uint8_t hiz=expRead(PI4_REG_HIZ); hiz&=~(1u<<IO_EXP_PIN);  // drive the pin
    expWrite(PI4_REG_HIZ,hiz);
    g_expCache=expRead(PI4_REG_OUT); g_expCache&=~(1u<<IO_EXP_PIN);
    expWrite(PI4_REG_OUT,g_expCache);
    Serial.printf("[EXP] 0x%02X P%d ready\n",IO_EXP_ADDR,IO_EXP_PIN);
}
void setAntenna(AntMode m){ g_antQueued=m; g_antPending=true; }
void doAntSwitch(){
    AntMode m=g_antQueued; g_antPending=false; g_ant=m;
    Serial.printf("[ANT-SW] doAntSwitch entered, mode=%s\n", antStr(m));
    Serial.flush();
    if(m==ANT_EXT) g_expCache|=(1u<<IO_EXP_PIN);
    else           g_expCache&=~(1u<<IO_EXP_PIN);
    bool ok=expWrite(PI4_REG_OUT,g_expCache);
    Serial.printf("[ANT] -> %s P%d=%s %s\n",
        antStr(m),IO_EXP_PIN,m==ANT_EXT?"HIGH":"LOW",ok?"ok":"ERR");
    Serial.flush();
    if(m==ANT_INT) g_si.reset(); else g_se.reset();
    g_varHead=0; g_varFull=false; g_varStd=0;   // variance window resets per antenna
    g_locked=true;  // KEEP locked — we already selected the BSSID
    g_tSample=millis();
    g_settleUntil=millis()+1000; g_dirty=true;
    Serial.println("[ANT-SW] doAntSwitch exit");
    Serial.flush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  CORE: SCAN + DATA
// ═══════════════════════════════════════════════════════════════════════════
void kickScan(){
    if(g_scanning){ Serial.println("[SCAN] already scanning, skip"); return; }
    g_scanning=true;
    uint32_t scanStart = millis();
    
    // Scan ONLY the target channel (massive speedup vs all-channel scan)
    // WiFi.scanNetworks(async, show_hidden, passive, max_ms_per_channel, channel)
    int n;
    if(g_targetChannel >= 1 && g_targetChannel <= 13){
        Serial.printf("[SCAN] single-channel scan ch=%d\n", g_targetChannel);
        n = WiFi.scanNetworks(false, false, false, SCAN_DWELL, g_targetChannel);
    } else {
        Serial.println("[SCAN] full scan (no target channel set)");
        n = WiFi.scanNetworks(false, false, false, SCAN_DWELL);
    }
    
    uint32_t scanDur = millis() - scanStart;
    Serial.printf("[SCAN] returned n=%d in %lums\n", n, scanDur);
    Serial.flush();
    if(n >= 0){
        if(g_screen==SCR_SCOPE || g_screen==SCR_WALK || g_screen==SCR_MULTIPATH
           || g_screen==SCR_ABDWELL || g_screen==SCR_POL){
            onScanComplete(n);
        }
        WiFi.scanDelete();
    }
    g_scanning=false;
    g_dirty = true;
    
    // Force draw NOW (don't wait for redraw gate) so user sees something
    if(g_screen==SCR_SCOPE || g_screen==SCR_WALK || g_screen==SCR_MULTIPATH
       || g_screen==SCR_ABDWELL || g_screen==SCR_POL){
        drawCurrentScreen();
        g_dirty = false;
        g_tDraw = millis();
    }
}

void onScanComplete(int n){
    Serial.printf("[SCAN] complete, found %d networks, locked=%d, looking for %02X:%02X:%02X:%02X:%02X:%02X\n",
        n, g_locked, g_bssid[0],g_bssid[1],g_bssid[2],g_bssid[3],g_bssid[4],g_bssid[5]);
    
    // ── Main RSSI tracking ──────────────────────────────────────────────
    if(g_locked){
        bool found=false;
        for(int i=0;i<n;i++){
            uint8_t* bssid=WiFi.BSSID(i);
            if(memcmp(bssid,g_bssid,6)==0){
                int8_t rssi=(int8_t)WiFi.RSSI(i);
                Serial.printf("[SCAN] found target AP at index %d: %d dBm\n", i, rssi);
                pushSample(rssi);
                found=true; break;
            }
        }
        if(!found) Serial.println("[SCAN] target BSSID not found in scan results");
    }
    // ── Channel scanner ─────────────────────────────────────────────────
    for(int ch=1;ch<=13;ch++){g_channels[ch].count=0;g_channels[ch].best=-100;}
    for(int i=0;i<n;i++){
        int ch=WiFi.channel(i);
        if(ch>=1&&ch<=13){
            g_channels[ch].count++;
            int8_t r=(int8_t)WiFi.RSSI(i);
            if(r>g_channels[ch].best) g_channels[ch].best=r;
        }
    }
    // ── AP timeline ─────────────────────────────────────────────────────
    g_timeline[g_tlHead]=(uint8_t)min(n,255);
    g_tlHead=(g_tlHead+1)%TL_LEN;
    if(!g_tlHead) g_tlFull=true;

    g_dirty=true;
}

void pushSample(int8_t rssi){
    Serial.printf("[SAMPLE] enter: %s = %d dBm\n", antStr(g_ant), rssi);
    Serial.flush();
    
    g_hist[g_head]=rssi; g_hant[g_head]=g_ant;
    g_head=(g_head+1)%PLOT_SAMPLES; if(!g_head) g_full=true;
    
    Serial.println("[SAMPLE] history updated");
    Serial.flush();
    
    if(g_ant==ANT_INT) g_si.push(rssi); else g_se.push(rssi);
    
    Serial.printf("[SAMPLE] stats updated: n=%lu avg=%.1f\n",
        (g_ant==ANT_INT?g_si.n:g_se.n),
        (g_ant==ANT_INT?g_si.avg:g_se.avg));
    Serial.flush();
    
    // multipath variance window
    g_varBuf[g_varHead]=rssi; g_varHead=(g_varHead+1)%VAR_WIN;
    if(!g_varHead) g_varFull=true;
    int count=g_varFull?VAR_WIN:g_varHead;
    if(count>=2) g_varStd=g_si.stddev(g_varBuf,count);
    
    g_dirty=true;  // <<< CRITICAL: trigger screen redraw!
    
    Serial.println("[SAMPLE] exit OK");
    Serial.flush();
    
    if(g_logging) writeLog(millis(),rssi);
}

// ═══════════════════════════════════════════════════════════════════════════
//  NAVIGATION
// ═══════════════════════════════════════════════════════════════════════════
void navigateTo(Screen s){
    // Leaving the packet-loss screen: disconnect, back to scan mode
    if(g_screen==SCR_PKTLOSS && s!=SCR_PKTLOSS && g_pktConnected){
        WiFi.disconnect();
        g_pktConnected=false; g_pktConnecting=false;
        Serial.println("[PKT] disconnected, resume scan mode");
    }
    g_screen=s; g_dirty=true;
    if(s==SCR_ABDWELL||s==SCR_POL){ g_abRunning=false; g_abLast=millis(); }
    M5.Display.fillScreen(C_BG);
    drawCurrentScreen();
}

void drawCurrentScreen(){
    switch(g_screen){
        case SCR_SCOPE:     drawScopeScreen();     break;
        case SCR_CHANNEL:   drawChannelScreen();   break;
        case SCR_POLAR:     drawPolarScreen();     break;
        case SCR_WALK:      drawWalkScreen();      break;
        case SCR_TIMELINE:  drawTimelineScreen();  break;
        case SCR_MULTIPATH: drawMultipathScreen(); break;
        case SCR_ABDWELL:   drawAbDwellScreen();   break;
        case SCR_POL:       drawPolScreen();       break;
        case SCR_PKTLOSS:   drawPktLossScreen();   break;
        case SCR_SETTINGS:  drawSettingsScreen();  break;
        case SCR_MENU:      drawMenuScreen();      break;
        default: break;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SHARED DRAWING HELPERS
// ═══════════════════════════════════════════════════════════════════════════
int rssiY(int8_t v){
    int c=(int)v; if(c>RSSI_TOP)c=RSSI_TOP; if(c<RSSI_BOT)c=RSSI_BOT;
    return PY+(int)((float)(RSSI_TOP-c)/(float)(RSSI_TOP-RSSI_BOT)*PH);
}
int rssiToSignalBars(int8_t r){
    return r>=-50?5:r>=-60?4:r>=-70?3:r>=-80?2:1;
}
const char* antStr(AntMode m){ return m==ANT_INT?"INT":"EXT"; }

// 4-button bar at bottom
void drawBtnBar(const char* b0,uint16_t c0,const char* b1,uint16_t c1,
                const char* b2,uint16_t c2,const char* b3,uint16_t c3){
    M5.Display.drawLine(0,BY,W,BY,C_SEP);
    int bw=W/4;
    const char* lbl[4]={b0,b1,b2,b3};
    uint16_t    col[4]={c0,c1,c2,c3};
    for(int i=0;i<4;i++){
        int x=i*bw;
        M5.Display.fillRect(x,BY,bw,BH,col[i]);
        M5.Display.drawRect(x,BY,bw,BH,C_SEP);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(C_WHITE,col[i]);
        int len=strlen(lbl[i])*12;
        M5.Display.setCursor(x+(bw-len)/2, BY+12);
        M5.Display.print(lbl[i]);
    }
}

// Standard screen header bar
void drawScreenHeader(const char* title, uint16_t col){
    M5.Display.fillRect(0,0,W,HH,C_HDRBG);
    M5.Display.drawLine(0,HH-1,W,HH-1,C_SEP);
    M5.Display.setTextSize(2); M5.Display.setTextColor(col,C_HDRBG);
    M5.Display.setCursor(12,14); M5.Display.print(title);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: OSCILLOSCOPE (default)
// ═══════════════════════════════════════════════════════════════════════════
void drawHeader(){
    M5.Display.fillRect(0,0,W,HH,C_HDRBG);
    M5.Display.drawLine(0,HH-1,W,HH-1,C_SEP);
    Stats& st=(g_ant==ANT_INT)?g_si:g_se;
    uint16_t vc=(st.cur>=-65)?C_INT:(st.cur>=-80)?C_EXT:C_RED;
    M5.Display.setTextSize(3); M5.Display.setTextColor(vc,C_HDRBG);
    M5.Display.setCursor(4,4); M5.Display.printf("%4d",(int)st.cur);
    M5.Display.setTextSize(1); M5.Display.setTextColor(0x67E0,C_HDRBG);
    M5.Display.setCursor(76,10); M5.Display.print("dBm");
    uint16_t ac=(g_ant==ANT_INT)?C_INT:C_EXT;
    M5.Display.setTextSize(2); M5.Display.setTextColor(ac,C_HDRBG);
    M5.Display.setCursor(130,4); M5.Display.printf("[%s]",antStr(g_ant));
    if(g_si.n>0&&g_se.n>0){
        float d=g_se.avg-g_si.avg;
        uint16_t dc=(d>=1.0f)?C_EXT:(d<=-1.0f)?C_INT:C_AXIS;
        M5.Display.setTextSize(1); M5.Display.setTextColor(dc,C_HDRBG);
        M5.Display.setCursor(240,4);
        if(d>=1.0f)       M5.Display.printf("d:EXT+%4.1fdB  ",d);
        else if(d<=-1.0f) M5.Display.printf("d:INT+%4.1fdB  ",-d);
        else              M5.Display.print( "d:  equal     ");
    }
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_HDRBG);
    M5.Display.setCursor(W-220,4);
    if(strlen(g_ssid)) M5.Display.printf("AP: %-20.20s",g_ssid);
    else               M5.Display.printf("%-24s",g_scanning?"scanning...":"no AP");
    if(g_logging){
        M5.Display.fillRoundRect(W-44,2,40,14,3,C_RED);
        M5.Display.setTextColor(C_WHITE,C_RED);
        M5.Display.setCursor(W-40,5); M5.Display.print(" REC");
    }
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(g_si.n?C_INT:C_HDRBG,C_HDRBG);
    M5.Display.setCursor(4,36);
    if(g_si.n) M5.Display.printf("INT  n:%-4u  mn:%4d  mx:%4d  av:%5.1f",
                                  g_si.n,g_si.mn,g_si.mx,g_si.avg);
    else       M5.Display.print( "INT  --                              ");
    M5.Display.setTextColor(g_se.n?C_EXT:C_HDRBG,C_HDRBG);
    M5.Display.setCursor(W/2+10,36);
    if(g_se.n) M5.Display.printf("EXT  n:%-4u  mn:%4d  mx:%4d  av:%5.1f",
                                  g_se.n,g_se.mn,g_se.mx,g_se.avg);
    else       M5.Display.print( "EXT  --                              ");
}

void drawGrid(){
    M5.Display.fillRect(PX,PY,PW,PH,C_BG);
    for(int d=RSSI_TOP;d>=RSSI_BOT;d-=10){
        int y=rssiY((int8_t)d);
        M5.Display.drawLine(PX,y,PX+PW,y,C_GRID);
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
        M5.Display.setCursor(1,y-4); M5.Display.printf("%d",d);
    }
    for(int i=1;i<10;i++){
        int x=PX+PW*i/10;
        M5.Display.drawLine(x,PY,x,BY,C_GRID);
    }
    M5.Display.drawRect(PX,PY,PW,PH,C_SEP);
}

void drawTrace(){
    int count=g_full?PLOT_SAMPLES:g_head; if(count<2)return;
    for(int i=1;i<count;i++){
        int ia=(g_head-count+i-1+PLOT_SAMPLES)%PLOT_SAMPLES;
        int ib=(g_head-count+i  +PLOT_SAMPLES)%PLOT_SAMPLES;
        int x1=PX+(i-1)*PW/PLOT_SAMPLES, x2=PX+i*PW/PLOT_SAMPLES;
        int y1=rssiY(g_hist[ia]),         y2=rssiY(g_hist[ib]);
        uint16_t col=(g_hant[ib]==ANT_INT)?C_INT:C_EXT;
        M5.Display.drawLine(x1,y1,x2,y2,col);
        M5.Display.drawLine(x1,y1+1,x2,y2+1,col);
    }
    if(g_si.n){ int y=rssiY((int8_t)g_si.avg);
        for(int x=PX;x<PX+PW;x+=8) M5.Display.drawLine(x,y,min(x+4,PX+PW),y,C_INT);
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_INT,C_BG);
        M5.Display.setCursor(PX+4,y-9); M5.Display.printf("INT avg:%.1f",g_si.avg);
    }
    if(g_se.n){ int y=rssiY((int8_t)g_se.avg);
        for(int x=PX;x<PX+PW;x+=8) M5.Display.drawLine(x,y,min(x+4,PX+PW),y,C_EXT);
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_EXT,C_BG);
        M5.Display.setCursor(PX+4,y-9); M5.Display.printf("EXT avg:%.1f",g_se.avg);
    }
}

void drawScopeScreen(){
    drawHeader(); drawGrid(); drawTrace();
    drawBtnBar("INT ANT",(g_ant==ANT_INT)?C_BINT:C_INACT,
               "EXT MMCX",(g_ant==ANT_EXT)?C_BEXT:C_INACT,
               g_logging?"STOP LOG":"LOG CSV",g_logging?C_BLOG:C_INACT,
               "MENU",C_BMENU);
}

void handleScopeTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    Serial.printf("[SCOPE-TOUCH] tap x=%d y=%d BY=%d\n", t.x, t.y, BY);
    Serial.flush();
    if(t.y<BY)return;
    int bw=W/4;
    if     (t.x<bw){   Serial.println("[SCOPE-TOUCH] INT button"); setAntenna(ANT_INT); }
    else if(t.x<bw*2){ Serial.println("[SCOPE-TOUCH] EXT button"); setAntenna(ANT_EXT); }
    else if(t.x<bw*3){ Serial.println("[SCOPE-TOUCH] LOG button"); if(!g_sdOk)return; g_logging?stopLog():startLog(); g_dirty=true; }
    else             { Serial.println("[SCOPE-TOUCH] MENU button"); navigateTo(SCR_MENU); }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: CHANNEL SCANNER
// ═══════════════════════════════════════════════════════════════════════════
void drawChannelScreen(){
    drawScreenHeader("Channel Scanner  2.4 GHz",C_CYAN);
    int plotY=HH, plotH=BY-HH;
    M5.Display.fillRect(0,plotY,W,plotH,C_BG);
    // 13 channels evenly spaced
    int bw=(W-40)/13, bx=20;
    int maxH=plotH-40;
    // find max count for scaling
    int maxCount=1;
    for(int ch=1;ch<=13;ch++) if(g_channels[ch].count>maxCount)maxCount=g_channels[ch].count;

    for(int ch=1;ch<=13;ch++){
        int x=bx+(ch-1)*bw;
        // Highlight common non-overlapping channels
        bool primary=(ch==1||ch==6||ch==11);
        uint16_t barCol=primary?C_INT:C_AXIS;
        if(g_channels[ch].count==0){
            M5.Display.setTextSize(1); M5.Display.setTextColor(C_GRID,C_BG);
            M5.Display.setCursor(x+bw/2-4, plotY+plotH-32);
            M5.Display.printf("%d",ch);
            continue;
        }
        // RSSI bar height (signal strength)
        int rssiH=(int)((float)(g_channels[ch].best-RSSI_BOT)/(RSSI_TOP-RSSI_BOT)*maxH);
        rssiH=max(rssiH,4);
        int barY=plotY+plotH-36-rssiH;
        // Colour by RSSI strength
        int8_t best=g_channels[ch].best;
        uint16_t col=(best>=-55)?C_RED:(best>=-70)?C_EXT:barCol;
        M5.Display.fillRect(x,barY,bw-4,rssiH,col);
        // AP count badge
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_WHITE,col);
        M5.Display.setCursor(x+2,barY+2);
        M5.Display.printf("%d",g_channels[ch].count);
        // Channel label
        M5.Display.setTextColor(primary?C_INT:C_AXIS,C_BG);
        M5.Display.setCursor(x+bw/2-4,plotY+plotH-32);
        M5.Display.printf("%d",ch);
        // RSSI value
        M5.Display.setTextColor(C_AXIS,C_BG);
        M5.Display.setCursor(x+2,barY-10);
        M5.Display.printf("%d",best);
    }
    // Legend
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_GRID,C_BG);
    M5.Display.setCursor(4,plotY+4);
    M5.Display.print("bar=signal  number=AP count  red=busy  ch1/6/11=highlighted");
    drawBtnBar("BACK",C_INACT,"",C_BG,"",C_BG,"",C_BG);
}

void handleChannelTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y>=BY && t.x<W/4) navigateTo(SCR_SCOPE);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: POLAR PLOT
// ═══════════════════════════════════════════════════════════════════════════
void drawPolarScreen(){
    drawScreenHeader("Polar Plot — Rotate & Sample",C_GOLD);
    int cx=W/2, cy=HH+(BY-HH)/2;
    int radius=min((BY-HH)/2,(W/2))-30;
    M5.Display.fillRect(0,HH,W,BY-HH,C_BG);
    // Rings at -40,-60,-80,-100
    int rings[]={-40,-60,-80,-100};
    uint16_t rcols[]={C_RED,C_EXT,C_INT,C_GRID};
    for(int i=0;i<4;i++){
        float frac=(float)(RSSI_TOP-rings[i])/(RSSI_TOP-RSSI_BOT);
        int r=(int)(radius*frac);
        M5.Display.drawCircle(cx,cy,r,rcols[i]);
        M5.Display.setTextSize(1); M5.Display.setTextColor(rcols[i],C_BG);
        M5.Display.setCursor(cx+2,cy-r+2);
        M5.Display.printf("%d",rings[i]);
    }
    // Radial lines every 30°
    for(int a=0;a<360;a+=30){
        float rad=(a-90)*M_PI/180.0f;
        int x1=cx+(int)(radius*0.05f*cosf(rad)), y1=cy+(int)(radius*0.05f*sinf(rad));
        int x2=cx+(int)(radius*cosf(rad)),        y2=cy+(int)(radius*sinf(rad));
        M5.Display.drawLine(x1,y1,x2,y2,C_GRID);
        int lx=cx+(int)((radius+16)*cosf(rad))-8;
        int ly=cy+(int)((radius+16)*sinf(rad))-4;
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
        M5.Display.setCursor(lx,ly); M5.Display.printf("%d",a);
    }
    // Plot INT samples
    for(int i=0;i<POLAR_N;i++){
        if(g_polInt[i]==-127) continue;
        float ang=(i*10-90)*M_PI/180.0f;
        float frac=(float)(RSSI_TOP-g_polInt[i])/(RSSI_TOP-RSSI_BOT);
        frac=max(0.0f,min(1.0f,frac));
        int r=(int)(radius*frac);
        int x=cx+(int)(r*cosf(ang)), y=cy+(int)(r*sinf(ang));
        M5.Display.fillCircle(x,y,4,C_INT);
    }
    // Plot EXT samples — connect with lines
    int prevX=-1,prevY=-1;
    for(int i=0;i<POLAR_N;i++){
        if(g_polExt[i]==-127){prevX=-1;continue;}
        float ang=(i*10-90)*M_PI/180.0f;
        float frac=(float)(RSSI_TOP-g_polExt[i])/(RSSI_TOP-RSSI_BOT);
        frac=max(0.0f,min(1.0f,frac));
        int r=(int)(radius*frac);
        int x=cx+(int)(r*cosf(ang)), y=cy+(int)(r*sinf(ang));
        M5.Display.fillCircle(x,y,4,C_EXT);
        if(prevX>=0) M5.Display.drawLine(prevX,prevY,x,y,C_EXT);
        prevX=x; prevY=y;
    }
    // Current angle indicator arrow
    float arad=(g_polAngle*10-90)*M_PI/180.0f;
    int ax=cx+(int)(radius*0.9f*cosf(arad)), ay=cy+(int)(radius*0.9f*sinf(arad));
    M5.Display.drawLine(cx,cy,ax,ay,C_GOLD);
    M5.Display.fillCircle(ax,ay,5,C_GOLD);
    // Angle readout
    M5.Display.setTextSize(2); M5.Display.setTextColor(C_GOLD,C_BG);
    M5.Display.setCursor(cx-40,cy-10);
    M5.Display.printf("%3d deg",g_polAngle*10);
    // Legend
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(C_INT,C_BG); M5.Display.setCursor(4,BY-50); M5.Display.print("● INT");
    M5.Display.setTextColor(C_EXT,C_BG); M5.Display.setCursor(60,BY-50);M5.Display.print("● EXT");

    drawBtnBar("BACK",C_INACT,"<-10",C_INACT,"+10->",C_INACT,"SAMPLE",C_GOLD);
}

void handlePolarTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y<BY)return;
    int bw=W/4;
    if(t.x<bw){ navigateTo(SCR_SCOPE); return; }
    if(t.x<bw*2){ g_polAngle=(g_polAngle+POLAR_N-1)%POLAR_N; g_dirty=true; return; }
    if(t.x<bw*3){ g_polAngle=(g_polAngle+1)%POLAR_N;         g_dirty=true; return; }
    // SAMPLE button — record current antenna RSSI at this angle
    Stats& st=(g_ant==ANT_INT)?g_si:g_se;
    if(st.cur!=-127){
        if(g_ant==ANT_INT) g_polInt[g_polAngle]=st.cur;
        else               g_polExt[g_polAngle]=st.cur;
        Serial.printf("[POLAR] %d° %s = %d dBm\n",g_polAngle*10,antStr(g_ant),st.cur);
        g_polAngle=(g_polAngle+1)%POLAR_N;
        g_dirty=true;
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: WALK TEST
// ═══════════════════════════════════════════════════════════════════════════
void drawWalkScreen(){
    drawHeader();
    drawGrid(); drawTrace();
    // Walk-specific overlay: distance estimate, variance
    int8_t cur=(g_ant==ANT_INT)?g_si.cur:g_se.cur;
    // Free-space path loss rough estimate (assumes ref = -35 dBm @ 1m)
    float meters = (cur>-90) ? powf(10.0f,((-35.0f-(float)cur)/20.0f)) : 99;
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_GOLD,C_BG);
    M5.Display.setCursor(PX+4, PY+4);
    M5.Display.printf("~%.1fm  std:%.1f dB", meters, g_varStd);
    drawBtnBar("BACK",C_INACT,
               "INT ANT",(g_ant==ANT_INT)?C_BINT:C_INACT,
               "EXT MMCX",(g_ant==ANT_EXT)?C_BEXT:C_INACT,
               g_logging?"STOP LOG":"LOG CSV",g_logging?C_BLOG:C_INACT);
}

void handleWalkTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y<BY)return;
    int bw=W/4;
    if     (t.x<bw)   navigateTo(SCR_SCOPE);
    else if(t.x<bw*2) setAntenna(ANT_INT);
    else if(t.x<bw*3) setAntenna(ANT_EXT);
    else{ if(g_sdOk){ g_logging?stopLog():startLog(); g_dirty=true; }}
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: AP TIMELINE
// ═══════════════════════════════════════════════════════════════════════════
void drawTimelineScreen(){
    drawScreenHeader("AP Count Timeline",C_PURP);
    int py=HH, ph=BY-HH;
    M5.Display.fillRect(0,py,W,ph,C_BG);
    int count=g_tlFull?TL_LEN:g_tlHead; if(count<2){ drawBtnBar("BACK",C_INACT,"",C_BG,"",C_BG,"",C_BG); return; }
    // Find max
    uint8_t mx=1;
    for(int i=0;i<count;i++) if(g_timeline[(g_tlHead-count+i+TL_LEN)%TL_LEN]>mx)
        mx=g_timeline[(g_tlHead-count+i+TL_LEN)%TL_LEN];
    // Grid lines
    for(int v=0;v<=mx;v+=5){
        int y=py+ph-4-(int)((float)v/mx*(ph-20));
        M5.Display.drawLine(0,y,W,y,C_GRID);
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
        M5.Display.setCursor(2,y-4); M5.Display.printf("%d",v);
    }
    // Plot
    for(int i=1;i<count;i++){
        int ia=(g_tlHead-count+i-1+TL_LEN)%TL_LEN;
        int ib=(g_tlHead-count+i  +TL_LEN)%TL_LEN;
        int x1=i*W/count, x2=(i+1)*W/count;
        int y1=py+ph-4-(int)((float)g_timeline[ia]/mx*(ph-20));
        int y2=py+ph-4-(int)((float)g_timeline[ib]/mx*(ph-20));
        M5.Display.drawLine(x1,y1,x2,y2,C_PURP);
        M5.Display.drawLine(x1,y1+1,x2,y2+1,C_PURP);
    }
    // Current value
    int latest=g_timeline[(g_tlHead+TL_LEN-1)%TL_LEN];
    M5.Display.setTextSize(3); M5.Display.setTextColor(C_PURP,C_BG);
    M5.Display.setCursor(W-100,HH+8); M5.Display.printf("%3d",latest);
    M5.Display.setTextSize(1); M5.Display.setCursor(W-100,HH+36); M5.Display.print("APs now");
    drawBtnBar("BACK",C_INACT,"",C_BG,"",C_BG,"",C_BG);
}

void handleTimelineTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y>=BY && t.x<W/4) navigateTo(SCR_SCOPE);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: MULTIPATH VISUALISER
// ═══════════════════════════════════════════════════════════════════════════
void drawMultipathScreen(){
    // Upper 60%: RSSI trace | Lower 40%: variance bar
    int traceH=(int)((BY-HH)*0.6f);
    int varY=HH+traceH, varH=BY-varY;
    // Temporarily override PH for trace section
    int savedPH=PH; PH=traceH; PY=HH;
    drawGrid(); drawTrace();
    PH=savedPH; PY=HH;
    // Variance section
    M5.Display.fillRect(0,varY,W,varH,C_BG);
    M5.Display.drawLine(0,varY,W,varY,C_SEP);
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(2,varY+4); M5.Display.print("Multipath variance (std dev dBm):");
    // Variance bar
    float maxStd=10.0f;
    float frac=min(g_varStd/maxStd,1.0f);
    int barW=(int)(frac*(W-60));
    uint16_t vc=(g_varStd<2)?C_INT:(g_varStd<5)?C_EXT:C_RED;
    M5.Display.fillRect(30,varY+20,barW,varH-30,vc);
    M5.Display.drawRect(30,varY+20,W-60,varH-30,C_SEP);
    M5.Display.setTextSize(2); M5.Display.setTextColor(vc,C_BG);
    M5.Display.setCursor(W/2-40,varY+20);
    M5.Display.printf("%.2f dB",g_varStd);
    // Label
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(W-130,varY+4);
    if(g_varStd<2)      M5.Display.print("STABLE");
    else if(g_varStd<5) M5.Display.print("MODERATE");
    else                M5.Display.print("HIGH MULTIPATH");
    // Header
    drawHeader();
    drawBtnBar("BACK",C_INACT,"INT ANT",(g_ant==ANT_INT)?C_BINT:C_INACT,
               "EXT MMCX",(g_ant==ANT_EXT)?C_BEXT:C_INACT,"",C_BG);
}

void handleMultipathTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y<BY)return;
    int bw=W/4;
    if     (t.x<bw)   navigateTo(SCR_SCOPE);
    else if(t.x<bw*2) setAntenna(ANT_INT);
    else if(t.x<bw*3) setAntenna(ANT_EXT);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: A/B DWELL TIMER
// ═══════════════════════════════════════════════════════════════════════════
void drawAbDwellScreen(){
    drawHeader(); drawGrid(); drawTrace();
    // Countdown overlay
    uint32_t elapsed=millis()-g_abLast;
    uint32_t remain=(g_abRunning && elapsed<g_cfg.dwellSec*1000UL)
                    ? g_cfg.dwellSec-(elapsed/1000) : 0;
    M5.Display.setTextSize(2); M5.Display.setTextColor(C_GOLD,C_BG);
    M5.Display.setCursor(PX+8,PY+8);
    if(g_abRunning) M5.Display.printf("Next switch: %lus",remain);
    else            M5.Display.print("Press START");
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(PX+8,PY+28);
    M5.Display.printf("Dwell: %lus per antenna",g_cfg.dwellSec);
    drawBtnBar("BACK",C_INACT,
               g_abRunning?"STOP":"START",g_abRunning?C_BLOG:C_BINT,
               "DWELL+",C_INACT,"DWELL-",C_INACT);
}

void handleAbDwellTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y<BY)return;
    int bw=W/4;
    if(t.x<bw){ navigateTo(SCR_SCOPE); return; }
    if(t.x<bw*2){ g_abRunning=!g_abRunning; g_abLast=millis(); g_dirty=true; return; }
    if(t.x<bw*3){
        if(g_cfg.dwellSec<120) g_cfg.dwellSec+=10; g_dirty=true; return;
    }
    if(g_cfg.dwellSec>10) g_cfg.dwellSec-=10; g_dirty=true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: POLARISATION TEST
// ═══════════════════════════════════════════════════════════════════════════
void drawPolScreen(){
    drawHeader(); drawGrid(); drawTrace();
    M5.Display.setTextSize(2);
    uint16_t instrCol=(g_ant==ANT_INT)?C_INT:C_EXT;
    M5.Display.setTextColor(instrCol,C_BG);
    M5.Display.setCursor(PX+8,PY+8);
    if(g_ant==ANT_INT)
        M5.Display.print("H-POL: Hold device landscape  ");
    else
        M5.Display.print("V-POL: Rotate device portrait ");
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(PX+8,PY+30);
    uint32_t elapsed=millis()-g_abLast;
    uint32_t remain=(g_abRunning && elapsed<g_cfg.dwellSec*1000UL)
                    ? g_cfg.dwellSec-(elapsed/1000) : 0;
    if(g_abRunning) M5.Display.printf("Auto-switch in: %lus", remain);
    else            M5.Display.print("Press START to auto-switch");
    drawBtnBar("BACK",C_INACT,
               g_abRunning?"STOP":"START",g_abRunning?C_BLOG:C_BINT,
               "H-POL",(g_ant==ANT_INT)?C_BINT:C_INACT,
               "V-POL",(g_ant==ANT_EXT)?C_BEXT:C_INACT);
}

void handlePolTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y<BY)return;
    int bw=W/4;
    if(t.x<bw){ navigateTo(SCR_SCOPE); return; }
    if(t.x<bw*2){ g_abRunning=!g_abRunning; g_abLast=millis(); g_dirty=true; return; }
    if(t.x<bw*3){ setAntenna(ANT_INT); return; }  // H polarisation
    setAntenna(ANT_EXT);                           // V polarisation
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: SETTINGS
// ═══════════════════════════════════════════════════════════════════════════
void drawSettingsScreen(){
    drawScreenHeader("Settings — Enable / Disable Modules",C_GOLD);
    struct Toggle{ const char* name; bool* val; uint16_t col; };
    Toggle tgl[]={
        {"Channel Scanner", &g_cfg.channelScan,  C_CYAN},
        {"Polar Plot",      &g_cfg.polarPlot,    C_GOLD},
        {"Walk Test",       &g_cfg.walkTest,     C_INT},
        {"AP Timeline",     &g_cfg.apTimeline,   C_PURP},
        {"Multipath",       &g_cfg.multipath,    C_EXT},
        {"A/B Dwell",       &g_cfg.abDwell,      C_BEXT},
        {"Polarisation",    &g_cfg.polarisation, C_RED},
        {"Packet Loss",     &g_cfg.pktLoss,      C_CYAN},
    };
    int n=8, cols=2, rows=(n+1)/2;
    int bw=W/cols, bh=(BY-HH-8)/rows;
    for(int i=0;i<n;i++){
        int col=i%cols, row=i/cols;
        int x=col*bw+4, y=HH+4+row*bh;
        bool on=*tgl[i].val;
        uint16_t bg=on?tgl[i].col:C_INACT;
        M5.Display.fillRoundRect(x,y,bw-8,bh-6,6,bg);
        M5.Display.drawRoundRect(x,y,bw-8,bh-6,6,C_SEP);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(C_WHITE,bg);
        M5.Display.setCursor(x+10,y+bh/2-12);
        M5.Display.printf("%s %s",on?"ON ":"OFF",tgl[i].name);
    }
    // Dwell time config
    int y=HH+4+rows*bh;
    M5.Display.fillRect(0,y,W,BY-y,C_HDRBG);
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_HDRBG);
    M5.Display.setCursor(8,y+6);
    M5.Display.printf("A/B dwell: %lus   Walk sample: %lums",g_cfg.dwellSec,g_cfg.walkMs);
    drawBtnBar("BACK",C_INACT,"DWELL+",C_INACT,"DWELL-",C_INACT,"WALK MS",C_INACT);
}

void handleSettingsTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    // Bottom bar
    if(t.y>=BY){
        int bw=W/4;
        if(t.x<bw){ navigateTo(SCR_MENU); return; }
        if(t.x<bw*2){ if(g_cfg.dwellSec<120) g_cfg.dwellSec+=10; g_dirty=true; return; }
        if(t.x<bw*3){ if(g_cfg.dwellSec>10)  g_cfg.dwellSec-=10; g_dirty=true; return; }
        // Walk MS: cycle 200/500/1000/2000
        uint32_t opts[]={200,500,1000,2000}; int nopt=4;
        for(int i=0;i<nopt;i++) if(g_cfg.walkMs==opts[i]){ g_cfg.walkMs=opts[(i+1)%nopt]; break; }
        g_dirty=true; return;
    }
    // Toggle buttons
    bool* vals[]={&g_cfg.channelScan,&g_cfg.polarPlot,&g_cfg.walkTest,
                  &g_cfg.apTimeline,&g_cfg.multipath,&g_cfg.abDwell,
                  &g_cfg.polarisation,&g_cfg.pktLoss};
    int n=8, cols=2, rows=(n+1)/2;
    int bw=W/cols, bh=(BY-HH-8)/rows;
    for(int i=0;i<n;i++){
        int col=i%cols, row=i/cols;
        int x=col*bw+4, y=HH+4+row*bh;
        if(t.x>=x && t.x<x+bw-8 && t.y>=y && t.y<y+bh-6){
            *vals[i]=!(*vals[i]); g_dirty=true; return;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: MENU
// ═══════════════════════════════════════════════════════════════════════════
void drawMenuScreen(){
    drawScreenHeader("Test Menu",C_WHITE);
    struct MenuItem{ const char* name; Screen scr; bool enabled; uint16_t col; };
    MenuItem items[]={
        {"Oscilloscope",  SCR_SCOPE,     true,                 C_INT},
        {"Channel Scan",  SCR_CHANNEL,   g_cfg.channelScan,    C_CYAN},
        {"Polar Plot",    SCR_POLAR,     g_cfg.polarPlot,      C_GOLD},
        {"Walk Test",     SCR_WALK,      g_cfg.walkTest,       C_INT},
        {"AP Timeline",   SCR_TIMELINE,  g_cfg.apTimeline,     C_PURP},
        {"Multipath",     SCR_MULTIPATH, g_cfg.multipath,      C_EXT},
        {"A/B Dwell",     SCR_ABDWELL,   g_cfg.abDwell,        C_BEXT},
        {"Polarisation",  SCR_POL,       g_cfg.polarisation,   C_RED},
        {"Packet Loss",   SCR_PKTLOSS,   g_cfg.pktLoss,        C_CYAN},
    };
    int n=9, cols=3, rows=3;
    int bw=W/cols, bh=(BY-HH-8)/rows;
    for(int i=0;i<n;i++){
        int col=i%cols, row=i/cols;
        int x=col*bw+4, y=HH+4+row*bh;
        uint16_t bg=items[i].enabled?items[i].col:C_INACT;
        M5.Display.fillRoundRect(x,y,bw-8,bh-6,8,bg);
        M5.Display.drawRoundRect(x,y,bw-8,bh-6,8,C_SEP);
        M5.Display.setTextSize(2);
        M5.Display.setTextColor(C_WHITE,bg);
        int len=strlen(items[i].name)*12;
        M5.Display.setCursor(x+(bw-8-len)/2, y+bh/2-12);
        M5.Display.print(items[i].name);
        if(!items[i].enabled){
            M5.Display.setTextSize(1);
            M5.Display.setTextColor(C_GRID,bg);
            M5.Display.setCursor(x+8,y+bh/2+8);
            M5.Display.print("(enable in Settings)");
        }
    }
    drawBtnBar("BACK",C_INACT,"EXIT APP",C_RED,"",C_BG,"SETTINGS",C_GOLD);
}

void handleMenuTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y>=BY){
        if(t.x<W/4)         navigateTo(SCR_SCOPE);
        else if(t.x<W/2)    g_wantExit=true;        // leave the applet -> M5View home
        else if(t.x>=W*3/4) navigateTo(SCR_SETTINGS);
        return;
    }
    // Menu grid
    bool* enabled[]={nullptr,&g_cfg.channelScan,&g_cfg.polarPlot,&g_cfg.walkTest,
                     &g_cfg.apTimeline,&g_cfg.multipath,&g_cfg.abDwell,
                     &g_cfg.polarisation,&g_cfg.pktLoss};
    Screen scrs[]={SCR_SCOPE,SCR_CHANNEL,SCR_POLAR,SCR_WALK,
                   SCR_TIMELINE,SCR_MULTIPATH,SCR_ABDWELL,SCR_POL,SCR_PKTLOSS};
    int cols=3, rows=3, n=9;
    int bw=W/cols, bh=(BY-HH-8)/rows;
    for(int i=0;i<n;i++){
        int col=i%cols, row=i/cols;
        int x=col*bw+4, y=HH+4+row*bh;
        if(t.x>=x&&t.x<x+bw-8&&t.y>=y&&t.y<y+bh-6){
            bool ok=(enabled[i]==nullptr||*enabled[i]);
            if(ok) navigateTo(scrs[i]);
            return;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: NETWORK SELECT
// ═══════════════════════════════════════════════════════════════════════════
void scanForSelection(){
    M5.Display.fillScreen(C_BG);
    M5.Display.setTextSize(2); M5.Display.setTextColor(C_INT,C_BG);
    M5.Display.setCursor(W/2-140,H/2-16); M5.Display.print("Scanning for WiFi...");
    Serial.println("[SCAN-SEL] starting initial scan");
    Serial.flush();
    delay(100);
    int n=WiFi.scanNetworks(false,false);  // blocking, no hidden, no passive
    Serial.printf("[SCAN-SEL] returned n=%d\n", n);
    Serial.flush();
    g_apCount=0;
    if(n>0){
        int idx[40], cnt=min(n,40);
        for(int i=0;i<cnt;i++) idx[i]=i;
        for(int i=1;i<cnt;i++){
            int key=idx[i]; int j=i-1;
            while(j>=0&&WiFi.RSSI(idx[j])<WiFi.RSSI(key)){idx[j+1]=idx[j];j--;}
            idx[j+1]=key;
        }
        for(int i=0;i<cnt;i++){
            int k=idx[i];
            strncpy(g_apList[i].ssid,WiFi.SSID(k).c_str(),63);
            g_apList[i].ssid[63]='\0';
            memcpy(g_apList[i].bssid,WiFi.BSSID(k),6);
            g_apList[i].rssi=(int8_t)WiFi.RSSI(k);
            g_apList[i].channel=WiFi.channel(k);
            g_apList[i].encrypted=(WiFi.encryptionType(k)!=WIFI_AUTH_OPEN);
        }
        g_apCount=cnt;
    }
    WiFi.scanDelete(); g_apScroll=0;
}

void drawSelectScreen(){
    const int HDR=50,BTNBAR=52,ROW_H=56;
    const int VISIBLE=(H-HDR-BTNBAR)/ROW_H;
    M5.Display.fillScreen(C_BG);
    M5.Display.fillRect(0,0,W,HDR,C_HDRBG);
    M5.Display.drawLine(0,HDR-1,W,HDR-1,C_SEP);
    M5.Display.setTextSize(2); M5.Display.setTextColor(C_INT,C_HDRBG);
    M5.Display.setCursor(12,12); M5.Display.print("Select WiFi Network to Test");
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_HDRBG);
    M5.Display.setCursor(W-160,18); M5.Display.printf("%d networks found",g_apCount);
    for(int i=0;i<VISIBLE;i++){
        int ai=g_apScroll+i; if(ai>=g_apCount) break;
        APInfo& ap=g_apList[ai]; int y=HDR+i*ROW_H;
        uint16_t rb=(i%2==0)?0x0821u:0x1041u;
        M5.Display.fillRect(0,y,W,ROW_H,rb);
        M5.Display.drawLine(0,y+ROW_H-1,W,y+ROW_H-1,C_SEP);
        int bars=rssiToSignalBars(ap.rssi);
        uint16_t bc=(bars>=4)?C_INT:(bars>=2)?C_EXT:C_RED;
        for(int b=0;b<5;b++){
            int bh=6+b*5, bx=10+b*12, by2=y+ROW_H/2+4-bh;
            M5.Display.fillRect(bx,by2,10,bh,(b<bars)?bc:C_GRID);
        }
        M5.Display.setTextSize(1); M5.Display.setTextColor(bc,rb);
        M5.Display.setCursor(10,y+ROW_H-14); M5.Display.printf("%4d",(int)ap.rssi);
        M5.Display.setTextSize(2); M5.Display.setTextColor(C_WHITE,rb);
        M5.Display.setCursor(80,y+8);
        char buf[28]; snprintf(buf,28,"%.26s",ap.ssid); M5.Display.print(buf);
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,rb);
        M5.Display.setCursor(W-110,y+8); M5.Display.printf("ch%2d",ap.channel);
        M5.Display.setTextColor(ap.encrypted?C_EXT:C_INT,rb);
        M5.Display.setCursor(W-60,y+8); M5.Display.print(ap.encrypted?"[ENC]":"[OPEN]");
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,rb);
        M5.Display.setCursor(80,y+36);
        M5.Display.printf("%02X:%02X:%02X:%02X:%02X:%02X",
            ap.bssid[0],ap.bssid[1],ap.bssid[2],ap.bssid[3],ap.bssid[4],ap.bssid[5]);
    }
    int btnY=H-BTNBAR;
    M5.Display.fillRect(0,btnY,W,BTNBAR,C_HDRBG);
    M5.Display.drawLine(0,btnY,W,btnY,C_SEP);
    int bw=W/3;
    auto selBtn=[&](int i,const char* lbl,uint16_t bg){
        int x=i*bw;
        M5.Display.fillRect(x+4,btnY+4,bw-8,BTNBAR-8,bg);
        M5.Display.setTextSize(2); M5.Display.setTextColor(C_WHITE,bg);
        M5.Display.setCursor(x+bw/2-strlen(lbl)*6,btnY+14);
        M5.Display.print(lbl);
    };
    selBtn(0,"^ UP",C_INACT); selBtn(1,"v DN",C_INACT); selBtn(2,"REFRESH",C_BINT);
}

void handleSelectTouch(){
    static uint32_t last=0; 
    if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); 
    if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    
    Serial.printf("[SEL-TOUCH] tap at x=%d y=%d\n", t.x, t.y);
    Serial.flush();
    const int HDR=50,BTNBAR=52,ROW_H=56;
    const int VISIBLE=(H-HDR-BTNBAR)/ROW_H;
    int btnY=H-BTNBAR;
    if(t.y>=btnY){
        int bw=W/3;
        if(t.x<bw)      { if(g_apScroll>0){g_apScroll--;drawSelectScreen();} }
        else if(t.x<bw*2){ if(g_apScroll+VISIBLE<g_apCount){g_apScroll++;drawSelectScreen();} }
        else             { scanForSelection(); drawSelectScreen(); }
        return;
    }
    if(t.y>=HDR&&t.y<btnY){
        int row=(t.y-HDR)/ROW_H, ai=g_apScroll+row;
        if(ai>=g_apCount) return;
        APInfo& ap=g_apList[ai];
        strncpy(g_ssid,ap.ssid,63);
        memcpy(g_bssid,ap.bssid,6);
        g_targetChannel = ap.channel;
        g_locked=true;
        Serial.printf("[SEL] selected SSID: \"%s\" BSSID: %02X:%02X:%02X:%02X:%02X:%02X ch=%d RSSI: %d dBm\n",
            g_ssid, ap.bssid[0],ap.bssid[1],ap.bssid[2],ap.bssid[3],ap.bssid[4],ap.bssid[5], ap.channel, (int)ap.rssi);
        
        int ry=HDR+row*ROW_H;
        M5.Display.fillRect(0,ry,W,ROW_H,C_BINT);
        M5.Display.setTextSize(2); M5.Display.setTextColor(C_WHITE,C_BINT);
        M5.Display.setCursor(80,ry+16); M5.Display.printf("Selected: %.26s",g_ssid);
        delay(500);
        
        // CRITICAL: clean up WiFi state before transitioning
        WiFi.scanDelete();
        WiFi.disconnect();
        delay(200);
        Serial.println("[SEL] WiFi state reset, transitioning to scope");
        
        g_screen=SCR_SCOPE;
        g_scanning=false;
        g_tSample=millis();
        g_settleUntil=millis()+500;
        M5.Display.fillScreen(C_BG);
        drawScopeScreen();
        kickScan();
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOGGING
// ═══════════════════════════════════════════════════════════════════════════
void startLog(){
    g_logFile=SD_MMC.open(LOG_FILE,FILE_APPEND);
    if(!g_logFile){Serial.println("[LOG] open failed");return;}
    g_logFile.println("ts_ms,antenna,rssi_dbm,ssid,bssid,int_avg,ext_avg,std_dev");
    g_logFile.flush(); g_logging=true;
}
void stopLog(){ if(g_logFile) g_logFile.close(); g_logging=false; }
void writeLog(uint32_t ts,int8_t rssi){
    if(!g_logFile) return;
    char b[18];
    snprintf(b,18,"%02X:%02X:%02X:%02X:%02X:%02X",
             g_bssid[0],g_bssid[1],g_bssid[2],g_bssid[3],g_bssid[4],g_bssid[5]);
    g_logFile.printf("%lu,%s,%d,\"%s\",%s,%.2f,%.2f,%.2f\n",
        (unsigned long)ts,antStr(g_ant),rssi,g_ssid,b,g_si.avg,g_se.avg,g_varStd);
    g_logFile.flush();
}

// ═══════════════════════════════════════════════════════════════════════════
//  SCREEN: PACKET LOSS TEST
//  Connects to the selected network (password set via PKT_WIFI_PASSWORD)
//  and pings the gateway once per second. Tracks loss %, RTT, history strip.
// ═══════════════════════════════════════════════════════════════════════════

void pktLossService(){
    uint32_t now = millis();

    // ── Connection management ───────────────────────────────────────────
    if(!g_pktConnected && !g_pktConnecting){
        if(strlen(PKT_WIFI_PASSWORD)==0) return;   // no password configured
        Serial.printf("[PKT] connecting to \"%s\"...\n", g_ssid);
        WiFi.begin(g_ssid, PKT_WIFI_PASSWORD);
        g_pktConnecting=true; g_pktConnStart=now;
        g_dirty=true;
        return;
    }
    if(g_pktConnecting){
        if(WiFi.status()==WL_CONNECTED){
            g_pktConnecting=false; g_pktConnected=true;
            g_pktSent=0; g_pktLost=0; g_pktAvgRtt=0;
            memset(g_pktHist,0,sizeof(g_pktHist)); g_pktHead=0;
            Serial.printf("[PKT] connected, IP %s, GW %s\n",
                WiFi.localIP().toString().c_str(),
                WiFi.gatewayIP().toString().c_str());
            g_dirty=true;
        } else if(now - g_pktConnStart > 15000){
            Serial.println("[PKT] connect timeout");
            WiFi.disconnect();
            g_pktConnecting=false;
            g_dirty=true;
        }
        return;
    }

    // ── Ping once per second ────────────────────────────────────────────
    if(g_pktConnected && now-g_pktLastPing >= 1000){
        g_pktLastPing=now;
        IPAddress gw = WiFi.gatewayIP();
        g_pktSent++;
#if ANT_ENABLE_PKTLOSS
        bool ok = Ping.ping(gw, 1);    // 1 attempt
        if(ok){
            g_pktRttMs=(int32_t)Ping.averageTime();
            // running average RTT
            g_pktAvgRtt = (g_pktAvgRtt*(g_pktSent-g_pktLost-1)+g_pktRttMs)
                          / (float)(g_pktSent-g_pktLost);
        } else {
            g_pktLost++; g_pktRttMs=-1;
        }
#else
        (void)gw; g_pktLost++; g_pktRttMs=-1;   // ping disabled
        bool ok=false; (void)ok;
#endif
        g_pktHist[g_pktHead]= ok?1:0;
        g_pktHead=(g_pktHead+1)%PKT_HIST;
        g_dirty=true;
    }
}

void drawPktLossScreen(){
    drawScreenHeader("Packet Loss / Ping Test",C_CYAN);
    int py=HH, ph=BY-HH;
    M5.Display.fillRect(0,py,W,ph,C_BG);

    // ── No password configured ───────────────────────────────────────────
    if(strlen(PKT_WIFI_PASSWORD)==0){
        M5.Display.setTextSize(2); M5.Display.setTextColor(C_EXT,C_BG);
        M5.Display.setCursor(20,py+30);
        M5.Display.print("No WiFi password configured.");
        M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
        M5.Display.setCursor(20,py+70);
        M5.Display.print("Edit the sketch and set PKT_WIFI_PASSWORD");
        M5.Display.setCursor(20,py+86);
        M5.Display.print("near the top of the file, then re-flash.");
        M5.Display.setCursor(20,py+110);
        M5.Display.print("The test connects to your selected network and");
        M5.Display.setCursor(20,py+126);
        M5.Display.print("pings the gateway 1x/sec to measure real loss.");
        drawBtnBar("BACK",C_INACT,"",C_BG,"",C_BG,"",C_BG);
        return;
    }

    // ── Connecting state ─────────────────────────────────────────────────
    if(g_pktConnecting){
        M5.Display.setTextSize(2); M5.Display.setTextColor(C_CYAN,C_BG);
        M5.Display.setCursor(20,py+40);
        M5.Display.printf("Connecting to %.20s ...",g_ssid);
        drawBtnBar("BACK",C_INACT,"",C_BG,"",C_BG,"",C_BG);
        return;
    }
    if(!g_pktConnected){
        M5.Display.setTextSize(2); M5.Display.setTextColor(C_RED,C_BG);
        M5.Display.setCursor(20,py+40);
        M5.Display.print("Not connected (check password)");
        drawBtnBar("BACK",C_INACT,"RETRY",C_BINT,"",C_BG,"",C_BG);
        return;
    }

    // ── Connected — stats ─────────────────────────────────────────────────
    float lossPct = g_pktSent ? 100.0f*g_pktLost/g_pktSent : 0;
    uint16_t lc = (lossPct<1)?C_INT:(lossPct<5)?C_EXT:C_RED;

    M5.Display.setTextSize(4); M5.Display.setTextColor(lc,C_BG);
    M5.Display.setCursor(20,py+16);
    M5.Display.printf("%.1f%%",lossPct);
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(22,py+52); M5.Display.print("packet loss");

    M5.Display.setTextSize(3);
    uint16_t rc=(g_pktRttMs<0)?C_RED:(g_pktRttMs<20)?C_INT:(g_pktRttMs<80)?C_EXT:C_RED;
    M5.Display.setTextColor(rc,C_BG);
    M5.Display.setCursor(W/2,py+16);
    if(g_pktRttMs>=0) M5.Display.printf("%ldms",(long)g_pktRttMs);
    else              M5.Display.print("LOST");
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(W/2+2,py+52);
    M5.Display.printf("RTT (avg %.1fms)",g_pktAvgRtt);

    // Sent/lost counters + antenna
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(20,py+72);
    M5.Display.printf("sent:%lu  lost:%lu  ant:[%s]  IP:%s  GW:%s",
        (unsigned long)g_pktSent,(unsigned long)g_pktLost,antStr(g_ant),
        WiFi.localIP().toString().c_str(),
        WiFi.gatewayIP().toString().c_str());

    // ── History strip — last 60 pings as green/red ticks ─────────────────
    int stripY=py+95, stripH=40;
    int tw=max(2,(W-40)/PKT_HIST);
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(20,stripY-12); M5.Display.print("last 60 pings:");
    for(int i=0;i<PKT_HIST;i++){
        int idx=(g_pktHead+i)%PKT_HIST;
        int x=20+i*tw;
        uint16_t c=g_pktHist[idx]?C_INT:C_RED;
        M5.Display.fillRect(x,stripY,tw-1,stripH,c);
    }

    // ── RSSI while connected (WiFi.RSSI() works on live connection) ──────
    int8_t liveRssi=(int8_t)WiFi.RSSI();
    M5.Display.setTextSize(2);
    uint16_t vc=(liveRssi>=-65)?C_INT:(liveRssi>=-80)?C_EXT:C_RED;
    M5.Display.setTextColor(vc,C_BG);
    M5.Display.setCursor(20,stripY+stripH+14);
    M5.Display.printf("%d dBm",liveRssi);
    M5.Display.setTextSize(1); M5.Display.setTextColor(C_AXIS,C_BG);
    M5.Display.setCursor(110,stripY+stripH+20);
    M5.Display.print("live RSSI on connection");

    drawBtnBar("BACK",C_INACT,
               "INT ANT",(g_ant==ANT_INT)?C_BINT:C_INACT,
               "EXT MMCX",(g_ant==ANT_EXT)?C_BEXT:C_INACT,
               "RESET",C_BLOG);
}

void handlePktLossTouch(){
    static uint32_t last=0; if(!M5.Touch.getCount())return;
    auto t=M5.Touch.getDetail(0); if(!t.isPressed())return;
    uint32_t now=millis(); if(now-last<400)return; last=now;
    if(t.y<BY)return;
    int bw=W/4;
    if(t.x<bw){ navigateTo(SCR_SCOPE); return; }
    if(t.x<bw*2){
        // Antenna switch keeps connection alive — RF path swaps live!
        setAntenna(ANT_INT); return;
    }
    if(t.x<bw*3){ setAntenna(ANT_EXT); return; }
    // RESET counters (also RETRY when disconnected)
    if(!g_pktConnected && !g_pktConnecting){
        g_pktConnecting=false; g_pktConnected=false;  // service() will retry
    } else {
        g_pktSent=0; g_pktLost=0; g_pktAvgRtt=0;
        memset(g_pktHist,0,sizeof(g_pktHist)); g_pktHead=0;
    }
    g_dirty=true;
}

} // namespace antenna

// ═══════════════════════════════════════════════════════════════════════════
//  Applet wrapper — bridges the shell lifecycle to the sketch's setup/loop.
// ═══════════════════════════════════════════════════════════════════════════
void AntennaApplet::on_enter()  { antenna::app_setup(); }
void AntennaApplet::on_exit()   { antenna::app_teardown(); }
void AntennaApplet::on_render() { /* drawing happens inside on_update() */ }
bool AntennaApplet::on_update() {
    antenna::app_loop();
    return !antenna::g_wantExit;   // false -> shell exits the applet to home
}
