# Antenna applet — WiFi crash debug (RESOLVED)

> **Update 2026-06-18/19:** WiFi fixed with `WiFi.setPins(12,13,11,10,9,8,15)`
> (Tab5 C6 hosted-SDIO pins) before WiFi init — verified on hardware. The
> RF-switch I²C issue below is also fixed in code: `expWrite/expRead` now use
> `M5.In_I2C` (the Tab5's internal bus, already initialised by `M5.begin()`)
> instead of raw un-begun `Wire`. INT/EXT switching awaits an on-device test.
> Original investigation kept below for reference.

Status as of 2026-06-16. Firmware is flashed and running on the Tab5 (COM6).
**3D Viewer + Room Scanner work.** The **Antenna applet crashes on launch** (blue
screen → reboot → back to M5View home).

## Root cause (confirmed from serial)

The app reaches `on_enter` (`[ANT] 1280x720`) then dies bringing up WiFi:

```
E sdmmc_common: sdmmc_init_ocr: send_op_cond (1) returned 0x107   (ESP_ERR_TIMEOUT)
E sdio_wrapper:  sdmmc_card_init failed
E H_SDIO_DRV:    sdio card init failed
FreeRTOS: Task "sdio_read" should not return, Aborting now!   → reboot
```

The ESP32-P4 has **no radio**; `WiFi.*` talks to the onboard **ESP32-C6 over an
esp-hosted SDIO link**. That link's init times out and the hosted driver aborts
the system. Cause: `platformio.ini` uses **`board = esp32-p4-evboard`** (generic
EVB) — the installed platform has **no M5Stack Tab5 profile** (only
`esp32-p4-evboard.json` / `esp32-p4.json`), so the C6 SDIO pins/sdkconfig don't
match the Tab5. The standalone Arduino sketch worked because the Arduino-IDE
M5Stack package ships the correct Tab5 config. **This is a build-config gap, not
an app bug.**

## Second, non-fatal issue
Just before the crash: `Wire.cpp: NULL TX buffer pointer` (repeated). The I²C bus
for the RF-switch expander (`0x43`) is never `begin()`'d under this build, so
INT/EXT antenna switching won't work even once WiFi is fixed. The standalone
sketch relied on `M5.begin()` setting up that bus. Fix: explicitly init the
correct Tab5 I²C bus (likely `Wire1`/`M5.In_I2C`) in `app_setup()`.

## Next session plan
1. Get the Tab5 C6 esp-hosted SDIO pin config. Fastest: **which board did the
   user select in the Arduino IDE** for the working antenna sketch (+ M5Stack
   board-package version)? That encodes the right pins/sdkconfig. Otherwise
   research M5Stack Tab5 C6 SDIO mapping.
2. Port that into `platformio.ini` as `build_flags` / sdkconfig overrides
   (esp_wifi_remote / esp_hosted SDIO pins), or switch to a real Tab5 board def
   if one becomes available.
3. Fix the RF-switch I²C bus init in the antenna applet.
4. Re-flash (remember `PYTHONIOENCODING=utf-8`), monitor, re-test Antenna.

## Note
SD is mounted via `SD_MMC` (SLOT 0) in `main.cpp`. Watch for SD ↔ C6 SDIO
interaction once WiFi config is corrected (may or may not be related).
