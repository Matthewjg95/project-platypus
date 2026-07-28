# Phase 1.1 — Applet Ecosystem Hardening

A guide to evolving the M5View shell from "a launcher that works for a couple of
apps" into a **robust multi-app platform** that can host the 3D Viewer, the Room
Scanner, and your external-antenna Oscilloscope side by side without stepping on
each other.

---

## Verdict: is the current structure robust?

**Short answer: it's a solid foundation, but not yet robust for 3+ apps.**

What's good:
- Clean `Applet` interface (`on_enter/exit/update/render/back`) — the right shape.
- Shell owns navigation; applets are self-contained and never call each other.
- Dirty-flag redraw model keeps the home screen cheap.

What breaks as the app count grows (and why it matters for the Oscilloscope):

| # | Problem | Symptom with 3 apps |
|---|---------|---------------------|
| 1 | `Shell` knows the concrete `ViewerApplet` type and `static_cast`s the active applet to it (`_active_viewer()`) | Opening **Settings over the Scanner or Oscilloscope** calls `ViewerApplet::set_paused()` on the wrong object → **undefined behavior / crash** |
| 2 | Home grid is hard-capped at `TILE_COLS*TILE_ROWS` (= 6) with no paging | A 7th app silently vanishes; no scroll |
| 3 | No pause/resume in the base interface | The Oscilloscope's continuous sampling keeps running under the Settings panel and corrupts the screen, like the Viewer used to |
| 4 | Settings menu is hardcoded + inert | "Brightness/Rotation/Lighting" do nothing and can't be extended per-app |
| 5 | No shared-resource ownership or memory budget | Viewer (65k-vert buffer), Scanner (canvases + SfM cloud), Oscilloscope (sample buffers + ADC/SPI) all grab globals (`M5`, `SD_MMC`, I²C 0x63) with no arbitration |

The five upgrades below fix exactly these, in priority order.

---

## Upgrade 1 — Lifecycle hooks in the base interface (kills the `static_cast` bug)

**Problem.** The shell reaches into a concrete applet type:

```cpp
// shell.h (current) — UNSAFE once a non-Viewer applet is active
ViewerApplet* _active_viewer() {
    if (_state == STATE_APPLET && _active >= 0)
        return static_cast<ViewerApplet*>(_applets[_active]);   // wrong type!
    return nullptr;
}
void _open_settings() { if (auto* v = _active_viewer()) v->set_paused(true); ... }
```

**Fix.** Move pause/resume into `Applet` so the shell talks to every app uniformly,
and delete the `ViewerApplet` include + cast from the shell.

```cpp
// applet.h
class Applet {
public:
    // ... existing ...
    virtual void on_pause()  {}   // settings opened / app backgrounded
    virtual void on_resume() {}   // settings closed / app foregrounded
};
```

```cpp
// shell.h — generic, no concrete type knowledge
void _open_settings()  { if (_active >= 0) _applets[_active]->on_pause();  ... }
void _close_settings() { if (_active >= 0) _applets[_active]->on_resume(); ... }
```

`ViewerApplet` renames `set_paused()` → `on_pause()/on_resume()`. The Scanner
pauses its UART drain + render; the Oscilloscope halts sampling. **Remove
`#include "viewer_applet.h"` from `shell.h`.**

**Effort:** ~30 min. **Risk:** low. **Priority: do this first** — it's a live defect.

---

## Upgrade 2 — Scalable launcher: paging + capacity guard

**Problem.** `_draw_tiles()` loops `i < TILE_COLS*TILE_ROWS` and `_tile_at()` maps
to a single fixed page. App #7 disappears with no warning.

**Fix.** Add horizontal paging (swipe) and a page indicator; derive page count
from the applet list.

```cpp
int _page = 0;
int _pages() const {
    int per = TILE_COLS * TILE_ROWS;
    return ((int)_applets.size() + per - 1) / per;   // ceil
}
// _draw_tiles(): iterate applets [_page*per, _page*per+per)
// _tile_at():   add _page*per to the computed index
// _handle_touch(): horizontal swipe (dx > threshold) changes _page, clamp [0,_pages()-1]
```

Add a row of dots at the bottom showing `_page`. Now the catalogue grows without
code changes.

**Effort:** ~1–2 h. **Risk:** low.

---

## Upgrade 3 — `AppContext`: inject shared services instead of reaching for globals

**Problem.** Every applet hard-codes globals: `M5.Display`, `SD_MMC`, the joystick
at I²C `0x63`, UART pins. The Oscilloscope wants the ADC/SPI front-end and maybe
the same I²C bus — there's no arbitration, and nothing documents who owns what.

**Fix.** Pass a context object to each applet at registration / enter. It exposes
display geometry, the SD mount, a settings store (Upgrade 4), and a **peripheral
broker** that hands out shared buses so two applets can't double-claim them.

```cpp
struct AppContext {
    LovyanGFX* display;
    int        screen_w, screen_h, content_y;   // content_y = BAR_H
    fs::FS*    sd;                                // &SD_MMC
    SettingsStore* settings;
    Peripherals*   periph;     // request_i2c(addr), request_uart(num,rx,tx), release()
};

class Applet {
public:
    virtual void attach(AppContext* ctx) { _ctx = ctx; }   // shell calls once
protected:
    AppContext* _ctx = nullptr;
};
```

The shell builds one `AppContext` in `begin()` and `attach()`es it to every applet.
Applets stop hard-coding `M5.Display` / `SD_MMC`; the Oscilloscope calls
`_ctx->periph->request_adc(...)` in `on_enter` and `release()` in `on_exit`, so the
Scanner's UART and the Oscilloscope's front-end never collide.

**Effort:** ~half day (broker is the bulk). **Risk:** medium — touches every applet,
but mechanical. Migrate one applet at a time; globals keep working until then.

---

## Upgrade 4 — Extensible, working Settings

**Problem.** The Settings panel lists four labels that do nothing, and there's no
way for an app to add its own controls (Oscilloscope: timebase, trigger, V/div;
Scanner: Phase 1/2 toggle, scan duration).

**Fix.** A tiny `SettingItem` model + a persisted `SettingsStore` (JSON on SD).
Global items live in the shell; the active applet contributes its own.

```cpp
enum class SettingType { Toggle, Range, Choice, Action };
struct SettingItem {
    const char* label; SettingType type;
    float value, min, max; const char* const* choices; int choice_count;
    void (*on_change)(AppContext*, float);   // applied live
};

class Applet {
public:
    virtual int settings_items(SettingItem* out, int max) { return 0; }
};
```

The shell renders global items (brightness → `M5.Display.setBrightness`, actually
wired) then appends `active->settings_items(...)`. `SettingsStore` persists to
`/meshscan/settings.json` (or `/system/settings.json`) and is read back on boot.

**Effort:** ~half day. **Risk:** low–medium. Big usability win across all apps.

---

## Upgrade 5 — Resource & memory discipline (so three heavy apps coexist)

**Problem.** Applets allocate large PSRAM buffers in `on_enter` and **never free
them** (the Viewer keeps its pipeline; the Scanner keeps canvases + a 20k-point
cloud; the Oscilloscope will keep sample ring buffers). Three resident heavyweights
can exhaust PSRAM, and there's no budget or teardown contract.

**Fix.** Make the lifecycle explicit and add a shared canvas pool + a budget:

1. **Contract:** `on_enter` allocates, `on_exit` **frees** heavy buffers (or
   implement `release_resources()` the shell calls on background/low-memory).
   Today both Viewer and Scanner deliberately keep their pipeline for fast
   re-entry — fine for 2 apps, but document it and switch to free-on-exit once a
   third resident app exists.
2. **Shared canvas pool** in `AppContext`: one set of `M5Canvas` render buffers
   borrowed by whichever applet is foreground, instead of each owning its own.
3. **Budget + guard:** document a PSRAM budget per applet and log
   `heap_caps_get_free_size(MALLOC_CAP_SPIRAM)` on enter/exit; refuse to launch if
   the requested reservation won't fit.

```cpp
// applet.h
virtual size_t psram_estimate() const { return 0; }   // declared need, bytes
virtual void   release_resources() {}                  // free on memory pressure
```

```cpp
// shell.h _launch_applet(): guard before entering
if (_applets[i]->psram_estimate() > heap_caps_get_free_size(MALLOC_CAP_SPIRAM)) {
    _toast("Not enough memory"); return;
}
```

**Effort:** ~half day to a day. **Risk:** medium. Pays off the moment the
Oscilloscope is resident alongside the Viewer/Scanner.

---

## Suggested order & effort

| Order | Upgrade | Why now | Effort |
|------:|---------|---------|--------|
| 1 | Lifecycle/pause hooks | Fixes a live UB bug; unblocks the Oscilloscope | 0.5 h |
| 2 | Launcher paging | Trivial, removes the 6-app ceiling | 1–2 h |
| 3 | `AppContext` injection | Arbitrates buses; decouples from globals | 0.5 d |
| 4 | Extensible Settings | Per-app controls actually work | 0.5 d |
| 5 | Memory discipline | Lets 3 heavy apps coexist | 0.5–1 d |

Upgrades 1–2 are quick wins you can land before adding the Oscilloscope. 3–5 are
the structural work that turns the shell into a real platform; do them as you bring
the Oscilloscope in, migrating one applet at a time so nothing breaks mid-flight.

---

## Oscilloscope-specific notes

- It is **timing-sensitive**: the current `loop()` runs `shell.update()` +
  `shell.render()` flat-out with no dt. Give the Oscilloscope its own sampling
  cadence (a dedicated FreeRTOS task like the Viewer's display task, or a hardware
  timer/DMA on the ADC) rather than sampling inside `on_update()`.
- It will want a **peripheral claim** (ADC/SPI front-end) — that's Upgrade 3's
  broker. Declare it in `on_enter`, release in `on_exit`.
- It benefits most from **Upgrade 1** (pause sampling under Settings) and
  **Upgrade 4** (timebase / trigger / V-div as live settings items).
