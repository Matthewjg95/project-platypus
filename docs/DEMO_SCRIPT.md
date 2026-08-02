# Demo Script & Pre-Flight Checklist

The two-minute story: **"A WiFi weather map, drawn by the instrument that
measured the room."**

---

## Pre-flight (run this before filming — every item has bitten us)

| # | Check | Why |
|---|-------|-----|
| 1 | microSD seated in the Tab5 (hear the click) | rooms/settings live there; a loose card = "SD card not found" |
| 2 | microSD seated in the Unit V | the kmodel loads from `/sd` |
| 3 | Grove cable firmly in both ends | half-seated = frames but no detections |
| 4 | Battery charged, or USB-C power attached | a mid-demo brownout ends the take |
| 5 | Power-cycle once and let it boot clean | proves the splash + startup path on camera |
| 6 | Room lit brightly, blinds open | the K210 detects far better with light |
| 7 | Demo room contains a **monitor/TV, a chair, and a sofa or table** | the classes this model is most confident on |
| 8 | Phone/second camera on a tripod or propped | one-take handheld filming of a 360° sweep is hard |
| 9 | Delete practice rooms first (long-press → Delete) | a clean list looks intentional |
| 10 | Know your AP's SSID and have it selected in RF | avoids fumbling the picker on camera |

---

## Shot list (target 2:00–2:30)

**0:00–0:15 — Cold open.** Power button. Splash comes up (PROJECT PLATYPUS /
Room Scanner + RF Survey), then the three tiles. Say what it is in one
sentence: *"This is an M5Stack Tab5 that scans a room with an AI camera and
then paints the WiFi signal onto the floor plan it just drew."*

**0:15–0:30 — The hardware.** Slow pan of the rig: Tab5, Unit V camera on
top, the custom antenna PCB on the back, battery sled. Name the parts.

**0:30–1:05 — The scan (hero shot).** Room Scan → Scan. Hold still, wait for
the tick, then sweep. Film the screen: the progress ring filling clockwise,
detection pips dropping onto the dial at the bearing where each object was
found, live camera preview inside the ring. Let the auto-finish land.

**1:05–1:25 — The result.** The 3D room appears: carved floor plan, object
markers with name labels, the start arrow at the corner you faced. Orbit it
with a finger. Point out that **the floor shape came from what the camera
could see** — sight-lines to objects prove open floor, so the room is not a
rectangle.

**1:25–1:55 — The payoff.** Tap RF. Walk the room, tapping your position on
the plan and sampling RSSI at each spot. The heatmap fills in. Show the
legend and the **DEAD** / **BEST** callouts. Then hit the antenna toggle and
show the *same room, second layer* — internal antenna vs external antenna
coverage, measured on a map the device drew itself.

**1:55–2:15 — Close.** Building map: the rooms as puzzle pieces. One line on
where it goes next (ToF-measured walls, custom door/window model). End on the
device in hand.

---

## What NOT to film

- Anything that misbehaves in rehearsal — cut it, don't fix it under deadline
- The OTA/dev tooling (mention it in the writeup instead; it reads as
  engineering maturity there, as filler on video)
- Long silent pauses while the camera is thinking — cut to the result

---

## Framing notes

- Screen-record where possible; the 1280×720 display films better straight-on
  than over the shoulder
- The serial screenshot tool (`tools/tab5_screenshot.py`) produces
  pixel-perfect stills for the writeup — use those, not photos of the screen
- Keep the tablet's brightness up; the survey greens/reds wash out otherwise
