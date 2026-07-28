// antenna_applet.h - 2.4 GHz antenna-test "oscilloscope" as an M5View applet.
//
// Wraps the standalone M5Tab_AntennaTest sketch (WiFi-RSSI scope + channel
// scanner, polar plot, walk test, multipath, A/B dwell, packet loss, etc.).
// It is a FULLSCREEN applet: it draws and handles touch over the whole screen
// with its own header/nav, and exits back to the M5View home via its own
// Menu -> EXIT (on_update() returns false). All of the sketch's globals and
// functions live in an anonymous `antenna` namespace inside the .cpp so they
// don't collide with the rest of the firmware.

#pragma once
#include "../applet.h"

class AntennaApplet : public Applet {
public:
    void        on_enter()  override;
    void        on_exit()   override;
    bool        on_update() override;   // returns false when the user picks EXIT
    void        on_render() override;   // no-op: drawing happens in on_update()
    bool        is_fullscreen() const override { return true; }
    const char* name() const override { return "Antenna"; }
    const char* icon() const override { return "~"; }
};
