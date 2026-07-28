#pragma once
#include <M5Unified.h>
#include <M5GFX.h>

// ============================================================
// applet.h — Base interface for all applets
//
// Every applet implements these four methods.
// The shell calls them at the appropriate times.
// Applets never call each other directly.
// ============================================================

class Applet {
public:
    virtual ~Applet() {}

    // Called once when the applet becomes the active screen.
    // Use for resource allocation, loading data, drawing initial UI.
    virtual void on_enter() = 0;

    // Called once when leaving the applet.
    // Use for cleanup, saving state, releasing resources.
    virtual void on_exit() = 0;

    // Called every frame. Handle input and update state here.
    // Return false to signal the shell to go back to home screen.
    virtual bool on_update() = 0;

    // Called every frame after on_update(). Draw here.
    virtual void on_render() = 0;

    // Called when the back button is tapped while this applet is active.
    // Return true to handle it internally (stay in applet).
    // Return false to let the shell exit the applet (go to home screen).
    virtual bool on_back() { return false; }

    // Called when the applet is backgrounded (e.g. the Settings panel opens
    // over it) and when it returns to the foreground. Applets that draw from a
    // background task or run continuous sampling should stop in on_pause() so
    // they don't overdraw the panel, and resume in on_resume(). Default no-op.
    virtual void on_pause()  {}
    virtual void on_resume() {}

    // Immersive applets that draw and handle touch over the ENTIRE screen
    // (their own header/nav, no shell top bar). The shell suppresses its bar
    // and stops intercepting touch for these; they must provide their own exit
    // (return false from on_update() to leave). Default false.
    virtual bool is_fullscreen() const { return false; }

    // Display name shown on the home screen icon
    virtual const char* name() const = 0;

    // Icon character (unicode or ASCII) shown on home screen tile
    virtual const char* icon() const = 0;
};