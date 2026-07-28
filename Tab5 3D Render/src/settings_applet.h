#pragma once
#include "applet.h"

// ============================================================
// settings_applet.h — Settings placeholder applet
//
// Currently opens via the shell's bottom panel overlay.
// This stub exists so settings can be launched as a full
// applet in the future when more options are added.
// ============================================================

class SettingsApplet : public Applet {
public:
    const char* name() const override { return "Settings"; }
    const char* icon() const override { return "*"; }

    void on_enter() override {
        // Nothing to init yet
    }

    void on_exit() override {
        // Nothing to clean up yet
    }

    bool on_update() override {
        // Settings is handled by the shell overlay for now
        // Return false immediately to go back home
        return false;
    }

    void on_render() override {
        // Rendered by shell overlay
    }
};
