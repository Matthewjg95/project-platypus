// project_manager.cpp
#include "project_manager.h"
#include <SD_MMC.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

static const char* ROOT = "/meshscan";
static const char* BUILDINGS = "/meshscan/buildings";

void ProjectManager::sanitize(const char* in, char* out, size_t out_sz) {
    size_t j = 0;
    for (size_t i = 0; in && in[i] && j + 1 < out_sz; ++i) {
        char c = in[i];
        if (isalnum((unsigned char)c) || c == ' ' || c == '-' || c == '_')
            out[j++] = c;
    }
    if (j == 0 && out_sz > 1) out[j++] = '_';
    out[j] = '\0';
}

bool ProjectManager::begin() {
    if (!SD_MMC.exists(ROOT))      SD_MMC.mkdir(ROOT);
    if (!SD_MMC.exists(BUILDINGS)) SD_MMC.mkdir(BUILDINGS);
    return SD_MMC.exists(BUILDINGS);
}

bool ProjectManager::createBuilding(const char* name) {
    char safe[PM_MAX_NAME]; sanitize(name, safe, sizeof(safe));
    char path[160]; snprintf(path, sizeof(path), "%s/%s", BUILDINGS, safe);
    if (SD_MMC.exists(path)) return true;
    return SD_MMC.mkdir(path);
}

bool ProjectManager::rmrf(const char* path) {
    File d = SD_MMC.open(path);
    if (!d) return false;
    if (!d.isDirectory()) { d.close(); return SD_MMC.remove(path); }
    for (File e = d.openNextFile(); e; e = d.openNextFile()) {
        char child[200];
        snprintf(child, sizeof(child), "%s/%s", path, e.name());
        bool dir = e.isDirectory();
        e.close();
        if (dir) rmrf(child); else SD_MMC.remove(child);
    }
    d.close();
    return SD_MMC.rmdir(path);
}

bool ProjectManager::deleteBuilding(const char* name) {
    char safe[PM_MAX_NAME]; sanitize(name, safe, sizeof(safe));
    char path[160]; snprintf(path, sizeof(path), "%s/%s", BUILDINGS, safe);
    return rmrf(path);
}

void ProjectManager::listBuildings(NameList& out) {
    out.count = 0;
    File dir = SD_MMC.open(BUILDINGS);
    if (!dir) return;
    for (File e = dir.openNextFile(); e && out.count < PM_MAX_LIST; e = dir.openNextFile()) {
        if (e.isDirectory()) {
            const char* nm = e.name();
            const char* base = strrchr(nm, '/'); base = base ? base + 1 : nm;
            strncpy(out.items[out.count], base, PM_MAX_NAME - 1);
            out.items[out.count][PM_MAX_NAME - 1] = '\0';
            out.count++;
        }
        e.close();
    }
    dir.close();
}

void ProjectManager::listRooms(const char* building, NameList& out) {
    out.count = 0;
    char safe[PM_MAX_NAME]; sanitize(building, safe, sizeof(safe));
    char path[160]; snprintf(path, sizeof(path), "%s/%s", BUILDINGS, safe);
    File dir = SD_MMC.open(path);
    if (!dir) return;
    for (File e = dir.openNextFile(); e && out.count < PM_MAX_LIST; e = dir.openNextFile()) {
        if (!e.isDirectory()) {
            const char* nm = e.name();
            const char* base = strrchr(nm, '/'); base = base ? base + 1 : nm;
            const char* dot = strstr(base, ".mesh");
            size_t len = dot ? (size_t)(dot - base) : strlen(base);
            if (len >= PM_MAX_NAME) len = PM_MAX_NAME - 1;
            memcpy(out.items[out.count], base, len);
            out.items[out.count][len] = '\0';
            out.count++;
        }
        e.close();
    }
    dir.close();
}

bool ProjectManager::deleteRoom(const char* building, const char* room) {
    char path[200];
    if (!meshPath(building, room, path, sizeof(path))) return false;
    bool ok = SD_MMC.remove(path);
    // remove sidecars too (.lbl object labels, .rf survey) — ignore absence
    char* dot = strrchr(path, '.');
    if (dot) {
        strcpy(dot, ".lbl"); SD_MMC.remove(path);
        strcpy(dot, ".rf");  SD_MMC.remove(path);
    }
    return ok;
}

bool ProjectManager::meshPath(const char* building, const char* room,
                              char* out, size_t out_sz) {
    char sb[PM_MAX_NAME], sr[PM_MAX_NAME];
    sanitize(building, sb, sizeof(sb));
    sanitize(room, sr, sizeof(sr));
    char dir[160]; snprintf(dir, sizeof(dir), "%s/%s", BUILDINGS, sb);
    if (!SD_MMC.exists(dir)) SD_MMC.mkdir(dir);
    int n = snprintf(out, out_sz, "%s/%s.mesh", dir, sr);
    return n > 0 && (size_t)n < out_sz;
}

void ProjectManager::suggestRoomName(const char* building, char* out, size_t out_sz) {
    NameList rooms; listRooms(building, rooms);
    for (int i = 1; i < 1000; ++i) {
        char cand[PM_MAX_NAME]; snprintf(cand, sizeof(cand), "room_%d", i);
        bool taken = false;
        for (uint16_t r = 0; r < rooms.count; ++r)
            if (strcmp(rooms.items[r], cand) == 0) { taken = true; break; }
        if (!taken) { strncpy(out, cand, out_sz - 1); out[out_sz - 1] = '\0'; return; }
    }
    strncpy(out, "room", out_sz - 1); out[out_sz - 1] = '\0';
}
