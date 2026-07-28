// project_manager.h - SD-card project layout: buildings -> rooms -> .mesh files.
//
// On disk:
//   /meshscan/
//     buildings/
//       <building name>/
//         <room name>.mesh
//
// Names are sanitised to a safe subset for FAT (alnum, space, dash, underscore).
// All methods are best-effort and return false on SD errors.

#pragma once
#include <stdint.h>
#include <stddef.h>   // size_t

#ifndef PM_MAX_NAME
#define PM_MAX_NAME 48
#endif
#ifndef PM_MAX_LIST
#define PM_MAX_LIST 64
#endif

struct NameList {
    char     items[PM_MAX_LIST][PM_MAX_NAME];
    uint16_t count;
};

class ProjectManager {
public:
    bool begin();                       // ensure /meshscan/buildings exists

    bool createBuilding(const char* name);
    bool deleteBuilding(const char* name);          // recursive
    void listBuildings(NameList& out);

    bool deleteRoom(const char* building, const char* room);
    void listRooms(const char* building, NameList& out);  // names without .mesh

    // Absolute path for a room's .mesh, creating the building dir if needed.
    // Returns false if the path would overflow `out`.
    bool meshPath(const char* building, const char* room, char* out, size_t out_sz);

    // Next unique room name like "room_3" inside a building (for quick scans).
    void suggestRoomName(const char* building, char* out, size_t out_sz);

private:
    static void sanitize(const char* in, char* out, size_t out_sz);
    static bool rmrf(const char* path);             // recursive delete
};
