// room_fitter.h - Phase 1 rectangular-room fitter.
//
// Over a 360 scan we accumulate two kinds of evidence:
//   1. Dominant line orientations from a Hough transform on each frame's edge
//      map (Sobel -> threshold -> Hough). Strong near-vertical lines are wall
//      edges; their distribution tells us how "boxy" / Manhattan the room is and
//      feeds a confidence score.
//   2. The XZ extent of objects the depth estimator has placed, which sets the
//      box footprint (a monocular scan can't recover absolute room size from
//      lines alone, so object distances anchor the scale).
//
// fit() combines both into an axis-aligned rectangular box (6 faces). This is a
// deliberately simple heuristic — Phase 2 replaces it with real triangulated
// geometry from a sparse point cloud.

#pragma once
#include <stdint.h>

struct RoomBox {
    float cx, cz;     // footprint centre (metres), floor plane
    float width;      // X extent (metres)
    float depth;      // Z extent (metres)
    float height;     // Y extent (metres, floor to ceiling)
    float confidence; // 0..1, how strongly the Hough evidence supports a box
};

class RoomFitter {
public:
    // gray_w/gray_h: resolution of the grayscale frames passed to addFrame().
    // Pass a downscaled gray image (e.g. 160x120) to keep the Hough cheap.
    RoomFitter(int gray_w = 160, int gray_h = 120);
    ~RoomFitter();

    bool begin();     // allocates Hough accumulator + edge buffer in PSRAM
    void reset();     // clear accumulators between scans

    // Feed one scan frame. `gray` is gray_w*gray_h bytes. yaw_rad unused for the
    // box fit itself but reserved for orientation-aware Phase-2 work.
    void addFrame(const uint8_t* gray, float yaw_rad);

    // Record where an object landed so the footprint can enclose it.
    void addObjectExtent(float x, float z);

    // Synthesize the room box from accumulated evidence.
    RoomBox fit() const;

    // Diagnostics
    int framesProcessed() const { return _frames; }

    static constexpr float DEFAULT_CEILING = 2.5f;   // metres
    static constexpr float FOOTPRINT_MARGIN = 0.5f;  // metres added around objects
    static constexpr float DEFAULT_SIDE = 4.0f;      // fallback room size, metres

private:
    int  _w, _h;
    int  _numTheta;          // angle bins across 0..180 deg
    int  _numRho;            // distance bins (2*diag+1)
    int  _rhoOffset;         // = diag, maps rho into [0, numRho)
    float* _cosT = nullptr;  // [_numTheta]
    float* _sinT = nullptr;  // [_numTheta]
    uint16_t* _acc = nullptr;// Hough accumulator [_numTheta*_numRho]
    uint8_t*  _edge = nullptr;// edge map [_w*_h]

    // global orientation histogram, summed peak strength per theta bin
    uint32_t* _thetaHist = nullptr; // [_numTheta]

    // object footprint extent
    bool  _haveExtent = false;
    float _minX, _maxX, _minZ, _maxZ;

    int _frames = 0;

    void sobelEdges(const uint8_t* gray);   // -> _edge (binary)
    void houghVote();                       // _edge -> _acc, accumulate peaks
};
