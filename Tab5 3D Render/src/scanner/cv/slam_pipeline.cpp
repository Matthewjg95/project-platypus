// slam_pipeline.cpp
#include "slam_pipeline.h"
#include "linalg.h"
#include "triangulator.h"
#include <string.h>
#include <stdlib.h>

#if defined(ESP_PLATFORM) || defined(ARDUINO)
  #include <esp_heap_caps.h>
  static void* sp_alloc(size_t n) { void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM); return p ? p : malloc(n); }
#else
  static void* sp_alloc(size_t n) { return malloc(n); }
#endif

namespace cv {

static void identity3(float R[9]) { for (int i=0;i<9;++i) R[i]=0; R[0]=R[4]=R[8]=1; }

bool SlamPipeline::begin(const CameraIntrinsics& K, uint32_t cloud_capacity) {
    _K = K;
    if (!_cloud.begin(cloud_capacity)) return false;
    _prev = (Keypoints*)sp_alloc(sizeof(Keypoints));
    _cur  = (Keypoints*)sp_alloc(sizeof(Keypoints));
    _matchBuf = (Match*)sp_alloc(sizeof(Match) * Keypoints::MAX);
    _p1 = (float*)sp_alloc(sizeof(float) * Keypoints::MAX * 2);
    _p2 = (float*)sp_alloc(sizeof(float) * Keypoints::MAX * 2);
    if (!_prev || !_cur || !_matchBuf || !_p1 || !_p2) return false;
    reset();
    return true;
}

void SlamPipeline::reset() {
    _cloud.clear();
    _hasPrev = false;
    _frames = _lastMatches = _lastInliers = 0;
    identity3(_Rg); _tg[0]=_tg[1]=_tg[2]=0;
}

void SlamPipeline::freeMem() {
    _cloud.freeMem();
    free(_prev); free(_cur); free(_matchBuf); free(_p1); free(_p2);
    _prev=_cur=nullptr; _matchBuf=nullptr; _p1=_p2=nullptr;
}

// Triangulate the current matched pair and fold points into the world cloud.
void SlamPipeline::triangulatePair(int nmatch, const float R[9], const float t[3]) {
    // projection matrices in normalised coords: P1=[I|0], P2=[R | t*scale]
    float ts[3] = { t[0]*_scale, t[1]*_scale, t[2]*_scale };
    float P1[12] = {1,0,0,0, 0,1,0,0, 0,0,1,0};
    float P2[12] = { R[0],R[1],R[2],ts[0], R[3],R[4],R[5],ts[1], R[6],R[7],R[8],ts[2] };

    float Rgt[9]; transpose(_Rg, 3, 3, Rgt);     // world<-prevcam rotation

    int added = 0;
    for (int i = 0; i < nmatch; ++i) {
        float x1,y1,x2,y2;
        _K.normalize(_p1[2*i], _p1[2*i+1], x1, y1);
        _K.normalize(_p2[2*i], _p2[2*i+1], x2, y2);
        float Xp[3];
        if (!triangulate_dlt(P1, P2, x1,y1, x2,y2, Xp)) continue;
        if (Xp[2] <= 0 || Xp[2] > max_point_depth) continue;   // cheirality + range
        // cam(cur) depth check
        float Xc2[3]; mat3_mulvec(R, Xp, Xc2);
        Xc2[0]+=ts[0]; Xc2[1]+=ts[1]; Xc2[2]+=ts[2];
        if (Xc2[2] <= 0) continue;

        // prev-cam -> world:  Xw = Rg^T (Xp - tg)
        float d[3] = { Xp[0]-_tg[0], Xp[1]-_tg[1], Xp[2]-_tg[2] };
        float Xw[3]; mat3_mulvec(Rgt, d, Xw);

        // colour from the previous frame's intensity at the feature (gray)
        uint8_t g = 200;   // TODO: sample actual gray; kept neutral for now
        _cloud.add(Xw[0], Xw[1], Xw[2], g, g, g);
        ++added;
    }
    _lastInliers = added;

    // advance global pose to the current frame: R_cur = R Rg, t_cur = R tg + t*scale
    float Rcur[9]; mat3_mul(R, _Rg, Rcur);
    float Rtg[3]; mat3_mulvec(R, _tg, Rtg);
    float tcur[3] = { Rtg[0]+ts[0], Rtg[1]+ts[1], Rtg[2]+ts[2] };
    memcpy(_Rg, Rcur, sizeof(Rcur));
    _tg[0]=tcur[0]; _tg[1]=tcur[1]; _tg[2]=tcur[2];
}

bool SlamPipeline::addFrame(const uint8_t* gray, int w, int h) {
    _orb.extract(gray, w, h, *_cur);
    ++_frames;

    if (!_hasPrev) {
        memcpy(_prev, _cur, sizeof(Keypoints));
        identity3(_Rg); _tg[0]=_tg[1]=_tg[2]=0;
        _hasPrev = true;
        return false;
    }

    int nm = _matcher.match(*_prev, *_cur, _matchBuf, Keypoints::MAX);
    _lastMatches = nm;
    if (nm < min_matches) { memcpy(_prev, _cur, sizeof(Keypoints)); return false; }

    for (int i = 0; i < nm; ++i) {
        int a = _matchBuf[i].a, b = _matchBuf[i].b;
        _p1[2*i] = _prev->x[a]; _p1[2*i+1] = _prev->y[a];
        _p2[2*i] = _cur->x[b];  _p2[2*i+1] = _cur->y[b];
    }

    PoseResult pose = estimate_relative_pose(_p1, _p2, nm, _K);
    bool triangulated = false;
    if (pose.valid && pose.planarity <= planarity_reject) {
        triangulatePair(nm, pose.R, pose.t);
        triangulated = true;
    }

    memcpy(_prev, _cur, sizeof(Keypoints));
    return triangulated;
}

} // namespace cv
