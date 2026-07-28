// surface_recon.cpp
#include "surface_recon.h"
#include <stdlib.h>
#include <math.h>

namespace cv {

static inline float dist2(const float* p, uint32_t i, uint32_t j) {
    float dx=p[i*3]-p[j*3], dy=p[i*3+1]-p[j*3+1], dz=p[i*3+2]-p[j*3+2];
    return dx*dx+dy*dy+dz*dz;
}

// Open-addressing set of canonical (sorted) triangle keys for de-duplication.
struct FaceSet {
    uint64_t* keys; uint32_t cap; bool ok;
    bool begin(uint32_t n) {
        cap = 1; while (cap < n*2) cap <<= 1;     // power-of-two, ~50% load
        keys = (uint64_t*)malloc(sizeof(uint64_t)*cap);
        ok = keys != nullptr;
        if (ok) for (uint32_t i=0;i<cap;++i) keys[i]=UINT64_MAX;
        return ok;
    }
    void end() { free(keys); keys=nullptr; }
    static uint64_t makeKey(uint16_t a, uint16_t b, uint16_t c) {
        uint16_t t;
        if (a>b){t=a;a=b;b=t;} if (b>c){t=b;b=c;c=t;} if (a>b){t=a;a=b;b=t;}
        return ((uint64_t)a<<32)|((uint64_t)b<<16)|c;
    }
    // returns true if newly inserted (not seen before)
    bool insert(uint64_t key) {
        if (!ok) return true;                      // set full/unavailable: allow dup
        uint32_t mask = cap-1;
        uint32_t h = (uint32_t)((key*0x9E3779B97F4A7C15ull) >> 32) & mask;
        for (uint32_t probe=0; probe<cap; ++probe) {
            uint32_t s=(h+probe)&mask;
            if (keys[s]==UINT64_MAX) { keys[s]=key; return true; }
            if (keys[s]==key) return false;
        }
        return true;
    }
};

uint32_t greedy_triangulate(const PointCloud& pc, const SurfaceReconParams& prm,
                            FaceSink sink, void* user) {
    const uint32_t n = pc.count;
    if (n < 3 || !sink) return 0;
    const float* P = pc.xyz;
    const float me2 = prm.max_edge * prm.max_edge;
    const int K = prm.k < 2 ? 2 : prm.k;

    FaceSet fs; fs.begin(prm.max_faces);

    // per-query neighbour scratch
    int*   nbr  = (int*)malloc(sizeof(int)*K);
    float* nbd  = (float*)malloc(sizeof(float)*K);
    uint32_t faces = 0;

    for (uint32_t i = 0; i < n && faces < prm.max_faces; ++i) {
        // brute-force k nearest (TODO: spatial grid for O(n) instead of O(n^2))
        int found = 0;
        for (int q=0;q<K;++q){ nbr[q]=-1; nbd[q]=1e30f; }
        for (uint32_t j = 0; j < n; ++j) {
            if (j == i) continue;
            float d = dist2(P, i, j);
            if (d > me2) continue;
            // insert into the K-best (simple insertion)
            if (d < nbd[K-1]) {
                int pos = K-1;
                while (pos>0 && nbd[pos-1]>d) { nbd[pos]=nbd[pos-1]; nbr[pos]=nbr[pos-1]; --pos; }
                nbd[pos]=d; nbr[pos]=(int)j;
                if (found<K) ++found;
            }
        }
        // form triangles from neighbour pairs with all edges <= max_edge
        for (int a = 0; a < found && faces < prm.max_faces; ++a) {
            int j = nbr[a]; if (j < 0) continue;
            for (int b = a+1; b < found; ++b) {
                int l = nbr[b]; if (l < 0) continue;
                if (dist2(P, (uint32_t)j, (uint32_t)l) > me2) continue;
                uint64_t key = FaceSet::makeKey((uint16_t)i,(uint16_t)j,(uint16_t)l);
                if (!fs.insert(key)) continue;
                sink((uint16_t)i, (uint16_t)j, (uint16_t)l, user);
                if (++faces >= prm.max_faces) break;
            }
        }
    }

    free(nbr); free(nbd); fs.end();
    return faces;
}

} // namespace cv
