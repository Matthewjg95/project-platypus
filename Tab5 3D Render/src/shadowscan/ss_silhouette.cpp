// ss_silhouette.cpp - see ss_silhouette.h for the pipeline description.
#include "ss_silhouette.h"
#include <esp_heap_caps.h>
#include <string.h>

namespace ss {

static int otsu_threshold(const uint8_t* gray, int n) {
    uint32_t hist[256] = {0};
    for (int i = 0; i < n; ++i) hist[gray[i]]++;
    uint64_t total_sum = 0;
    for (int i = 0; i < 256; ++i) total_sum += (uint64_t)i * hist[i];

    uint64_t sum_b = 0;
    uint32_t w_b = 0;
    float best_var = -1;
    int best_t = 128;
    for (int t = 0; t < 256; ++t) {
        w_b += hist[t];
        if (w_b == 0) continue;
        uint32_t w_f = n - w_b;
        if (w_f == 0) break;
        sum_b += (uint64_t)t * hist[t];
        float m_b = (float)sum_b / w_b;
        float m_f = (float)(total_sum - sum_b) / w_f;
        float var = (float)w_b * w_f * (m_b - m_f) * (m_b - m_f);
        if (var > best_var) { best_var = var; best_t = t; }
    }
    return best_t;
}

bool extract_silhouette(const uint8_t* gray, int w, int h,
                        int manual_thresh, Silhouette& sil, size_t max_points) {
    sil = Silhouette();
    const int n = w * h;
    sil.threshold = (manual_thresh >= 0) ? manual_thresh : otsu_threshold(gray, n);

    // Polarity: sample the frame border; the majority class there is the
    // background, so the object is the other one.
    int border_dark = 0, border_total = 0;
    for (int x = 0; x < w; x += 4) {
        border_dark += (gray[x] <= sil.threshold) + (gray[(h - 1) * w + x] <= sil.threshold);
        border_total += 2;
    }
    for (int y = 0; y < h; y += 4) {
        border_dark += (gray[y * w] <= sil.threshold) + (gray[y * w + w - 1] <= sil.threshold);
        border_total += 2;
    }
    sil.dark_object = (border_dark * 2 < border_total);   // border mostly light => dark object

    // Object mask.
    uint8_t* mask = (uint8_t*)heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
    int32_t* stack = (int32_t*)heap_caps_malloc(n * sizeof(int32_t), MALLOC_CAP_SPIRAM);
    if (!mask || !stack) {
        if (mask) free(mask);
        if (stack) free(stack);
        sil.error = "Out of memory";
        return false;
    }
    for (int i = 0; i < n; ++i)
        mask[i] = sil.dark_object ? (gray[i] <= sil.threshold) : (gray[i] >= sil.threshold);

    // Largest 4-connected component: label by flood fill, keep the biggest.
    // mask: 1 = unvisited object, 0 = background, 2 = visited (other blob),
    // 3 = visited (current best blob's final value after the sweep below).
    int best_pixels = 0, best_seed = -1;
    for (int i = 0; i < n; ++i) {
        if (mask[i] != 1) continue;
        int count = 0, sp = 0;
        stack[sp++] = i;
        mask[i] = 2;
        while (sp) {
            int p = stack[--sp];
            ++count;
            int x = p % w, y = p / w;
            if (x > 0     && mask[p - 1] == 1) { mask[p - 1] = 2; stack[sp++] = p - 1; }
            if (x < w - 1 && mask[p + 1] == 1) { mask[p + 1] = 2; stack[sp++] = p + 1; }
            if (y > 0     && mask[p - w] == 1) { mask[p - w] = 2; stack[sp++] = p - w; }
            if (y < h - 1 && mask[p + w] == 1) { mask[p + w] = 2; stack[sp++] = p + w; }
        }
        if (count > best_pixels) { best_pixels = count; best_seed = i; }
    }
    free(stack);
    sil.blob_pixels = best_pixels;

    if (best_seed < 0 || best_pixels < 300) {
        free(mask);
        sil.error = "No object found - check contrast";
        return false;
    }
    if (best_pixels > n * 3 / 4) {
        free(mask);
        sil.error = "Object fills the frame - move back";
        return false;
    }

    // Re-isolate ONLY the best blob (mask currently has all blobs == 2):
    // reflood from the seed with value 3, then treat 3 as "inside".
    {
        int32_t* st2 = (int32_t*)heap_caps_malloc(best_pixels * sizeof(int32_t) + 64,
                                                  MALLOC_CAP_SPIRAM);
        if (!st2) { free(mask); sil.error = "Out of memory"; return false; }
        int sp = 0;
        st2[sp++] = best_seed;
        mask[best_seed] = 3;
        while (sp) {
            int p = st2[--sp];
            int x = p % w, y = p / w;
            if (x > 0     && mask[p - 1] == 2) { mask[p - 1] = 3; st2[sp++] = p - 1; }
            if (x < w - 1 && mask[p + 1] == 2) { mask[p + 1] = 3; st2[sp++] = p + 1; }
            if (y > 0     && mask[p - w] == 2) { mask[p - w] = 3; st2[sp++] = p - w; }
            if (y < h - 1 && mask[p + w] == 2) { mask[p + w] = 3; st2[sp++] = p + w; }
        }
        free(st2);
    }
    auto inside = [&](int x, int y) -> bool {
        return x >= 0 && x < w && y >= 0 && y < h && mask[y * w + x] == 3;
    };

    // Start pixel: topmost-leftmost of the blob (guaranteed boundary).
    int sx = -1, sy = -1;
    for (int i = 0; i < n; ++i)
        if (mask[i] == 3) { sx = i % w; sy = i / w; break; }

    // Moore-neighbour trace, clockwise neighbourhood.
    static const int8_t NB[8][2] = {{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1}};
    Poly contour;
    contour.push_back(V2((float)sx, (float)sy));
    int cx = sx, cy = sy, prev_dir = 0;
    const int max_trace = 4 * (w + h) + best_pixels;   // generous perimeter bound
    for (int step = 0; step < max_trace; ++step) {
        bool found = false;
        for (int i = 0; i < 8; ++i) {
            int d = (prev_dir + 6 + i) % 8;      // back up two, sweep clockwise
            int nx = cx + NB[d][0], ny = cy + NB[d][1];
            if (inside(nx, ny)) {
                cx = nx; cy = ny; prev_dir = d;
                found = true;
                break;
            }
        }
        if (!found) break;                        // isolated pixel
        if (cx == sx && cy == sy && contour.size() > 2) break;   // closed
        contour.push_back(V2((float)cx, (float)cy));
    }
    free(mask);

    if (contour.size() < 8) {
        sil.error = "Contour trace failed - retry";
        return false;
    }

    // Adaptive RDP: loosen epsilon until under the cap.
    float eps = 1.2f;
    Poly out = simplify(contour, eps);
    while (out.size() > max_points && eps < 24.0f) {
        eps *= 1.5f;
        out = simplify(contour, eps);
    }
    ensure_ccw(out);
    sil.pts = out;
    return true;
}

}  // namespace ss
