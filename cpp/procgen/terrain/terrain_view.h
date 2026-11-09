#pragma once

// 两个地形演示共用的绘制：高度着色 + 光照，以及一个简单的"斜视 3D"视图
// （逐列从前往后画竖条，只画比已经画过的更高的部分——早期体素地形游戏的画法，没有用 GPU 的 3D）

#include <algorithm>
#include <cmath>
#include <vector>

#include "raylib.h"
#include "terrain.h"

namespace ga::terrain::view {

inline Color lerpColor(Color a, Color b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return {(unsigned char)(a.r + (b.r - a.r) * t), (unsigned char)(a.g + (b.g - a.g) * t), (unsigned char)(a.b + (b.b - a.b) * t), 255};
}

inline Color scale(Color c, float k) {
    auto f = [&](unsigned char v) { return (unsigned char)std::clamp(v * k, 0.0f, 255.0f); };
    return {f(c.r), f(c.g), f(c.b), c.a};
}

// 陆地按高度（0 = 海岸，1 = 最高）分段着色
inline Color landColor(float t) {
    const Color stops[] = {{214, 200, 150, 255}, {120, 160, 80, 255}, {80, 125, 60, 255}, {135, 115, 85, 255}, {150, 140, 130, 255}, {245, 245, 245, 255}};
    const float at[] = {0.0f, 0.06f, 0.35f, 0.62f, 0.82f, 0.95f};
    for (int i = 0; i < 5; ++i)
        if (t <= at[i + 1]) return lerpColor(stops[i], stops[i + 1], (t - at[i]) / (at[i + 1] - at[i]));
    return stops[5];
}

inline Color waterColor(float depth) {  // depth：0 = 海岸，1 = 最深
    return lerpColor({90, 150, 200, 255}, {30, 60, 120, 255}, depth);
}

// 光照：光从左上方来。relief 是高度放大倍数（高度差 × relief 才是"格子"单位）
inline std::vector<float> hillshade(const Heightmap& hm, float relief) {
    std::vector<float> s(hm.v.size(), 1.0f);
    const float lx = -0.6f, ly = -0.6f, lz = 0.53f;
    for (int y = 0; y < hm.h; ++y)
        for (int x = 0; x < hm.w; ++x) {
            int x0 = std::max(0, x - 1), x1 = std::min(hm.w - 1, x + 1), y0 = std::max(0, y - 1), y1 = std::min(hm.h - 1, y + 1);
            float gx = (hm.at(x1, y) - hm.at(x0, y)) / float(x1 - x0) * relief;
            float gy = (hm.at(x, y1) - hm.at(x, y0)) / float(y1 - y0) * relief;
            float len = std::sqrt(gx * gx + gy * gy + 1);
            float d = (-gx * lx - gy * ly + lz) / len;
            s[size_t(y) * hm.w + x] = std::clamp(0.35f + 0.85f * d, 0.25f, 1.25f);
        }
    return s;
}

// 斜视 3D：画到 w × h 的像素缓冲（没画到的地方透明）。
// 地图第 y 行在屏幕上的基线是 top + y·rowStep，高度向上偏移 height·hScale；从前（y 大）往后画
inline void renderOblique(const Heightmap& hm, const std::vector<Color>& colors, float seaLevel, std::vector<Color>& out, int w, int h,
                          float hScale) {
    out.assign(size_t(w) * h, Color{0, 0, 0, 0});
    const float margin = hScale + 4;
    const float rowStep = (h - margin - 6) / float(hm.h);
    for (int sx = 0; sx < w; ++sx) {
        int mx = std::min(hm.w - 1, int((sx + 0.5f) * hm.w / w));
        float ybuf = margin + (hm.h - 1) * rowStep + 1;  // 高度 0 的平面：最前一行的竖条画到这里，形成一个切开的剖面
        for (int my = hm.h - 1; my >= 0; --my) {
            float e = std::max(hm.at(mx, my), seaLevel);
            float base = margin + my * rowStep;
            float top = base - e * hScale;
            if (top >= ybuf) continue;
            Color c = colors[size_t(my) * hm.w + mx];
            int y0 = std::max(0, int(std::floor(top))), y1 = std::min(h, int(std::ceil(ybuf)));
            if (my == hm.h - 1) c = Color{110, 95, 80, 255};  // 剖面：统一的土色
            for (int sy = y0; sy < y1; ++sy) out[size_t(sy) * w + sx] = c;
            ybuf = top;
        }
    }
}

}  // namespace ga::terrain::view
