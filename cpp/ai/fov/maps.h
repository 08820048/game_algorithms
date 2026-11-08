#pragma once

// 演示和测试用的地图生成：房间 + L 形走廊，部分房间里有柱子
#include "../../foundations/random/random.h"
#include "fov.h"

namespace ga::fov {

inline Grid makeDungeon(Pcg32& rng, int w, int h, int rooms) {
    Grid g(w, h);
    std::fill(g.wall.begin(), g.wall.end(), 1);
    int px = -1, py = -1;
    for (int k = 0; k < rooms; ++k) {
        int rw = 5 + rng.below(9), rh = 4 + rng.below(6);
        int x0 = 1 + rng.below(w - rw - 2), y0 = 1 + rng.below(h - rh - 2);
        for (int y = y0; y < y0 + rh; ++y)
            for (int x = x0; x < x0 + rw; ++x) g.wall[g.index(x, y)] = 0;
        if (rng.below(3) == 0)
            for (int y = y0 + 1; y < y0 + rh - 1; y += 2)
                for (int x = x0 + 1; x < x0 + rw - 1; x += 3) g.wall[g.index(x, y)] = 1;
        int cx = x0 + rw / 2, cy = y0 + rh / 2;
        if (px >= 0) {
            for (int x = std::min(px, cx); x <= std::max(px, cx); ++x) g.wall[g.index(x, py)] = 0;
            for (int y = std::min(py, cy); y <= std::max(py, cy); ++y) g.wall[g.index(cx, y)] = 0;
        }
        px = cx, py = cy;
    }
    return g;
}

}  // namespace ga::fov
