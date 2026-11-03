// JPS 与 A* 的性能对比（8 方向，代价统一），256×256 地图，两种地形：
//   rooms ：大片空地 + 随机矩形障碍（类似室内 / 城镇地图）
//   noise ：每格 25% 概率是墙（大量零碎障碍，JPS 的最坏情况之一）
// 用法（在 cpp/ 目录下）：
//   c++ -O2 -std=c++20 -I common pathfinding/jps/jps_bench.cpp -o jps_bench && ./jps_bench

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "jps.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

struct Stats {
    double micros = 0;
    double expanded = 0;
    double scanned = 0;
};

static Stats measure(const Grid& g, const std::vector<std::pair<Cell, Cell>>& queries, Grid8Mode mode) {
    Stats st;
    double best = 1e30;
    for (int run = 0; run < 5; ++run) {
        long expanded = 0, scanned = 0;
        auto t0 = Clock::now();
        for (auto [a, b] : queries) {
            Grid8Search s(g, a, b, mode);
            while (s.step()) {}
            expanded += s.expandedCount();
            scanned += s.scanCount();
        }
        best = std::min(best, std::chrono::duration<double, std::micro>(Clock::now() - t0).count());
        st.expanded = static_cast<double>(expanded) / queries.size();
        st.scanned = static_cast<double>(scanned) / queries.size();
    }
    st.micros = best / queries.size();
    return st;
}

int main() {
    constexpr int kSize = 256;
    for (const char* kind : {"rooms", "noise"}) {
        Pcg32 rng(11);
        Grid g(kSize, kSize);
        if (kind[0] == 'r') {
            for (int k = 0; k < 220; ++k) {  // 随机矩形障碍
                int x = static_cast<int>(rng.below(kSize)), y = static_cast<int>(rng.below(kSize));
                int w = 2 + static_cast<int>(rng.below(14)), h = 2 + static_cast<int>(rng.below(14));
                for (int yy = y; yy < std::min(kSize, y + h); ++yy)
                    for (int xx = x; xx < std::min(kSize, x + w); ++xx) g.setWall({xx, yy}, true);
            }
        } else {
            for (int i = 0; i < g.size(); ++i) {
                if (rng.below(100) < 25) g.setWall(g.cellAt(i), true);
            }
        }
        std::vector<std::pair<Cell, Cell>> queries;
        while (queries.size() < 200) {
            Cell a{static_cast<int>(rng.below(kSize)), static_cast<int>(rng.below(kSize))};
            Cell b{static_cast<int>(rng.below(kSize)), static_cast<int>(rng.below(kSize))};
            if (!g.passable(a) || !g.passable(b)) continue;
            Grid8Search s(g, a, b, Grid8Mode::Jps);
            while (s.step()) {}
            if (s.found()) queries.push_back({a, b});
        }
        Stats a = measure(g, queries, Grid8Mode::AStar);
        Stats j = measure(g, queries, Grid8Mode::Jps);
        std::printf("[%s]\n", kind);
        std::printf("  A*   %9.1f us/query  expanded %7.0f\n", a.micros, a.expanded);
        std::printf("  JPS  %9.1f us/query  expanded %7.0f  scanned %7.0f\n", j.micros, j.expanded, j.scanned);
    }
}
