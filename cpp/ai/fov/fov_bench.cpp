// 视野：各算法的开销、与精确定义的差异、对称性、单点视线的开销（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/fov/fov_bench.cpp -o fov_bench && ./fov_bench
#include <chrono>
#include <cstdio>
#include <functional>

#include "../../foundations/random/random.h"
#include "fov.h"

using namespace ga;
using namespace ga::fov;
using Clock = std::chrono::steady_clock;

static Grid randomGrid(Pcg32& rng, int w, int h, double density) {
    Grid g(w, h);
    for (auto& c : g.wall) c = rng.nextDouble() < density;
    return g;
}

// 房间加走廊：在全是墙的地图上挖出矩形房间，再用 L 形走廊连接
static Grid dungeon(Pcg32& rng, int w, int h) {
    Grid g(w, h);
    std::fill(g.wall.begin(), g.wall.end(), 1);
    int px = -1, py = -1;
    for (int k = 0; k < w * h / 180; ++k) {
        int rw = 4 + rng.below(9), rh = 4 + rng.below(7);
        int x0 = 1 + rng.below(w - rw - 2), y0 = 1 + rng.below(h - rh - 2);
        for (int y = y0; y < y0 + rh; ++y)
            for (int x = x0; x < x0 + rw; ++x) g.wall[g.index(x, y)] = 0;
        if (rng.below(3) == 0)  // 有的房间里有柱子
            for (int y = y0 + 1; y < y0 + rh - 1; y += 2)
                for (int x = x0 + 1; x < x0 + rw - 1; x += 2) g.wall[g.index(x, y)] = 1;
        int cx = x0 + rw / 2, cy = y0 + rh / 2;
        if (px >= 0) {
            for (int x = std::min(px, cx); x <= std::max(px, cx); ++x) g.wall[g.index(x, py)] = 0;
            for (int y = std::min(py, cy); y <= std::max(py, cy); ++y) g.wall[g.index(cx, y)] = 0;
        }
        px = cx, py = cy;
    }
    return g;
}

using Fn = std::function<void(const Grid&, int, int, int, std::vector<uint8_t>&)>;

static std::vector<std::pair<int, int>> openCells(Pcg32& rng, const Grid& g, int n) {
    std::vector<std::pair<int, int>> out;
    while (int(out.size()) < n) {
        int x = rng.below(g.w), y = rng.below(g.h);
        if (!g.wall[g.index(x, y)]) out.push_back({x, y});
    }
    return out;
}

static double usPer(const Grid& g, const std::vector<std::pair<int, int>>& origins, int radius, const Fn& f) {
    std::vector<uint8_t> out;
    double best = 1e30;
    for (int round = 0; round < 5; ++round) {
        auto t0 = Clock::now();
        for (auto [x, y] : origins) f(g, x, y, radius, out);
        best = std::min(best, std::chrono::duration<double, std::micro>(Clock::now() - t0).count() / origins.size());
    }
    return best;
}

int main() {
    Pcg32 rng(2026);
    struct Map {
        const char* name;
        Grid g;
    };
    std::vector<Map> maps = {{"open 10%", randomGrid(rng, 128, 128, 0.10)},
                             {"cave 30%", randomGrid(rng, 128, 128, 0.30)},
                             {"dungeon", dungeon(rng, 128, 128)}};
    const Fn fns[] = {shadowcast, classicShadowcast, rayCast, fovBrute};
    const char* names[] = {"shadowcast", "classic", "ray cast", "brute"};

    std::printf("== time per FOV (microseconds), 128x128 maps ==\n%-10s %6s", "map", "radius");
    for (const char* n : names) std::printf(" %11s", n);
    std::printf("  visible cells\n");
    for (Map& m : maps) {
        auto origins = openCells(rng, m.g, 200);
        for (int radius : {8, 16, 32}) {
            std::printf("%-10s %6d", m.name, radius);
            for (int k = 0; k < 4; ++k) {
                if (k == 3 && radius > 16) {
                    std::printf(" %11s", "-");
                    continue;
                }
                std::printf(" %11.1f", usPer(m.g, origins, radius, fns[k]));
            }
            std::vector<uint8_t> out;
            double vis = 0;
            for (auto [x, y] : origins) {
                shadowcast(m.g, x, y, radius, out);
                for (uint8_t v : out) vis += v;
            }
            std::printf("  %.0f\n", vis / origins.size());
        }
    }

    std::printf("\n== difference from the exact definition (radius 16, 300 origins per map) ==\n");
    std::printf("%-10s %-10s %14s %14s %14s %14s\n", "map", "method", "extra floor", "missing floor", "extra wall", "missing wall");
    for (Map& m : maps) {
        auto origins = openCells(rng, m.g, 300);
        for (int k : {1, 2}) {
            long long exF = 0, miF = 0, exW = 0, miW = 0, visF = 0, visW = 0;
            std::vector<uint8_t> ref, out;
            for (auto [x, y] : origins) {
                shadowcast(m.g, x, y, 16, ref);
                fns[k](m.g, x, y, 16, out);
                for (size_t i = 0; i < ref.size(); ++i) {
                    bool wall = m.g.wall[i];
                    (wall ? visW : visF) += ref[i];
                    if (out[i] && !ref[i]) ++(wall ? exW : exF);
                    if (!out[i] && ref[i]) ++(wall ? miW : miF);
                }
            }
            std::printf("%-10s %-10s %13.2f%% %13.2f%% %13.2f%% %13.2f%%\n", m.name, names[k], 100.0 * exF / visF, 100.0 * miF / visF,
                        100.0 * exW / visW, 100.0 * miW / visW);
        }
    }

    std::printf("\n== symmetry: of floor pairs where at least one side sees the other, share that is one-way (radius 16) ==\n");
    for (Map& m : maps) {
        Grid g = m.g;
        g.w = g.h = 48;  // 取左上角 48x48，两两比较
        std::vector<uint8_t> wall(48 * 48);
        for (int y = 0; y < 48; ++y)
            for (int x = 0; x < 48; ++x) wall[y * 48 + x] = m.g.wall[m.g.index(x, y)];
        g.wall = wall;
        int n = 48 * 48;
        for (int k : {0, 1, 2}) {
            std::vector<std::vector<uint8_t>> v(n);
            for (int i = 0; i < n; ++i)
                if (!g.wall[i]) fns[k](g, i % 48, i / 48, 16, v[i]);
            long long seen = 0, oneWay = 0;
            for (int a = 0; a < n; ++a)
                for (int b = a + 1; b < n; ++b) {
                    if (g.wall[a] || g.wall[b] || !(v[a][b] || v[b][a])) continue;
                    ++seen, oneWay += v[a][b] != v[b][a];
                }
            std::printf("%-10s %-10s %6.2f%%  (%lld of %lld pairs)\n", m.name, names[k], 100.0 * oneWay / seen, oneWay, seen);
        }
    }

    std::printf("\n== single line-of-sight checks (canSee), dungeon map ==\n");
    for (int dist : {4, 8, 16, 32}) {
        std::vector<std::array<int, 4>> q;
        while (q.size() < 20000) {
            int ax = rng.below(128), ay = rng.below(128);
            double ang = rng.nextDouble() * 6.2831853;
            int bx = ax + int(std::lround(dist * std::cos(ang))), by = ay + int(std::lround(dist * std::sin(ang)));
            if (maps[2].g.inside(bx, by)) q.push_back({ax, ay, bx, by});
        }
        double best = 1e30;
        int vis = 0;
        for (int round = 0; round < 5; ++round) {
            vis = 0;
            auto t0 = Clock::now();
            for (auto& e : q) vis += canSee(maps[2].g, e[0], e[1], e[2], e[3]);
            best = std::min(best, std::chrono::duration<double, std::nano>(Clock::now() - t0).count() / q.size());
        }
        std::printf("distance %2d: %6.0f ns per check (%d%% visible)\n", dist, best, vis * 100 / int(q.size()));
    }
}
