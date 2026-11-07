// 影响力图：各种算法的开销、每帧只传播一轮带来的延迟、盖章穿墙的程度、避开火力的路径（不由 CMake 构建）：
//   c++ -std=c++20 -O2 -Icpp/common cpp/ai/influence/influence_bench.cpp -o influence_bench && ./influence_bench
#include <chrono>
#include <cstdio>

#include "../../foundations/random/random.h"
#include "influence.h"

using namespace ga;
using namespace ga::influence;
using Clock = std::chrono::steady_clock;

static Map makeMap(Pcg32& rng, int w, int h) {  // 15% 零散的墙 + 若干道长墙
    Map m(w, h);
    for (auto& c : m.wall) c = rng.nextFloat() < 0.15f;
    for (int k = 0; k < w * h / 400; ++k) {
        int x = rng.below(w), y = rng.below(h), len = 5 + rng.below(w / 3);
        bool hor = rng.below(2);
        for (int t = 0; t < len; ++t) {
            int cx = hor ? x + t : x, cy = hor ? y : y + t;
            if (cx < w && cy < h) m.wall[m.index(cx, cy)] = 1;
        }
    }
    return m;
}

static std::vector<Source> openSources(Pcg32& rng, const Map& m, int n) {
    std::vector<Source> s;
    while (int(s.size()) < n) {
        int x = rng.below(m.w), y = rng.below(m.h);
        if (m.open(x, y)) s.push_back({x, y, 1});
    }
    return s;
}

template <class F>
static double minMs(int rounds, F&& f) {
    double best = 1e30;
    for (int r = 0; r < rounds; ++r) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    }
    return best;
}

int main() {
    const float R = 28;  // 盖章半径；传播的衰减取 0.01^(1/R)，影响力在 R 格处降到阈值 0.01，两者的范围相当
    Decay decay(std::pow(0.01f, 1 / R), 0.01f);

    // ---- 1) 开销 ----
    std::printf("1) full recompute, ms (min of rounds). stamp radius %.0f, propagation decay %.3f per cell\n", R, decay.straight);
    std::printf("  %-9s %-6s %10s %10s | %10s %10s %10s %12s %8s %8s\n", "map", "units", "brute", "stamp", "dijkstra", "jacobi", "sweeps",
                "1 jacobi rnd", "rounds", "passes");
    for (int size : {64, 128, 256})
        for (int units : {10, 50, 200}) {
            Pcg32 rng(size * 1000 + units);
            Map m = makeMap(rng, size, size);
            auto src = openSources(rng, m, units);
            Field f, seeds, scratch;
            int rounds = 0, passes = 0;
            int reps = size <= 64 ? 20 : size <= 128 ? 8 : 3;
            double brute = minMs(reps, [&] { stampBrute(m, src, R, f); });
            Stamper st(R);
            double stamp = minMs(reps, [&] { st.stamp(m, src, f); });
            double dijk = minMs(reps, [&] { geodesicBrute(m, src, decay, f); });
            double jac = minMs(reps, [&] { rounds = propagate(m, src, decay, f); });
            double sw = minMs(reps, [&] { passes = propagateSweeps(m, src, decay, f); });
            seed(m, src, seeds);
            double one = minMs(reps * 3, [&] { propagateStep(m, seeds, decay, 1, f, scratch); });
            std::printf("  %3dx%-5d %-6d %10.3f %10.3f | %10.3f %10.3f %10.3f %12.3f %8d %8d\n", size, size, units, brute, stamp, dijk, jac, sw, one,
                        rounds, passes);
        }

    // ---- 2) 盖章穿墙：盖章覆盖到、但沿路走超过半径才能到的格子 ----
    std::printf("\n2) stamp leaking through walls (64x64 maps, 20 seeds, 1 unit each)\n");
    {
        long long stampCells = 0, leaked = 0, strongLeak = 0;
        for (int seed = 1; seed <= 20; ++seed) {
            Pcg32 rng(seed);
            Map m = makeMap(rng, 64, 64);
            auto src = openSources(rng, m, 1);
            Field s, g;
            stampBrute(m, src, R, s);
            geodesicBrute(m, src, decay, g);
            for (size_t i = 0; i < s.size(); ++i) {
                if (m.wall[i] || s[i] <= 0) continue;
                ++stampCells;
                if (g[i] == 0) {
                    ++leaked;
                    if (s[i] > 0.5f) ++strongLeak;
                }
            }
        }
        std::printf("  %.1f%% of the cells the stamp reaches are more than %.0f steps away by path; %.1f%% of them with stamp value > 0.5\n",
                    100.0 * leaked / stampCells, R, 100.0 * strongLeak / std::max(1LL, leaked));
    }

    // ---- 3) 每帧只传播一轮：移动单位的影响力落后多少 ----
    std::printf("\n3) one unit moving at 0.25 cells/frame on an open 96x32 map; error vs fully converged map, averaged over frames 100-300\n");
    std::printf("  %-10s %-14s %14s %14s %16s %16s\n", "momentum", "rounds/frame", "mean |error|", "max |error|", "8 cells ahead", "8 cells behind");
    for (float momentum : {1.0f, 0.5f, 0.2f})
        for (int perFrame : {1, 2, 4}) {
            Map m(96, 32);
            Field cur, seeds, scratch, target;
            std::vector<Source> s = {{4, 16, 1}};
            seed(m, s, seeds);
            cur = seeds;
            double meanErr = 0, maxErr = 0, ahead = 0, behind = 0;
            int samples = 0;
            for (int frame = 0; frame < 300; ++frame) {
                s[0].x = 4 + int(frame * 0.25f);
                seed(m, s, seeds);
                for (int k = 0; k < perFrame; ++k) propagateStep(m, seeds, decay, momentum, cur, scratch);
                if (frame >= 100) {
                    propagate(m, s, decay, target);
                    double sum = 0, mx = 0;
                    for (size_t i = 0; i < cur.size(); ++i)
                        sum += std::abs(cur[i] - target[i]), mx = std::max(mx, double(std::abs(cur[i] - target[i])));
                    meanErr += sum / cur.size(), maxErr = std::max(maxErr, mx);
                    int a = m.index(s[0].x + 8, 16), b = m.index(s[0].x - 8, 16);  // 前方 / 后方 8 格：实际值 ÷ 应有的值
                    ahead += cur[a] / target[a], behind += cur[b] / target[b];
                    ++samples;
                }
            }
            std::printf("  %-10.1f %-14d %14.4f %14.3f %15.0f%% %15.0f%%\n", momentum, perFrame, meanErr / samples, maxErr, 100 * ahead / samples,
                        100 * behind / samples);
        }

    // ---- 4) 避开火力的路径 ----
    std::printf("\n4) safe path across a 64x48 map with 6 enemies (decay 0.85), 300 maps\n");
    std::printf("  %-8s %10s %12s %14s %12s\n", "danger", "length", "exposure", "max threat", "ms");
    for (float danger : {0.0f, 1.0f, 3.0f, 10.0f, 30.0f, 100.0f}) {
        Pcg32 rng(77);
        double len = 0, exposure = 0, peak = 0, ms = 0;
        int n = 0;
        for (int t = 0; t < 300; ++t) {
            Map m = makeMap(rng, 64, 48);
            auto enemies = openSources(rng, m, 6);
            Field threat;
            propagate(m, enemies, Decay(0.85f), threat);
            int start = m.index(0, rng.below(48)), goal = m.index(63, rng.below(48));
            m.wall[start] = m.wall[goal] = 0;
            PathResult p;
            double t1 = minMs(1, [&] { p = safePath(m, threat, start, goal, danger); });
            if (p.cells.empty()) continue;
            double l = 0, e = 0, mx = 0;
            for (size_t k = 1; k < p.cells.size(); ++k) {
                int a = p.cells[k - 1], b = p.cells[k];
                double step = (a % m.w != b % m.w && a / m.w != b / m.w) ? std::sqrt(2.0) : 1.0;
                l += step, e += step * threat[b], mx = std::max(mx, double(threat[b]));
            }
            len += l, exposure += e, peak += mx, ms += t1, ++n;
        }
        std::printf("  %-8.0f %10.1f %12.2f %14.2f %12.3f\n", danger, len / n, exposure / n, peak / n, ms / n);
    }
}
